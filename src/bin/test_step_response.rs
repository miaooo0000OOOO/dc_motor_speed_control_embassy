#![no_std]
#![no_main]

use core::fmt::Write;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{AfioRemap, AfioRemapBool, Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{TIM1, TIM2, USART1};
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPreDiv, PllSource, Sysclk,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::qei::{Config as QeiConfig, Qei};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm, SimplePwmChannels};
use embassy_stm32::timer::{Ch1, Ch2, Ch3, Ch4};
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::Config;
use embassy_time::{Instant, Timer};
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

/// 编码器每转计数值（11线 × 4倍频 × 21减速比）
const COUNTS_PER_REV: f32 = (11 * 21 * 4) as f32; // 924.0
/// 转速测量采样周期（ms）——10ms 以捕捉快速上升沿（τ 实测约 16ms）
const SAMPLE_PERIOD_MS: u64 = 10;
/// 每次阶跃实验的总时长（ms）——1000ms 已覆盖 60τ，足够进入稳态
const STEP_DURATION_MS: u64 = 1000;
/// 阶跃前预稳态时长（ms），确保电机已静止
const PRE_SETTLE_MS: u64 = 300;
/// 阶跃后保持时长（ms），覆盖完整的上升过程
const HOLD_DURATION_MS: u64 = 800;
/// 轮间停转等待（ms），消除惯性并散热
const ROUND_PAUSE_MS: u64 = 1000;
/// 每组占空比的重复实验次数
const REPETITIONS: usize = 5;
/// 实验占空比序列（%），从死区以上开始，覆盖典型工作区
const DUTY_LEVELS: &[i32] = &[40, 60, 80, 100];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // ── 时钟配置：72 MHz ──
    let mut config = Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz(8_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll = Some(Pll {
        src: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL9,
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV2;
    config.rcc.apb2_pre = APBPrescaler::DIV1;
    let p = embassy_stm32::init(config);

    // ── L298N ENA 使能（始终高电平）──
    let _ena = Output::new(p.PA7, Level::High, Speed::Low);

    // ── 编码器（QEI，TIM1，PA8/PA9）──
    let qei_config = QeiConfig {
        ch1_pull: Pull::Up,
        ch2_pull: Pull::Up,
        ..Default::default()
    };
    let qei = Qei::new::<Ch1, Ch2, AfioRemap<0>>(p.TIM1, p.PA8, p.PA9, qei_config);

    // ── TIM2 PWM（PB10 CH3 / PB11 CH4，AFIO 部分重映射2）──
    let ch3: PwmPin<'_, TIM2, Ch3, AfioRemap<2>> =
        PwmPin::new(p.PB10, embassy_stm32::gpio::OutputType::PushPull);
    let ch4: PwmPin<'_, TIM2, Ch4, AfioRemap<2>> =
        PwmPin::new(p.PB11, embassy_stm32::gpio::OutputType::PushPull);

    let pwm = SimplePwm::new(
        p.TIM2,
        None,
        None,
        Some(ch3),
        Some(ch4),
        Hertz::khz(20),
        CountingMode::EdgeAlignedUp,
    );
    let mut channels = pwm.split();
    channels.ch3.enable();
    channels.ch4.enable();

    // ── UART1（PB7 RX / PB6 TX，AFIO 重映射）──
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 115200;
    let mut uart = Uart::new_blocking::<USART1, AfioRemapBool<true>>(
        p.USART1,
        p.PB7,
        p.PB6,
        uart_config,
    )
    .unwrap();

    // ── OLED 初始化 ──
    let mut i2c_config = embassy_stm32::i2c::Config::default();
    i2c_config.frequency = Hertz::khz(400);
    let i2c = embassy_stm32::i2c::I2c::new_blocking(p.I2C1, p.PB8, p.PB9, i2c_config);
    let mut oled = dc_motor_speed_control_embassy::sh1106::Sh1106::new(i2c);
    oled.init();
    oled.clear();
    oled.draw_string_8x16(0, 0, "STEP RESPONSE", &dc_motor_speed_control_embassy::font::F8X16);
    oled.draw_string_6x8(2, 0, "Wait for cmd...", &dc_motor_speed_control_embassy::font::F6X8);

    defmt::info!("Step response test ready. Waiting for 'S' command...");

    // ── 等待 PC 端发送开始命令 ──
    let mut rx_buf = [0u8; 1];
    uart.blocking_read(&mut rx_buf).unwrap();
    if rx_buf[0] != b'S' {
        defmt::info!("Unknown command: {}. Exiting.", rx_buf[0]);
        return;
    }

    defmt::info!(
        "Starting step response test: duty levels={:?}, reps={}",
        DUTY_LEVELS,
        REPETITIONS
    );

    // 发送实验元信息
    send_meta(&mut uart).await;

    // ── 主实验循环 ──
    let total_runs = DUTY_LEVELS.len() * REPETITIONS;
    let mut run_idx = 0usize;

    for &duty in DUTY_LEVELS {
        for rep in 1..=REPETITIONS {
            run_idx += 1;

            // OLED 显示进度
            let mut buf = String::<32>::new();
            let _ = write!(buf, "D={:3}% {}/{}", duty, run_idx, total_runs);
            oled.draw_string_6x8(3, 0, &buf, &dc_motor_speed_control_embassy::font::F6X8);

            defmt::info!("--- Run {}/{}: duty={}% rep={} ---", run_idx, total_runs, duty, rep);

            // 发送当前运行标记
            send_run_header(&mut uart, duty, rep);

            // 1. 归零并等待静止
            set_pwm(&mut channels, 0);
            Timer::after_millis(PRE_SETTLE_MS).await;

            // 2. 施加阶跃并高速采集
            let start_time = Instant::now();
            set_pwm(&mut channels, duty);

            // 初始化编码器测量基准
            let mut last_count = qei.count();
            let mut last_time = Instant::now().as_micros();

            // 记录阶跃后数据
            let samples = (STEP_DURATION_MS / SAMPLE_PERIOD_MS) as usize;
            for _ in 0..samples {
                Timer::after_millis(SAMPLE_PERIOD_MS).await;

                let elapsed = start_time.elapsed().as_millis();
                let (rpm, new_count, new_time) = measure_rpm_single(&qei, last_count, last_time);
                last_count = new_count;
                last_time = new_time;

                // 输出: time_ms,rpm,duty
                let mut line = String::<32>::new();
                let _ = write!(line, "{},{},{}", elapsed, rpm as i32, duty);
                uart.blocking_write(line.as_bytes()).unwrap();
                uart.blocking_write(b"\n").unwrap();

                // 前 200ms 使用 defmt 高频输出，便于调试
                if elapsed < 200 {
                    defmt::info!("t={}ms rpm={} duty={}", elapsed, rpm as i32, duty);
                }
            }

            // 3. 结束当前运行
            send_run_footer(&mut uart, duty, rep);

            // 4. 归零并等待，消除惯性和散热
            set_pwm(&mut channels, 0);
            Timer::after_millis(ROUND_PAUSE_MS).await;
        }
    }

    // ── 全部完成 ──
    uart.blocking_write(b"ALL_DONE\n").unwrap();
    defmt::info!("All step response tests completed.");

    oled.clear();
    oled.draw_string_8x16(0, 0, "TEST DONE", &dc_motor_speed_control_embassy::font::F8X16);
    let mut buf = String::<32>::new();
    let _ = write!(buf, "{} runs saved", total_runs);
    oled.draw_string_6x8(3, 0, &buf, &dc_motor_speed_control_embassy::font::F6X8);

    loop {
        Timer::after_secs(1).await;
    }
}

/// 发送实验元信息
async fn send_meta(uart: &mut Uart<'_, embassy_stm32::mode::Blocking>) {
    let mut buf = String::<128>::new();
    let _ = write!(
        buf,
        "META,sample_period_ms={},hold_ms={},pause_ms={},reps={},duty_levels=\n",
        SAMPLE_PERIOD_MS, HOLD_DURATION_MS, ROUND_PAUSE_MS, REPETITIONS
    );
    uart.blocking_write(buf.as_bytes()).unwrap();
    for (i, &d) in DUTY_LEVELS.iter().enumerate() {
        let mut tmp = String::<8>::new();
        let _ = write!(tmp, "{}{}", if i == 0 { "" } else { "," }, d);
        uart.blocking_write(tmp.as_bytes()).unwrap();
    }
    uart.blocking_write(b"\n").unwrap();
}

/// 发送单次运行头部标记
fn send_run_header(uart: &mut Uart<'_, embassy_stm32::mode::Blocking>, duty: i32, rep: usize) {
    let mut buf = String::<32>::new();
    let _ = write!(buf, "START,duty={},rep={}", duty, rep);
    uart.blocking_write(buf.as_bytes()).unwrap();
    uart.blocking_write(b"\n").unwrap();
}

/// 发送单次运行尾部标记
fn send_run_footer(uart: &mut Uart<'_, embassy_stm32::mode::Blocking>, duty: i32, rep: usize) {
    let mut buf = String::<32>::new();
    let _ = write!(buf, "END,duty={},rep={}", duty, rep);
    uart.blocking_write(buf.as_bytes()).unwrap();
    uart.blocking_write(b"\n").unwrap();
}

/// 设置 PWM 输出（duty: 0 ~ 100，仅正向）
fn set_pwm(channels: &mut SimplePwmChannels<'_, TIM2>, duty: i32) {
    let max_duty = channels.ch3.max_duty_cycle();
    let duty_abs = duty.unsigned_abs() as u32;
    let compare = duty_abs.saturating_mul(max_duty) / 100;

    if duty > 0 {
        channels.ch3.set_duty_cycle(compare);
        channels.ch4.set_duty_cycle(0);
    } else {
        channels.ch3.set_duty_cycle(0);
        channels.ch4.set_duty_cycle(0);
    }
}

/// 单次转速测量（rpm），基于 ~50ms 的瞬时速度
///
/// 利用编码器计数值差分，在调用间隔约为 SAMPLE_PERIOD_MS 时，
/// 可直接得到该周期内的平均转速。
///
/// 返回 (rpm, new_count, new_time)，调用方需保存 new_count/new_time 作为下一次基准。
fn measure_rpm_single(qei: &Qei<'_, TIM1>, last_count: u16, last_time: u64) -> (f32, u16, u64) {
    let count = qei.count();
    let time = Instant::now().as_micros();

    let delta_count = count.wrapping_sub(last_count) as i16 as i32;
    let delta_time = time.wrapping_sub(last_time);

    let rpm = if delta_time > 0 {
        (delta_count as f32 * 1_000_000.0 * 60.0) / (COUNTS_PER_REV * delta_time as f32)
    } else {
        0.0
    };

    (rpm, count, time)
}
