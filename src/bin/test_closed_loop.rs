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

use dc_motor_speed_control_embassy::controller::CompositeController;

// ═══════════════════════════════════════════════════════════════════════════
// 系统常数
// ═══════════════════════════════════════════════════════════════════════════
/// 编码器每转计数值（11线 × 4倍频 × 21减速比）
const COUNTS_PER_REV: f32 = (11 * 21 * 4) as f32; // 924.0
/// 数据采样周期（ms）——10ms 保证上升沿分辨力
const SAMPLE_PERIOD_MS: u64 = 10;
/// 控制周期（ms）——保持与主程序一致 50ms，PI 参数无需换算
const CONTROL_PERIOD_MS: u64 = 50;
/// 控制周期内包含的采样周期数
const SAMPLES_PER_CONTROL: u64 = CONTROL_PERIOD_MS / SAMPLE_PERIOD_MS; // 5
/// 阶跃前预稳态时长（ms），确保电机静止
const PRE_SETTLE_MS: u64 = 300;
/// 阶跃后保持时长（ms），覆盖完整调节过程（>60τ）
const STEP_HOLD_MS: u64 = 1200;
/// 扰动实验总时长（ms）
const DISTURB_TOTAL_MS: u64 = 2000;
/// 扰动施加起始时刻（ms）
const DISTURB_START_MS: u64 = 500;
/// 扰动施加结束时刻（ms）
const DISTURB_END_MS: u64 = 1000;
/// 轮间停转等待（ms）
const ROUND_PAUSE_MS: u64 = 1000;
/// 每组条件的重复实验次数
const REPETITIONS: usize = 5;
/// 闭环阶跃设定值（rpm），覆盖低/中/高速，均高于死区
const STEP_SETPOINTS: &[f32] = &[60.0, 100.0, 140.0];
/// 扰动实验基础设定值（rpm）
const DISTURB_SETPOINT: f32 = 100.0;
/// 阶跃扰动占空比幅度（%）——等效负载突变
const DISTURB_DUTY: f32 = 8.0;
/// 死区转速边界（rpm）
const DEAD_ZONE_RPM: f32 = 14.0;
/// PI 参数（与 main.rs 保持一致，Ts = 50ms）
const PID_KP: f32 = 0.132;
const PID_KI: f32 = 7.770;
const PID_KD: f32 = 0.0;
/// 测速低通滤波系数
const SPEED_FILTER_ALPHA: f32 = 0.6;

// ═══════════════════════════════════════════════════════════════════════════
// 实验类型
// ═══════════════════════════════════════════════════════════════════════════
#[derive(Clone, Copy, Debug, PartialEq)]
enum ExpType {
    Step,
    Disturbance,
}

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
    oled.draw_string_8x16(0, 0, "CLOSED LOOP", &dc_motor_speed_control_embassy::font::F8X16);
    oled.draw_string_6x8(2, 0, "Wait for cmd...", &dc_motor_speed_control_embassy::font::F6X8);

    defmt::info!("Closed-loop response test ready. Waiting for 'C' command...");

    // ── 等待 PC 端发送开始命令 ──
    let mut rx_buf = [0u8; 1];
    uart.blocking_read(&mut rx_buf).unwrap();
    if rx_buf[0] != b'C' {
        defmt::info!("Unknown command: {}. Exiting.", rx_buf[0]);
        return;
    }

    // 发送实验元信息
    send_meta(&mut uart).await;

    // ── 阶段 1：闭环阶跃响应 ──
    let total_step_runs = STEP_SETPOINTS.len() * REPETITIONS;
    let mut run_idx = 0usize;

    for &sp in STEP_SETPOINTS {
        for rep in 1..=REPETITIONS {
            run_idx += 1;
            let mut buf = String::<32>::new();
            let _ = write!(buf, "SP={:3.0} {}/{}", sp, run_idx, total_step_runs);
            oled.draw_string_6x8(3, 0, &buf, &dc_motor_speed_control_embassy::font::F6X8);

            defmt::info!("--- Step run {}/{}: sp={}rpm rep={} ---", run_idx, total_step_runs, sp as i32, rep);
            send_run_header(&mut uart, ExpType::Step, sp, 0.0, rep);
            run_one_experiment(&mut uart, &qei, &mut channels, ExpType::Step, sp, 0.0).await;
            send_run_footer(&mut uart, ExpType::Step, sp, 0.0, rep);

            set_pwm(&mut channels, 0.0);
            Timer::after_millis(ROUND_PAUSE_MS).await;
        }
    }

    // ── 阶段 2：阶跃扰动响应 ──
    for rep in 1..=REPETITIONS {
        let mut buf = String::<32>::new();
        let _ = write!(buf, "DIST {} / {}", rep, REPETITIONS);
        oled.draw_string_6x8(3, 0, &buf, &dc_motor_speed_control_embassy::font::F6X8);

        defmt::info!("--- Disturbance run {}/{}: sp={}rpm disturb={}% ---", rep, REPETITIONS, DISTURB_SETPOINT as i32, DISTURB_DUTY as i32);
        send_run_header(&mut uart, ExpType::Disturbance, DISTURB_SETPOINT, DISTURB_DUTY, rep);
        run_one_experiment(&mut uart, &qei, &mut channels, ExpType::Disturbance, DISTURB_SETPOINT, DISTURB_DUTY).await;
        send_run_footer(&mut uart, ExpType::Disturbance, DISTURB_SETPOINT, DISTURB_DUTY, rep);

        set_pwm(&mut channels, 0.0);
        Timer::after_millis(ROUND_PAUSE_MS).await;
    }

    // ── 全部完成 ──
    uart.blocking_write(b"ALL_DONE\n").unwrap();
    defmt::info!("All closed-loop tests completed.");
    oled.clear();
    oled.draw_string_8x16(0, 0, "TEST DONE", &dc_motor_speed_control_embassy::font::F8X16);
    loop { Timer::after_secs(1).await; }
}

// ═══════════════════════════════════════════════════════════════════════════
// 单次实验运行
// ═══════════════════════════════════════════════════════════════════════════
async fn run_one_experiment(
    uart: &mut Uart<'_, embassy_stm32::mode::Blocking>,
    qei: &Qei<'_, TIM1>,
    channels: &mut SimplePwmChannels<'_, TIM2>,
    exp_type: ExpType,
    setpoint: f32,
    disturb_duty: f32,
) {
    let total_duration = if exp_type == ExpType::Step {
        PRE_SETTLE_MS + STEP_HOLD_MS
    } else {
        DISTURB_TOTAL_MS
    };

    let start_time = Instant::now();

    // 初始化控制器
    let mut controller = CompositeController::with_gains(PID_KP, PID_KI, PID_KD);
    controller.set_integral_limits(-30.0, 30.0);
    controller.set_output_limits(-100.0, 100.0);

    let mut last_count = qei.count();
    let mut last_time = Instant::now().as_micros();
    let mut prev_rpm: f32 = 0.0;
    let mut tick_counter: u64 = 0;
    let mut duty_total: f32 = 0.0;
    let mut duty_ff: f32 = 0.0;
    let mut duty_fb: f32 = 0.0;

    let samples = (total_duration / SAMPLE_PERIOD_MS) as usize;

    for _ in 0..samples {
        Timer::after_millis(SAMPLE_PERIOD_MS).await;
        let elapsed = start_time.elapsed().as_millis();

        // ── 转速测量 ──
        let (raw_rpm, new_count, new_time) = measure_rpm_single(qei, last_count, last_time);
        last_count = new_count;
        last_time = new_time;
        let rpm = SPEED_FILTER_ALPHA * raw_rpm + (1.0 - SPEED_FILTER_ALPHA) * prev_rpm;
        prev_rpm = rpm;

        // ── 设定值与扰动 ──
        let (sp, disturbance) = if exp_type == ExpType::Step {
            if elapsed < PRE_SETTLE_MS {
                (0.0, 0.0)
            } else {
                (setpoint, 0.0)
            }
        } else {
            let dist = if elapsed >= DISTURB_START_MS && elapsed < DISTURB_END_MS {
                disturb_duty
            } else {
                0.0
            };
            (setpoint, dist)
        };

        // ── 控制计算（每 CONTROL_PERIOD_MS）──
        if tick_counter % SAMPLES_PER_CONTROL == 0 {
            if sp.abs() < DEAD_ZONE_RPM {
                controller.reset();
                duty_total = 0.0;
                duty_ff = 0.0;
                duty_fb = 0.0;
            } else {
                duty_total = controller.compute(sp, rpm);
                duty_ff = controller.feedforward_duty(sp);
                duty_fb = duty_total - duty_ff;
            }
        }

        // ── PWM 输出（含扰动叠加）──
        let duty_out = (duty_total + disturbance).clamp(-100.0, 100.0);
        set_pwm(channels, duty_out);

        // ── 串口输出 ──
        // time_ms,sp_rpm,pv_rpm,duty_total,duty_ff,duty_fb,disturbance
        let mut line = String::<48>::new();
        let _ = write!(
            line,
            "{},{},{},{:.1},{:.1},{:.1},{:.1}",
            elapsed,
            sp as i32,
            rpm as i32,
            duty_total,
            duty_ff,
            duty_fb,
            disturbance,
        );
        uart.blocking_write(line.as_bytes()).unwrap();
        uart.blocking_write(b"\n").unwrap();

        tick_counter += 1;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 串口协议辅助函数
// ═══════════════════════════════════════════════════════════════════════════
async fn send_meta(uart: &mut Uart<'_, embassy_stm32::mode::Blocking>) {
    let mut buf = String::<256>::new();
    let _ = write!(
        buf,
        "META,exp=closed_loop,sample_period_ms={},control_period_ms={},pre_settle_ms={},step_hold_ms={},disturb_total_ms={},disturb_start_ms={},disturb_end_ms={},reps={},step_setpoints=",
        SAMPLE_PERIOD_MS, CONTROL_PERIOD_MS, PRE_SETTLE_MS, STEP_HOLD_MS,
        DISTURB_TOTAL_MS, DISTURB_START_MS, DISTURB_END_MS, REPETITIONS,
    );
    uart.blocking_write(buf.as_bytes()).unwrap();
    for (i, &sp) in STEP_SETPOINTS.iter().enumerate() {
        let mut tmp = String::<16>::new();
        let _ = write!(tmp, "{}{:.0}", if i == 0 { "" } else { "," }, sp);
        uart.blocking_write(tmp.as_bytes()).unwrap();
    }
    uart.blocking_write(b",disturb_duty=").unwrap();
    let mut tmp = String::<8>::new();
    let _ = write!(tmp, "{:.1}", DISTURB_DUTY);
    uart.blocking_write(tmp.as_bytes()).unwrap();
    uart.blocking_write(b"\n").unwrap();
}

fn send_run_header(
    uart: &mut Uart<'_, embassy_stm32::mode::Blocking>,
    exp_type: ExpType,
    setpoint: f32,
    disturb: f32,
    rep: usize,
) {
    let mut buf = String::<48>::new();
    let typ = match exp_type {
        ExpType::Step => "STEP",
        ExpType::Disturbance => "DISTURB",
    };
    let _ = write!(buf, "START,typ={},sp={:.0},disturb={:.1},rep={}", typ, setpoint, disturb, rep);
    uart.blocking_write(buf.as_bytes()).unwrap();
    uart.blocking_write(b"\n").unwrap();
}

fn send_run_footer(
    uart: &mut Uart<'_, embassy_stm32::mode::Blocking>,
    exp_type: ExpType,
    setpoint: f32,
    disturb: f32,
    rep: usize,
) {
    let mut buf = String::<48>::new();
    let typ = match exp_type {
        ExpType::Step => "STEP",
        ExpType::Disturbance => "DISTURB",
    };
    let _ = write!(buf, "END,typ={},sp={:.0},disturb={:.1},rep={}", typ, setpoint, disturb, rep);
    uart.blocking_write(buf.as_bytes()).unwrap();
    uart.blocking_write(b"\n").unwrap();
}

// ═══════════════════════════════════════════════════════════════════════════
// 硬件辅助函数
// ═══════════════════════════════════════════════════════════════════════════
fn set_pwm(channels: &mut SimplePwmChannels<'_, TIM2>, duty: f32) {
    let max_duty = channels.ch3.max_duty_cycle();
    let duty_abs = duty.abs().clamp(0.0, 100.0) as u32;
    let compare = duty_abs.saturating_mul(max_duty) / 100;

    if duty > 0.0 {
        channels.ch3.set_duty_cycle(compare);
        channels.ch4.set_duty_cycle(0);
    } else if duty < 0.0 {
        channels.ch3.set_duty_cycle(0);
        channels.ch4.set_duty_cycle(compare);
    } else {
        channels.ch3.set_duty_cycle(0);
        channels.ch4.set_duty_cycle(0);
    }
}

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
