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
/// 转速测量采样间隔（ms）
const MEASURE_INTERVAL_MS: u64 = 200;
/// 每点采样次数
const MEASURE_SAMPLES: u8 = 5;
/// 每点稳定等待时间（ms），包含测量时间
const SETTLE_MS: u64 = 2000;
/// PWM 占空比步长（%）
const STEP_DUTY: usize = 5;
/// 往返扫描轮数
const ROUNDS: usize = 3;

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

    defmt::info!("Speed-Voltage test ready. Waiting for 'S' command...");

    // ── 等待 PC 端发送开始命令 ──
    let mut rx_buf = [0u8; 1];
    uart.blocking_read(&mut rx_buf).unwrap();
    if rx_buf[0] != b'S' {
        defmt::info!("Unknown command: {}. Exiting.", rx_buf[0]);
        return;
    }

    defmt::info!("Starting test: {} rounds, {}% step", ROUNDS, STEP_DUTY);

    // ── 多轮往返扫描 ──
    for round in 0..ROUNDS {
        defmt::info!("--- Round {}/{} ---", round + 1, ROUNDS);

        // 正向：0% → 100%
        for duty in (0..=100).step_by(STEP_DUTY) {
            set_pwm(&mut channels, duty as i32);
            Timer::after_millis(SETTLE_MS).await;
            let rpm = measure_rpm(&qei).await;
            send_line(&mut uart, duty as i32, rpm);
            defmt::info!("FWD duty={}% rpm={}", duty, rpm as i32);
        }

        // 反向：100% → 0%
        let steps = 100 / STEP_DUTY;
        for i in 0..=steps {
            let duty = 100 - i * STEP_DUTY;
            set_pwm(&mut channels, duty as i32);
            Timer::after_millis(SETTLE_MS).await;
            let rpm = measure_rpm(&qei).await;
            send_line(&mut uart, duty as i32, rpm);
            defmt::info!("REV duty={}% rpm={}", duty, rpm as i32);
        }
    }

    // ── 结束 ──
    set_pwm(&mut channels, 0);
    uart.blocking_write(b"DONE\n").unwrap();
    defmt::info!("Test completed.");

    loop {
        Timer::after_secs(1).await;
    }
}

/// 设置 PWM 输出（duty: 0 ~ 100）
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

/// 测量平均转速（rpm）
async fn measure_rpm(qei: &Qei<'_, TIM1>) -> f32 {
    let mut sum_rpm = 0.0f32;
    let mut last_count = qei.count();
    let mut last_time = Instant::now().as_micros();

    for _ in 0..MEASURE_SAMPLES {
        Timer::after_millis(MEASURE_INTERVAL_MS).await;

        let count = qei.count();
        let time = Instant::now().as_micros();

        let delta_count = count.wrapping_sub(last_count) as i16 as i32;
        let delta_time = time.wrapping_sub(last_time);

        let rpm = if delta_time > 0 {
            (delta_count as f32 * 1_000_000.0 * 60.0) / (COUNTS_PER_REV * delta_time as f32)
        } else {
            0.0
        };
        sum_rpm += rpm;

        last_count = count;
        last_time = time;
    }

    sum_rpm / MEASURE_SAMPLES as f32
}

/// 通过 UART 发送一行数据：duty,rpm\n
fn send_line(uart: &mut Uart<'_, embassy_stm32::mode::Blocking>, duty: i32, rpm: f32) {
    let mut buf = String::<32>::new();
    let _ = write!(buf, "{},{}\n", duty, rpm as i32);
    uart.blocking_write(buf.as_bytes()).unwrap();
}
