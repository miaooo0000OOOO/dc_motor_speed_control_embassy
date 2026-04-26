#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::gpio::AfioRemap;
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPreDiv, PllSource, Sysclk,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::qei::{Config as QeiConfig, Qei};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::{Ch1, Ch2, Ch3, Ch4};
use embassy_stm32::Config;
use embassy_time::{Duration, Instant, Ticker};

use {defmt_rtt as _, panic_probe as _};

/// 编码器每转计数值（11线 × 4倍频 × 21减速比）
const COUNTS_PER_REV: f32 = (11 * 21 * 4) as f32; // 924.0

#[embassy_executor::main]
async fn main(spawner: Spawner) {
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

    // ── TIM2 PWM 初始化（PB10 CH3 / PB11 CH4，AFIO 部分重映射2）──
    let ch3: PwmPin<'_, embassy_stm32::peripherals::TIM2, Ch3, AfioRemap<2>> =
        PwmPin::new(p.PB10, embassy_stm32::gpio::OutputType::PushPull);
    let ch4: PwmPin<'_, embassy_stm32::peripherals::TIM2, Ch4, AfioRemap<2>> =
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
    let ch3_pwm = &mut channels.ch3;
    let ch4_pwm = &mut channels.ch4;
    ch3_pwm.enable();
    ch4_pwm.enable();

    // ── 固定占空比 70% 正转 ──
    let max_duty = ch3_pwm.max_duty_cycle();
    let compare = 70 * max_duty / 100;
    ch3_pwm.set_duty_cycle(compare);
    ch4_pwm.set_duty_cycle(0);
    defmt::info!("PWM fixed at 70%");

    // ── 编码器（QEI，TIM1，PA8/PA9）──
    let qei_config = QeiConfig {
        ch1_pull: Pull::Up,
        ch2_pull: Pull::Up,
        ..Default::default()
    };
    let qei = Qei::new::<Ch1, Ch2, AfioRemap<0>>(p.TIM1, p.PA8, p.PA9, qei_config);

    // 启动采样任务
    spawner.spawn(sample_task(qei)).ok();

    // 主循环空闲
    loop {
        embassy_time::Timer::after_millis(500).await;
    }
}

/// 执行 M/T 测速并打印 RPM
#[embassy_executor::task]
async fn sample_task(qei: Qei<'static, embassy_stm32::peripherals::TIM1>) {
    let mut last_count: u16 = qei.count();
    let mut last_time: u64 = Instant::now().as_micros();

    let mut ticker = Ticker::every(Duration::from_millis(100));
    loop {
        ticker.next().await;

        let count = qei.count();
        let time = Instant::now().as_micros();

        // 自动处理溢出
        let delta_count = count.wrapping_sub(last_count) as i16 as i32;
        let delta_time = time.wrapping_sub(last_time);

        if delta_time > 0 {
            let rpm = (delta_count as f32 * 1_000_000.0 * 60.0)
                / (COUNTS_PER_REV * delta_time as f32);
            defmt::info!("RPM: {}", rpm);
        }

        last_count = count;
        last_time = time;
    }
}
