#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPreDiv, PllSource, Sysclk,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::gpio::AfioRemap;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::Config;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

/// 梯形循环更新周期（ms）
const UPDATE_INTERVAL_MS: u64 = 50;
/// 每周期占空比变化量（%），决定斜率绝对值
const RAMP_STEP: i32 = 2;
/// 平顶维持时间（ms）
const PLATEAU_MS: u64 = 2000;

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

    // ── TIM2 PWM 初始化 ──
    // PB10 -> TIM2_CH3 (AFIO 部分重映射2)
    // PB11 -> TIM2_CH4 (AFIO 部分重映射2)
    let ch3: PwmPin<'_, embassy_stm32::peripherals::TIM2, embassy_stm32::timer::Ch3, AfioRemap<2>> =
        PwmPin::new(p.PB10, embassy_stm32::gpio::OutputType::PushPull);
    let ch4: PwmPin<'_, embassy_stm32::peripherals::TIM2, embassy_stm32::timer::Ch4, AfioRemap<2>> =
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

    // 状态机变量
    let mut duty: i32 = 0;            // 当前占空比：-100 ~ +100
    let mut state: u8 = 0;            // 梯形阶段
    let mut plateau_counter: u64 = 0; // 平顶计时

    defmt::info!("L298N trapezoid PWM test started");
    defmt::info!("Ramp step: {}%/{}ms, Plateau: {}ms", RAMP_STEP, UPDATE_INTERVAL_MS, PLATEAU_MS);

    loop {
        Timer::after_millis(UPDATE_INTERVAL_MS).await;

        match state {
            // ── 阶段0：0% -> +100% ──
            0 => {
                duty += RAMP_STEP;
                if duty >= 100 {
                    duty = 100;
                    state = 1;
                    plateau_counter = 0;
                    defmt::info!("Reach +100%, holding...");
                }
            }
            // ── 阶段1：维持 +100% ──
            1 => {
                plateau_counter += UPDATE_INTERVAL_MS;
                if plateau_counter >= PLATEAU_MS {
                    state = 2;
                    defmt::info!("Start ramp +100% -> -100%");
                }
            }
            // ── 阶段2：+100% -> -100% ──
            2 => {
                duty -= RAMP_STEP;
                if duty <= -100 {
                    duty = -100;
                    state = 3;
                    plateau_counter = 0;
                    defmt::info!("Reach -100%, holding...");
                }
            }
            // ── 阶段3：维持 -100% ──
            3 => {
                plateau_counter += UPDATE_INTERVAL_MS;
                if plateau_counter >= PLATEAU_MS {
                    state = 4;
                    defmt::info!("Start ramp -100% -> 0%");
                }
            }
            // ── 阶段4：-100% -> 0% ──
            4 => {
                duty += RAMP_STEP;
                if duty >= 0 {
                    duty = 0;
                    state = 0;
                    defmt::info!("Reach 0%, cycle restart");
                }
            }
            _ => state = 0,
        }

        // ── 输出 PWM ──
        let max_duty = ch3_pwm.max_duty_cycle();
        let duty_abs = duty.unsigned_abs();
        let compare = duty_abs * max_duty / 100;

        if duty > 0 {
            // 正转：IN1 有 PWM，IN2 = 0
            ch3_pwm.set_duty_cycle(compare);
            ch4_pwm.set_duty_cycle(0);
        } else if duty < 0 {
            // 反转：IN1 = 0，IN2 有 PWM
            ch3_pwm.set_duty_cycle(0);
            ch4_pwm.set_duty_cycle(compare);
        } else {
            // 停止
            ch3_pwm.set_duty_cycle(0);
            ch4_pwm.set_duty_cycle(0);
        }

        defmt::info!("PWM setpoint = {}%", duty);
    }
}
