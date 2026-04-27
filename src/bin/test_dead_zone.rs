#![no_std]
#![no_main]

use core::fmt::Write;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{AfioRemap, Level, Output, Pull, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c, Master};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::{TIM1, TIM2};
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPreDiv, PllSource, Sysclk,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::qei::{Config as QeiConfig, Qei};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm, SimplePwmChannels};
use embassy_stm32::timer::{Ch1, Ch2, Ch3, Ch4};
use embassy_stm32::Config;
use embassy_time::{Instant, Timer};
use heapless::String;

use dc_motor_speed_control_embassy::font::{F6X8, F8X16};
use dc_motor_speed_control_embassy::sh1106::Sh1106;
use {defmt_rtt as _, panic_probe as _};

/// 编码器每转计数值（11线 × 4倍频 × 21减速比）
const COUNTS_PER_REV: f32 = (11 * 21 * 4) as f32; // 924.0
/// 认为电机开始转动 / 已经停止的转速阈值（rpm）
const RPM_THRESHOLD: f32 = 2.0;
/// PWM 占空比步长（%）
const STEP_DUTY: i32 = 1;
/// 每次改变占空比后的测量采样间隔（ms）
const MEASURE_INTERVAL_MS: u64 = 150;
/// 去程每步采样次数
const SAMPLES_UP: u8 = 3;
/// 回程每步采样次数（可略少，加快测试）
const SAMPLES_DOWN: u8 = 2;
/// 连续几次超/低于阈值才确认边界
const STABLE_COUNT: u8 = 2;
/// 往返扫描轮数
const ROUNDS: usize = 3;
/// 轮间停转等待（ms）
const ROUND_PAUSE_MS: u64 = 2000;

type Oled = Sh1106<I2c<'static, Blocking, Master>>;

#[derive(Clone, Copy)]
struct Boundary {
    duty: i32,
    rpm: f32,
}

#[derive(Clone, Copy)]
struct RoundResult {
    pos_start: Boundary, // 正向去程：启动边界
    pos_stop: Boundary,  // 正向回程：停止边界
    neg_start: Boundary, // 负向去程：启动边界
    neg_stop: Boundary,  // 负向回程：停止边界
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

    // ── OLED 初始化 ──
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = Hertz::khz(400);
    let i2c = I2c::new_blocking(p.I2C1, p.PB8, p.PB9, i2c_config);
    let mut oled = Sh1106::new(i2c);
    oled.init();
    oled.clear();

    oled.draw_string_8x16(0, 0, "DEAD ZONE TEST", &F8X16);
    oled.draw_string_6x8(2, 0, "Rounds=3 Avg+Hyst", &F6X8);
    defmt::info!("=== Dead Zone Test Start ({} rounds) ===", ROUNDS);

    let mut results: heapless::Vec<RoundResult, 5> = heapless::Vec::new();

    for round in 0..ROUNDS {
        let mut buf = String::<32>::new();
        let _ = write!(buf, "Round {}/{}...", round + 1, ROUNDS);
        oled.draw_string_6x8(3, 0, &buf, &F6X8);
        defmt::info!("--- Round {}/{} ---", round + 1, ROUNDS);

        // ── 正向去程：从小到大扫描，找启动边界 ──
        let pos_start = find_cross_above(&mut channels, &qei, 0, 100, &mut oled, "P+").await;

        // ── 正向回程：从启动边界往回扫，找停止边界 ──
        let pos_stop = match pos_start {
            Some(b) => find_cross_below(&mut channels, &qei, b.duty, 0, &mut oled, "P-").await,
            None => {
                defmt::warn!("Round {}: positive start not found", round + 1);
                Some(Boundary { duty: 0, rpm: 0.0 })
            }
        };

        // 回零等待，消除惯性
        set_pwm(&mut channels, 0);
        Timer::after_millis(ROUND_PAUSE_MS).await;

        // ── 负向去程：从 0 到 -100，找启动边界 ──
        let neg_start = find_cross_above(&mut channels, &qei, 0, -100, &mut oled, "N+").await;

        // ── 负向回程：从启动边界往回扫，找停止边界 ──
        let neg_stop = match neg_start {
            Some(b) => find_cross_below(&mut channels, &qei, b.duty, 0, &mut oled, "N-").await,
            None => {
                defmt::warn!("Round {}: negative start not found", round + 1);
                Some(Boundary { duty: 0, rpm: 0.0 })
            }
        };

        set_pwm(&mut channels, 0);
        Timer::after_millis(ROUND_PAUSE_MS).await;

        if let (Some(ps), Some(pstp), Some(ns), Some(nstp)) = (pos_start, pos_stop, neg_start, neg_stop) {
            defmt::info!(
                "Round {}: pos_start(d={},r={}) pos_stop(d={},r={}) neg_start(d={},r={}) neg_stop(d={},r={})",
                round + 1, ps.duty, ps.rpm as i32, pstp.duty, pstp.rpm as i32,
                ns.duty, ns.rpm as i32, nstp.duty, nstp.rpm as i32
            );
            let _ = results.push(RoundResult {
                pos_start: ps,
                pos_stop: pstp,
                neg_start: ns,
                neg_stop: nstp,
            });
        }
    }

    // ── 计算平均值（每轮先做迟滞平均，再跨轮平均）──
    let mut pos_avg_list: heapless::Vec<Boundary, 5> = heapless::Vec::new();
    let mut neg_avg_list: heapless::Vec<Boundary, 5> = heapless::Vec::new();

    for r in &results {
        let _ = pos_avg_list.push(Boundary {
            duty: (r.pos_start.duty + r.pos_stop.duty) / 2,
            rpm: (r.pos_start.rpm + r.pos_stop.rpm) / 2.0,
        });
        let _ = neg_avg_list.push(Boundary {
            duty: (r.neg_start.duty + r.neg_stop.duty) / 2,
            rpm: (r.neg_start.rpm + r.neg_stop.rpm) / 2.0,
        });
    }

    let pos_final = avg_boundary(&pos_avg_list);
    let neg_final = avg_boundary(&neg_avg_list);

    // ── 输出结果 ──
    defmt::info!("=== Dead Zone Test Result ===");
    defmt::info!(
        "Positive: duty={}% rpm={}",
        pos_final.duty,
        pos_final.rpm as i32
    );
    defmt::info!(
        "Negative: duty={}% rpm={}",
        neg_final.duty,
        neg_final.rpm as i32
    );
    defmt::info!(
        "Dead zone speed range: {} ~ {} rpm",
        neg_final.rpm as i32,
        pos_final.rpm as i32
    );

    // ── OLED 显示最终结果 ──
    oled.clear();
    oled.draw_string_8x16(0, 0, "DEAD ZONE", &F8X16);

    let mut buf = String::<32>::new();
    let _ = write!(buf, "+D:{}% +R:{:.0}", pos_final.duty, pos_final.rpm);
    oled.draw_string_6x8(2, 0, &buf, &F6X8);

    buf.clear();
    let _ = write!(buf, "-D:{}% -R:{:.0}", neg_final.duty, neg_final.rpm);
    oled.draw_string_6x8(3, 0, &buf, &F6X8);

    buf.clear();
    let _ = write!(buf, "RANGE:{:.0}~{:.0}", neg_final.rpm, pos_final.rpm);
    oled.draw_string_6x8(5, 0, &buf, &F6X8);

    loop {
        Timer::after_secs(1).await;
    }
}

/// 找边界：|rpm| 首次持续超过阈值（找“启动”点）
async fn find_cross_above(
    channels: &mut SimplePwmChannels<'_, TIM2>,
    qei: &Qei<'_, TIM1>,
    start_duty: i32,
    end_duty: i32,
    oled: &mut Oled,
    label: &str,
) -> Option<Boundary> {
    let step = if end_duty >= start_duty {
        STEP_DUTY
    } else {
        -STEP_DUTY
    };
    let mut duty = start_duty;
    let mut stable = 0u8;

    while (step > 0 && duty <= end_duty) || (step < 0 && duty >= end_duty) {
        set_pwm(channels, duty);
        let rpm = measure_rpm(qei, SAMPLES_UP).await;

        let mut buf = String::<32>::new();
        let _ = write!(buf, "{} D={:3} R={:5.1}", label, duty, rpm);
        oled.draw_string_6x8(3, 0, &buf, &F6X8);
        defmt::info!("{}: duty={}% -> rpm={}", label, duty, rpm);

        if rpm.abs() > RPM_THRESHOLD {
            stable += 1;
            if stable >= STABLE_COUNT {
                return Some(Boundary { duty, rpm });
            }
        } else {
            stable = 0;
        }

        duty += step;
    }
    None
}

/// 找边界：|rpm| 首次持续低于阈值（找“停止”点）
async fn find_cross_below(
    channels: &mut SimplePwmChannels<'_, TIM2>,
    qei: &Qei<'_, TIM1>,
    start_duty: i32,
    end_duty: i32,
    oled: &mut Oled,
    label: &str,
) -> Option<Boundary> {
    let step = if end_duty >= start_duty {
        STEP_DUTY
    } else {
        -STEP_DUTY
    };
    let mut duty = start_duty;
    let mut stable = 0u8;

    while (step > 0 && duty <= end_duty) || (step < 0 && duty >= end_duty) {
        set_pwm(channels, duty);
        let rpm = measure_rpm(qei, SAMPLES_DOWN).await;

        let mut buf = String::<32>::new();
        let _ = write!(buf, "{} D={:3} R={:5.1}", label, duty, rpm);
        oled.draw_string_6x8(4, 0, &buf, &F6X8);
        defmt::info!("{}: duty={}% -> rpm={}", label, duty, rpm);

        if rpm.abs() < RPM_THRESHOLD {
            stable += 1;
            if stable >= STABLE_COUNT {
                return Some(Boundary { duty, rpm });
            }
        } else {
            stable = 0;
        }

        duty += step;
    }
    // 扫到终点仍未停止，返回终点作为保守估计
    Some(Boundary {
        duty: end_duty,
        rpm: 0.0,
    })
}

/// 对一组 Boundary 取平均
fn avg_boundary(list: &[Boundary]) -> Boundary {
    if list.is_empty() {
        return Boundary { duty: 0, rpm: 0.0 };
    }
    let sum_duty: i32 = list.iter().map(|b| b.duty).sum();
    let sum_rpm: f32 = list.iter().map(|b| b.rpm).sum();
    let n = list.len() as i32;
    Boundary {
        duty: sum_duty / n,
        rpm: sum_rpm / n as f32,
    }
}

/// 设置 PWM 输出（duty: -100 ~ 100）
fn set_pwm(channels: &mut SimplePwmChannels<'_, TIM2>, duty: i32) {
    let max_duty = channels.ch3.max_duty_cycle();
    let duty_abs = duty.unsigned_abs() as u32;
    let compare = duty_abs.saturating_mul(max_duty) / 100;

    if duty > 0 {
        channels.ch3.set_duty_cycle(compare);
        channels.ch4.set_duty_cycle(0);
    } else if duty < 0 {
        channels.ch3.set_duty_cycle(0);
        channels.ch4.set_duty_cycle(compare);
    } else {
        channels.ch3.set_duty_cycle(0);
        channels.ch4.set_duty_cycle(0);
    }
}

/// 测量平均转速（rpm）
async fn measure_rpm(qei: &Qei<'_, TIM1>, samples: u8) -> f32 {
    let mut sum_rpm = 0.0f32;
    let mut last_count = qei.count();
    let mut last_time = Instant::now().as_micros();

    for _ in 0..samples {
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

    sum_rpm / samples as f32
}
