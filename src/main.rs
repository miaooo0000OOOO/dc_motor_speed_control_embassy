#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c, Master};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::{TIM1, TIM2};
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPreDiv, PllSource, Sysclk,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::gpio::AfioRemap;
use embassy_stm32::timer::qei::{Config as QeiConfig, Qei};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm, SimplePwmChannels};
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::{Ch1, Ch2, Ch3, Ch4};
use embassy_stm32::Config;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer};

use dc_motor_speed_control_embassy::font::{F6X8, F8X16};
use dc_motor_speed_control_embassy::keypad::Keypad;
use dc_motor_speed_control_embassy::menu::AppState;
use dc_motor_speed_control_embassy::pid::{DerivativeMode, Pid};
use dc_motor_speed_control_embassy::sh1106::Sh1106;
use {defmt_rtt as _, panic_probe as _};

/// 编码器每转计数值（11线 × 4倍频 × 21减速比）
const COUNTS_PER_REV: f32 = (11 * 21 * 4) as f32; // 924.0
/// 控制周期
const CONTROL_PERIOD_MS: u64 = 100;
/// PID 采样周期（秒），由控制周期编译期计算得到
const TS_S: f32 = (CONTROL_PERIOD_MS as f32) / 1000.0;
/// PID 初始参数（唯一真值源）
const PID_KP: f32 = 0.5;
const PID_KI: f32 = 0.4;
const PID_KD: f32 = 0.0;

/// 全局应用状态（OLED 菜单、设定值、PI 参数等）
static APP_STATE: Mutex<CriticalSectionRawMutex, AppState> = Mutex::new(AppState::new());

/// OLED 类型别名（减少 task 参数签名长度）
type Oled = Sh1106<I2c<'static, Blocking, Master>>;

#[defmt::panic_handler]
fn panic() -> ! {
    panic_probe::hard_fault();
}

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
    oled.draw_string_8x16(0, 0, "DC MOTOR PID", &F8X16);
    oled.draw_string_6x8(2, 0, "Initializing...", &F6X8);
    Timer::after_millis(500).await;

    // ── 键盘初始化 ──
    let keypad = Keypad::new(p.PA3, p.PA4, p.PA5, p.PA6);

    // ── PID 初始化 ──
    let pid = Pid::new(PID_KP, PID_KI, PID_KD)
        .with_sample_time(TS_S)
        .with_output_limits(-100.0, 100.0)
        .with_derivative_mode(DerivativeMode::OnFeedback);

    // 同步 PID 初始参数到共享状态，避免 menu.rs 与 main.rs 重复定义
    {
        let mut state = APP_STATE.lock().await;
        state.p_val = PID_KP;
        state.i_val = PID_KI;
        state.d_val = PID_KD;
    }

    // ── 启动任务 ──
    spawner.spawn(control_task(qei, channels, pid)).ok();
    spawner.spawn(display_task(oled)).ok();
    spawner.spawn(keypad_task(keypad)).ok();

    // 主循环：心跳 LED
    let mut led = Output::new(p.PC13, Level::High, Speed::Low);
    loop {
        led.toggle();
        Timer::after_millis(500).await;
    }
}

/// 控制任务：编码器测速 → PID → PWM 输出
#[embassy_executor::task]
async fn control_task(
    qei: Qei<'static, TIM1>,
    mut channels: SimplePwmChannels<'static, TIM2>,
    mut pid: Pid,
) {
    let mut last_count: u16 = qei.count();
    let mut last_time: u64 = Instant::now().as_micros();
    let mut ticker = Ticker::every(Duration::from_millis(CONTROL_PERIOD_MS));

    loop {
        ticker.next().await;

        let count = qei.count();
        let time = Instant::now().as_micros();

        // 自动处理 u16 溢出
        let delta_count = count.wrapping_sub(last_count) as i16 as i32;
        let delta_time = time.wrapping_sub(last_time);

        let rpm = if delta_time > 0 {
            (delta_count as f32 * 1_000_000.0 * 60.0) / (COUNTS_PER_REV * delta_time as f32)
        } else {
            0.0
        };

        let mut state = APP_STATE.lock().await;
        state.actual = rpm;

        // 阶跃模式下以 hold 速度为目标，否则以设定值为目标
        let setpoint = if state.speed_sub == dc_motor_speed_control_embassy::menu::SpeedSubMode::Step {
            state.step_hold_speed
        } else {
            state.setpoint
        };
        let feedback = rpm;

        // 动态更新 PID 参数（支持在线调参）
        pid.set_kp(state.p_val);
        pid.set_ki(state.i_val);
        pid.set_kd(state.d_val);

        let control = pid.compute(setpoint, feedback);
        let duty = control as i32;
        state.pwm_duty = duty;
        drop(state);

        // ── 输出 PWM ──
        let ch3_pwm = &mut channels.ch3;
        let ch4_pwm = &mut channels.ch4;
        let max_duty = ch3_pwm.max_duty_cycle();
        let duty_abs = duty.unsigned_abs() as u32;
        let compare = duty_abs.saturating_mul(max_duty) / 100;

        if duty > 0 {
            ch3_pwm.set_duty_cycle(compare);
            ch4_pwm.set_duty_cycle(0);
        } else if duty < 0 {
            ch3_pwm.set_duty_cycle(0);
            ch4_pwm.set_duty_cycle(compare);
        } else {
            ch3_pwm.set_duty_cycle(0);
            ch4_pwm.set_duty_cycle(0);
        }

        defmt::info!("sp={} rpm, pv={} rpm, duty={}%", setpoint as i32, rpm, duty);

        last_count = count;
        last_time = time;
    }
}

/// 显示任务：50ms 周期刷新 OLED
#[embassy_executor::task]
async fn display_task(mut oled: Oled) {
    let mut ticker = Ticker::every(Duration::from_millis(50));
    loop {
        ticker.next().await;
        let mut state = APP_STATE.lock().await;
        state.update_motor(); // 保持原有调用周期，内部为空
        state.render(&mut oled);
    }
}

/// 键盘任务：10ms 周期扫描按键
#[embassy_executor::task]
async fn keypad_task(mut keypad: Keypad) {
    let mut ticker = Ticker::every(Duration::from_millis(10));
    loop {
        ticker.next().await;
        let events = keypad.scan();
        if !events.is_empty() {
            let mut state = APP_STATE.lock().await;
            for ev in events {
                state.handle_event(ev);
            }
        }
    }
}
