#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPreDiv, PllSource, Sysclk,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use dc_motor_speed_control_embassy::pid::{DerivativeMode, Pid};

/// 模拟一阶惯性系统：y(k) = a*y(k-1) + b*u(k-1)
struct FirstOrderPlant {
    a: f32,
    b: f32,
    y: f32,
}

impl FirstOrderPlant {
    fn new(ts: f32, tau: f32) -> Self {
        // 一阶惯性离散化（欧拉近似），a = exp(-Ts/Tau) ≈ 1 - Ts/Tau
        let a = 1.0 - ts / tau;
        let b = ts / tau;
        Self { a, b, y: 0.0 }
    }

    fn step(&mut self, u: f32) -> f32 {
        self.y = self.a * self.y + self.b * u;
        self.y
    }
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
    let _p = embassy_stm32::init(config);

    // ── PID 参数 ──
    const TS_S: f32 = 0.01; // 10 ms 控制周期

    let mut pid = Pid::new(2.0, 0.5, 0.01)
        .with_sample_time(TS_S)
        .with_integral_limits(-50.0, 50.0)
        .with_output_limits(-100.0, 100.0)
        .with_derivative_mode(DerivativeMode::OnFeedback);

    let mut plant = FirstOrderPlant::new(TS_S, 0.1); // 时间常数 100 ms

    defmt::info!("PID unit test started");

    // ── 测试1：积分限幅 ──
    defmt::info!("=== Test 1: Integral windup ===");
    pid.reset();
    let mut pid_i = Pid::new(0.0, 10.0, 0.0)
        .with_sample_time(TS_S)
        .with_integral_limits(-5.0, 5.0)
        .with_output_limits(-100.0, 100.0);

    for _ in 0..10 {
        let out = pid_i.compute(100.0, 0.0);
        defmt::info!("setpoint=100, feedback=0, output={}", out);
        Timer::after_millis((TS_S * 1000.0) as u64).await;
    }

    // ── 测试2：输出限幅 ──
    defmt::info!("=== Test 2: Output clamp ===");
    pid.reset();
    let mut pid_p = Pid::new(20.0, 0.0, 0.0)
        .with_output_limits(-80.0, 80.0);

    let out_pos = pid_p.compute(10.0, 0.0);
    let out_neg = pid_p.compute(0.0, 10.0);
    defmt::info!("error=+10, output={}; error=-10, output={}", out_pos, out_neg);
    Timer::after_millis(100).await;

    // ── 测试3：阶跃响应（模拟一阶对象）──
    defmt::info!("=== Test 3: Step response with plant ===");
    pid.reset();
    plant.y = 0.0;

    let setpoint = 50.0;
    for i in 0..300 {
        let feedback = plant.y;
        let control = pid.compute(setpoint, feedback);
        plant.step(control);

        // 每 10 个周期（100ms）打印一次，避免日志过多
        if i % 10 == 0 {
            defmt::info!(
                "t={}ms, sp={}, pv={}, u={}",
                i as u32 * (TS_S * 1000.0) as u32,
                setpoint,
                feedback,
                control
            );
        }

        Timer::after_millis((TS_S * 1000.0) as u64).await;
    }

    // ── 测试4：设定值突变（微分先行对比）──
    defmt::info!("=== Test 4: Setpoint jump (derivative on feedback) ===");
    pid.reset();
    plant.y = 40.0;

    for i in 0..100 {
        let sp = if i < 50 { 40.0 } else { 80.0 }; // 50步后设定值从40跳到80
        let control = pid.compute(sp, plant.y);
        plant.step(control);

        if i % 10 == 0 || (i >= 48 && i <= 52) {
            defmt::info!("step={}, sp={}, pv={}, u={}", i, sp, plant.y, control);
        }

        Timer::after_millis((TS_S * 1000.0) as u64).await;
    }

    defmt::info!("All tests completed.");

    // 主循环空闲闪烁
    let mut led = Output::new(_p.PC13, Level::High, Speed::Low);
    loop {
        led.toggle();
        Timer::after_millis(500).await;
    }
}
