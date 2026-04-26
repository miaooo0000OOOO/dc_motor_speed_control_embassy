#![no_std]
#![no_main]

mod menu;

use embassy_executor::Spawner;
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPreDiv, PllSource, Sysclk,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use dc_motor_speed_control_embassy::keypad::Keypad;
use dc_motor_speed_control_embassy::sh1106::Sh1106;
use menu::AppState;

#[defmt::panic_handler]
fn panic() -> ! {
    panic_probe::hard_fault();
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
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

    // OLED init
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = Hertz::khz(400);
    let i2c = I2c::new_blocking(p.I2C1, p.PB8, p.PB9, i2c_config);
    let mut oled = Sh1106::new(i2c);
    oled.init();
    oled.clear();

    // Keypad init
    let mut keypad = Keypad::new(p.PA3, p.PA4, p.PA5, p.PA6);
    let mut state = AppState::new();

    defmt::info!("Menu test started.");
    state.render(&mut oled);

    let mut tick: u32 = 0;

    loop {
        // 10ms 扫描周期
        let events = keypad.scan();
        for ev in events {
            state.handle_event(ev);
        }

        tick += 1;
        if tick % 10 == 0 {
            // 100ms 更新电机模拟 + OLED 刷新
            state.update_motor();
            state.render(&mut oled);
        }

        Timer::after_millis(10).await;
    }
}
