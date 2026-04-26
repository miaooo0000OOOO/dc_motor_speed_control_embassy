#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPreDiv, PllSource, Sysclk,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[defmt::panic_handler]
fn panic() -> ! {
    panic_probe::hard_fault();
}

use dc_motor_speed_control_embassy::font::{F6X8, F8X16};
use dc_motor_speed_control_embassy::sh1106::Sh1106;

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

    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = Hertz::khz(400);
    let i2c = I2c::new_blocking(p.I2C1, p.PB8, p.PB9, i2c_config);

    let mut oled = Sh1106::new(i2c);
    oled.init();
    oled.clear();

    oled.draw_string_8x16(0, 0, "SH1106 OLED", &F8X16);
    oled.draw_string_6x8(2, 0, "STM32F103C8T6", &F6X8);
    oled.draw_string_6x8(3, 0, "Embassy + Rust", &F6X8);
    oled.draw_string_6x8(4, 0, "I2C1 PB8/PB9", &F6X8);
    oled.draw_string_6x8(5, 0, "72MHz SYSCLK", &F6X8);

    let mut counter: u32 = 0;
    let mut buf = heapless::String::<16>::new();

    loop {
        buf.clear();
        use core::fmt::Write;
        let _ = write!(buf, "Count: {}", counter);
        oled.draw_string_6x8(7, 0, &buf, &F6X8);
        counter += 1;
        Timer::after_secs(1).await;
    }
}
