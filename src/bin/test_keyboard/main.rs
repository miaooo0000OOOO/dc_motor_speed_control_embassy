#![no_std]
#![no_main]

use embassy_executor::Spawner;
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

use dc_motor_speed_control_embassy::keypad::{KeyEvent, Keypad};

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
    let mut keypad = Keypad::new(p.PA3, p.PA4, p.PA5, p.PA6);

    defmt::info!("Matrix keyboard test started. Scanning every 10ms...");

    loop {
        let events = keypad.scan();
        for ev in events {
            match ev {
                KeyEvent::Pressed(key) => {
                    defmt::info!("Pressed: {}", key.name());
                }
                KeyEvent::Released(key) => {
                    defmt::info!("Released: {}", key.name());
                }
            }
        }
        Timer::after_millis(10).await;
    }
}
