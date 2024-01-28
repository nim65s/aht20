#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use aht20::Aht20Async;
use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::i2c::I2c;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, i2c, peripherals};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

const ADDRESS: u8 = 96;

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello world!");
    let p = embassy_stm32::init(Default::default());

    let mut i2c = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH7,
        Hertz(10_000),
        Default::default(),
    );

    info!("Create aht20 device");
    let mut aht20 = Aht20Async::new(i2c)
        .await
        .expect("aht20 device creation error");
    info!("aht20 device created");

    loop {
        let (h, t) = aht20.read().await.expect("aht20 device read error");
        info!("read {:?} {:?}", h.rh(), t.celsius());
        Timer::after(Duration::from_secs(10)).await;
    }
}
