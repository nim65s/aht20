#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use aht20::Aht20;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp32c3_hal::{
    clock::ClockControl,
    embassy,
    i2c::I2C,
    interrupt,
    peripherals::{Interrupt, Peripherals},
    prelude::*,
    IO,
};
use esp_backtrace as _;
use esp_println::println;

#[main]
async fn main(_spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let timer_group0 = esp32c3_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0.timer0);

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio4,
        io.pins.gpio5,
        1u32.kHz(),
        &clocks,
    );

    interrupt::enable(Interrupt::I2C_EXT0, interrupt::Priority::Priority1).unwrap();

    let mut aht20 = Aht20::new(i2c).await.expect("aht20 device creation error");

    loop {
        let (h, t) = aht20.read_ht().await.expect("aht20 device read error");
        println!("read {:?} {:?}", h.rh(), t.celsius());
        Timer::after(Duration::from_secs(10)).await;
    }
}
