#![no_std]
#![no_main]

use aht20::Aht20;
use esp32c3_hal::{
    clock::ClockControl, gpio::IO, i2c::I2C, peripherals::Peripherals, prelude::*, Delay,
};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    println!("Hello");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio4,
        io.pins.gpio5,
        10u32.kHz(),
        &clocks,
    );

    let delay = Delay::new(&clocks);
    let mut aht20 = Aht20::new(i2c, delay).expect("aht20 device creation error");

    let mut delay = Delay::new(&clocks);
    loop {
        let (h, t) = aht20.read().expect("aht20 device read error");
        println!("read {:?} {:?}", h.rh(), t.celsius());
        delay.delay_ms(10_000u32);
    }
}
