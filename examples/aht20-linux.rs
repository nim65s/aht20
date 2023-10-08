//! Linux I2C Demo

use std::{env, process};

use aht20::Aht20;
use embedded_hal::blocking::delay::DelayMs;
use linux_embedded_hal as hal;

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        println!("usage: {} /dev/i2c-N", args[0]);
        process::exit(1);
    }

    let Ok(i2c) = hal::I2cdev::new(&args[1]) else {
        eprintln!("Couldn't open I2C device");
        return;
    };

    let Ok(mut dev) = Aht20::new(i2c, hal::Delay) else {
        eprintln!("Couldn't contact aht20");
        return;
    };

    loop {
        match dev.read() {
            Ok((h, t)) => println!(
                "relative humidity={0}%; temperature={1}C",
                h.rh(),
                t.celsius()
            ),
            Err(e) => eprintln!("error reading aht20: {e:?}"),
        }

        hal::Delay.delay_ms(1000u16);
    }
}
