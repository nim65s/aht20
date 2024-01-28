use crate::{Aht20NoDelay, Error, Humidity, Temperature};
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

/// AHT20 driver.
pub struct Aht20<I2C, D> {
    aht20: Aht20NoDelay<I2C>,
    delay: D,
}

impl<I2C, D> Aht20<I2C, D>
where
    I2C: I2c,
    D: DelayNs,
{
    /// Creates a new AHT20 device from an I2C peripheral and a Delay.
    /// # Errors
    /// will return `Err` in case of i2c and/or calibration issue
    pub fn new(i2c: I2C, delay: D) -> Result<Self, Error<I2C::Error>> {
        let aht20 = Aht20NoDelay::new(i2c);
        let mut dev = Self { aht20, delay };

        dev.calibrate()?;

        Ok(dev)
    }

    fn delay_ms(&mut self, delay: u32) {
        self.delay.delay_ms(delay);
    }

    /// Self-calibrate the sensor.
    /// # Errors
    /// will return `Err` in case of i2c and/or calibration issue
    pub fn calibrate(&mut self) -> Result<(), Error<I2C::Error>> {
        // Send calibrate command
        self.aht20.start_calibration()?;

        self.delay_ms(10);

        // Wait until not busy
        while self.aht20.busy()? {
            self.delay_ms(10);
        }

        self.aht20.calibrated()
    }

    /// Soft resets the sensor.
    /// # Errors
    /// will return `Err` in case of i2c issue
    pub fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
        // Send soft reset command
        self.aht20.start_reset()?;

        // Wait 20ms as stated in specification
        self.delay_ms(20);

        Ok(())
    }

    /// Reads humidity and temperature.
    /// # Errors
    /// will return `Err` in case of i2c and/or calibration issue
    pub fn read(&mut self) -> Result<(Humidity, Temperature), Error<I2C::Error>> {
        // Send trigger measurement command
        self.aht20.start_read()?;
        self.delay_ms(80);

        // Wait until not busy
        while self.aht20.busy()? {
            self.delay_ms(10);
        }

        // Read in sensor data
        self.aht20.end_read()
    }
}
