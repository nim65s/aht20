use crate::{Error, Humidity, StatusFlags, Temperature, CRC_ALGO, I2C_ADDRESS};
use crc::Crc;
use embedded_hal::i2c::I2c;

/// AHT20 driver without delay.
pub struct Aht20NoDelay<I2C> {
    i2c: I2C,
}

impl<I2C> Aht20NoDelay<I2C>
where
    I2C: I2c,
{
    /// Creates a new AHT20 device from an I2C peripheral.
    /// # Errors
    /// will return `Err` in case of i2c and/or calibration issue
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    /// Gets the sensor status.
    fn status(&mut self) -> Result<StatusFlags, Error<I2C::Error>> {
        let buf = &mut [0u8; 1];
        self.i2c.write_read(I2C_ADDRESS, &[0u8], buf)?;

        Ok(StatusFlags::from_bits_truncate(buf[0]))
    }

    /// Self-calibrate the sensor.
    /// # Errors
    /// will return `Err` in case of i2c and/or calibration issue
    pub fn start_calibration(&mut self) -> Result<(), Error<I2C::Error>> {
        // Send calibrate command
        self.i2c.write(I2C_ADDRESS, &[0xE1, 0x08, 0x00])?;
        Ok(())
    }

    /// Check if the device is still busy
    /// # Errors
    /// will return `Err` in case of i2c issue
    pub fn busy(&mut self) -> Result<bool, Error<I2C::Error>> {
        Ok(self.status()?.contains(StatusFlags::BUSY))
    }

    /// Confirm sensor is calibrated
    /// # Errors
    /// will return `Err` in case of i2c issue
    pub fn calibrated(&mut self) -> Result<(), Error<I2C::Error>> {
        if !self.status()?.contains(StatusFlags::CALIBRATION_ENABLE) {
            return Err(Error::Uncalibrated);
        }

        Ok(())
    }

    /// Soft resets the sensor.
    /// # Errors
    /// will return `Err` in case of i2c issue
    pub fn start_reset(&mut self) -> Result<(), Error<I2C::Error>> {
        // Send soft reset command
        self.i2c.write(I2C_ADDRESS, &[0xBA])?;
        Ok(())
    }

    /// Start reading humidity and temperature.
    /// # Errors
    /// will return `Err` in case of i2c issue
    pub fn start_read(&mut self) -> Result<(), Error<I2C::Error>> {
        // Send trigger measurement command
        self.i2c.write(I2C_ADDRESS, &[0xAC, 0x33, 0x00])?;
        Ok(())
    }

    /// Conclude reading humidity and temperature.
    /// # Errors
    /// will return `Err` in case of i2c and/or calibration issue
    pub fn end_read(&mut self) -> Result<(Humidity, Temperature), Error<I2C::Error>> {
        let crc = Crc::<u8>::new(&CRC_ALGO);
        let mut digest = crc.digest();

        // Read in sensor data
        let buf = &mut [0u8; 7];
        self.i2c.write_read(I2C_ADDRESS, &[0u8], buf)?;

        // Check for CRC mismatch
        digest.update(&buf[..=5]);
        if digest.finalize() != buf[6] {
            return Err(Error::Checksum);
        };

        // Check calibration
        let status = StatusFlags::from_bits_truncate(buf[0]);
        if !status.contains(StatusFlags::CALIBRATION_ENABLE) {
            return Err(Error::Uncalibrated);
        }

        // Extract humitidy and temperature values from data
        let buf: [u32; 7] = [
            u32::from(buf[0]),
            u32::from(buf[1]),
            u32::from(buf[2]),
            u32::from(buf[3]),
            u32::from(buf[4]),
            u32::from(buf[5]),
            u32::from(buf[6]),
        ];
        let hum = (buf[1] << 12) | (buf[2] << 4) | (buf[3] >> 4);
        let temp = ((buf[3] & 0x0f) << 16) | (buf[4] << 8) | buf[5];

        Ok((Humidity::new(hum), Temperature::new(temp)))
    }
}
