use crate::{Error, Humidity, StatusFlags, Temperature, CRC_ALGO, I2C_ADDRESS};
use crc::Crc;
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;

/// AHT20 async driver
pub struct Aht20Async<I2C> {
    i2c: I2C,
}

impl<I2C, E> Aht20Async<I2C>
where
    I2C: I2c<Error = E>,
{
    /// Creates a new AHT20 device from an I2C peripheral.
    /// # Errors
    /// will return `Err` in case of i2c and/or calibration issue
    pub async fn new(i2c: I2C) -> Result<Self, Error<E>> {
        let mut dev = Self { i2c };

        dev.calibrate().await?;

        Ok(dev)
    }

    /// Gets the sensor status.
    async fn status(&mut self) -> Result<StatusFlags, Error<E>> {
        let buf = &mut [0u8; 1];
        self.i2c.write_read(I2C_ADDRESS, &[0u8], buf).await?;

        Ok(StatusFlags::from_bits_truncate(buf[0]))
    }

    /// Self-calibrate the sensor.
    /// # Errors
    /// will return `Err` in case of i2c and/or calibration issue
    pub async fn start_calibration(&mut self) -> Result<(), Error<E>> {
        // Send calibrate command
        self.i2c.write(I2C_ADDRESS, &[0xE1, 0x08, 0x00]).await?;
        Ok(())
    }

    /// Check if the device is still busy
    /// # Errors
    /// will return `Err` in case of i2c issue
    pub async fn busy(&mut self) -> Result<bool, Error<E>> {
        Ok(self.status().await?.contains(StatusFlags::BUSY))
    }

    /// Confirm sensor is calibrated
    /// # Errors
    /// will return `Err` in case of i2c issue
    pub async fn calibrated(&mut self) -> Result<(), Error<E>> {
        if !self
            .status()
            .await?
            .contains(StatusFlags::CALIBRATION_ENABLE)
        {
            return Err(Error::Uncalibrated);
        }

        Ok(())
    }

    /// Soft resets the sensor.
    /// # Errors
    /// will return `Err` in case of i2c issue
    pub async fn start_reset(&mut self) -> Result<(), Error<E>> {
        // Send soft reset command
        self.i2c.write(I2C_ADDRESS, &[0xBA]).await?;
        Ok(())
    }

    /// Start reading humidity and temperature.
    /// # Errors
    /// will return `Err` in case of i2c issue
    pub async fn start_read(&mut self) -> Result<(), Error<E>> {
        // Send trigger measurement command
        self.i2c.write(I2C_ADDRESS, &[0xAC, 0x33, 0x00]).await?;
        Ok(())
    }

    /// Conclude reading humidity and temperature.
    /// # Errors
    /// will return `Err` in case of i2c and/or calibration issue
    pub async fn end_read(&mut self) -> Result<(Humidity, Temperature), Error<E>> {
        let crc = Crc::<u8>::new(&CRC_ALGO);
        let mut digest = crc.digest();

        // Read in sensor data
        let buf = &mut [0u8; 7];
        self.i2c.write_read(I2C_ADDRESS, &[0u8], buf).await?;

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

    /// await for some time
    pub async fn delay_ms(&mut self, delay: u64) {
        Timer::after(Duration::from_millis(delay)).await;
    }

    /// Self-calibrate the sensor.
    /// # Errors
    /// will return `Err` in case of i2c and/or calibration issue
    pub async fn calibrate(&mut self) -> Result<(), Error<E>> {
        // Send calibrate command
        self.start_calibration().await?;

        self.delay_ms(10).await;

        // Wait until not busy
        while self.busy().await? {
            self.delay_ms(10).await;
        }

        self.calibrated().await
    }

    /// Soft resets the sensor.
    /// # Errors
    /// will return `Err` in case of i2c issue
    pub async fn reset(&mut self) -> Result<(), Error<E>> {
        // Send soft reset command
        self.start_reset().await?;

        // Wait 20ms as stated in specification
        self.delay_ms(20).await;

        Ok(())
    }

    /// Reads humidity and temperature.
    /// # Errors
    /// will return `Err` in case of i2c and/or calibration issue
    pub async fn read_ht(&mut self) -> Result<(Humidity, Temperature), Error<E>> {
        // Send trigger measurement command
        self.start_read().await?;
        self.delay_ms(80).await;

        // Wait until not busy
        while self.busy().await? {
            self.delay_ms(10).await;
        }

        // Read in sensor data
        self.end_read().await
    }
}
