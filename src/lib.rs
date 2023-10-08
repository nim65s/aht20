//! A platform agnostic driver to interface with the AHT20 temperature and
//! humidity sensor.
//!
//! This driver was built using [`embedded-hal`] traits and is a fork of Anthony
//! Romano's [AHT10 crate].
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.2
//! [AHT10 crate]: https://github.com/heyitsanthony/aht10

#![deny(missing_docs)]
#![no_std]
#![feature(async_fn_in_trait)]

use bitflags::bitflags;
use crc::{Algorithm, Crc};
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

const I2C_ADDRESS: u8 = 0x38;
const CRC_ALGO: Algorithm<u8> = Algorithm {
    width: 8,
    poly: 0b11_0001,
    init: 0xFF,
    refin: false,
    refout: false,
    xorout: 0,
    check: 0xf7,
    residue: 0,
};

bitflags! {
    struct StatusFlags: u8 {
        const BUSY = (1 << 7);
        const MODE = ((1 << 6) | (1 << 5));
        const CRC = (1 << 4);
        const CALIBRATION_ENABLE = (1 << 3);
        const FIFO_ENABLE = (1 << 2);
        const FIFO_FULL = (1 << 1);
        const FIFO_EMPTY = (1 << 0);
    }
}

/// AHT20 Error.
#[derive(Debug, Copy, Clone)]
pub enum Error<E> {
    /// Underlying bus error.
    Bus(E),
    /// Checksum mismatch.
    Checksum,
    /// Device is not calibrated.
    Uncalibrated,
}

impl<E> core::convert::From<E> for Error<E> {
    fn from(e: E) -> Self {
        Self::Bus(e)
    }
}

/// Humidity reading from AHT20.
#[derive(Deserialize, Serialize, MaxSize, Debug, Eq, PartialEq, Copy, Clone)]
pub struct Humidity {
    h: u32,
}

impl Humidity {
    /// Humidity converted to Relative Humidity %.
    #[must_use]
    pub fn rh(&self) -> f64 {
        100.0 * f64::from(self.h) / f64::from(1 << 20)
    }

    /// Raw humidity reading.
    #[must_use]
    pub const fn raw(&self) -> u32 {
        self.h
    }
}

/// Temperature reading from AHT20.
#[derive(Deserialize, Serialize, MaxSize, Debug, Eq, PartialEq, Copy, Clone)]
pub struct Temperature {
    t: u32,
}

impl Temperature {
    /// Temperature converted to Celsius.
    #[must_use]
    pub fn celsius(&self) -> f64 {
        200.0 * f64::from(self.t) / f64::from(1 << 20) - 50.0
    }

    /// Raw temperature reading.
    #[must_use]
    pub const fn raw(&self) -> u32 {
        self.t
    }
}

/// AHT20 driver without delay.
pub struct Aht20<I2C> {
    i2c: I2C,
}

impl<I2C, E> Aht20<I2C>
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

        Ok((Humidity { h: hum }, Temperature { t: temp }))
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
