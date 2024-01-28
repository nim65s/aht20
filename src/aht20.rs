use bitflags::bitflags;
use crc::Algorithm;

#[cfg(feature = "postcard")]
use postcard::experimental::max_size::MaxSize;
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// I2c adress used by aht20
pub const I2C_ADDRESS: u8 = 0x38;

/// Crc algorithm used by aht20
pub const CRC_ALGO: Algorithm<u8> = Algorithm {
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
    pub struct StatusFlags: u8 {
        const BUSY = (1 << 7);
        const MODE = ((1 << 6) | (1 << 5));
        const CRC = (1 << 4);
        const CALIBRATION_ENABLE = (1 << 3);
        const FIFO_ENABLE = (1 << 2);
        const FIFO_FULL = (1 << 1);
        const FIFO_EMPTY = (1 << 0);
    }
}

/// Humidity reading from AHT20.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[cfg_attr(feature = "postcard", derive(MaxSize))]
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub struct Humidity {
    h: u32,
}

impl Humidity {
    /// Create humidity value
    #[must_use]
    pub fn new(h: u32) -> Self {
        Self { h }
    }

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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[cfg_attr(feature = "postcard", derive(MaxSize))]
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub struct Temperature {
    t: u32,
}

impl Temperature {
    /// Create temperature  value
    #[must_use]
    pub fn new(t: u32) -> Self {
        Self { t }
    }

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
