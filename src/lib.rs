//! A platform agnostic driver to interface with the AHT20 temperature and
//! humidity sensor.
//!
//! This driver was built using [`embedded-hal`] traits and is a fork of Anthony
//! Romano's [AHT10 crate].
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.2
//! [AHT10 crate]: https://github.com/heyitsanthony/aht10

//#![deny(missing_docs)]
#![no_std]
#![feature(error_in_core)]

mod aht20;
pub use aht20::{Humidity, StatusFlags, Temperature, CRC_ALGO, I2C_ADDRESS};

mod error;
pub use error::Error;

mod async_driver;
pub use async_driver::Aht20Async;
