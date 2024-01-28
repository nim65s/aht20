/// AHT20 Error.
#[derive(thiserror::Error, Debug, Copy, Clone)]
pub enum Error<E> {
    /// Underlying bus error.
    #[error("bus error: {0:?}")]
    Bus(E),

    /// Checksum mismatch.
    #[error("checksum error")]
    Checksum,

    /// Device is not calibrated.
    #[error("uncalibrated error")]
    Uncalibrated,
}

impl<E> core::convert::From<E> for Error<E> {
    fn from(e: E) -> Self {
        Self::Bus(e)
    }
}
