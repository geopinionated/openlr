use std::io::ErrorKind;

use thiserror::Error;

#[derive(Error, Debug, PartialEq, Eq, Clone, Copy)]
pub enum DecodeError {
    #[error("OpenLR invalid Base 64")]
    InvalidBase64,
    #[error("OpenLR buffer I/O error: {0:?}")]
    IO(ErrorKind),
    #[error("OpenLR version {0} not supported")]
    VersionNotSupported(u8),
    #[error("OpenLR header is not valid: {0:08b}")]
    InvalidHeader(u8),
    #[error("OpenLR FRC is not valid: {0}")]
    InvalidFrc(u8),
    #[error("OpenLR FOW is not valid: {0}")]
    InvalidFow(u8),
    #[error("OpenLR Orientation is not valid: {0}")]
    InvalidOrientation(u8),
    #[error("OpenLR Side of Road is not valid: {0}")]
    InvalidSideOfRoad(u8),
}

impl From<base64::DecodeError> for DecodeError {
    fn from(_: base64::DecodeError) -> Self {
        Self::InvalidBase64
    }
}

impl From<std::io::Error> for DecodeError {
    fn from(error: std::io::Error) -> Self {
        Self::IO(error.kind())
    }
}
