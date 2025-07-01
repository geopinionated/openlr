use std::io::ErrorKind;

use thiserror::Error;

#[derive(Error, Debug, PartialEq, Clone, Copy)]
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

#[derive(Error, Debug, PartialEq, Clone, Copy)]
pub enum EncodeError {
    #[error("OpenLR buffer I/O error: {0:?}")]
    IO(ErrorKind),
    #[error("OpenLR Bearing is not valid, expected [0, 360): {0}")]
    InvalidBearing(u16),
    #[error("OpenLR Offset is not valid, expected [0, 1): {0}")]
    InvalidOffset(f64),
    #[error("OpenLR Line consists of at least 2 LR-points")]
    InvalidLine,
    #[error("OpenLR Polygon consists of at least 3 LR-points")]
    InvalidPolygon,
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

impl From<std::io::Error> for EncodeError {
    fn from(error: std::io::Error) -> Self {
        Self::IO(error.kind())
    }
}
