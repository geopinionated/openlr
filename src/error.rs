use std::io::ErrorKind;

use thiserror::Error;

use crate::{Length, LocationType, Point};

#[derive(Error, Debug, PartialEq, Clone, Copy)]
pub enum DeserializeError {
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
pub enum SerializeError {
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
    #[error("OpenLR Rectangle consists of 2 different coordinates")]
    InvalidRectangle,
    #[error("OpenLR Grid size must have number of columns and rows > 1")]
    InvalidGridSize,
}

#[derive(Error, Debug, PartialEq, Clone, Copy)]
pub enum DecodeError {
    #[error("Decoding {0:?} is not supported")]
    LocationTypeNotSupported(LocationType),
    #[error("Cannot deserialize: {0}")]
    InvalidData(DeserializeError),
    #[error("Cannot find candidates for {0:?}")]
    CandidatesNotFound(Point),
    #[error("Cannot find route between LRPs {0:?}")]
    RouteNotFound((Point, Point)),
    #[error("Cannot connect route to shortest path {0:?}")]
    AlternativeRouteNotFound((Point, Point)),
    #[error("Cannot decode invalid location {0:?}")]
    InvalidLocation(LocationError),
}

#[derive(Error, Debug, PartialEq, Clone, Copy)]
pub enum EncoderError {
    #[error("Cannot encode invalid location: {0:?}")]
    InvalidLocation(LocationError),
    #[error("Cannot serialize location: {0:?}")]
    InvalidLocationReference(SerializeError),
    #[error("Cannot compute intermediate at location index {0}")]
    IntermediateError(usize),
    #[error("Cannot find route between LRPs")]
    RouteNotFound,
    #[error("Cannot construct LRP for location")]
    LrpConstructionFailed,
    #[error("Maximum distance between consecutive LRPs exceeded")]
    MaxDistanceExceeded,
    #[error("Cannot construct LRPs for location after trimming")]
    LrpOffsetTrimmingFailed,
}

#[derive(Error, Debug, PartialEq, Clone, Copy)]
pub enum LocationError {
    #[error("Invalid offsets {0:?}")]
    InvalidOffsets((Length, Length)),
    #[error("Location is empty")]
    Empty,
    #[error("Location is not connected")]
    NotConnected,
}

impl From<LocationError> for DecodeError {
    fn from(error: LocationError) -> Self {
        Self::InvalidLocation(error)
    }
}

impl From<LocationError> for EncoderError {
    fn from(error: LocationError) -> Self {
        Self::InvalidLocation(error)
    }
}

impl From<DeserializeError> for DecodeError {
    fn from(error: DeserializeError) -> Self {
        Self::InvalidData(error)
    }
}

impl From<SerializeError> for EncoderError {
    fn from(error: SerializeError) -> Self {
        Self::InvalidLocationReference(error)
    }
}

impl From<base64::DecodeError> for DeserializeError {
    fn from(_: base64::DecodeError) -> Self {
        Self::InvalidBase64
    }
}

impl From<std::io::Error> for DeserializeError {
    fn from(error: std::io::Error) -> Self {
        Self::IO(error.kind())
    }
}

impl From<std::io::Error> for SerializeError {
    fn from(error: std::io::Error) -> Self {
        Self::IO(error.kind())
    }
}
