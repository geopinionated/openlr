#![doc = include_str!("../README.md")]

mod binary;
mod error;
mod model;

pub use binary::{
    decode_base64_openlr, decode_binary_openlr, encode_base64_openlr, encode_binary_openlr,
};
pub use error::{DecodeError, EncodeError};
pub use model::{
    Bearing, Circle, ClosedLine, Coordinate, Fow, Frc, Grid, GridSize, Length, Line,
    LineAttributes, LocationReference, LocationType, Offset, Orientation, PathAttributes, Poi,
    Point, PointAlongLine, Polygon, Rectangle, SideOfRoad,
};
