#![doc = include_str!("../README.md")]
#![forbid(unsafe_code)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::panic)]
#![deny(clippy::wildcard_enum_match_arm)]

mod decoder;
mod encoder;
mod error;
mod format;
mod graph;
mod location;
mod model;

pub use decoder::{DecoderConfig, decode_base64_openlr, decode_binary_openlr};
pub use encoder::{EncoderConfig, encode_base64_openlr, encode_binary_openlr};
pub use error::{DecodeError, DeserializeError, EncodeError, LocationError, SerializeError};
pub use format::binary::{
    deserialize_base64_openlr, deserialize_binary_openlr, serialize_base64_openlr,
    serialize_binary_openlr,
};
pub use graph::DirectedGraph;
pub use location::{
    ClosedLineLocation, LineLocation, Location, PoiLocation, PointAlongLineLocation,
};
pub use model::{
    Bearing, Circle, ClosedLine, Coordinate, Fow, Frc, Grid, GridSize, Length, Line,
    LineAttributes, LocationReference, LocationType, Offset, Offsets, Orientation, PathAttributes,
    Poi, Point, PointAlongLine, Polygon, Rating, RatingScore, Rectangle, SideOfRoad,
};
