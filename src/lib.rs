#![doc = include_str!("../README.md")]

mod decoder;
mod encoder;
mod error;
mod format;
mod graph;
mod location;
mod model;
mod path;

pub use decoder::{DecoderConfig, decode_base64_openlr, decode_binary_openlr};
pub use encoder::{EncoderConfig, encode_base64_openlr, encode_binary_openlr};
pub use error::{DecodeError, DeserializeError, EncoderError, LocationError, SerializeError};
pub use format::binary::{
    deserialize_base64_openlr, deserialize_binary_openlr, serialize_base64_openlr,
    serialize_binary_openlr,
};
pub use graph::DirectedGraph;
pub use location::{LineLocation, Location};
pub use model::{
    Bearing, Circle, ClosedLine, Coordinate, Fow, Frc, Grid, GridSize, Length, Line,
    LineAttributes, LocationReference, LocationType, Offset, Offsets, Orientation, PathAttributes,
    Poi, Point, PointAlongLine, Polygon, Rating, RatingScore, Rectangle, SideOfRoad,
};
