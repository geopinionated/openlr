#![doc = include_str!("../README.md")]

mod binary;
mod decoder;
pub mod decoder_graph;
mod dijkstra;
mod error;
mod graph;
mod model;

pub use binary::{
    deserialize_base64_openlr, deserialize_binary_openlr, serialize_base64_openlr,
    serialize_binary_openlr,
};
// --------- TODO --------
pub use decoder::{DecodeError, decode_base64_openlr};
pub use error::{DeserializeError, SerializeError};
pub use graph::DirectedGraph;
pub use model::{
    Bearing, Circle, ClosedLine, Coordinate, Fow, Frc, Grid, GridSize, Length, Line,
    LineAttributes, LocationReference, LocationType, Offset, Orientation, PathAttributes, Poi,
    Point, PointAlongLine, Polygon, Rating, Rectangle, SideOfRoad,
};
