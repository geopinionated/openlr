#![doc = include_str!("../README.md")]

mod binary;
mod decoder;
mod error;
mod graph;
mod model;

pub use binary::{
    deserialize_base64_openlr, deserialize_binary_openlr, serialize_base64_openlr,
    serialize_binary_openlr,
};
pub use decoder::candidates::{CandidateNode, CandidateNodes, find_candidate_nodes};
pub use decoder::{DecoderConfig, decode_base64_openlr, decode_binary_openlr};
pub use error::{DecodeError, DeserializeError, SerializeError};
pub use graph::DirectedGraph;
pub use model::{
    Bearing, Circle, ClosedLine, Coordinate, Fow, Frc, Grid, GridSize, Length, Line,
    LineAttributes, Location, LocationReference, LocationType, Offset, Orientation, PathAttributes,
    Poi, Point, PointAlongLine, Polygon, Rectangle, SideOfRoad,
};
