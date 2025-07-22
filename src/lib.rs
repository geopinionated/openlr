#![doc = include_str!("../README.md")]

mod binary;
mod decoder;
mod encoder;
mod error;
mod graph;
mod location;
mod model;
mod routing;

pub use binary::{
    deserialize_base64_openlr, deserialize_binary_openlr, serialize_base64_openlr,
    serialize_binary_openlr,
};
pub use decoder::candidates::{
    CandidateLine, CandidateLinePair, CandidateLines, CandidateNode, CandidateNodes,
    find_candidate_lines, find_candidate_nodes,
};
pub use decoder::line::decode_line;
pub use decoder::resolver::resolve_routes;
pub use decoder::{DecoderConfig, decode_base64_openlr, decode_binary_openlr};
pub use encoder::expansion::{
    ExpandedPath, edge_backward_expansion, edge_forward_expansion, is_node_valid,
    is_opposite_direction, select_edge_expansion_candidate,
};
pub use encoder::line::encode_line;
pub use encoder::{EncoderConfig, encode_base64_openlr, encode_binary_openlr};
pub use error::{DecodeError, DeserializeError, EncoderError, LocationError, SerializeError};
pub use graph::{DirectedGraph, is_path_connected};
pub use location::{LineLocation, Location, ensure_line_is_valid};
pub use model::{
    Bearing, Circle, ClosedLine, Coordinate, Fow, Frc, Grid, GridSize, Length, Line,
    LineAttributes, LocationReference, LocationType, Offset, Offsets, Orientation, PathAttributes,
    Poi, Point, PointAlongLine, Polygon, Rating, RatingScore, Rectangle, SideOfRoad,
};
pub use routing::{Path, Route, Routes, ShortestPathConfig, shortest_path};
