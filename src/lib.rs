#![doc = include_str!("../README.md")]

mod binary;
mod decoder;
mod encoder;
mod error;
mod graph;
mod location;
mod model;
mod path;

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
pub use decoder::route::{Route, Routes};
pub use decoder::shortest_path::shortest_path;
pub use decoder::{DecoderConfig, decode_base64_openlr, decode_binary_openlr};
pub use encoder::expansion::{
    edge_backward_expansion, edge_forward_expansion, is_node_valid, is_opposite_direction,
    line_location_expansion, select_edge_expansion_candidate,
};
pub use encoder::line::encode_line;
pub use encoder::lrp::{LocRefPoint, LocRefPoints};
pub use encoder::resolver::resolve_lrps;
pub use encoder::shortest_path::{IntermediateLocation, ShortestRoute, shortest_path_location};
pub use encoder::{EncoderConfig, encode_base64_openlr, encode_binary_openlr};
pub use error::{DecodeError, DeserializeError, EncoderError, LocationError, SerializeError};
pub use graph::DirectedGraph;
pub use location::{LineLocation, Location, ensure_line_is_valid};
pub use model::{
    Bearing, Circle, ClosedLine, Coordinate, Fow, Frc, Grid, GridSize, Length, Line,
    LineAttributes, LocationReference, LocationType, Offset, Offsets, Orientation, PathAttributes,
    Poi, Point, PointAlongLine, Polygon, Rating, RatingScore, Rectangle, SideOfRoad,
};
pub use path::{Path, is_path_connected};
