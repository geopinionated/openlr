//! [White Paper](https://download.tomtom.com/open/banners/openlr-whitepaper_v1.5.pdf)

mod error;
mod model;
mod reader;

pub use error::DecodeError;
pub use model::{
    Bearing, Circle, ClosedLine, Coordinate, Fow, Frc, Grid, GridSize, Length, Line,
    LineAttributes, LocationReference, LocationType, Offset, Orientation, PathAttributes, Poi,
    Point, PointAlongLine, Polygon, Rectangle, SideOfRoad,
};
pub use reader::{decode_base64_openlr, decode_binary_openlr};
