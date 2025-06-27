//! https://download.tomtom.com/open/banners/openlr-whitepaper_v1.5.pdf

mod error;
mod model;
mod reader;

pub use error::OpenLrError;
pub use model::{
    Bearing, CircleLocationReference, Coordinate, Fow, Frc, GridLocationReference, GridSize,
    Length, LineAttributes, LineLocationReference, LocationReference, LocationReferencePoint,
    LocationType, Offset, Orientation, PathAttributes, PoiLocationReference,
    PointAlongLineLocationReference, PolygonLocationReference, RectangleLocationReference,
    SideOfRoad,
};
pub use reader::{decode_base64_openlr, decode_binary_openlr};
