use crate::{Coordinate, Length};

/// TODO
/// Geo-spatial index + Road Network Graph
pub trait Graph {
    type Node;

    fn nearest_neighbours_within_distance(
        &self,
        coordinate: Coordinate,
        max_distance: Length,
    ) -> impl Iterator<Item = Self::Node>;
}
