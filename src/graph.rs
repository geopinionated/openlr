use std::fmt::Debug;
use std::hash::Hash;

use crate::{Bearing, Coordinate, Fow, Frc, Length};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EdgeProperty<EdgeId> {
    pub id: EdgeId, // TODO: this should not be needed
    pub length: Length,
    pub frc: Frc,
    pub fow: Fow,
}

/// Geospatial index + Road Network Graph
pub trait Graph {
    type VertexId: Debug + Copy + Ord + Hash + Eq;
    type EdgeId: Debug + Copy + Ord;
    //type Meter: Debug + Copy + Ord + Default + From<f64> + Into<f64>;

    // TODO many methods for each property?
    // get_cost(), get_frc(), etc..
    fn get_edge_properties(&self, edge: Self::EdgeId) -> Option<&EdgeProperty<Self::EdgeId>>;

    fn vertex_exiting_edges(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = (Self::EdgeId, Self::VertexId)>;

    fn vertex_entering_edges(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = (Self::EdgeId, Self::VertexId)>;

    fn nearest_vertices_within_distance(
        &self,
        coordinate: Coordinate,
        max_distance: Length,
    ) -> impl Iterator<Item = (Self::VertexId, Length)>;

    fn connected_vertices(&self, vertex: Self::VertexId) -> impl Iterator<Item = Self::VertexId> {
        let mut nodes: Vec<_> = self
            .vertex_exiting_edges(vertex)
            .chain(self.vertex_entering_edges(vertex))
            .map(|(_, v)| v)
            .filter(|&v| v != vertex)
            .collect();
        nodes.sort_unstable();
        nodes.dedup();
        nodes.into_iter()
    }

    fn is_junction(&self, vertex: Self::VertexId) -> bool {
        self.connected_vertices(vertex).count() > 2
    }
}
