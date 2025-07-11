use std::fmt::Debug;
use std::hash::Hash;

use crate::{Bearing, Coordinate, Fow, Frc, Length};

/// Geospatial index + Road Network Graph
pub trait Graph {
    type VertexId: Debug + Copy + Ord + Hash + Eq;
    type EdgeId: Debug + Copy + Ord;
    //type Meter: Debug + Copy + Ord + Default + From<f64> + Into<f64>;

    fn get_edge_start_vertex(&self, edge: Self::EdgeId) -> Option<Self::VertexId>;
    fn get_edge_end_vertex(&self, edge: Self::EdgeId) -> Option<Self::VertexId>;

    fn get_edge_length(&self, edge: Self::EdgeId) -> Option<Length>;
    fn get_edge_frc(&self, edge: Self::EdgeId) -> Option<Frc>;
    fn get_edge_fow(&self, edge: Self::EdgeId) -> Option<Fow>;
    fn get_edge_coordinates(&self, edge: Self::EdgeId) -> impl Iterator<Item = Coordinate>;

    fn get_edge_bearing(&self, edge: Self::EdgeId) -> Option<Bearing>;
    fn get_edge_bearing_between(
        &self,
        edge: Self::EdgeId,
        distance_start: Length, // distance between start vertex and first point
        offset: Length,         // distance between first point and second point
    ) -> Option<Bearing>;

    fn get_distance_from_start_vertex(
        &self,
        edge: Self::EdgeId,
        coordinate: Coordinate,
    ) -> Option<Length>;

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

    fn nearest_edges_within_distance(
        &self,
        coordinate: Coordinate,
        max_distance: Length,
    ) -> impl Iterator<Item = (Self::EdgeId, Length)>;

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
