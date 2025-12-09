use std::fmt::Debug;
use std::hash::Hash;

use crate::{Bearing, Coordinate, Fow, Frc, Length};

/// Directed graph.
/// Exposes the behavior of a Geospatial Index and of a Road Network Graph.
/// Should be implemented by the graph the represents the map the decoder and encoder run on.
pub trait DirectedGraph {
    /// Uniquely identify a vertex that belongs to the graph.
    type VertexId: Debug + Copy + Ord + Hash;
    /// Uniquely identify a directed edge that belongs to the graph.
    type EdgeId: Debug + Copy + Ord + Hash;

    /// Gets the vertex coordinate.
    fn get_vertex_coordinate(&self, vertex: Self::VertexId) -> Coordinate;

    /// Gets the start vertex of the directed edge.
    fn get_edge_start_vertex(&self, edge: Self::EdgeId) -> Self::VertexId;

    /// Gets the end vertex of the directed edge.
    fn get_edge_end_vertex(&self, edge: Self::EdgeId) -> Self::VertexId;

    /// Gets the total length of the directed edge.
    fn get_edge_length(&self, edge: Self::EdgeId) -> Length;

    /// Gets the Functional Road Class (FRC) of the directed edge.
    fn get_edge_frc(&self, edge: Self::EdgeId) -> Frc;

    /// Gets the Form of Way (FOW) of the directed edge.
    fn get_edge_fow(&self, edge: Self::EdgeId) -> Fow;

    /// Gets an iterator over all the outgoing edges from the given vertex.
    /// For each edge returns the edge ID and the edge end vertex.
    fn vertex_exiting_edges(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = (Self::EdgeId, Self::VertexId)>;

    /// Gets an iterator over all the incoming edges to the given vertex.
    /// For each edge returns the edge ID and the edge start vertex.
    fn vertex_entering_edges(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = (Self::EdgeId, Self::VertexId)>;

    /// Gets an iterator over all the vertices that are within a max distance from the coordinate.
    /// For each vertex also returns the distance from the coordinate.
    /// Vertices must be returned sorted by their distance to the coordinate.
    /// Returns an empty iterator if no vertex can be found within max distance.
    fn nearest_vertices_within_distance(
        &self,
        coordinate: Coordinate,
        max_distance: Length,
    ) -> impl Iterator<Item = (Self::VertexId, Length)>;

    /// Gets an iterator over all the edges that are within a max distance from the coordinate.
    /// For each edges also returns the distance from the coordinate.
    /// Edges must be returned sorted by their distance to the coordinate.
    /// Returns an empty iterator if no vertex can be found within max distance.
    fn nearest_edges_within_distance(
        &self,
        coordinate: Coordinate,
        max_distance: Length,
    ) -> impl Iterator<Item = (Self::EdgeId, Length)>;

    /// Gets the distance of the projected coordinate to the start vertex of the edge when following
    /// the edge coordinates.
    /// The returned length should be clamped between 0 and the edge length.
    ///
    /// The projection point shall be that coordinate on the line with the smallest distance between
    /// the line and the given coordinate.
    fn get_distance_along_edge(&self, edge: Self::EdgeId, coordinate: Coordinate) -> Length;

    /// Gets the coordinate along the edge geometry which is at the given distance from the edge
    /// start vertex.
    ///
    /// The distance is clamped within the edge length, therefore for distances lower of equal to
    /// zero the edge start vertex coordinate will be returned and for distances greater or equal to
    /// the edge length the edge end vertex coordinate will be returned.
    fn get_coordinate_along_edge(&self, edge: Self::EdgeId, distance: Length) -> Coordinate;

    /// Gets the bearing of a subsection A-B of the edge that goes from the coordinate (A) at the
    /// given distance from the start vertex, and the coordinate (B) that is at the given distance
    /// from A. The segment length can be negative.
    fn get_edge_bearing(
        &self,
        edge: Self::EdgeId,
        distance_from_start: Length,
        segment_length: Length,
    ) -> Bearing;

    /// Returns true if turning from the start edge to the end edge is not allowed.
    fn is_turn_restricted(&self, start: Self::EdgeId, end: Self::EdgeId) -> bool;

    /// Returns the total number of edges that are connected to the vertex, that is, the sum of the
    /// number of entering edges and the exiting edges.
    fn vertex_degree(&self, vertex: Self::VertexId) -> usize {
        self.vertex_edges(vertex).count()
    }

    /// Gets an iterator over all the edges (entering and exiting) into/from the given vertex.
    /// For each edge returns the edge ID and the edge end/start vertex respectively.
    /// Returns an empty iterator if the vertex doesn't belong to the graph.
    fn vertex_edges(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = (Self::EdgeId, Self::VertexId)> {
        self.vertex_entering_edges(vertex)
            .chain(self.vertex_exiting_edges(vertex))
    }
}

pub mod dijkstra;
pub mod path;

#[cfg(test)]
pub mod tests {
    #![allow(clippy::panic)]
    #![allow(clippy::disallowed_types)]

    mod geojson;
    mod network;

    pub use network::{EdgeId, NETWORK_GRAPH, NetworkGraph, VertexId};
}
