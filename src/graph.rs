use std::fmt::Debug;
use std::hash::Hash;

use crate::{Bearing, Coordinate, Fow, Frc, Length};

/// Directed graph.
/// Exposes the behavior of a Geospatial Index and a Road Network Graph.
/// Should be implemented by the graph the represents the map the decoder is supposed to run on.
pub trait DirectedGraph {
    /// Uniquely identify a vertex that belongs to the graph.
    type VertexId: Debug + Copy + Ord + Hash;
    /// Uniquely identify a directed edge that belongs to the graph.
    type EdgeId: Debug + Copy + PartialEq;

    /// Gets the start vertex of the directed edge.
    /// Returns None if the edge doesn't belong to the graph.
    fn get_edge_start_vertex(&self, edge: Self::EdgeId) -> Option<Self::VertexId>;

    /// Gets the end vertex of the directed edge.
    /// Returns None if the edge doesn't belong to the graph.
    fn get_edge_end_vertex(&self, edge: Self::EdgeId) -> Option<Self::VertexId>;

    /// Gets the total length of the directed edge.
    /// Returns None if the edge doesn't belong to the graph.
    fn get_edge_length(&self, edge: Self::EdgeId) -> Option<Length>;

    /// Gets the Functional Road Class of the directed edge.
    /// Returns None if the edge doesn't belong to the graph.
    fn get_edge_frc(&self, edge: Self::EdgeId) -> Option<Frc>;

    /// Gets the Form of Way of the directed edge.
    /// Returns None if the edge doesn't belong to the graph.
    fn get_edge_fow(&self, edge: Self::EdgeId) -> Option<Fow>;

    /// Gets an iterator over all the coordinates of the directed edge.
    /// The coordinates will be sorted from the first vertex to the last vertex.
    /// Returns an empty iterator if the edge doesn't belong to the graph.
    fn get_edge_coordinates(&self, edge: Self::EdgeId) -> impl Iterator<Item = Coordinate>;

    /// Gets an iterator over all the outgoing edges from the given vertex.
    /// For each edge returns the edge ID and the edge end vertex.
    /// Returns an empty iterator if the vertex doesn't belong to the graph.
    fn vertex_exiting_edges(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = (Self::EdgeId, Self::VertexId)>;

    /// Gets an iterator over all the incoming edges to the given vertex.
    /// For each edge returns the edge ID and the edge start vertex.
    /// Returns an empty iterator if the vertex doesn't belong to the graph.
    fn vertex_entering_edges(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = (Self::EdgeId, Self::VertexId)>;

    /// Gets an iterator over all the vertices that are within a max distance from the coordinate.
    /// For each vertex also returns the distance from the coordinate.
    /// Vertices must be returned sorted by their distance to the coordinate.
    /// Returns an empty iterator if no vertex could be found within distance.
    fn nearest_vertices_within_distance(
        &self,
        coordinate: Coordinate,
        max_distance: Length,
    ) -> impl Iterator<Item = (Self::VertexId, Length)>;

    /// Gets an iterator over all the edges that are within a max distance from the coordinate.
    /// For each edges also returns the distance from the coordinate.
    /// Edges must be returned sorted by their distance to the coordinate.
    /// Returns an empty iterator if no vertex could be found within distance.
    fn nearest_edges_within_distance(
        &self,
        coordinate: Coordinate,
        max_distance: Length,
    ) -> impl Iterator<Item = (Self::EdgeId, Length)>;

    /// Gets the distance of the projected coordinate to the start vertex of the edge when following
    /// the edge coordinates.
    /// Returns None if the edge doesn't belong to the graph or if the coordinate cannot be projected.
    fn get_distance_from_start_vertex(
        &self,
        edge: Self::EdgeId,
        coordinate: Coordinate,
    ) -> Option<Length>;

    /// Gets the bearing of a subsection A-B of the edge that goes from the coordinate (A) at the
    /// given distance from the start vertex, and the coordinate (B) that is at the given distance
    /// from A. The segment length can be negative.
    /// Returns None if the edge doesn't belong to the graph or if the segment cannot be constructed.
    fn get_edge_bearing_between(
        &self,
        edge: Self::EdgeId,
        distance_from_start: Length,
        segment_length: Length,
    ) -> Option<Bearing>;

    /// Returns true if turning from the start edge to the end edge is not allowed.
    /// If any of the given edges doesn't belog to the path returns true.
    fn is_turn_restricted(&self, start: Self::EdgeId, end: Self::EdgeId) -> bool;

    /// Returns the number of edges that are connected to the vertex, that is, the sum of the
    /// number of outgoing directed edges plus the incoming ones.
    fn vertex_degree(&self, vertex: Self::VertexId) -> usize {
        self.vertex_entering_edges(vertex).count() + self.vertex_exiting_edges(vertex).count()
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
