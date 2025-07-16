//! https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm

use crate::{DirectedGraph, Frc, Length};

#[derive(Debug, Clone, Copy)]
pub struct ShortestPathConfig {
    pub lowest_frc: Frc,
    pub max_length: Length,
}

impl Default for ShortestPathConfig {
    fn default() -> Self {
        Self {
            lowest_frc: Frc::Frc7,
            max_length: Length::MAX,
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub struct ShortestPath<EdgeId> {
    pub length: Length,
    pub edges: Vec<EdgeId>,
}

pub fn shortest_path<G: DirectedGraph>(
    config: &ShortestPathConfig,
    graph: &G,
    origin: G::VertexId,
    destination: G::VertexId,
) -> Option<ShortestPath<G::EdgeId>> {
    todo!()
}
