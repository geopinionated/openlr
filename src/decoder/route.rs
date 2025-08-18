use std::fmt::Debug;
use std::ops::Deref;

use crate::{CandidateLine, CandidateLinePair, DirectedGraph, Length, Offsets, Path};

#[derive(Debug, Clone, PartialEq)]
pub struct Route<EdgeId> {
    pub path: Path<EdgeId>,
    pub candidates: CandidateLinePair<EdgeId>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Routes<EdgeId>(Vec<Route<EdgeId>>);

impl<EdgeId> From<Vec<Route<EdgeId>>> for Routes<EdgeId> {
    fn from(routes: Vec<Route<EdgeId>>) -> Self {
        Self(routes)
    }
}

impl<EdgeId> Deref for Routes<EdgeId> {
    type Target = Vec<Route<EdgeId>>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<EdgeId: Debug + Copy + PartialEq> Routes<EdgeId> {
    pub fn edges(&self) -> impl DoubleEndedIterator<Item = EdgeId> {
        self.0.iter().flat_map(|r| &r.path.edges).copied()
    }

    pub fn to_path(&self) -> Vec<EdgeId> {
        self.edges().collect()
    }

    /// Gets the positive and negative offsets calculated from the projections of the LRPs
    /// into the first and last route (sub-path) respectively.
    pub fn calculate_offsets<G>(&self, graph: &G, offsets: Offsets) -> Option<(Length, Length)>
    where
        G: DirectedGraph<EdgeId = EdgeId>,
    {
        let first_route = self.first()?; // LRP1 -> LRP2
        let last_route = self.last()?; // Last LRP - 1 -> Last LRP

        let distance_from_start = first_route.distance_from_start();
        let distance_to_end = last_route.distance_to_end(graph);

        let mut head_length = first_route.path.length - distance_from_start;
        let mut tail_length = last_route.path.length - distance_to_end;

        if self.len() == 1 {
            // cut other opposite if start and end are in the same and only route
            head_length -= distance_to_end;
            tail_length -= distance_from_start;
        } else {
            if let Some(distance) = first_route.last_candidate().distance_to_projection {
                // the second route (sub-path) doesn't start at the beginning of the line
                // add this distance to the length of the first route
                head_length += distance;
            }

            if let Some(distance) = last_route.first_candidate().distance_to_projection {
                // the last route (sub-path) doesn't start at the beginning of the line
                // subtract this distance to the length of the last route
                tail_length -= distance;
            }
        }

        let pos_offset = offsets.distance_from_start(head_length) + distance_from_start;
        let neg_offset = offsets.distance_to_end(tail_length) + distance_to_end;

        Some((pos_offset, neg_offset))
    }
}

impl<EdgeId: Copy> Route<EdgeId> {
    pub fn distance_from_start(&self) -> Length {
        self.first_candidate()
            .distance_to_projection
            .unwrap_or(Length::ZERO)
    }

    pub fn distance_to_end<G>(&self, graph: &G) -> Length
    where
        G: DirectedGraph<EdgeId = EdgeId>,
    {
        let CandidateLine {
            edge,
            distance_to_projection,
            ..
        } = self.last_candidate();

        if let Some(projection) = distance_to_projection {
            let length = graph.get_edge_length(*edge).unwrap_or(Length::ZERO);
            length - *projection
        } else {
            Length::ZERO
        }
    }

    pub const fn first_candidate(&self) -> &CandidateLine<EdgeId> {
        &self.candidates.line_lrp1
    }

    pub const fn last_candidate(&self) -> &CandidateLine<EdgeId> {
        &self.candidates.line_lrp2
    }

    pub const fn first_candidate_edge(&self) -> EdgeId {
        self.first_candidate().edge
    }

    pub const fn last_candidate_edge(&self) -> EdgeId {
        self.last_candidate().edge
    }
}
