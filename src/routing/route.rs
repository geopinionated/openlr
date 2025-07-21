use std::fmt::Debug;
use std::ops::Deref;

use crate::{
    CandidateLine, CandidateLinePair, DecodeError, DirectedGraph, Length, LineLocation, Offsets,
};

#[derive(Debug, Clone, PartialEq)]
pub struct Route<EdgeId> {
    pub edges: Vec<EdgeId>,
    pub length: Length,
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
        self.0.iter().flat_map(|r| &r.edges).copied()
    }

    pub fn path_length(&self) -> Length {
        self.iter().map(|r| r.length).sum()
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

        let mut head_length = first_route.length - distance_from_start;
        let mut tail_length = last_route.length - distance_to_end;

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

    /// Concatenates all the routes into a single Line location trimed by the given offsets.
    pub fn into_line_location<G>(
        self,
        graph: &G,
        mut pos_offset: Length,
        mut neg_offset: Length,
    ) -> Result<LineLocation<G::EdgeId>, DecodeError>
    where
        G: DirectedGraph<EdgeId = EdgeId>,
    {
        if pos_offset + neg_offset > self.path_length() {
            return Err(DecodeError::InvalidOffsets((pos_offset, neg_offset)));
        }

        /// Returns the cut index and the total cut length.
        fn get_cut_index<G: DirectedGraph>(
            graph: &G,
            edges: impl IntoIterator<Item = G::EdgeId>,
            offset: Length,
        ) -> Option<(usize, Length)> {
            edges
                .into_iter()
                .enumerate()
                .scan(Length::ZERO, |length, (i, edge)| {
                    let current_length = *length;
                    if current_length <= offset {
                        *length += graph.get_edge_length(edge).unwrap_or(Length::ZERO);
                        Some((i, current_length))
                    } else {
                        None
                    }
                })
                .last()
        }

        let edges_count = self.edges().count();

        let start_cut = get_cut_index(graph, self.edges(), pos_offset);
        let (start, cut_length) = start_cut.unwrap_or((0, Length::ZERO));
        pos_offset -= cut_length;

        let end_cut = get_cut_index(graph, self.edges().rev(), neg_offset);
        let (end, cut_length) = end_cut
            .map(|(i, length)| (edges_count - i, length))
            .unwrap_or((edges_count, Length::ZERO));
        neg_offset -= cut_length;

        Ok(LineLocation {
            edges: self.edges().skip(start).take(end - start).collect(),
            pos_offset,
            neg_offset,
        })
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
