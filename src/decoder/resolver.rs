use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};
use std::fmt::Debug;
use std::ops::Deref;

use tracing::debug;

use crate::{
    CandidateLine, CandidateLinePair, CandidateLines, DecodeError, DecoderConfig, DirectedGraph,
    Frc, Length, LineLocation, Offsets, RatingScore, ShortestPathConfig, shortest_path,
};

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
        let path_length: Length = self.iter().map(|r| r.length).sum();
        let edges_count = self.edges().count();

        if pos_offset + neg_offset > path_length {
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

    fn is_connected<G>(&self, graph: &G) -> bool
    where
        G: DirectedGraph<EdgeId = EdgeId>,
    {
        let edges: Vec<_> = self.edges().collect();

        for window in edges.windows(2) {
            let [e1, e2] = [window[0], window[1]];
            match graph.get_edge_end_vertex(e1) {
                Some(v) if !graph.vertex_exiting_edges(v).any(|(e, _)| e == e2) => return false,
                None => return false,
                Some(_) => (),
            };
        }

        true
    }
}

/// Shortest path from the LRP to the next one.
#[derive(Debug, Clone, PartialEq)]
pub struct Route<EdgeId> {
    pub edges: Vec<EdgeId>,
    pub length: Length,
    pub candidates: CandidateLinePair<EdgeId>,
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

    pub fn first_candidate(&self) -> &CandidateLine<EdgeId> {
        &self.candidates.line_lrp1
    }

    pub fn last_candidate(&self) -> &CandidateLine<EdgeId> {
        &self.candidates.line_lrp2
    }

    fn first_candidate_edge(&self) -> EdgeId {
        self.first_candidate().edge
    }

    fn last_candidate_edge(&self) -> EdgeId {
        self.last_candidate().edge
    }
}

/// The decoder needs to compute a shortest-path between each pair of subsequent location reference
/// points. For each pair of location reference points suitable candidate lines must be chosen. The
/// candidate line of the first LRPs of this pair acts as start of the shortest-path calculation.
/// The candidate line of the second location reference point of this pair is the end of the
/// shortest-path calculation. If the chosen lines are equal no shortest-path calculation needs to
/// be started.
///
/// The shortest path algorithm should take the part of the network into account which contains all
/// lines having a functional road class lower than or equal to the lowest functional road class of
/// the first location reference point of the pair. This value might be altered if the decoder
/// anticipates having different functional road class values than the encoder map.
///
/// Additionally the shortest-path algorithm should fulfill the following constraints:
/// - All lengths of the lines should be measured in meters and should also be converted to
///   integer values, float values need to be rounded correctly.
/// - The search is node based and will start at the start node of the first line and will end at
///   the end node of the last line.
/// - The algorithm shall return an ordered list of lines representing the calculated shortest-path.
///
/// If no shortest-path can be calculated for two subsequent location reference points, the decoder
/// might try a different pair of candidate lines or finally fail and report an error. If a
/// different pair of candidate lines is tried it might happen that the start line needs to be
/// changed. In such a case this also affects the end line of the previous shortest-path and this
/// path also needs to be re-calculated and checked again. The number of retries of shortest-path
/// calculations should be limited in order to guarantee a fast decoding process.
pub fn resolve_routes<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    candidate_lines: &[CandidateLines<G::EdgeId>],
) -> Result<Routes<G::EdgeId>, DecodeError> {
    debug!("Resolving routes for {} candidates", candidate_lines.len());

    if let Some(routes) = resolve_single_line_routes(graph, candidate_lines) {
        debug_assert!(routes.is_connected(graph));
        return Ok(routes);
    }

    let mut routes: Vec<Route<G::EdgeId>> = Vec::with_capacity(candidate_lines.len() - 1);

    for window in candidate_lines.windows(2) {
        let [candidates_lrp1, candidates_lrp2] = [&window[0], &window[1]];
        let pairs = resolve_top_k_candidate_pairs(config, candidates_lrp1, candidates_lrp2)?;

        let CandidateLines { lrp: lrp1, .. } = candidates_lrp1;
        let CandidateLines { lrp: lrp2, .. } = candidates_lrp2;

        let lowest_frc_value = lrp1.lfrcnp().value() + Frc::variance(&lrp1.lfrcnp());
        let lowest_frc = Frc::from_value(lowest_frc_value).unwrap_or(Frc::Frc7);

        if let Some(route) = resolve_candidate_pairs_path(config, graph, pairs, lowest_frc) {
            // TODO: if the previous route ends on a line that is not the start of this new route
            // the previous route needs to be re-computed
            if let Some(last_edge) = routes.last().map(|r| r.last_candidate_edge())
                && last_edge != route.first_candidate_edge()
            {
                return Err(DecodeError::AlternativeRouteNotFound((*lrp1, *lrp2)));
            }

            routes.push(route);
        } else {
            return Err(DecodeError::RouteNotFound((*lrp1, *lrp2)));
        }
    }

    let routes = Routes(routes);
    debug_assert!(routes.is_connected(graph));
    Ok(routes)
}

/// If all the best candidate lines are equal there is no need to compute top K candidates and
/// their shortest paths, we can just return the best candidate line for each LRP.
fn resolve_single_line_routes<G: DirectedGraph>(
    graph: &G,
    candidate_lines: &[CandidateLines<G::EdgeId>],
) -> Option<Routes<G::EdgeId>> {
    if let Some(best_candidate) = candidate_lines.first().and_then(|c| c.best_candidate())
        && candidate_lines
            .iter()
            .all(|c| c.best_candidate().map(|line| line.edge) == Some(best_candidate.edge))
    {
        let pairs = candidate_lines.windows(2).filter_map(|window| {
            Some(CandidateLinePair {
                line_lrp1: window[0].best_candidate()?,
                line_lrp2: window[1].best_candidate()?,
            })
        });

        let routes = pairs
            .enumerate()
            .map(|(i, candidates)| {
                let edges = if i == 0 {
                    vec![best_candidate.edge]
                } else {
                    vec![]
                };

                Route {
                    length: edges.iter().filter_map(|&e| graph.get_edge_length(e)).sum(),
                    edges,
                    candidates,
                }
            })
            .collect();

        Some(Routes(routes))
    } else {
        None
    }
}

fn resolve_candidate_pairs_path<G, I>(
    config: &DecoderConfig,
    graph: &G,
    pairs: I,
    lowest_frc: Frc,
) -> Option<Route<G::EdgeId>>
where
    G: DirectedGraph,
    I: IntoIterator<Item = CandidateLinePair<G::EdgeId>>,
{
    debug!("Resolving candidate pairs with lowest {lowest_frc:?}");

    for candidates in pairs {
        let CandidateLinePair {
            line_lrp1,
            line_lrp2,
        } = candidates;

        if line_lrp1.edge == line_lrp2.edge {
            let edges = if line_lrp2.lrp.is_last() {
                vec![line_lrp1.edge]
            } else {
                vec![]
            };

            return Some(Route {
                length: edges.iter().filter_map(|&e| graph.get_edge_length(e)).sum(),
                edges,
                candidates,
            });
        }

        let origin = graph.get_edge_start_vertex(line_lrp1.edge)?;
        let destination = if line_lrp2.lrp.is_last() {
            graph.get_edge_end_vertex(line_lrp2.edge)?
        } else {
            graph.get_edge_start_vertex(line_lrp2.edge)?
        };

        let path_config = ShortestPathConfig {
            lowest_frc,
            max_length: max_route_length(config, graph, &line_lrp1, &line_lrp2),
        };

        if let Some(path) = shortest_path(&path_config, graph, origin, destination) {
            debug_assert!(path.length <= path_config.max_length);
            let min_length = line_lrp1.lrp.dnp() - config.next_point_variance;

            if path.length >= min_length {
                return Some(Route {
                    edges: path.edges,
                    length: path.length,
                    candidates,
                });
            }
        }
    }

    None
}

fn max_route_length<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    line_lrp1: &CandidateLine<G::EdgeId>,
    line_lrp2: &CandidateLine<G::EdgeId>,
) -> Length {
    let mut max_distance = line_lrp1.lrp.dnp() + config.next_point_variance;

    // shortest path can only stop at distances between real vertices, therefore we need to
    // add the complete length when computing max distance upper bound if the lines were projected
    if line_lrp1.is_projected() {
        max_distance += graph
            .get_edge_length(line_lrp1.edge)
            .unwrap_or(Length::ZERO);
    }
    if line_lrp2.is_projected() {
        max_distance += graph
            .get_edge_length(line_lrp2.edge)
            .unwrap_or(Length::ZERO);
    }

    max_distance
}

fn resolve_top_k_candidate_pairs<EdgeId: Debug + Copy>(
    config: &DecoderConfig,
    lines_lrp1: &CandidateLines<EdgeId>,
    lines_lrp2: &CandidateLines<EdgeId>,
) -> Result<Vec<CandidateLinePair<EdgeId>>, DecodeError> {
    let max_size = lines_lrp1.lines.len() * lines_lrp2.lines.len();
    let k_size = max_size.min(config.max_number_retries + 1);
    debug!("Resolving candidate pair ratings with size={k_size}");

    let mut pair_ratings: BinaryHeap<Reverse<RatingScore>> = BinaryHeap::with_capacity(k_size + 1);
    let mut rating_pairs: HashMap<RatingScore, Vec<_>> = HashMap::with_capacity(k_size + 1);

    for &line_lrp1 in &lines_lrp1.lines {
        for &line_lrp2 in &lines_lrp2.lines {
            let pair_rating = line_lrp1.rating * line_lrp2.rating;
            pair_ratings.push(Reverse(pair_rating));

            if pair_ratings.len() <= k_size {
                rating_pairs
                    .entry(pair_rating)
                    .or_default()
                    .push(CandidateLinePair {
                        line_lrp1,
                        line_lrp2,
                    });

                continue;
            }

            let worst_rating = match pair_ratings.pop() {
                Some(Reverse(rating)) if pair_rating <= rating => continue,
                Some(Reverse(rating)) => rating,
                None => continue,
            };

            rating_pairs
                .entry(pair_rating)
                .or_default()
                .push(CandidateLinePair {
                    line_lrp1,
                    line_lrp2,
                });

            if let Some(pairs) = rating_pairs.get_mut(&worst_rating)
                && pairs.len() > 1
            {
                pairs.pop();
            } else {
                rating_pairs.remove(&worst_rating);
            }
        }
    }

    let mut candidates = Vec::with_capacity(k_size);
    while let Some(Reverse(rating)) = pair_ratings.pop() {
        candidates.extend(rating_pairs.remove(&rating).into_iter().flatten());
    }
    candidates.reverse();

    debug_assert_eq!(candidates.len(), k_size);
    debug_assert!(candidates.is_sorted_by_key(|pair| Reverse(pair.rating())));

    Ok(candidates)
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::{CandidateLine, Point};

    #[test]
    fn resolve_top_k_candidate_pairs_001() {
        let config = DecoderConfig {
            max_number_retries: 3,
            ..Default::default()
        };

        let line1 = CandidateLine {
            lrp: Point::default(),
            edge: 1,
            distance_to_projection: None,
            rating: RatingScore::from(926.3),
        };

        let line2 = CandidateLine {
            lrp: Point::default(),
            edge: 2,
            distance_to_projection: Some(Length::from_meters(141.6)),
            rating: RatingScore::from(880.4),
        };

        let line3 = CandidateLine {
            lrp: Point::default(),
            edge: 3,
            distance_to_projection: None,
            rating: RatingScore::from(924.9),
        };

        let line4 = CandidateLine {
            lrp: Point::default(),
            edge: 4,
            distance_to_projection: None,
            rating: RatingScore::from(100.0),
        };

        let line5 = CandidateLine {
            lrp: Point::default(),
            edge: 5,
            distance_to_projection: None,
            rating: RatingScore::from(10.0),
        };

        let pairs = resolve_top_k_candidate_pairs(
            &config,
            &CandidateLines {
                lrp: Point::default(),
                lines: vec![line1, line2],
            },
            &CandidateLines {
                lrp: Point::default(),
                lines: vec![line3, line4, line5],
            },
        )
        .unwrap();

        assert_eq!(
            pairs,
            [
                CandidateLinePair {
                    line_lrp1: line1,
                    line_lrp2: line3
                },
                CandidateLinePair {
                    line_lrp1: line2,
                    line_lrp2: line3
                },
                CandidateLinePair {
                    line_lrp1: line1,
                    line_lrp2: line4
                },
                CandidateLinePair {
                    line_lrp1: line2,
                    line_lrp2: line4
                }
            ]
        );
    }
}
