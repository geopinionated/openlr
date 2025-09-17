use std::cmp::Reverse;
use std::fmt::Debug;
use std::hash::Hash;

use radix_heap::RadixHeapMap;
use rustc_hash::FxHashMap;
use tracing::{debug, warn};

use crate::graph::dijkstra::unpack_path;
use crate::graph::path::{is_node_valid, is_path_loop};
use crate::{DirectedGraph, EncoderError, Length, LocationError};

/// Represents a subset, or the totality, of the location that is a shortest path.
#[derive(Debug, Clone, PartialEq)]
pub enum ShortestPath {
    /// The whole location is the shortest path, and therefore it has no intermediate edges.
    Location,
    /// Route of the location converges to the shortest path up to the intermediate edge.
    Intermediate(Intermediate),
    /// Route not found.
    NotFound,
}

/// An intermediate is a line in the location where the shortest path diverges.
/// Intermediates are good places where to split the location with a new LRP.
#[derive(Debug, Clone, PartialEq)]
pub struct Intermediate {
    /// Index of the intermediate edge in the location edges list, where the location can be split
    /// at its start to add a new LRP (since there is a deviation from the location path to the
    /// actual shortest path). The intermediate edge will act as the new starting line for the
    /// location subset that comes next.
    pub location_index: usize,
}

/// Returns the shortest route that follow the given location up until the location path diverges
/// from the shortest path, in which case the path route is split at an intermediate edge.
///
/// The encoder calculates a shortest-path between the current start line and the end line of the
/// location. This step will be executed until coverage of the location has been found.
///
/// If no intermediate location reference point was detected so far, the current start line is
/// identical to the start line of the location. If an intermediate location reference point was
/// detected in a previous step, then the line corresponding to the intermediate location reference
/// point acts as current start line. The start line is always part of the location.
///
/// The shortest path algorithm should take the whole network or a well-defined subset of the
/// network into account in order to calculate a shortest path between the current start and end.
/// Additionally it should fulfill the following constraints:
/// - All lengths of the lines should be measured in meters and should also be converted to integer
///   values, so that float values need to be rounded correctly.
/// - The search is node based and starts at the start node of the first line and ends at the end
///   node of the last line.
/// - The algorithm shall return an ordered list of lines representing the calculated shortest-path.
///
/// If no shortest-path can be calculated the encoding should fail. But this should never happen as
/// the location consists of concatenated lines.
pub fn shortest_path_location<G: DirectedGraph>(
    graph: &G,
    location: &[G::EdgeId],
    max_lrp_distance: Length,
) -> Result<ShortestPath, EncoderError> {
    debug!("Computing shortest path following {location:?}");

    let (origin, destination) = match location.first().zip(location.last()) {
        Some((origin, destination)) => (*origin, *destination),
        _ => return Err(LocationError::Empty.into()),
    };

    if origin == destination && location.len() > 1 {
        // origin and destination are equals but there is a path in between
        // skip the origin and proceed with the next line in the location
        return Ok(ShortestPath::Intermediate(Intermediate {
            location_index: 1,
        }));
    }

    // Find the indices of eventual loops into origin and destination
    let mut postfix = location.get(1..).into_iter().flatten();
    let origin_loop_index = postfix.position(|&e| e == origin);

    let mut prefix = location.get(..location.len() - 1).into_iter().flatten();
    let destination_loop_index = prefix.position(|&e| e == destination);

    if let Some(0) = origin_loop_index {
        // origin loops onto itself
        return Ok(ShortestPath::Intermediate(Intermediate {
            location_index: 1,
        }));
    }

    let max_length = location
        .iter()
        .filter_map(|&e| graph.get_edge_length(e))
        .sum();

    let origin_length = graph.get_edge_length(origin).unwrap_or(Length::MAX);

    let mut shortest_distances = FxHashMap::from_iter([(origin, origin_length)]);
    let mut previous_map: FxHashMap<G::EdgeId, G::EdgeId> = FxHashMap::default();
    let mut heap = RadixHeapMap::from_iter([(Reverse(origin_length), origin)]);
    let mut intermediator = Intermediator::new(graph, location, max_lrp_distance)?;

    while let Some((Reverse(h_distance), h_edge)) = heap.pop() {
        if let Some(location_index) = location.iter().position(|&e| e == h_edge) {
            // Step – 5 Determine the position of a new intermediate location reference point
            if let Some(intermediate) =
                intermediator.get_intermediate(h_edge, h_distance, &previous_map)?
            {
                return Ok(ShortestPath::Intermediate(intermediate));
            }

            let path = &location[..=location_index];
            debug_assert_eq!(path, unpack_path(&previous_map, h_edge));
            if is_path_loop(graph, path, Length::ZERO, Length::ZERO) {
                return Ok(ShortestPath::Intermediate(Intermediate { location_index }));
            }
        }

        if h_edge == destination {
            if let Some(location_index) = destination_loop_index {
                // route found until the destination loop ends
                debug_assert_eq!(location[location_index], destination);
                return Ok(ShortestPath::Intermediate(Intermediate { location_index }));
            }

            debug_assert_eq!(location, unpack_path(&previous_map, destination));
            return Ok(ShortestPath::Location);
        }

        if let Some(loop_index) = origin_loop_index
            && intermediator.last_edge_index == loop_index
        {
            // the loop ending at origin has been completely followed
            return Ok(ShortestPath::Intermediate(Intermediate {
                location_index: loop_index + 1,
            }));
        }

        // check if we already know a cheaper way to get to the end of this path from the origin
        let shortest_distance = *shortest_distances.get(&h_edge).unwrap_or(&Length::MAX);
        if h_distance > shortest_distance {
            continue;
        }

        let exiting_edges = graph
            .get_edge_end_vertex(h_edge)
            .into_iter()
            .flat_map(|v| graph.vertex_exiting_edges(v))
            .filter(|&(e, _)| !graph.is_turn_restricted(h_edge, e));

        for (edge, _) in exiting_edges {
            let edge_length = graph.get_edge_length(edge).unwrap_or(Length::MAX);
            let distance = h_distance + edge_length;

            if distance > max_length {
                continue;
            }

            let shortest_distance = *shortest_distances.get(&edge).unwrap_or(&Length::MAX);

            // check if we can follow the current path to reach the neighbor in a cheaper way
            if distance < shortest_distance {
                // Relax: we have now found a better way that we are going to explore
                shortest_distances.insert(edge, distance);
                previous_map.insert(edge, h_edge);
                heap.push(Reverse(distance), edge);
            }
        }
    }

    Ok(ShortestPath::NotFound)
}

/// Splits the location, if this doesn't follow the shortest path, at intermediate edges.
#[derive(Debug)]
struct Intermediator<'a, G: DirectedGraph> {
    graph: &'a G,
    location: &'a [G::EdgeId],
    max_lrp_distance: Length,
    last_edge: G::EdgeId,
    last_edge_index: usize,
}

impl<'a, G: DirectedGraph> Intermediator<'a, G> {
    fn new(
        graph: &'a G,
        location: &'a [G::EdgeId],
        max_lrp_distance: Length,
    ) -> Result<Self, EncoderError> {
        let last_edge = location.first().copied().ok_or(LocationError::Empty)?;
        let last_edge_index = 0;

        Ok(Self {
            graph,
            location,
            max_lrp_distance,
            last_edge,
            last_edge_index,
        })
    }

    /// Checks if the location should be split because it diverges from the shortest path.
    /// If it does, returns the intermediate (up to the diversion), otherwise returns None.
    ///
    /// If the location (between current start and end) is not fully part of the shortest-path, or
    /// the order of the lines is mixed up, then a proper intermediate location reference point
    /// needs to be determined. This intermediate must fulfill the following constraints:
    /// - The shortest-path between the current start and the line indicated by the intermediate
    ///   location reference point must cover the corresponding part of the location completely.
    /// - The start node of the line indicated by the intermediate location reference point shall be
    ///   positioned on a valid node (if no valid node can be determined, an invalid node may be
    ///   chosen).
    fn get_intermediate(
        &mut self,
        h_edge: G::EdgeId,
        h_distance: Length,
        previous_map: &FxHashMap<G::EdgeId, G::EdgeId>,
    ) -> Result<Option<Intermediate>, EncoderError> {
        if h_edge == self.location[0] {
            // the first line is always found because all paths start from the origin
            return Ok(None);
        }

        if let Some(next_edge) = self.get_location_successor(previous_map, h_edge) {
            self.last_edge = next_edge;
            self.last_edge_index += 1;

            let intermediate = if h_distance > self.max_lrp_distance {
                let location_index =
                    self.rfind_intermediate_index(previous_map).ok_or_else(|| {
                        warn!("Cannot rfind valid intermediate to split max LRP distance");
                        EncoderError::IntermediateError(self.last_edge_index)
                    })?;

                Some(Intermediate { location_index })
            } else {
                None
            };

            return Ok(intermediate);
        }

        // The location deviates from the shortest path that would allow to reach this element.
        // Find the start of this deviation along the current path, at least the start line should
        // be found because all the paths go back to the origin.
        let common_edge =
            find_common_edge(self.location, previous_map, h_edge).ok_or_else(|| {
                warn!("Cannot find common edge of {h_edge:?}");
                EncoderError::IntermediateError(self.last_edge_index)
            })?;

        // check if the deviation starts at the last element found so far or earlier in the path
        if common_edge == self.last_edge {
            let location_index = self.last_edge_index + 1;
            let intermediate = self.location[location_index];

            let previous_edge = *previous_map.get(&intermediate).ok_or_else(|| {
                warn!("Cannot find previous edge of intermediate {intermediate:?}");
                EncoderError::IntermediateError(self.last_edge_index)
            })?;

            if previous_edge != self.last_edge {
                // the shortest path to the intermediate (next location edge) does not include
                // the last (location) edge, therefore there is an additional (2nd) deviation
                let location_index = self.rfind_intermediate_index(previous_map);
                warn!("Multiple deviations from shortest path at {location_index:?}");
            }

            Ok(Some(Intermediate { location_index }))
        } else {
            let location_index = self.rfind_intermediate_index(previous_map).ok_or_else(|| {
                warn!("Cannot rfind valid intermediate earlier in the path");
                EncoderError::IntermediateError(self.last_edge_index)
            })?;

            Ok(Some(Intermediate { location_index }))
        }
    }

    /// Returns the next edge only if the given element is a direct successor of the last element
    /// found in the location, and therefore the next (location) edge belongs to the shortest
    /// path. Otherwise returns None.
    fn get_location_successor(
        &mut self,
        previous_map: &FxHashMap<G::EdgeId, G::EdgeId>,
        h_edge: G::EdgeId,
    ) -> Option<G::EdgeId> {
        let previous_element_edge = previous_map.get(&h_edge).copied()?;
        let h_edge = Some(h_edge);
        let next_edge = self.location.get(self.last_edge_index + 1).copied();

        if self.last_edge == previous_element_edge && next_edge == h_edge {
            next_edge
        } else {
            None
        }
    }

    /// Find an intermediate which has a valid start node.
    /// The method traverses the path from the last element found in the location back to the start
    /// and searches for a line having a valid start node.
    fn rfind_intermediate_index(
        &self,
        previous_map: &FxHashMap<G::EdgeId, G::EdgeId>,
    ) -> Option<usize> {
        let mut edge = self.last_edge;

        loop {
            // Check if we came back to the previous LRP, this means that we had a cycle in our path
            // and cannot find a valid node to place the intermediate. Fallback to the last element
            // found in the location.
            if edge == self.location[0] {
                return Some(self.last_edge_index);
            } else if is_node_valid(self.graph, self.graph.get_edge_start_vertex(edge)?) {
                return self.location.iter().position(|&e| e == edge);
            }

            edge = previous_map.get(&edge).copied()?;
        }
    }
}

/// Returns the first element that is part of both the location and the provided given edge path.
fn find_common_edge<EdgeId: Copy + Eq + Hash>(
    location: &[EdgeId],
    previous_map: &FxHashMap<EdgeId, EdgeId>,
    mut edge: EdgeId,
) -> Option<EdgeId> {
    while let Some(&previous_edge) = previous_map.get(&edge) {
        if location.contains(&previous_edge) {
            return Some(previous_edge);
        }
        edge = previous_edge;
    }
    None
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::graph::tests::{EdgeId, NETWORK_GRAPH, NetworkGraph};

    #[test]
    fn encoder_shortest_path_location_001() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = [EdgeId(-9044470), EdgeId(-9044471)];

        let route = shortest_path_location(graph, &location, Length::MAX).unwrap();

        assert_eq!(route, ShortestPath::Location);
    }

    #[test]
    fn encoder_shortest_path_location_002() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = [EdgeId(-9044470), EdgeId(-9044471), EdgeId(-9044472)];

        let route = shortest_path_location(graph, &location, Length::MAX).unwrap();

        assert_eq!(route, ShortestPath::Location);
    }

    #[test]
    fn encoder_shortest_path_location_003() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = [EdgeId(-9044472), EdgeId(4993083)];

        let route = shortest_path_location(graph, &location, Length::MAX).unwrap();

        assert_eq!(route, ShortestPath::Location);
    }

    #[test]
    fn encoder_shortest_path_location_004() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = [
            EdgeId(-7292030),
            EdgeId(-7292029),
            EdgeId(7516886),
            EdgeId(7516883),
            EdgeId(7516885),
        ];

        let route = shortest_path_location(graph, &location, Length::MAX).unwrap();

        assert_eq!(
            route,
            ShortestPath::Intermediate(Intermediate { location_index: 1 })
        );
    }

    #[test]
    fn encoder_shortest_path_location_005() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = [
            EdgeId(-4925290),
            EdgeId(-7292030),
            EdgeId(-7292029),
            EdgeId(7516886),
            EdgeId(7516883),
            EdgeId(7516885),
        ];

        let route = shortest_path_location(graph, &location, Length::MAX).unwrap();

        assert_eq!(
            route,
            ShortestPath::Intermediate(Intermediate { location_index: 2 })
        );
    }

    #[test]
    fn encoder_shortest_path_location_006() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = [EdgeId(-7519159), EdgeId(5104156), EdgeId(-7519157)];

        let route = shortest_path_location(graph, &location, Length::MAX).unwrap();

        assert_eq!(
            route,
            ShortestPath::Intermediate(Intermediate { location_index: 1 })
        );
    }

    #[test]
    fn encoder_shortest_path_location_007() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = [
            EdgeId(7531947),
            EdgeId(86727),
            EdgeId(4921654),
            EdgeId(-7144581),
            EdgeId(-7144582),
            EdgeId(-7144580),
            EdgeId(-7144579),
            EdgeId(-79715),
            EdgeId(7430361),
        ];

        let route = shortest_path_location(graph, &location, Length::MAX).unwrap();

        assert_eq!(
            route,
            ShortestPath::Intermediate(Intermediate { location_index: 2 })
        );
    }

    #[test]
    fn encoder_shortest_path_location_008() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = [
            EdgeId(7516884),
            EdgeId(-7292029),
            EdgeId(7516886),
            EdgeId(-7516883),
            EdgeId(7516884),
        ];

        let route = shortest_path_location(graph, &location, Length::MAX).unwrap();

        assert_eq!(
            route,
            ShortestPath::Intermediate(Intermediate { location_index: 1 })
        );
    }

    #[test]
    fn encoder_shortest_path_location_009() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = [
            EdgeId(7516884),
            EdgeId(7516884), // loop into first line
            EdgeId(7516885),
        ];

        let route = shortest_path_location(graph, &location, Length::MAX).unwrap();

        assert_eq!(
            route,
            ShortestPath::Intermediate(Intermediate { location_index: 1 })
        );
    }

    #[test]
    fn encoder_shortest_path_location_010() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = [
            EdgeId(-7516884),
            EdgeId(-7292029),
            EdgeId(7516886),
            EdgeId(7516883),
            EdgeId(-7516884), // loop into origin
            EdgeId(7292030),
        ];

        let route = shortest_path_location(graph, &location, Length::MAX).unwrap();

        assert_eq!(
            route,
            ShortestPath::Intermediate(Intermediate { location_index: 3 })
        );
    }

    #[test]
    fn encoder_shortest_path_location_011() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = [
            EdgeId(-7516885),
            EdgeId(-7516884), // loop into destination
            EdgeId(-7292029),
            EdgeId(7516886),
            EdgeId(7516883),
            EdgeId(-7516884),
        ];

        let route = shortest_path_location(graph, &location, Length::MAX).unwrap();

        assert_eq!(
            route,
            ShortestPath::Intermediate(Intermediate { location_index: 1 })
        );
    }

    #[test]
    fn encoder_shortest_path_location_012() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = [
            EdgeId(961825),
            EdgeId(7531950),
            EdgeId(-6770340),
            EdgeId(7531949),
            EdgeId(7430352),
            EdgeId(7430353),
            EdgeId(-4232179),
            EdgeId(961825),
            EdgeId(7531950),
            EdgeId(-6770340),
            EdgeId(7531949),
            EdgeId(7430352),
            EdgeId(7430353),
            EdgeId(-4232179),
            EdgeId(4993081),
            EdgeId(4957478),
            EdgeId(4957479),
            EdgeId(4957480),
            EdgeId(-869555),
            EdgeId(-869554),
        ];

        let route = shortest_path_location(graph, &location, Length::MAX).unwrap();

        assert_eq!(
            route,
            ShortestPath::Intermediate(Intermediate { location_index: 6 })
        );
    }

    #[test]
    fn encoder_shortest_path_location_013() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = [EdgeId(-9044470), EdgeId(-9044471), EdgeId(-9044472)];

        let route = shortest_path_location(graph, &location, Length::from_meters(19.0)).unwrap();

        assert_eq!(
            route,
            ShortestPath::Intermediate(Intermediate { location_index: 1 })
        );
    }

    #[test]
    fn encoder_shortest_path_location_014() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = [EdgeId(-9044470), EdgeId(-9044471), EdgeId(-9044472)];

        let route = shortest_path_location(graph, &location, Length::from_meters(30.0)).unwrap();

        assert_eq!(
            route,
            ShortestPath::Intermediate(Intermediate { location_index: 1 })
        );
    }

    #[test]
    fn encoder_shortest_path_location_015() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = [EdgeId(-9044470), EdgeId(-9044471), EdgeId(-9044472)];

        let route = shortest_path_location(graph, &location, Length::from_meters(31.0)).unwrap();

        assert_eq!(
            route,
            ShortestPath::Intermediate(Intermediate { location_index: 2 })
        );
    }

    #[test]
    fn encoder_shortest_path_location_016() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = [EdgeId(8717174), EdgeId(8717175), EdgeId(109783)];

        let route = shortest_path_location(graph, &location, Length::MAX).unwrap();

        assert_eq!(route, ShortestPath::Location);
    }

    #[test]
    fn encoder_shortest_path_location_017() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = [
            EdgeId(1653344),
            EdgeId(4997411),
            EdgeId(5359424),
            EdgeId(5359425),
        ];

        let route = shortest_path_location(graph, &location, Length::MAX).unwrap();

        assert_eq!(route, ShortestPath::Location);
    }
}
