use std::cmp::Reverse;

use radix_heap::RadixHeapMap;
use rustc_hash::FxHashMap;
use tracing::trace;

use crate::graph::dijkstra::unpack_path;
use crate::graph::path::{Path, is_path_connected};
use crate::{DecodeError, DirectedGraph, Frc, Length};

pub fn shortest_path<G: DirectedGraph>(
    graph: &G,
    origin: G::EdgeId,
    destination: G::EdgeId,
    lowest_frc: Frc,
    max_length: Length,
) -> Result<Option<Path<G::EdgeId>>, DecodeError<G::Error>> {
    trace!(
        "Computing shortest path {origin:?} {:?} -> {destination:?} {:?}",
        graph.get_edge_start_vertex(origin),
        graph.get_edge_end_vertex(destination)
    );

    let origin_length = graph.get_edge_length(origin)?;
    let mut shortest_distances = FxHashMap::from_iter([(origin, origin_length)]);
    let mut previous_map: FxHashMap<G::EdgeId, G::EdgeId> = FxHashMap::default();
    let mut heap = RadixHeapMap::from_iter([(Reverse(origin_length), origin)]);

    while let Some((Reverse(h_distance), h_edge)) = heap.pop() {
        if h_edge == destination {
            // Unpacking: the shortest path from destination back to origin
            let edges = unpack_path(&previous_map, destination);
            debug_assert!(is_path_connected(graph, &edges)?, "{edges:?}");

            return Ok(Some(Path {
                length: h_distance,
                edges,
            }));
        }

        // check if we already know a cheaper way to get to the end of this path from the origin
        let shortest_distance = *shortest_distances.get(&h_edge).unwrap_or(&Length::MAX);
        if h_distance > shortest_distance {
            continue;
        }

        let exiting_edges = graph.vertex_exiting_edges(graph.get_edge_end_vertex(h_edge)?)?;

        for (edge, _) in exiting_edges {
            if graph.is_turn_restricted(h_edge, edge)? {
                continue;
            }

            let distance = h_distance + graph.get_edge_length(edge)?;
            let frc = graph.get_edge_frc(edge)?;

            if distance > max_length {
                trace!("Element distance too far: {edge:?} {distance} > {max_length}");
                continue;
            }

            if frc > lowest_frc {
                trace!("Element FRC too low: {edge:?} {frc:?} > {lowest_frc:?}",);
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

    Ok(None)
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::graph::tests::{EdgeId, NETWORK_GRAPH, NetworkGraph};

    #[test]
    fn decoder_shortest_path_001() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert_eq!(
            shortest_path(
                graph,
                EdgeId(8717174),
                EdgeId(8717174),
                Frc::Frc7,
                Length::MAX
            )
            .unwrap()
            .unwrap(),
            Path {
                length: Length::from_meters(136.0),
                edges: vec![EdgeId(8717174)]
            }
        );
    }

    #[test]
    fn decoder_shortest_path_002() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert_eq!(
            shortest_path(
                graph,
                EdgeId(8717174),
                EdgeId(109783),
                Frc::Frc7,
                Length::MAX
            )
            .unwrap()
            .unwrap(),
            Path {
                length: Length::from_meters(379.0),
                edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
            }
        );
    }

    #[test]
    fn decoder_shortest_path_003() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert_eq!(
            shortest_path(graph, EdgeId(16218), EdgeId(961826), Frc::Frc7, Length::MAX)
                .unwrap()
                .unwrap(),
            Path {
                length: Length::from_meters(753.0),
                edges: vec![
                    EdgeId(16218),
                    EdgeId(16219),
                    EdgeId(7430347),
                    EdgeId(4232179),
                    EdgeId(961826)
                ],
            }
        );
    }

    #[test]
    fn decoder_shortest_path_004() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert_eq!(
            shortest_path(
                graph,
                EdgeId(16218),
                EdgeId(961826),
                Frc::Frc7,
                Length::from_meters(752.0)
            )
            .unwrap(),
            None
        );
    }

    #[test]
    fn decoder_shortest_path_005() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert_eq!(
            shortest_path(
                graph,
                EdgeId(-4232179),
                EdgeId(-4232179),
                Frc::Frc7,
                Length::MAX
            )
            .unwrap()
            .unwrap(),
            Path {
                length: Length::from_meters(16.0),
                edges: vec![EdgeId(-4232179)],
            }
        );
    }

    #[test]
    fn decoder_shortest_path_006() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert_eq!(
            shortest_path(
                graph,
                EdgeId(16218),
                EdgeId(3227046),
                Frc::Frc7,
                Length::MAX
            )
            .unwrap()
            .unwrap(),
            Path {
                length: Length::from_meters(1462.0),
                edges: vec![
                    EdgeId(16218),
                    EdgeId(16219),
                    EdgeId(7430347),
                    EdgeId(961825),
                    EdgeId(7531950),
                    EdgeId(7531947),
                    EdgeId(7430351),
                    EdgeId(7430360),
                    EdgeId(7430361),
                    EdgeId(7430362),
                    EdgeId(7430348),
                    EdgeId(-244115),
                    EdgeId(-9497548),
                    EdgeId(-9497547),
                    EdgeId(3227046)
                ],
            }
        );
    }

    #[test]
    fn decoder_shortest_path_007() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert_eq!(
            shortest_path(
                graph,
                EdgeId(1653344),
                EdgeId(5359425),
                Frc::Frc7,
                Length::MAX
            )
            .unwrap()
            .unwrap(),
            Path {
                length: Length::from_meters(489.0),
                edges: vec![
                    EdgeId(1653344),
                    EdgeId(4997411),
                    EdgeId(5359424),
                    EdgeId(5359425),
                ],
            }
        );
    }
}
