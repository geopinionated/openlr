use rustc_hash::FxHashSet;

use crate::{DirectedGraph, Length};

#[derive(Debug, Clone, PartialEq)]
pub struct Path<EdgeId> {
    pub length: Length,
    pub edges: Vec<EdgeId>,
}

impl<EdgeId> Default for Path<EdgeId> {
    fn default() -> Self {
        Self {
            length: Length::ZERO,
            edges: vec![],
        }
    }
}

/// Returns true only if the path contains a loop when considering positive and negative offsets.
pub fn is_path_loop<G: DirectedGraph>(
    graph: &G,
    path: &[G::EdgeId],
    pos_offset: Length,
    neg_offset: Length,
) -> bool {
    let vertices = {
        let first = path
            .first()
            .filter(|_| pos_offset.is_zero())
            .and_then(|&e| graph.get_edge_start_vertex(e));

        let middle = path
            .iter()
            .skip(1)
            .flat_map(|&e| graph.get_edge_start_vertex(e));

        let last = path
            .last()
            .filter(|_| neg_offset.is_zero())
            .and_then(|&e| graph.get_edge_end_vertex(e));

        first.into_iter().chain(middle).chain(last)
    };

    let mut seen = FxHashSet::default();
    for v in vertices {
        if !seen.insert(v) {
            return true;
        }
    }

    false
}

/// Returns true only if all the edges of the path are sequentially connected in the given graph.
/// If turning between any of the sequentlay edges is not allowed returns false.
pub fn is_path_connected<G: DirectedGraph>(graph: &G, path: &[G::EdgeId]) -> bool {
    for window in path.windows(2) {
        let [e1, e2] = [window[0], window[1]];

        if graph.is_turn_restricted(e1, e2) {
            return false;
        }

        match graph.get_edge_end_vertex(e1) {
            Some(v) if !graph.vertex_exiting_edges(v).any(|(e, _)| e == e2) => return false,
            None => return false,
            Some(_) => (),
        };
    }

    true
}

/// Returns true if a node is valid and therefore the path starting/ending from/into this node
/// will not be further expanded.
///
/// A node is invalid if:
/// 1. The node has a degree of 2 and it is not a dead-end street:
///   - If a node has only one incoming and one outgoing line, then it can be skipped during route
///     search because no deviation at this point is possible.
///   - If the node is part of a dead-end street, then the node is still valid because otherwise at
///     this point it is only possible to go back the line.
///   - If such a dead-end node is the start or end of a location, it needs to be valid; otherwise,
///     every extension might be useless.
/// 2. The node has a degree of 4 and the incoming/outgoing lines are pairwise:
///    - If the incoming/outgoing lines form two pairs, then the node connects only two other nodes
///      and no deviation is possible (u-turns are also not allowed).
///    - If there are more connected lines, then the node is valid.
pub fn is_node_valid<G: DirectedGraph>(graph: &G, vertex: G::VertexId) -> bool {
    match graph.vertex_degree(vertex) {
        2 => {
            let edges: Vec<_> = graph.vertex_edges(vertex).map(|(e, _)| e).collect();
            debug_assert_eq!(edges.len(), 2);

            is_opposite_direction(graph, edges[0], edges[1]) // true: dead end
        }
        4 => {
            let mut edges: Vec<_> = graph.vertex_edges(vertex).map(|(e, _)| e).collect();
            debug_assert_eq!(edges.len(), 4);

            let opposite_index = edges
                .iter()
                .skip(1)
                .position(|&e| is_opposite_direction(graph, edges[0], e))
                .map(|i| i + 1);

            match opposite_index {
                None => true,
                Some(index) => {
                    // check the remaining edges
                    edges.swap_remove(index);
                    edges.swap_remove(0);
                    !is_opposite_direction(graph, edges[0], edges[1])
                }
            }
        }
        _ => true,
    }
}

/// Returns true only if the first edge is the directed edge that goes into the opposite
/// direction of the second edge, and they both connect at the same vertices.
pub fn is_opposite_direction<G: DirectedGraph>(graph: &G, e1: G::EdgeId, e2: G::EdgeId) -> bool {
    // n1 < ==== > n2
    graph.get_edge_start_vertex(e1) == graph.get_edge_end_vertex(e2)
        && graph.get_edge_end_vertex(e1) == graph.get_edge_start_vertex(e2)
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::graph::tests::{EdgeId, NETWORK_GRAPH, NetworkGraph, VertexId};

    #[test]
    fn is_opposite_direction_001() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert!(!is_opposite_direction(graph, EdgeId(16218), EdgeId(16218)));
        assert!(!is_opposite_direction(graph, EdgeId(16218), EdgeId(16219)));
        assert!(!is_opposite_direction(graph, EdgeId(16219), EdgeId(16218)));
        assert!(!is_opposite_direction(graph, EdgeId(16218), EdgeId(961826)));
        assert!(!is_opposite_direction(
            graph,
            EdgeId(-5707439),
            EdgeId(-8717174)
        ));

        assert!(is_opposite_direction(
            graph,
            EdgeId(4925290),
            EdgeId(-4925290)
        ));
        assert!(is_opposite_direction(
            graph,
            EdgeId(8345025),
            EdgeId(-8345025)
        ));
    }

    #[test]
    fn is_valid_node_001() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        // 1 ---> 2
        assert_eq!(graph.vertex_degree(VertexId(1)), 1);
        assert!(is_node_valid(graph, VertexId(1)));

        // 2 ---> 3 ---> 34
        assert_eq!(graph.vertex_degree(VertexId(3)), 2);
        assert!(
            !is_node_valid(graph, VertexId(3)),
            "2nd degree and not a dead-end"
        );

        // 105 <====> 58
        assert_eq!(graph.vertex_degree(VertexId(105)), 2);
        assert!(is_node_valid(graph, VertexId(105)), "2nd degree dead-end");

        // 1 ---> 2 ---> 3
        //       ||
        //       58
        assert_eq!(graph.vertex_degree(VertexId(2)), 4);
        assert!(is_node_valid(graph, VertexId(2)));

        // 139 <====> 138 <====> 140
        assert_eq!(graph.vertex_degree(VertexId(138)), 4);
        assert!(!is_node_valid(graph, VertexId(138)));

        assert_eq!(graph.vertex_degree(VertexId(75)), 6);
        assert!(is_node_valid(graph, VertexId(75)));

        assert_eq!(graph.vertex_degree(VertexId(20)), 6);
        assert!(is_node_valid(graph, VertexId(20)));

        assert_eq!(graph.vertex_degree(VertexId(68)), 8);
        assert!(is_node_valid(graph, VertexId(68)));
    }
}
