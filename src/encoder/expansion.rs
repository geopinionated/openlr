use crate::path::Path;
use crate::{DirectedGraph, EncoderConfig, Length, LineLocation};

/// Returns the line expanded by forward and backward paths so that the start and the end of the
/// location are in valid nodes.
///
/// Data format rules recommends to place location reference points on valid nodes.
/// Valid nodes are such nodes where a shortest-path calculation needs to decide between several
/// different ways. Invalid nodes, on the contrary, are such nodes where a shortest path
/// calculation can step over.
///
/// Since the start and end of a location will become a location reference point these nodes need
/// to be adjusted to valid nodes if necessary (expansion of location). The expansion shall take
/// rules into account so that the maximum distance between two location reference points will not
/// be exceeded.
///
/// For line locations the real start of the location can then be referenced using offsets
/// (positive offset for the start node and negative offset for the end node, respectively).
pub fn line_location_expansion<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: &LineLocation<G::EdgeId>,
) -> LineLocation<G::EdgeId> {
    let prefix = edge_backward_expansion(config, graph, line);
    let postfix = edge_forward_expansion(config, graph, line);

    let path = prefix
        .edges
        .into_iter()
        .chain(line.path.iter().copied())
        .chain(postfix.edges)
        .collect();

    LineLocation {
        path,
        pos_offset: line.pos_offset + prefix.length,
        neg_offset: line.neg_offset + postfix.length,
    }
}

/// Returns the expansion path in forward direction (from the line end).
pub fn edge_forward_expansion<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: &LineLocation<G::EdgeId>,
) -> Path<G::EdgeId> {
    let mut expansion = Path::default();
    let mut edge = line.path[line.path.len() - 1];
    let mut offset = line.neg_offset;

    while let Some(vertex) = graph
        .get_edge_end_vertex(edge)
        .filter(|&v| !is_node_valid(graph, v))
    {
        let candidates = graph.vertex_exiting_edges(vertex).map(|(e, _)| e);

        match resolve_edge_expansion(config, graph, line, offset, &expansion, edge, candidates) {
            Some((e, length)) => {
                if let Some(last_edge) = expansion.edges.last()
                    && graph.is_turn_restricted(*last_edge, e)
                {
                    return Path::default();
                }

                expansion.edges.push(e);
                expansion.length += length;
                offset += length;
                edge = e;
            }
            None => break,
        };
    }

    if let Some(&e) = expansion.edges.first()
        && graph.is_turn_restricted(edge, e)
    {
        return Path::default();
    }

    expansion
}

/// Returns the expansion path in backward direction (from the line start).
pub fn edge_backward_expansion<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: &LineLocation<G::EdgeId>,
) -> Path<G::EdgeId> {
    let mut expansion = Path::default();
    let mut edge = line.path[0];
    let mut offset = line.pos_offset;

    while let Some(vertex) = graph
        .get_edge_start_vertex(edge)
        .filter(|&v| !is_node_valid(graph, v))
    {
        let candidates = graph.vertex_entering_edges(vertex).map(|(e, _)| e);

        match resolve_edge_expansion(config, graph, line, offset, &expansion, edge, candidates) {
            Some((e, length)) => {
                if let Some(last_edge) = expansion.edges.last()
                    && graph.is_turn_restricted(e, *last_edge)
                {
                    return Path::default();
                }

                expansion.edges.push(e);
                expansion.length += length;
                offset += length;
                edge = e;
            }
            None => break,
        };
    }

    expansion.edges.reverse();

    if let Some(&e) = expansion.edges.last()
        && graph.is_turn_restricted(e, edge)
    {
        return Path::default();
    }

    expansion
}

/// Selects the next valid edge candidate for the given line.
/// Returns the selected edge and its length, otherwise None if no candidate could be selected.
pub fn resolve_edge_expansion<G, I>(
    config: &EncoderConfig,
    graph: &G,
    line: &LineLocation<G::EdgeId>,
    offset: Length,
    expansion: &Path<G::EdgeId>,
    edge: G::EdgeId,
    candidates: I,
) -> Option<(G::EdgeId, Length)>
where
    G: DirectedGraph,
    I: IntoIterator<Item = G::EdgeId>,
{
    let candidate = select_edge_expansion_candidate(graph, edge, candidates)?;
    let length = graph.get_edge_length(candidate).unwrap_or(Length::MAX);

    // including the edge in the expansion must not exceed the max distance or form a loop
    if offset + length > config.max_lrp_distance
        || line.path.contains(&candidate)
        || expansion.edges.contains(&candidate)
    {
        return None;
    }

    Some((candidate, length))
}

/// Selects a single expansion edge from a list of candidates.
/// If no expansion is possible returns None.
pub fn select_edge_expansion_candidate<G, I>(
    graph: &G,
    edge: G::EdgeId,
    candidates: I,
) -> Option<G::EdgeId>
where
    G: DirectedGraph,
    I: IntoIterator<Item = G::EdgeId>,
{
    let candidates: Vec<_> = candidates.into_iter().take(3).collect();

    if candidates.is_empty() {
        return None;
    } else if candidates.len() == 1 {
        return Some(candidates[0]);
    } else if candidates.len() > 2 {
        return None;
    }

    debug_assert_eq!(candidates.len(), 2);
    let (e1, e2) = (candidates[0], candidates[1]);
    let is_e1_opposite = is_opposite_direction(graph, edge, e1);
    let is_e2_opposite = is_opposite_direction(graph, edge, e2);

    if is_e1_opposite && !is_e2_opposite {
        return Some(e2);
    } else if is_e2_opposite && !is_e1_opposite {
        return Some(e1);
    } else if is_e1_opposite && is_e2_opposite {
        let length = graph.get_edge_length(edge)?;

        let is_length_similar =
            |e| Some((length - graph.get_edge_length(e)?).meters().abs() <= 1.0);

        match (is_length_similar(e1)?, is_length_similar(e2)?) {
            (false, true) => return Some(e1),
            (true, false) => return Some(e2),
            _ => return None,
        }
    }

    None
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
            is_opposite_direction(graph, edges[0], edges[1]) // true: dead end
        }
        4 => {
            let mut edges: Vec<_> = graph.vertex_edges(vertex).map(|(e, _)| e).collect();

            let position = edges
                .iter()
                .skip(1)
                .position(|&e| is_opposite_direction(graph, edges[0], e))
                .map(|i| i + 1);

            match position {
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
    fn encoder_is_opposite_direction_001() {
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
    fn encoder_is_valid_node_001() {
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

    #[test]
    fn encoder_select_edge_expansion_candidate_001() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert_eq!(
            select_edge_expansion_candidate(graph, EdgeId(16218), []),
            None
        );

        assert_eq!(
            select_edge_expansion_candidate(graph, EdgeId(16218), [EdgeId(16219)]),
            Some(EdgeId(16219))
        );

        assert_eq!(
            select_edge_expansion_candidate(graph, EdgeId(16218), [EdgeId(16219), EdgeId(3622025)]),
            None
        );

        assert_eq!(
            select_edge_expansion_candidate(graph, EdgeId(16218), [EdgeId(16219), EdgeId(3622025)]),
            None
        );

        assert_eq!(
            select_edge_expansion_candidate(
                graph,
                EdgeId(3622025),
                [EdgeId(-3622025), EdgeId(16219)]
            ),
            Some(EdgeId(16219))
        );

        assert_eq!(
            select_edge_expansion_candidate(
                graph,
                EdgeId(7020005),
                [EdgeId(-7020005), EdgeId(3622025), EdgeId(3622026)]
            ),
            None
        );
    }

    #[test]
    fn encoder_edge_expansion_001() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = EncoderConfig::default();

        let line = LineLocation {
            path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        };

        assert_eq!(
            edge_forward_expansion(&config, graph, &line),
            Path::default(),
            "End VertexId(20) is a valid node"
        );

        assert_eq!(
            edge_backward_expansion(&config, graph, &line),
            Path::default(),
            "Start VertexId(68) is a valid node"
        );
    }

    #[test]
    fn encoder_edge_expansion_002() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = EncoderConfig::default();

        let line = LineLocation {
            path: vec![EdgeId(16219)],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        };

        assert_eq!(
            edge_forward_expansion(&config, graph, &line),
            Path {
                edges: vec![EdgeId(7430347)],
                length: Length::from_meters(78.0)
            },
            "End VertexId(3) is not a valid node"
        );

        assert_eq!(
            edge_backward_expansion(&config, graph, &line),
            Path::default(),
            "Start VertexId(2) is a valid node"
        );
    }

    #[test]
    fn encoder_edge_expansion_003() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = EncoderConfig::default();

        let line = LineLocation {
            path: vec![EdgeId(7430347)],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        };

        assert_eq!(
            edge_forward_expansion(&config, graph, &line),
            Path::default(),
            "End VertexId(34) is a valid node"
        );

        assert_eq!(
            edge_backward_expansion(&config, graph, &line),
            Path {
                edges: vec![EdgeId(16219)],
                length: Length::from_meters(109.0)
            },
            "Start VertexId(3) is not a valid node"
        );
    }

    #[test]
    fn encoder_edge_expansion_004() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = EncoderConfig::default();

        let line = LineLocation {
            path: vec![EdgeId(-9044470)],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        };

        assert_eq!(
            edge_forward_expansion(&config, graph, &line),
            Path {
                edges: vec![EdgeId(-9044471), EdgeId(-9044472)],
                length: Length::from_meters(26.0)
            },
            "End VertexId(138) is not a valid node"
        );
    }

    #[test]
    fn encoder_edge_expansion_005() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = EncoderConfig::default();

        let line = LineLocation {
            path: vec![EdgeId(-9044472)],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        };

        assert_eq!(
            edge_backward_expansion(&config, graph, &line),
            Path {
                edges: vec![EdgeId(-9044470), EdgeId(-9044471)],
                length: Length::from_meters(31.0)
            },
            "Start VertexId(140) is not a valid node"
        );
    }
}
