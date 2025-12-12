use crate::graph::path::{Path, is_node_valid, is_opposite_direction};
use crate::{DirectedGraph, EncodeError, EncoderConfig, Length, LineLocation};

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
pub fn line_location_with_expansion<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    mut line: LineLocation<G::EdgeId>,
) -> Result<LineLocation<G::EdgeId>, EncodeError<G::Error>> {
    let prefix = edge_backward_expansion(config, graph, &line)?;
    let mut postfix = edge_forward_expansion(config, graph, &line)?;

    let mut path = prefix.edges;
    path.reserve_exact(line.path.len() + postfix.edges.len());
    path.append(&mut line.path);
    path.append(&mut postfix.edges);

    line.path = path;
    line.pos_offset += prefix.length;
    line.neg_offset += postfix.length;

    Ok(line)
}

/// Returns the expansion path in forward direction (from the line last edge).
/// The path cannot contain any turn restriction, in which case this function returns an empty one.
fn edge_forward_expansion<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: &LineLocation<G::EdgeId>,
) -> Result<Path<G::EdgeId>, EncodeError<G::Error>> {
    let mut expansion = Path::default();
    let mut edge = line.path[line.path.len() - 1];
    let mut offset = line.neg_offset;

    while !is_node_valid(graph, graph.get_edge_end_vertex(edge)?)? {
        let vertex = graph.get_edge_end_vertex(edge)?;
        let candidates = graph.vertex_exiting_edges(vertex)?.map(|(e, _)| e);

        match resolve_edge_expansion(config, graph, line, offset, &expansion, edge, candidates)? {
            Some((e, length)) => {
                if let Some(last_edge) = expansion.edges.last()
                    && graph.is_turn_restricted(*last_edge, e)?
                {
                    return Ok(Path::default());
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
        && graph.is_turn_restricted(edge, e)?
    {
        return Ok(Path::default());
    }

    Ok(expansion)
}

/// Returns the expansion path in backward direction (into the line first edge).
/// The path cannot contain any turn restriction, in which case this function returns an empty one.
fn edge_backward_expansion<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: &LineLocation<G::EdgeId>,
) -> Result<Path<G::EdgeId>, EncodeError<G::Error>> {
    let mut expansion = Path::default();
    let mut edge = line.path[0];
    let mut offset = line.pos_offset;

    while !is_node_valid(graph, graph.get_edge_start_vertex(edge)?)? {
        let vertex = graph.get_edge_start_vertex(edge)?;
        let candidates = graph.vertex_entering_edges(vertex)?.map(|(e, _)| e);

        match resolve_edge_expansion(config, graph, line, offset, &expansion, edge, candidates)? {
            Some((e, length)) => {
                if let Some(last_edge) = expansion.edges.last()
                    && graph.is_turn_restricted(e, *last_edge)?
                {
                    return Ok(Path::default());
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
        && graph.is_turn_restricted(e, edge)?
    {
        return Ok(Path::default());
    }

    Ok(expansion)
}

/// Selects the next valid edge that can expand the line from the given candidate edge.
/// Returns the selected edge and its length, otherwise None if no edge could be selected.
#[allow(clippy::type_complexity)]
fn resolve_edge_expansion<G, I>(
    config: &EncoderConfig,
    graph: &G,
    line: &LineLocation<G::EdgeId>,
    offset: Length,
    expansion: &Path<G::EdgeId>,
    edge: G::EdgeId,
    candidates: I,
) -> Result<Option<(G::EdgeId, Length)>, EncodeError<G::Error>>
where
    G: DirectedGraph,
    I: IntoIterator<Item = G::EdgeId>,
{
    let Some(candidate) = select_edge_expansion_candidate(graph, edge, candidates)? else {
        return Ok(None);
    };

    let length = graph.get_edge_length(candidate)?;

    // including the edge in the expansion must not exceed the max distance or form a loop
    if offset + length > config.max_lrp_distance
        || line.path.contains(&candidate)
        || expansion.edges.contains(&candidate)
    {
        return Ok(None);
    }

    Ok(Some((candidate, length)))
}

/// Selects a single expansion edge from a list of candidates.
/// If no expansion is possible returns None.
fn select_edge_expansion_candidate<G, I>(
    graph: &G,
    edge: G::EdgeId,
    candidates: I,
) -> Result<Option<G::EdgeId>, EncodeError<G::Error>>
where
    G: DirectedGraph,
    I: IntoIterator<Item = G::EdgeId>,
{
    let candidates: Vec<_> = candidates.into_iter().take(3).collect();

    if candidates.is_empty() {
        return Ok(None);
    } else if candidates.len() == 1 {
        return Ok(Some(candidates[0]));
    } else if candidates.len() > 2 {
        return Ok(None);
    }

    debug_assert_eq!(candidates.len(), 2);
    let (e1, e2) = (candidates[0], candidates[1]);
    let is_e1_opposite = is_opposite_direction(graph, edge, e1)?;
    let is_e2_opposite = is_opposite_direction(graph, edge, e2)?;

    if is_e1_opposite && !is_e2_opposite {
        return Ok(Some(e2));
    } else if is_e2_opposite && !is_e1_opposite {
        return Ok(Some(e1));
    } else if is_e1_opposite && is_e2_opposite {
        let length = graph.get_edge_length(edge)?;

        let is_length_similar =
            |e| Ok::<_, G::Error>((length - graph.get_edge_length(e)?).meters().abs() <= 1.0);

        match (is_length_similar(e1)?, is_length_similar(e2)?) {
            (false, true) => return Ok(Some(e1)),
            (true, false) => return Ok(Some(e2)),
            _ => return Ok(None),
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
    fn encoder_select_edge_expansion_candidate_001() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert_eq!(
            select_edge_expansion_candidate(graph, EdgeId(16218), []).unwrap(),
            None
        );

        assert_eq!(
            select_edge_expansion_candidate(graph, EdgeId(16218), [EdgeId(16219)]).unwrap(),
            Some(EdgeId(16219))
        );

        assert_eq!(
            select_edge_expansion_candidate(graph, EdgeId(16218), [EdgeId(16219), EdgeId(3622025)])
                .unwrap(),
            None
        );

        assert_eq!(
            select_edge_expansion_candidate(graph, EdgeId(16218), [EdgeId(16219), EdgeId(3622025)])
                .unwrap(),
            None
        );

        assert_eq!(
            select_edge_expansion_candidate(
                graph,
                EdgeId(3622025),
                [EdgeId(-3622025), EdgeId(16219)]
            )
            .unwrap(),
            Some(EdgeId(16219))
        );

        assert_eq!(
            select_edge_expansion_candidate(
                graph,
                EdgeId(7020005),
                [EdgeId(-7020005), EdgeId(3622025), EdgeId(3622026)]
            )
            .unwrap(),
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
            line_location_with_expansion(&config, graph, line.clone()).unwrap(),
            line,
            "Start VertexId(68) and End VertexId(20) are both valid nodes"
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
            line_location_with_expansion(&config, graph, line).unwrap(),
            LineLocation {
                path: vec![EdgeId(16219), EdgeId(7430347)],
                pos_offset: Length::ZERO,
                neg_offset: Length::from_meters(78.0),
            },
            "Start VertexId(2) is a valid node but End VertexId(3) is not a valid node"
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
            line_location_with_expansion(&config, graph, line).unwrap(),
            LineLocation {
                path: vec![EdgeId(16219), EdgeId(7430347)],
                pos_offset: Length::from_meters(109.0),
                neg_offset: Length::ZERO,
            },
            "Start VertexId(3) is not a valid node but End VertexId(34) is a valid node"
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
            edge_forward_expansion(&config, graph, &line).unwrap(),
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
            edge_backward_expansion(&config, graph, &line).unwrap(),
            Path {
                edges: vec![EdgeId(-9044470), EdgeId(-9044471)],
                length: Length::from_meters(31.0)
            },
            "Start VertexId(140) is not a valid node"
        );
    }
}
