use tracing::info;

use crate::model::LineLocation;
use crate::{
    DecodeError, DecoderConfig, DirectedGraph, Line, find_candidate_lines, find_candidate_nodes,
    resolve_routes,
};

pub fn decode_line<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    line: Line,
) -> Result<LineLocation<G::EdgeId>, DecodeError> {
    info!("Decoding {line:?} with {config:?}");

    // Step – 2 For each location reference point find candidate nodes
    let nodes = find_candidate_nodes(config, graph, &line.points);
    debug_assert_eq!(nodes.len(), line.points.len());

    // Step – 3 For each location reference point find candidate lines
    // Step – 4 Rate candidate lines for each location reference point
    let lines = find_candidate_lines(config, graph, nodes)?;
    debug_assert_eq!(lines.len(), line.points.len());

    // Step – 5 Determine shortest-path(s) between all subsequent location reference points
    // Step – 6 Check validity of the calculated shortest-path(s)
    let routes = resolve_routes(config, graph, &lines)?;
    debug_assert_eq!(routes.len(), line.points.len() - 1);

    // Step – 7 Concatenate and trim path according to the offsets
    let (pos_off, neg_off) = routes
        .calculate_offsets(graph, line.offsets)
        .unwrap_or_default();

    routes.into_line_location(graph, pos_off, neg_off)
}
