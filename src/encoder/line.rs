use tracing::info;

use crate::encoder::expansion::line_location_expansion;
use crate::{DirectedGraph, EncoderConfig, EncoderError, LineLocation, ensure_line_is_valid};

pub fn encode_line<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: LineLocation<G::EdgeId>,
) -> Result<Vec<u8>, EncoderError> {
    info!("Encoding {line:?} with {config:?}");

    // Step – 1 Check validity of the location and offsets to be encoded
    ensure_line_is_valid(graph, &line, config.max_lrp_distance)?;
    let line = line.trim(graph)?;

    // TODO: dedup line edges?

    // Step – 2 Adjust start and end node of the location to represent valid map nodes
    let expansion = line_location_expansion(config, graph, &line);

    todo!()
}

/*
#[derive(Debug)]
struct LocRefPoint;

pub fn generate_lrps<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: &LineLocation<G::EdgeId>,
    expansion: &ExpansionPaths<G::EdgeId>,
) -> Result<Vec<LocRefPoint>, EncoderError> {
    let expanded_path = expand_line_location_path(graph, line, expansion);

    if expanded_path.edges.is_empty() {
        return Err(LocationError::Empty.into());
    }

    let mut lrps = vec![];

    // find shortest-path(s) until the whole location is covered by a
    // concatenation of these shortest-path(s)

    while !expanded_path.edges.is_empty() {
        // do route search between current start and end of location
        let result = todo!();
    }

    Ok(lrps)
}

#[derive(Debug)]
enum Route<EdgeId> {
    Route,
    /// Lines which can be used as intermediates which are good lines to split the location into
    /// several shortest-paths.
    Intermediate {
        route: Vec<EdgeId>,
        intermidiate: EdgeId,
        index: usize,
    },
    NotFound,
}

fn resolve_route<G: DirectedGraph>(
    graph: &G,
    path: Path<G::EdgeId>,
) -> Result<Route<G::EdgeId>, EncoderError> {
    debug_assert!(!path.edges.is_empty());

    // TODO
    let start_loop: Option<usize> = None;
    let end_loop: Option<usize> = None;

    let start_edge = path.edges[0];
    let end_edge = path.edges[path.edges.len() - 1];

    if start_edge == end_edge && path.edges.len() > 1 {
        // start and end are equals but there is a path in between
        // so skip the start and proceed with the next line in the location
        return Ok(Route::Intermediate {
            route: vec![start_edge],
            intermidiate: path.edges[1],
            index: 1,
        });
    } else if let Some(0) = start_loop {
        // there is a loop of a single line in the location
        return Ok(Route::Intermediate {
            route: vec![start_edge],
            intermidiate: path.edges[1],
            index: 1,
        });
    } else {
        // TODO: needs to be done only for "remaning location edges"
        let x = shortest_path_location(graph, &path.edges);
    }

    todo!()
}

fn expand_line_location_path<G: DirectedGraph>(
    graph: &G,
    line: &LineLocation<G::EdgeId>,
    expansion: &ExpansionPaths<G::EdgeId>,
) -> Path<G::EdgeId> {
    let ExpansionPaths { start, end } = expansion;

    let prefix = start.edges.iter();
    let suffix = end.edges.iter();
    let edges = prefix.chain(line.path.iter()).chain(suffix).copied();

    let length = start.length + line.path_length(graph) + end.length;

    Path {
        length,
        edges: edges.collect(),
    }
}
*/
