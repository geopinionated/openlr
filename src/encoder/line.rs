use tracing::info;

use crate::encoder::expansion::line_location_expansion;
use crate::{
    DirectedGraph, EncoderConfig, EncoderError, ExpansionPaths, LineLocation, LocationError, Path,
    ShortestRoute, ensure_line_is_valid, shortest_path_location,
};

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

#[derive(Debug)]
pub struct LocRefPoint<EdgeId> {
    /// The line this LRP refers to.
    pub edge: EdgeId,
    /// The shortest path to the next LRP.
    pub path: Vec<EdgeId>,
}

impl<EdgeId: Copy> LocRefPoint<EdgeId> {
    fn new(location: Vec<EdgeId>) -> Self {
        let edge = location[0];

        Self {
            edge,
            path: location,
        }
    }
}

pub fn resolve_lrps<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: &LineLocation<G::EdgeId>,
    expansion: &ExpansionPaths<G::EdgeId>,
) -> Result<Vec<LocRefPoint<G::EdgeId>>, EncoderError> {
    let mut location: Vec<G::EdgeId> = expansion.expand_line_path(line);

    if location.is_empty() {
        return Err(LocationError::Empty.into());
    }

    let mut lrps = vec![];

    // find shortest-path(s) until the whole location is covered by a
    // concatenation of these shortest-path(s)
    while !location.is_empty() {
        match shortest_path_location(graph, &location)? {
            ShortestRoute::Route(_) => {
                //
                todo!()
            }
            ShortestRoute::Intermediate(_) => {
                //
                todo!()
            }
            ShortestRoute::NotFound => {
                //
                todo!()
            }
        }
    }

    Ok(lrps)
}
