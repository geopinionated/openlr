use tracing::warn;

use crate::{
    DirectedGraph, EncoderConfig, EncoderError, IntermediateLocation, LineLocation, LocRefPoint,
    LocRefPoints, LocationError, ShortestRoute, shortest_path_location,
};

/// Resolves all the LRPs that should be necessary to encode the given line, and its expansion.
pub fn resolve_lrps<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: &LineLocation<G::EdgeId>,
) -> Result<LocRefPoints<G::EdgeId>, EncoderError> {
    let mut location: Vec<G::EdgeId> = line.path.clone();

    let last_edge = if let Some(&last_edge) = location.last() {
        last_edge
    } else {
        return Err(LocationError::Empty.into());
    };

    let last_lrp = LocRefPoint::from_last_node(config, graph, last_edge)?;
    let mut lrps = vec![];

    // Step - 3 Determine coverage of the location by a shortest-path.
    // Find shortest paths until the whole location is covered by a concatenation of these.
    while !location.is_empty() {
        match shortest_path_location(graph, &location, config.max_lrp_distance)? {
            // Step – 4 Check whether the calculated shortest-path covers the location completely.
            ShortestRoute::Location => {
                let lrp = LocRefPoint::from_node(config, graph, location)?;
                lrps.push(lrp);
                break;
            }
            // Step – 6 Restart shortest path calculation between the new intermediate location
            // reference point and the end of the location.
            ShortestRoute::Intermediate(intermediate) => {
                let IntermediateLocation { location_index, .. } = intermediate;
                let mut intermediates = intermediate_lrps(config, graph, &location, intermediate)?;
                lrps.append(&mut intermediates);
                location.drain(..location_index);
            }
            ShortestRoute::NotFound => {
                return Err(EncoderError::RouteNotFound);
            }
        }
    }

    lrps.push(last_lrp);

    // Step – 8 Check validity of the location reference path.
    debug_assert!(
        line.path
            .iter()
            .zip(lrps.iter().flat_map(|lrp| &lrp.edges))
            .all(|(e1, e2)| e1 == e2),
        "Resolved LRPs don't cover the exact expanded location edges"
    );

    // TODO: determine new intermediate LRPs if the maximum distance was exceeded.
    if let Some(lrp) = lrps
        .iter()
        .find(|lrp| lrp.point.dnp() > config.max_lrp_distance)
    {
        warn!("Maximum LRP distance exceeded by {lrp:?}");
        return Err(EncoderError::MaxDistanceExceeded);
    }

    Ok(LocRefPoints {
        lrps,
        pos_offset: line.pos_offset,
        neg_offset: line.neg_offset,
    })
}

fn intermediate_lrps<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    location: &[G::EdgeId],
    intermediate: IntermediateLocation,
) -> Result<Vec<LocRefPoint<G::EdgeId>>, EncoderError> {
    let location = location[..intermediate.location_index].to_vec();
    let lrp = LocRefPoint::from_node(config, graph, location)?;
    Ok(vec![lrp])
}
