use tracing::debug;

use crate::encoder::expansion::line_location_with_expansion;
use crate::encoder::resolver::resolve_lrps;
use crate::{
    ClosedLine, ClosedLineLocation, DirectedGraph, EncodeError, EncoderConfig, Length, Line,
    LineLocation, Offsets, Poi, PoiLocation, PointAlongLine, PointAlongLineLocation,
};

/// 1. Check validity of the location and offsets to be encoded.
/// 2. Adjust start and end node of the location to represent valid map nodes.
/// 3. Determine coverage of the location by a shortest-path.
/// 4. Check whether the calculated shortest-path covers the location completely. Go to step 5 if
///    the location is not covered completely, otherwise go to step 7.
/// 5. Determine the position of a new intermediate location reference point so that the part of the
///    location between the start of the shortest-path calculation and the new intermediate is
///    covered completely by a shortest-path.
/// 6. Go to step 3 and restart shortest path calculation between the new intermediate location
///    reference point and the end of the location.
/// 7. Concatenate the calculated shortest-paths for a complete coverage of the location and form an
///    ordered list of location reference points.
/// 8. Check validity of the location reference path. If the location reference path is invalid then
///    go to step 9, if the location reference path is valid then go to step 10.
/// 9. Add a sufficient number of additional intermediate location reference points if the distance
///    between two location reference points exceeds the maximum distance. Remove the start/ end
///    LR-point if the positive/ negative offset value exceeds the length of the corresponding path.
/// 10. Create physical representation of the location reference.
pub fn encode_line<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: LineLocation<G::EdgeId>,
) -> Result<Line, EncodeError<G::Error>> {
    debug!("Encoding {line:?} with {config:?}");

    // Step – 1 Check validity of the location and offsets to be encoded
    let line = line.trim(graph)?;

    // Step – 2 Adjust start and end node of the location to represent valid map nodes
    let line = line_location_with_expansion(config, graph, line)?;
    debug_assert!(!line.path.is_empty());

    // Step – 3..8 Split location into intermediate LRPs until full coverage
    let lrps = resolve_lrps(config, graph, line)?;
    debug_assert!(lrps.len() > 1);

    // Step – 9 Trim LRPs if the offset values exceeds the length of the corresponding path
    let lrps = lrps.trim(config, graph)?;

    Ok(lrps.into())
}

pub fn encode_point_along_line<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    point: PointAlongLineLocation<G::EdgeId>,
) -> Result<PointAlongLine, EncodeError<G::Error>> {
    let line = LineLocation {
        path: point.path,
        pos_offset: point.offset,
        neg_offset: Length::ZERO,
    };

    let line = encode_line(config, graph, line)?;

    Ok(PointAlongLine {
        points: [line.points[0], line.points[line.points.len() - 1]],
        offset: line.offsets.pos,
        orientation: point.orientation,
        side: point.side,
    })
}

pub fn encode_poi<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    poi: PoiLocation<G::EdgeId>,
) -> Result<Poi, EncodeError<G::Error>> {
    let point = encode_point_along_line(config, graph, poi.point)?;

    Ok(Poi {
        point,
        coordinate: poi.coordinate,
    })
}

pub fn encode_closed_line<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: ClosedLineLocation<G::EdgeId>,
) -> Result<ClosedLine, EncodeError<G::Error>> {
    let line = LineLocation {
        path: line.path,
        pos_offset: Length::ZERO,
        neg_offset: Length::ZERO,
    };

    let mut line = encode_line(config, graph, line)?;
    debug_assert_eq!(line.offsets, Offsets::ZERO);

    let last_line = line.points[line.points.len() - 1].line;
    line.points.pop();

    Ok(ClosedLine {
        points: line.points,
        last_line,
    })
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::graph::tests::{EdgeId, NETWORK_GRAPH, NetworkGraph};
    use crate::{DecoderConfig, Length, Location, decode_base64_openlr, encode_base64_openlr};

    #[test]
    fn encoder_encode_line_location_reference_001() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let line = Location::Line(LineLocation {
            path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        });

        let encoded = encode_base64_openlr(&EncoderConfig::default(), graph, line.clone()).unwrap();
        let decoded = decode_base64_openlr(&DecoderConfig::default(), graph, &encoded).unwrap();
        assert_eq!(decoded, line);
    }

    #[test]
    fn encoder_encode_line_location_reference_002() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let line = Location::Line(LineLocation {
            path: vec![
                EdgeId(1653344),
                EdgeId(4997411),
                EdgeId(5359424),
                EdgeId(5359425),
            ],
            pos_offset: Length::from_meters(11.0),
            neg_offset: Length::from_meters(14.0),
        });

        let encoded = encode_base64_openlr(&EncoderConfig::default(), graph, line.clone()).unwrap();
        let decoded = decode_base64_openlr(&DecoderConfig::default(), graph, &encoded).unwrap();
        assert_eq!(decoded, line);
    }
}
