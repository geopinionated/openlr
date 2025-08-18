use crate::graph::path::is_path_connected;
use crate::{DirectedGraph, Length, LocationError};

/// Defines a location (in a map) that can be encoded using the OpenLR encoder
/// and is also the result of the decoding process.
#[derive(Debug, Clone, PartialEq)]
pub enum Location<EdgeId> {
    Line(LineLocation<EdgeId>),
}

/// Location (in a map) that represents a Line Location Reference.
#[derive(Debug, Clone, PartialEq)]
pub struct LineLocation<EdgeId> {
    /// Complete list of edges that form the line.
    pub path: Vec<EdgeId>,
    /// Distance from the start of the first edge to the beginning of the location.
    pub pos_offset: Length,
    /// Distance from the end of the last edge to the end of the location.
    pub neg_offset: Length,
}

impl<EdgeId: Copy> LineLocation<EdgeId> {
    pub fn path_length<G>(&self, graph: &G) -> Length
    where
        G: DirectedGraph<EdgeId = EdgeId>,
    {
        self.path
            .iter()
            .filter_map(|&e| graph.get_edge_length(e))
            .sum()
    }

    /// Construct a valid Line location from the path trimed by its offsets.
    ///
    /// The offsets must fulfill the following constraints:
    /// - The sum of the positive and negative offset cannot be greater than the total length of the
    ///   location lines.
    /// - Positive offset value shall be less than the length of the first line:
    ///     - Otherwise the first line can be removed from the list of location lines and the offset
    ///       value must be reduced in the same way.
    ///     - This procedure shall be repeated until this constraint is fulfilled.
    /// - Negative offset value shall be less than the length of the last line:
    ///     - Otherwise the last line can be removed from the list of location lines and the offset
    ///       value must be reduced in the same way.
    ///     - This procedure shall be repeated until this constraint is fulfilled.
    pub fn trim<G>(self, graph: &G) -> Result<LineLocation<G::EdgeId>, LocationError>
    where
        G: DirectedGraph<EdgeId = EdgeId>,
    {
        let path_length = self.path_length(graph);

        let Self {
            mut path,
            mut pos_offset,
            mut neg_offset,
        } = self;

        if pos_offset + neg_offset >= path_length {
            return Err(LocationError::InvalidOffsets((pos_offset, neg_offset)));
        }

        let start_cut = get_path_cut(graph, path.iter().copied(), pos_offset);
        let (start, cut_length) = start_cut.unwrap_or((0, Length::ZERO));
        pos_offset -= cut_length;

        let end_cut = get_path_cut(graph, path.iter().rev().copied(), neg_offset);
        let end_cut = end_cut.map(|(i, length)| (path.len() - i, length));
        let (end, cut_length) = end_cut.unwrap_or((path.len(), Length::ZERO));
        neg_offset -= cut_length;

        if end < path.len() {
            path.drain(end..);
        }
        if start < path.len() {
            path.drain(..start);
        }

        let line = LineLocation {
            path,
            pos_offset,
            neg_offset,
        };

        ensure_line_is_valid(graph, &line, Length::MAX_BINARY_LRP_DISTANCE)?;

        Ok(line)
    }
}

/// Returns an error if the Line location is not valid.
///
/// A line location is valid if the following constraints are fulfilled:
/// - The location is a connected path.
/// - The location is traversable from its start to its end.
/// - The sum of the positive and negative offset cannot be greater than the total length of the
///   location lines.
fn ensure_line_is_valid<G: DirectedGraph>(
    graph: &G,
    line: &LineLocation<G::EdgeId>,
    max_lrp_distance: Length,
) -> Result<(), LocationError> {
    let LineLocation {
        ref path,
        pos_offset,
        neg_offset,
    } = *line;

    if path.is_empty() {
        return Err(LocationError::Empty);
    } else if !is_path_connected(graph, path) {
        return Err(LocationError::NotConnected);
    }

    if pos_offset > max_lrp_distance
        || neg_offset > max_lrp_distance
        || pos_offset + neg_offset >= line.path_length(graph)
    {
        return Err(LocationError::InvalidOffsets((pos_offset, neg_offset)));
    }

    Ok(())
}

/// Returns the cut index and the total cut length.
fn get_path_cut<G, I>(graph: &G, edges: I, offset: Length) -> Option<(usize, Length)>
where
    G: DirectedGraph,
    I: IntoIterator<Item = G::EdgeId>,
{
    edges
        .into_iter()
        .enumerate()
        .scan(Length::ZERO, |length, (i, edge)| {
            let current_length = *length;
            if current_length <= offset {
                *length += graph.get_edge_length(edge).unwrap_or(Length::ZERO);
                Some((i, current_length))
            } else {
                None
            }
        })
        .last()
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::graph::tests::{EdgeId, NETWORK_GRAPH, NetworkGraph};

    #[test]
    fn trim_line_location_001() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = LineLocation {
            path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        };

        assert_eq!(location.clone().trim(graph), Ok(location));
    }

    #[test]
    fn trim_line_location_002() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = LineLocation {
            path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)], // 136m + 51m + 192m
            pos_offset: Length::from_meters(10.0),
            neg_offset: Length::from_meters(10.0),
        };

        assert_eq!(location.clone().trim(graph), Ok(location));
    }

    #[test]
    fn trim_line_location_003() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = LineLocation {
            path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)], // 136m + 51m + 192m
            pos_offset: Length::from_meters(136.0),
            neg_offset: Length::ZERO,
        };

        assert_eq!(
            location.trim(graph),
            Ok(LineLocation {
                path: vec![EdgeId(8717175), EdgeId(109783)],
                pos_offset: Length::ZERO,
                neg_offset: Length::ZERO,
            })
        );
    }

    #[test]
    fn trim_line_location_004() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = LineLocation {
            path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)], // 136m + 51m + 192m
            pos_offset: Length::from_meters(137.0),
            neg_offset: Length::ZERO,
        };

        assert_eq!(
            location.trim(graph),
            Ok(LineLocation {
                path: vec![EdgeId(8717175), EdgeId(109783)],
                pos_offset: Length::from_meters(1.0),
                neg_offset: Length::ZERO,
            })
        );
    }

    #[test]
    fn trim_line_location_005() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let location = LineLocation {
            path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)], // 136m + 51m + 192m
            pos_offset: Length::from_meters(137.0),
            neg_offset: Length::from_meters(193.0),
        };

        assert_eq!(
            location.trim(graph),
            Ok(LineLocation {
                path: vec![EdgeId(8717175)],
                pos_offset: Length::from_meters(1.0),
                neg_offset: Length::from_meters(1.0),
            })
        );
    }

    #[test]
    fn trim_line_location_006() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let pos_offset = Length::from_meters(379.0);
        let neg_offset = Length::ZERO;

        let location = LineLocation {
            path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)], // 136m + 51m + 192m
            pos_offset,
            neg_offset,
        };

        assert_eq!(
            location.trim(graph),
            Err(LocationError::InvalidOffsets((pos_offset, neg_offset)))
        );
    }

    #[test]
    fn trim_line_location_007() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let pos_offset = Length::from_meters(279.0);
        let neg_offset = Length::from_meters(100.0);

        let location = LineLocation {
            path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)], // 136m + 51m + 192m
            pos_offset,
            neg_offset,
        };

        assert_eq!(
            location.trim(graph),
            Err(LocationError::InvalidOffsets((pos_offset, neg_offset)))
        );
    }
}
