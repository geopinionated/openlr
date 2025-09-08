use std::fmt::Debug;
use std::ops::{Deref, DerefMut};

use crate::decoder::candidates::{CandidateLine, CandidateLinePair};
use crate::graph::path::Path;
use crate::{DirectedGraph, Length, Offsets};

/// The shortest route between two (consecutive) LRPs.
#[derive(Debug, Clone, PartialEq)]
pub struct CandidateRoute<EdgeId> {
    pub path: Path<EdgeId>,
    pub candidates: CandidateLinePair<EdgeId>,
}

/// The sequence of all the shortest routes that connect each consecutive LRP pair.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CandidateRoutes<EdgeId>(Vec<CandidateRoute<EdgeId>>);

impl<EdgeId> From<Vec<CandidateRoute<EdgeId>>> for CandidateRoutes<EdgeId> {
    fn from(routes: Vec<CandidateRoute<EdgeId>>) -> Self {
        Self(routes)
    }
}

impl<EdgeId> Deref for CandidateRoutes<EdgeId> {
    type Target = Vec<CandidateRoute<EdgeId>>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<EdgeId> DerefMut for CandidateRoutes<EdgeId> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<EdgeId: Debug + Copy + PartialEq> CandidateRoutes<EdgeId> {
    pub fn edges(&self) -> impl DoubleEndedIterator<Item = EdgeId> {
        self.0.iter().flat_map(|r| &r.path.edges).copied()
    }

    pub fn path_length(&self) -> Length {
        self.0.iter().map(|r| r.path.length).sum()
    }

    pub fn to_path(&self) -> Vec<EdgeId> {
        self.edges().collect()
    }

    /// Gets the positive and negative offsets calculated from the projections of the LRPs
    /// into the first and last route (sub-path) respectively.
    pub fn calculate_offsets<G>(&self, graph: &G, offsets: Offsets) -> Option<(Length, Length)>
    where
        G: DirectedGraph<EdgeId = EdgeId>,
    {
        let first_route = self.first()?; // LRP1 -> LRP2
        let last_route = self.last()?; // Last LRP - 1 -> Last LRP

        let distance_from_start = first_route.distance_from_start();
        let distance_to_end = last_route.distance_to_end(graph);

        let mut head_length = first_route.path.length - distance_from_start;
        let mut tail_length = last_route.path.length - distance_to_end;

        if self.len() == 1 {
            // cut other opposite if start and end are in the same and only route
            head_length -= distance_to_end;
            tail_length -= distance_from_start;
        } else {
            if let Some(distance) = first_route.last_candidate().distance_to_projection {
                // the second route (sub-path) doesn't start at the beginning of the line
                // add this distance to the length of the first route
                head_length += distance;
            }

            if let Some(distance) = last_route.first_candidate().distance_to_projection {
                // the last route (sub-path) doesn't start at the beginning of the line
                // subtract this distance to the length of the last route
                tail_length -= distance;
            }
        }

        let pos_offset = offsets.distance_from_start(head_length) + distance_from_start;
        let neg_offset = offsets.distance_to_end(tail_length) + distance_to_end;

        Some((pos_offset.round(), neg_offset.round()))
    }
}

impl<EdgeId: Copy> CandidateRoute<EdgeId> {
    pub fn distance_from_start(&self) -> Length {
        self.first_candidate()
            .distance_to_projection
            .unwrap_or(Length::ZERO)
    }

    pub fn distance_to_end<G>(&self, graph: &G) -> Length
    where
        G: DirectedGraph<EdgeId = EdgeId>,
    {
        let CandidateLine {
            edge,
            distance_to_projection,
            ..
        } = self.last_candidate();

        if let Some(projection) = distance_to_projection {
            let length = graph.get_edge_length(edge).unwrap_or(Length::ZERO);
            (length - projection).max(Length::ZERO)
        } else {
            Length::ZERO
        }
    }

    /// Gets the positive and negative offsets calculated from the projections of the LRPs.
    pub fn calculate_offsets<G>(&self, graph: &G, offsets: Offsets) -> (Length, Length)
    where
        G: DirectedGraph<EdgeId = EdgeId>,
    {
        let distance_from_start = self.distance_from_start();
        let distance_to_end = self.distance_to_end(graph);
        let length = self.path.length - distance_from_start - distance_to_end;

        let pos_offset = offsets.distance_from_start(length) + distance_from_start;
        let neg_offset = offsets.distance_to_end(length) + distance_to_end;

        (pos_offset, neg_offset)
    }

    pub const fn first_candidate(&self) -> CandidateLine<EdgeId> {
        self.candidates.line_lrp1
    }

    pub const fn last_candidate(&self) -> CandidateLine<EdgeId> {
        self.candidates.line_lrp2
    }

    pub const fn first_candidate_edge(&self) -> EdgeId {
        self.first_candidate().edge
    }

    pub const fn last_candidate_edge(&self) -> EdgeId {
        self.last_candidate().edge
    }
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::graph::tests::{EdgeId, NETWORK_GRAPH, NetworkGraph};
    use crate::model::RatingScore;
    use crate::{Bearing, Coordinate, Fow, Frc, LineAttributes, PathAttributes, Point};

    #[test]
    fn decoder_calculate_offsets_001() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let first_lrp = Point {
            coordinate: Coordinate {
                lon: 13.46112,
                lat: 52.51711,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(107),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(381.0),
            }),
        };

        let last_lrp = Point {
            coordinate: Coordinate {
                lon: 13.46284,
                lat: 52.51500,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(17),
            },
            path: None,
        };

        let line_first_lrp = CandidateLine {
            lrp: first_lrp,
            edge: EdgeId(8717174),
            rating: RatingScore::from(926.3),
            distance_to_projection: None,
        };

        let line_last_lrp = CandidateLine {
            lrp: last_lrp,
            edge: EdgeId(109783),
            rating: RatingScore::from(924.9),
            distance_to_projection: None,
        };

        let routes = CandidateRoutes::from(vec![CandidateRoute {
            path: Path {
                edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
                length: Length::from_meters(379.0),
            },
            candidates: CandidateLinePair {
                line_lrp1: line_first_lrp,
                line_lrp2: line_last_lrp,
            },
        }]);

        let (offset_start, offset_end) =
            routes.calculate_offsets(graph, Offsets::default()).unwrap();

        assert_eq!(offset_start, Length::ZERO);
        assert_eq!(offset_end, Length::ZERO);
    }

    #[test]
    fn decoder_calculate_offsets_002() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let first_lrp = Point {
            coordinate: Coordinate {
                lon: 13.46112,
                lat: 52.51711,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(107),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(381.0),
            }),
        };

        let last_lrp = Point {
            coordinate: Coordinate {
                lon: 13.46284,
                lat: 52.51500,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(17),
            },
            path: None,
        };

        let line_first_lrp = CandidateLine {
            lrp: first_lrp,
            edge: EdgeId(8717174),
            rating: RatingScore::from(926.3),
            distance_to_projection: Some(Length::from_meters(10.0)),
        };

        let line_last_lrp = CandidateLine {
            lrp: last_lrp,
            edge: EdgeId(109783),
            rating: RatingScore::from(924.9),
            distance_to_projection: Some(Length::from_meters(92.0)),
        };

        let routes: CandidateRoutes<_> = vec![CandidateRoute {
            path: Path {
                edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
                length: Length::from_meters(379.0),
            },
            candidates: CandidateLinePair {
                line_lrp1: line_first_lrp,
                line_lrp2: line_last_lrp,
            },
        }]
        .into();

        let (offset_start, offset_end) =
            routes.calculate_offsets(graph, Offsets::default()).unwrap();

        assert_eq!(offset_start, Length::from_meters(10.0));
        assert_eq!(offset_end, Length::from_meters(100.0));
    }

    #[test]
    fn decoder_calculate_offsets_003() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let first_lrp = Point {
            coordinate: Coordinate {
                lon: 13.4615506,
                lat: 52.5170544,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(107),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(70.0),
            }),
        };

        let last_lrp = Point {
            coordinate: Coordinate {
                lon: 13.4625506,
                lat: 52.5168944,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(287),
            },
            path: None,
        };

        let line_first_lrp = CandidateLine {
            lrp: first_lrp,
            edge: EdgeId(8717174),
            rating: RatingScore::from(1128.7),
            distance_to_projection: Some(Length::from_meters(20.0)),
        };

        let line_last_lrp = CandidateLine {
            lrp: last_lrp,
            edge: EdgeId(8717174),
            rating: RatingScore::from(1122.7),
            distance_to_projection: Some(Length::from_meters(36.0)),
        };

        let routes: CandidateRoutes<_> = vec![CandidateRoute {
            path: Path {
                edges: vec![EdgeId(8717174)],
                length: Length::from_meters(136.0),
            },
            candidates: CandidateLinePair {
                line_lrp1: line_first_lrp,
                line_lrp2: line_last_lrp,
            },
        }]
        .into();

        let (offset_start, offset_end) =
            routes.calculate_offsets(graph, Offsets::default()).unwrap();

        assert_eq!(offset_start, Length::from_meters(20.0));
        assert_eq!(offset_end, Length::from_meters(100.0));
    }

    #[test]
    fn decoder_calculate_offsets_004() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let first_lrp = Point {
            coordinate: Coordinate {
                lon: 13.4615506,
                lat: 52.5170544,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(107),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(70.0),
            }),
        };

        let second_lrp = Point {
            coordinate: Coordinate {
                lon: 13.4625506,
                lat: 52.5168944,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(107),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(280.0),
            }),
        };

        let last_lrp = Point {
            coordinate: Coordinate {
                lon: 13.46284,
                lat: 52.51500,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(17),
            },
            path: None,
        };

        let line_first_lrp = CandidateLine {
            lrp: first_lrp,
            edge: EdgeId(8717174),
            rating: RatingScore::from(1128.7),
            distance_to_projection: Some(Length::from_meters(20.0)),
        };

        let line_second_lrp = CandidateLine {
            lrp: second_lrp,
            edge: EdgeId(8717174),
            rating: RatingScore::from(1122.7),
            distance_to_projection: Some(Length::from_meters(36.0)),
        };

        let line_last_lrp = CandidateLine {
            lrp: last_lrp,
            edge: EdgeId(109783),
            rating: RatingScore::from(924.9),
            distance_to_projection: None,
        };

        let routes: CandidateRoutes<_> = vec![
            CandidateRoute {
                path: Path {
                    edges: vec![], // first and second LRPs are on the same line
                    length: Length::ZERO,
                },
                candidates: CandidateLinePair {
                    line_lrp1: line_first_lrp,
                    line_lrp2: line_second_lrp,
                },
            },
            CandidateRoute {
                path: Path {
                    edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
                    length: Length::from_meters(379.0),
                },
                candidates: CandidateLinePair {
                    line_lrp1: line_second_lrp,
                    line_lrp2: line_last_lrp,
                },
            },
        ]
        .into();

        let (offset_start, offset_end) =
            routes.calculate_offsets(graph, Offsets::default()).unwrap();

        assert_eq!(offset_start, Length::from_meters(20.0));
        assert_eq!(offset_end, Length::ZERO);
    }

    #[test]
    fn decoder_calculate_offsets_005() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let first_lrp = Point {
            coordinate: Coordinate {
                lon: 13.46112,
                lat: 52.51711,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(107),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(381.0),
            }),
        };

        let second_lrp = Point {
            coordinate: Coordinate {
                lon: 13.46284,
                lat: 52.51500,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(197),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(45.0),
            }),
        };

        let last_lrp = Point {
            coordinate: Coordinate {
                lon: 13.4632200,
                lat: 52.5147507,
            },
            line: LineAttributes {
                frc: Frc::Frc2,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(280),
            },
            path: None,
        };

        let line_first_lrp = CandidateLine {
            lrp: first_lrp,
            edge: EdgeId(8717174),
            rating: RatingScore::from(1194.8),
            distance_to_projection: None,
        };

        let line_second_lrp = CandidateLine {
            lrp: second_lrp,
            edge: EdgeId(6770340),
            rating: RatingScore::from(1193.5),
            distance_to_projection: None,
        };

        let line_last_lrp = CandidateLine {
            lrp: last_lrp,
            edge: EdgeId(7531947),
            rating: RatingScore::from(1176.0),
            distance_to_projection: None,
        };

        let routes: CandidateRoutes<_> = vec![
            CandidateRoute {
                path: Path {
                    edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
                    length: Length::from_meters(379.0),
                },
                candidates: CandidateLinePair {
                    line_lrp1: line_first_lrp,
                    line_lrp2: line_second_lrp,
                },
            },
            CandidateRoute {
                path: Path {
                    edges: vec![EdgeId(6770340), EdgeId(7531947)],
                    length: Length::from_meters(53.0),
                },
                candidates: CandidateLinePair {
                    line_lrp1: line_second_lrp,
                    line_lrp2: line_last_lrp,
                },
            },
        ]
        .into();

        let (offset_start, offset_end) =
            routes.calculate_offsets(graph, Offsets::default()).unwrap();

        assert_eq!(offset_start, Length::ZERO);
        assert_eq!(offset_end, Length::ZERO);
    }

    #[test]
    fn decoder_calculate_offsets_006() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let first_lrp = Point {
            coordinate: Coordinate {
                lon: 13.46112,
                lat: 52.51711,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(107),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(381.0),
            }),
        };

        let second_lrp = Point {
            coordinate: Coordinate {
                lon: 13.46284,
                lat: 52.51500,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(197),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(45.0),
            }),
        };

        let last_lrp = Point {
            coordinate: Coordinate {
                lon: 13.4632200,
                lat: 52.5147507,
            },
            line: LineAttributes {
                frc: Frc::Frc2,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(280),
            },
            path: None,
        };

        let line_first_lrp = CandidateLine {
            lrp: first_lrp,
            edge: EdgeId(8717174),
            rating: RatingScore::from(1194.8),
            distance_to_projection: Some(Length::from_meters(10.0)),
        };

        let line_second_lrp = CandidateLine {
            lrp: second_lrp,
            edge: EdgeId(6770340),
            rating: RatingScore::from(1193.5),
            distance_to_projection: Some(Length::from_meters(5.0)),
        };

        let line_last_lrp = CandidateLine {
            lrp: last_lrp,
            edge: EdgeId(7531947),
            rating: RatingScore::from(1176.0),
            distance_to_projection: Some(Length::from_meters(27.0)),
        };

        let routes: CandidateRoutes<_> = vec![
            CandidateRoute {
                path: Path {
                    edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
                    length: Length::from_meters(379.0),
                },
                candidates: CandidateLinePair {
                    line_lrp1: line_first_lrp,
                    line_lrp2: line_second_lrp,
                },
            },
            CandidateRoute {
                path: Path {
                    edges: vec![EdgeId(6770340), EdgeId(7531947)],
                    length: Length::from_meters(53.0),
                },
                candidates: CandidateLinePair {
                    line_lrp1: line_second_lrp,
                    line_lrp2: line_last_lrp,
                },
            },
        ]
        .into();

        let (offset_start, offset_end) =
            routes.calculate_offsets(graph, Offsets::default()).unwrap();

        assert_eq!(offset_start, Length::from_meters(10.0));
        assert_eq!(offset_end, Length::from_meters(10.0));
    }
}
