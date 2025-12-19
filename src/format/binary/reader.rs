use std::io::{Cursor, Read};

use base64::Engine;
use base64::prelude::BASE64_STANDARD;

use crate::format::binary::encoding::EncodedAttributes;
use crate::{
    Bearing, Circle, ClosedLine, Coordinate, DeserializeError, Fow, Frc, Grid, GridSize, Length,
    Line, LineAttributes, LocationReference, LocationType, Offset, PathAttributes, Poi, Point,
    PointAlongLine, Polygon, Rectangle,
};

/// Deserializes an OpenLR Location Reference encoded in Base64.
pub fn deserialize_base64_openlr(
    data: impl AsRef<[u8]>,
) -> Result<LocationReference, DeserializeError> {
    let data = BASE64_STANDARD.decode(data)?;
    deserialize_binary_openlr(&data)
}

/// Deserializes a binary representation of an OpenLR Location Reference.
pub fn deserialize_binary_openlr(data: &[u8]) -> Result<LocationReference, DeserializeError> {
    use LocationReference::*;

    let mut reader = OpenLrBinaryReader::new(data);

    match reader.read_header()? {
        LocationType::Line => Ok(Line(reader.read_line()?)),
        LocationType::GeoCoordinate => Ok(GeoCoordinate(reader.read_coordinate()?)),
        LocationType::PointAlongLine => Ok(PointAlongLine(reader.read_point_along_line()?)),
        LocationType::PoiWithAccessPoint => Ok(Poi(reader.read_poi()?)),
        LocationType::Circle => Ok(Circle(reader.read_circle()?)),
        LocationType::Rectangle => Ok(Rectangle(reader.read_rectangle()?)),
        LocationType::Grid => Ok(Grid(reader.read_grid()?)),
        LocationType::Polygon => Ok(Polygon(reader.read_polygon()?)),
        LocationType::ClosedLine => Ok(ClosedLine(reader.read_closed_line()?)),
    }
}

#[derive(Debug)]
struct OpenLrBinaryReader<'a> {
    cursor: Cursor<&'a [u8]>,
}

impl<'a> OpenLrBinaryReader<'a> {
    const fn new(data: &'a [u8]) -> Self {
        Self {
            cursor: Cursor::new(data),
        }
    }

    const fn len(&self) -> usize {
        self.cursor.get_ref().len()
    }

    fn read_header(&mut self) -> Result<LocationType, DeserializeError> {
        let mut header = [0u8; 1];
        self.cursor.read_exact(&mut header)?;
        let header = header[0];

        let version = header & 0b111;
        if version != 3 {
            return Err(DeserializeError::VersionNotSupported(version));
        }

        let location_type = (header >> 3) & 0b1111;
        let location_type = match location_type {
            0 => LocationType::Circle,
            1 => LocationType::Line,
            2 => LocationType::Polygon,
            4 => LocationType::GeoCoordinate,
            5 if self.len() > 17 => LocationType::PoiWithAccessPoint,
            5 => LocationType::PointAlongLine,
            8 if self.len() > 13 => LocationType::Grid,
            8 => LocationType::Rectangle,
            11 => LocationType::ClosedLine,
            _ => return Err(DeserializeError::InvalidHeader(header)),
        };

        Ok(location_type)
    }

    fn read_line(&mut self) -> Result<Line, DeserializeError> {
        let relative_points_count = (self.len() - 9) / 7;
        let mut line = Line::with_capacity(1 + relative_points_count);

        let mut coordinate = self.read_coordinate()?;
        let mut attributes = self.read_attributes()?;

        for _ in 0..relative_points_count {
            let dnp = self.read_dnp()?;

            line.points.push(Point {
                coordinate,
                line: attributes.line,
                path: Some(PathAttributes {
                    lfrcnp: attributes.lfrcnp()?,
                    dnp,
                }),
            });

            coordinate = self.read_relative_coordinate(coordinate)?;
            attributes = self.read_attributes()?;
        }

        line.points.push(Point {
            coordinate,
            line: attributes.line,
            path: None,
        });

        let mut read_offset = |offset_flag| {
            if offset_flag {
                self.read_offset()
            } else {
                Ok(Offset::default())
            }
        };

        line.offsets.pos = read_offset(attributes.pos_offset_flag())?;
        line.offsets.neg = read_offset(attributes.neg_offset_flag())?;

        Ok(line)
    }

    fn read_closed_line(&mut self) -> Result<ClosedLine, DeserializeError> {
        let relative_points_count = (self.len() - 12) / 7;
        let mut line = ClosedLine::with_capacity(1 + relative_points_count);

        let mut coordinate = self.read_coordinate()?;
        let attributes = self.read_attributes()?;
        let dnp = self.read_dnp()?;
        line.points.push(Point {
            coordinate,
            line: attributes.line,
            path: Some(PathAttributes {
                lfrcnp: attributes.lfrcnp()?,
                dnp,
            }),
        });

        for _ in 0..relative_points_count {
            coordinate = self.read_relative_coordinate(coordinate)?;
            let attributes = self.read_attributes()?;
            let dnp = self.read_dnp()?;
            line.points.push(Point {
                coordinate,
                line: attributes.line,
                path: Some(PathAttributes {
                    lfrcnp: attributes.lfrcnp()?,
                    dnp,
                }),
            });
        }

        let attributes = self.read_attributes()?;
        line.last_line = attributes.line;

        Ok(line)
    }

    fn read_point_along_line(&mut self) -> Result<PointAlongLine, DeserializeError> {
        let coordinate = self.read_coordinate()?;
        let attributes = self.read_attributes()?;
        let dnp = self.read_dnp()?;
        let orientation = attributes.orientation()?;

        let point_1 = Point {
            coordinate,
            line: attributes.line,
            path: Some(PathAttributes {
                lfrcnp: attributes.lfrcnp()?,
                dnp,
            }),
        };

        let coordinate = self.read_relative_coordinate(coordinate)?;
        let attributes = self.read_attributes()?;
        let side = attributes.side()?;

        let point_2 = Point {
            coordinate,
            line: attributes.line,
            path: None,
        };

        let offset = if attributes.pos_offset_flag() {
            self.read_offset()?
        } else {
            Offset::default()
        };

        Ok(PointAlongLine {
            points: [point_1, point_2],
            offset,
            orientation,
            side,
        })
    }

    fn read_poi(&mut self) -> Result<Poi, DeserializeError> {
        let point = self.read_point_along_line()?;
        let coordinate = self.read_relative_coordinate(point.points[0].coordinate)?;
        Ok(Poi { point, coordinate })
    }

    fn read_circle(&mut self) -> Result<Circle, DeserializeError> {
        let center = self.read_coordinate()?;
        let radius = self.read_radius()?;
        Ok(Circle { center, radius })
    }

    fn read_rectangle(&mut self) -> Result<Rectangle, DeserializeError> {
        let lower_left = self.read_coordinate()?;

        let upper_right = if self.len() > 11 {
            self.read_coordinate()?
        } else {
            self.read_relative_coordinate(lower_left)?
        };

        Ok(Rectangle {
            lower_left,
            upper_right,
        })
    }

    fn read_grid(&mut self) -> Result<Grid, DeserializeError> {
        let lower_left = self.read_coordinate()?;

        let upper_right = if self.len() > 15 {
            self.read_coordinate()?
        } else {
            self.read_relative_coordinate(lower_left)?
        };

        let rect = Rectangle {
            lower_left,
            upper_right,
        };

        let size = self.read_grid_size()?;

        Ok(Grid { rect, size })
    }

    fn read_polygon(&mut self) -> Result<Polygon, DeserializeError> {
        let relative_corners_count = (self.len() - 7) / 4;
        let mut polygon = Polygon::with_capacity(1 + relative_corners_count);

        let mut coordinate = self.read_coordinate()?;
        polygon.corners.push(coordinate);

        for _ in 0..relative_corners_count {
            coordinate = self.read_relative_coordinate(coordinate)?;
            polygon.corners.push(coordinate);
        }

        Ok(polygon)
    }

    fn read_coordinate(&mut self) -> Result<Coordinate, DeserializeError> {
        let mut read_degrees = || -> Result<f64, DeserializeError> {
            let mut c = [0u8; 3];
            self.cursor.read_exact(&mut c)?;
            Ok(Coordinate::degrees_from_be_bytes(c))
        };

        let lon = read_degrees()?;
        let lat = read_degrees()?;
        let coordinate = Coordinate::new(lon, lat)?;
        Ok(coordinate)
    }

    fn read_relative_coordinate(
        &mut self,
        previous: Coordinate,
    ) -> Result<Coordinate, DeserializeError> {
        let mut read_degrees = |previous| -> Result<f64, DeserializeError> {
            let mut c = [0u8; 2];
            self.cursor.read_exact(&mut c)?;
            Ok(Coordinate::degrees_from_be_bytes_relative(c, previous))
        };

        let lon = read_degrees(previous.lon)?;
        let lat = read_degrees(previous.lat)?;
        let coordinate = Coordinate::new(lon, lat)?;
        Ok(coordinate)
    }

    fn read_attributes(&mut self) -> Result<EncodedAttributes, DeserializeError> {
        let mut attributes = [0u8; 2];
        self.cursor.read_exact(&mut attributes)?;

        let fow = Fow::try_from_byte(attributes[0] & 0b111)?;
        let frc = Frc::try_from_byte((attributes[0] >> 3) & 0b111)?;
        let orientation_or_side = (attributes[0] >> 6) & 0b11;
        let bearing = Bearing::from_byte(attributes[1] & 0b11111);
        let lfrcnp_or_flags = (attributes[1] >> 5) & 0b111;

        Ok(EncodedAttributes {
            line: LineAttributes { frc, fow, bearing },
            lfrcnp_or_flags,
            orientation_or_side,
        })
    }

    fn read_dnp(&mut self) -> Result<Length, DeserializeError> {
        let mut dnp = [0u8; 1];
        self.cursor.read_exact(&mut dnp)?;
        Ok(Length::dnp_from_byte(dnp[0]))
    }

    fn read_offset(&mut self) -> Result<Offset, DeserializeError> {
        let mut offset = [0u8; 1];
        self.cursor.read_exact(&mut offset)?;
        Ok(Offset::from_byte(offset[0]))
    }

    fn read_radius(&mut self) -> Result<Length, DeserializeError> {
        let mut radius = [0u8; 4];
        let length = self.cursor.read(&mut radius)?;
        Ok(Length::radius_from_be_bytes(&radius[..length]))
    }

    fn read_grid_size(&mut self) -> Result<GridSize, DeserializeError> {
        let mut size = [0u8; 4];
        self.cursor.read_exact(&mut size)?;
        Ok(GridSize::from_be_bytes(size))
    }
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::model::Offsets;
    use crate::{Orientation, SideOfRoad};

    #[test]
    fn openlr_deserialize_version_1_not_supported() {
        assert_eq!(
            deserialize_base64_openlr("CQcm6yX4vTPGFwM7AskzCw==").unwrap_err(),
            DeserializeError::VersionNotSupported(1)
        );
    }

    #[test]
    fn openlr_deserialize_version_2_not_supported() {
        assert_eq!(
            deserialize_base64_openlr("CgRbWyNG9BpsCQCb/jsbtAT/6/+jK1kC").unwrap_err(),
            DeserializeError::VersionNotSupported(2)
        );
    }

    #[test]
    fn openlr_deserialize_invalid_header() {
        assert_eq!(
            deserialize_base64_openlr("ewGkNSK5Wg==").unwrap_err(),
            DeserializeError::InvalidHeader(0b01111011)
        );
    }

    #[test]
    fn openlr_deserialize_line_location_reference_001() {
        let location = deserialize_base64_openlr("CwRbWyNG9RpsCQCb/jsbtAT/6/+jK1lE").unwrap();

        assert_eq!(
            location,
            LocationReference::Line(Line {
                points: vec![
                    Point {
                        coordinate: Coordinate {
                            lon: 6.1268198,
                            lat: 49.608_517
                        },
                        line: LineAttributes {
                            frc: Frc::Frc3,
                            fow: Fow::MultipleCarriageway,
                            bearing: Bearing::from_degrees(141)
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc3,
                            dnp: Length::from_meters(557.0)
                        })
                    },
                    Point {
                        coordinate: Coordinate {
                            lon: 6.128_37,
                            lat: 49.603_99
                        },
                        line: LineAttributes {
                            frc: Frc::Frc3,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(231)
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc5,
                            dnp: Length::from_meters(264.0)
                        })
                    },
                    Point {
                        coordinate: Coordinate {
                            lon: 6.128_16,
                            lat: 49.603_058
                        },
                        line: LineAttributes {
                            frc: Frc::Frc5,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(287)
                        },
                        path: None
                    }
                ],
                offsets: Offsets {
                    pos: Offset::from_range(0.26757812),
                    neg: Offset::default()
                }
            })
        );
    }

    #[test]
    fn openlr_deserialize_line_location_reference_002() {
        let location = deserialize_base64_openlr("CwB67CGukRxiCACyAbwaMXU=").unwrap();

        assert_eq!(
            location,
            LocationReference::Line(Line {
                points: vec![
                    Point {
                        coordinate: Coordinate {
                            lon: 0.6752192,
                            lat: 47.365_16
                        },
                        line: LineAttributes {
                            frc: Frc::Frc3,
                            fow: Fow::Roundabout,
                            bearing: Bearing::from_degrees(28)
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc3,
                            dnp: Length::from_meters(498.0)
                        })
                    },
                    Point {
                        coordinate: Coordinate {
                            lon: 0.6769992,
                            lat: 47.369_602
                        },
                        line: LineAttributes {
                            frc: Frc::Frc3,
                            fow: Fow::MultipleCarriageway,
                            bearing: Bearing::from_degrees(197)
                        },
                        path: None
                    },
                ],
                offsets: Offsets {
                    pos: Offset::default(),
                    neg: Offset::from_range(0.45898438)
                }
            })
        );
    }

    #[test]
    fn openlr_deserialize_line_location_reference_003() {
        let location = deserialize_base64_openlr("CwcX6CItqAs6AQAAAAALGg==").unwrap();

        assert_eq!(
            location,
            LocationReference::Line(Line {
                points: vec![
                    Point {
                        coordinate: Coordinate {
                            lon: 9.975_06,
                            lat: 48.063_286
                        },
                        line: LineAttributes {
                            frc: Frc::Frc1,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(298)
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc1,
                            dnp: Length::from_meters(88.0)
                        })
                    },
                    Point {
                        coordinate: Coordinate {
                            lon: 9.975_06,
                            lat: 48.063_286
                        },
                        line: LineAttributes {
                            frc: Frc::Frc1,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(298)
                        },
                        path: None
                    },
                ],
                offsets: Offsets::default()
            })
        );
    }

    #[test]
    fn openlr_deserialize_line_location_reference_004() {
        let location = deserialize_base64_openlr("CwRbWyNG9BpgAACa/jsboAD/6/+kKwA=").unwrap();

        assert_eq!(
            location,
            LocationReference::Line(Line {
                points: vec![
                    Point {
                        coordinate: Coordinate {
                            lon: 6.1268198,
                            lat: 49.608_498
                        },
                        line: LineAttributes {
                            frc: Frc::Frc3,
                            fow: Fow::MultipleCarriageway,
                            bearing: Bearing::from_degrees(6)
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc3,
                            dnp: Length::from_meters(29.0)
                        })
                    },
                    Point {
                        coordinate: Coordinate {
                            lon: 6.128_36,
                            lat: 49.603_966
                        },
                        line: LineAttributes {
                            frc: Frc::Frc3,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(6)
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc5,
                            dnp: Length::from_meters(29.0)
                        })
                    },
                    Point {
                        coordinate: Coordinate {
                            lon: 6.128_15,
                            lat: 49.603_046
                        },
                        line: LineAttributes {
                            frc: Frc::Frc5,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(6)
                        },
                        path: None
                    }
                ],
                offsets: Offsets::default()
            })
        );
    }

    #[test]
    fn openlr_deserialize_coordinate_location_reference_001() {
        let location = deserialize_base64_openlr("I+djotZ9eA==").unwrap();

        assert_eq!(
            location,
            LocationReference::GeoCoordinate(Coordinate {
                lon: -34.608_94,
                lat: -58.373_27
            })
        );
    }

    #[test]
    fn openlr_deserialize_coordinate_location_reference_002() {
        let location = deserialize_base64_openlr("IyVUdwmSoA==").unwrap();

        assert_eq!(
            location,
            LocationReference::GeoCoordinate(Coordinate {
                lon: 52.495_22,
                lat: 13.461_675
            })
        );
    }

    #[test]
    fn openlr_deserialize_point_along_line_location_reference_001() {
        let location = deserialize_base64_openlr("K/6P+SKSuBJGGAUn/1gSUyM=").unwrap();

        assert_eq!(
            location,
            LocationReference::PointAlongLine(PointAlongLine {
                points: [
                    Point {
                        coordinate: Coordinate {
                            lon: -2.0216238,
                            lat: 48.618_44
                        },
                        line: LineAttributes {
                            frc: Frc::Frc2,
                            fow: Fow::MultipleCarriageway,
                            bearing: Bearing::from_degrees(73)
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc2,
                            dnp: Length::from_meters(1436.0)
                        })
                    },
                    Point {
                        coordinate: Coordinate {
                            lon: -2.0084338,
                            lat: 48.616_76
                        },
                        line: LineAttributes {
                            frc: Frc::Frc2,
                            fow: Fow::MultipleCarriageway,
                            bearing: Bearing::from_degrees(219)
                        },
                        path: None
                    }
                ],
                offset: Offset::from_range(0.138_671_88),
                orientation: Orientation::Unknown,
                side: SideOfRoad::OnRoadOrUnknown,
            })
        );
    }

    #[test]
    fn openlr_deserialize_point_along_line_location_reference_002() {
        let location = deserialize_base64_openlr("KwBVwSCh+RRXAf/i/9AUXP8=").unwrap();

        assert_eq!(
            location,
            LocationReference::PointAlongLine(PointAlongLine {
                points: [
                    Point {
                        coordinate: Coordinate {
                            lon: 0.4710495,
                            lat: 45.889_732
                        },
                        line: LineAttributes {
                            frc: Frc::Frc2,
                            fow: Fow::Roundabout,
                            bearing: Bearing::from_degrees(264)
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc2,
                            dnp: Length::from_meters(88.0)
                        })
                    },
                    Point {
                        coordinate: Coordinate {
                            lon: 0.4707495,
                            lat: 45.889_25
                        },
                        line: LineAttributes {
                            frc: Frc::Frc2,
                            fow: Fow::Roundabout,
                            bearing: Bearing::from_degrees(321)
                        },
                        path: None
                    }
                ],
                offset: Offset::from_range(0.9980469),
                orientation: Orientation::Unknown,
                side: SideOfRoad::OnRoadOrUnknown,
            })
        );
    }

    #[test]
    fn openlr_deserialize_poi_location_reference_001() {
        let location = deserialize_base64_openlr("KwOg5iUNnCOTAv+D/5QjQ1j/gP/r").unwrap();

        assert_eq!(
            location,
            LocationReference::Poi(Poi {
                point: PointAlongLine {
                    points: [
                        Point {
                            coordinate: Coordinate {
                                lon: 5.1025807,
                                lat: 52.106
                            },
                            line: LineAttributes {
                                frc: Frc::Frc4,
                                fow: Fow::SingleCarriageway,
                                bearing: Bearing::from_degrees(219)
                            },
                            path: Some(PathAttributes {
                                lfrcnp: Frc::Frc4,
                                dnp: Length::from_meters(147.0)
                            })
                        },
                        Point {
                            coordinate: Coordinate {
                                lon: 5.1013307,
                                lat: 52.104_92
                            },
                            line: LineAttributes {
                                frc: Frc::Frc4,
                                fow: Fow::SingleCarriageway,
                                bearing: Bearing::from_degrees(39)
                            },
                            path: None
                        }
                    ],
                    offset: Offset::from_range(0.34570312),
                    orientation: Orientation::Unknown,
                    side: SideOfRoad::OnRoadOrUnknown,
                },
                coordinate: Coordinate {
                    lon: 5.1013007,
                    lat: 52.105_79
                }
            })
        );
    }

    #[test]
    fn openlr_deserialize_circle_location_reference_001() {
        let location = deserialize_base64_openlr("AwOgxCUNmwEs").unwrap();

        assert_eq!(
            location,
            LocationReference::Circle(Circle {
                center: Coordinate {
                    lon: 5.101_851,
                    lat: 52.105_976
                },
                radius: Length::from_meters(300.0)
            })
        );
    }

    #[test]
    fn openlr_deserialize_circle_location_reference_002() {
        let location = deserialize_base64_openlr("A/2lJCfIiAfQ").unwrap();

        assert_eq!(
            location,
            LocationReference::Circle(Circle {
                center: Coordinate {
                    lon: -3.3115947,
                    lat: 55.945_29
                },
                radius: Length::from_meters(2000.0)
            })
        );
    }

    #[test]
    fn openlr_deserialize_rectangle_location_reference_001() {
        let location = deserialize_base64_openlr("Qxl5HRKFDR33oB/agA==").unwrap();

        assert_eq!(
            location,
            LocationReference::Rectangle(Rectangle {
                lower_left: Coordinate {
                    lon: 35.821_533,
                    lat: 26.043_36
                },
                upper_right: Coordinate {
                    lon: 42.141_483,
                    lat: 44.793_995
                }
            })
        );
    }

    #[test]
    fn openlr_deserialize_rectangle_location_reference_002() {
        let location = deserialize_base64_openlr("QwOgcSUNGgGIAX8=").unwrap();

        assert_eq!(
            location,
            LocationReference::Rectangle(Rectangle {
                lower_left: Coordinate {
                    lon: 5.100_07,
                    lat: 52.103_207
                },
                upper_right: Coordinate {
                    lon: 5.103_99,
                    lat: 52.107_037
                }
            })
        );
    }

    #[test]
    fn openlr_deserialize_grid_location_reference_001() {
        let location = deserialize_base64_openlr("Q/xfwiMc5QsGuyx13wILASg=").unwrap();

        assert_eq!(
            location,
            LocationReference::Grid(Grid {
                rect: Rectangle {
                    lower_left: Coordinate {
                        lon: -5.0989758,
                        lat: 49.377_46
                    },
                    upper_right: Coordinate {
                        lon: 15.505_711,
                        lat: 62.522_476
                    }
                },
                size: GridSize {
                    columns: 523,
                    rows: 296
                }
            })
        );
    }

    #[test]
    fn openlr_deserialize_grid_location_reference_002() {
        let location = deserialize_base64_openlr("QwOgNiUM5wFVANsAAwAC").unwrap();

        assert_eq!(
            location,
            LocationReference::Grid(Grid {
                rect: Rectangle {
                    lower_left: Coordinate {
                        lon: 5.098_804,
                        lat: 52.102_116
                    },
                    upper_right: Coordinate {
                        lon: 5.1022142,
                        lat: 52.104_305
                    }
                },
                size: GridSize {
                    columns: 3,
                    rows: 2
                }
            })
        );
    }

    #[test]
    fn openlr_deserialize_polygon_location_reference_001() {
        let location = deserialize_base64_openlr("EwOgUCUNEwJFAH//yAEv/vIAxw==").unwrap();

        assert_eq!(
            location,
            LocationReference::Polygon(Polygon {
                corners: vec![
                    Coordinate {
                        lon: 5.099_362,
                        lat: 52.103_058
                    },
                    Coordinate {
                        lon: 5.105_172,
                        lat: 52.104_33
                    },
                    Coordinate {
                        lon: 5.104_617,
                        lat: 52.107_353
                    },
                    Coordinate {
                        lon: 5.101_919,
                        lat: 52.109_34
                    }
                ]
            })
        );
    }

    #[test]
    fn openlr_deserialize_closed_line_location_reference_001() {
        let location = deserialize_base64_openlr("WwRboCNGfhJrBAAJ/zkb9AgTFQ==").unwrap();

        assert_eq!(
            location,
            LocationReference::ClosedLine(ClosedLine {
                points: vec![
                    Point {
                        coordinate: Coordinate {
                            lon: 6.128_3,
                            lat: 49.605_965
                        },
                        line: LineAttributes {
                            frc: Frc::Frc2,
                            fow: Fow::MultipleCarriageway,
                            bearing: Bearing::from_degrees(129)
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc3,
                            dnp: Length::from_meters(264.0)
                        })
                    },
                    Point {
                        coordinate: Coordinate {
                            lon: 6.1283904,
                            lat: 49.603_973
                        },
                        line: LineAttributes {
                            frc: Frc::Frc3,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(231)
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc7,
                            dnp: Length::from_meters(498.0)
                        })
                    },
                ],
                last_line: LineAttributes {
                    frc: Frc::Frc2,
                    fow: Fow::SingleCarriageway,
                    bearing: Bearing::from_degrees(242)
                }
            })
        );
    }
}
