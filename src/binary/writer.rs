use std::io::{Cursor, Write};

use base64::Engine;
use base64::prelude::BASE64_STANDARD;

use crate::binary::encoding::EncodedAttributes;
use crate::model::Offsets;
use crate::{
    Circle, Coordinate, EncodeError, Length, Line, LocationReference, LocationType, Offset, Poi,
    PointAlongLine,
};

/// Encodes an OpenLR Location Reference into Base64.
pub fn encode_base64_openlr(location: &LocationReference) -> Result<String, EncodeError> {
    let data = encode_binary_openlr(location)?;
    Ok(BASE64_STANDARD.encode(data))
}

/// Encodes an OpenLR Location Reference into binary.
pub fn encode_binary_openlr(location: &LocationReference) -> Result<Vec<u8>, EncodeError> {
    use LocationReference::*;

    let mut writer = OpenLrBinaryWriter::default();
    writer.write_header(location.location_type())?;

    match location {
        Line(line) => writer.write_line(line)?,
        GeoCoordinate(coordinate) => writer.write_coordinate(coordinate)?,
        PointAlongLine(point) => writer.write_point_along_line(point)?,
        Poi(poi) => writer.write_poi(poi)?,
        Circle(circle) => writer.write_circle(circle)?,
        Rectangle(_) => unimplemented!(),
        Grid(_) => unimplemented!(),
        Polygon(_) => unimplemented!(),
        ClosedLine(_) => unimplemented!(),
    };

    Ok(writer.cursor.into_inner())
}

#[derive(Debug, Default)]
struct OpenLrBinaryWriter {
    cursor: Cursor<Vec<u8>>,
}

impl OpenLrBinaryWriter {
    fn write_header(&mut self, location_type: LocationType) -> Result<(), EncodeError> {
        const VERSION: u8 = 3;

        let location_type = match location_type {
            LocationType::Circle => 0,
            LocationType::Line => 1,
            LocationType::Polygon => 2,
            LocationType::GeoCoordinate => 4,
            LocationType::PoiWithAccessPoint | LocationType::PointAlongLine => 5,
            LocationType::Grid | LocationType::Rectangle => 8,
            LocationType::ClosedLine => 11,
        };

        let header = VERSION + (location_type << 3);
        self.cursor.write_all(&[header])?;
        Ok(())
    }

    fn write_line(&mut self, line: &Line) -> Result<(), EncodeError> {
        let Line { points, offsets } = line;

        let first_point = points.first().ok_or(EncodeError::InvalidLine)?;
        let mut coordinate = first_point.coordinate;
        self.write_coordinate(&coordinate)?;

        let path = first_point.path.unwrap_or_default();
        let attributes = EncodedAttributes::from(first_point.line).with_lfrcnp(path.lfrcnp);
        self.write_attributes(attributes)?;
        self.write_dnp(&path.dnp)?;

        let relative_points = points.get(1..points.len() - 1).into_iter().flatten();
        for point in relative_points {
            coordinate = self.write_relative_coordinate(point.coordinate, coordinate)?;
            let path = point.path.unwrap_or_default();
            let attributes = EncodedAttributes::from(point.line).with_lfrcnp(path.lfrcnp);
            self.write_attributes(attributes)?;
            self.write_dnp(&path.dnp)?;
        }

        let last_point = points.last().ok_or(EncodeError::InvalidLine)?;
        self.write_relative_coordinate(last_point.coordinate, coordinate)?;
        let attributes = EncodedAttributes::from(last_point.line).with_offsets(offsets);
        self.write_attributes(attributes)?;

        if attributes.pos_offset_flag() {
            self.write_offset(offsets.pos)?;
        }
        if attributes.neg_offset_flag() {
            self.write_offset(offsets.neg)?;
        }

        Ok(())
    }

    fn write_point_along_line(&mut self, point: &PointAlongLine) -> Result<(), EncodeError> {
        let PointAlongLine {
            points: [first_point, last_point],
            offset,
            orientation,
            side,
        } = point;

        self.write_coordinate(&first_point.coordinate)?;
        let path = first_point.path.unwrap_or_default();
        let attributes = EncodedAttributes::from(first_point.line)
            .with_lfrcnp(path.lfrcnp)
            .with_orientation(orientation);
        self.write_attributes(attributes)?;
        self.write_dnp(&path.dnp)?;

        self.write_relative_coordinate(last_point.coordinate, first_point.coordinate)?;
        let attributes = EncodedAttributes::from(last_point.line)
            .with_offsets(&Offsets::positive(*offset))
            .with_side(side);
        self.write_attributes(attributes)?;

        if attributes.pos_offset_flag() {
            self.write_offset(*offset)?;
        }

        Ok(())
    }

    fn write_poi(&mut self, poi: &Poi) -> Result<(), EncodeError> {
        let Poi { point, poi } = poi;
        self.write_point_along_line(point)?;
        self.write_relative_coordinate(*poi, point.points[0].coordinate)?;
        Ok(())
    }

    fn write_circle(&mut self, circle: &Circle) -> Result<(), EncodeError> {
        let Circle { center, radius } = circle;
        self.write_coordinate(center)?;
        self.write_radius(radius)?;
        Ok(())
    }

    fn write_coordinate(&mut self, coordinate: &Coordinate) -> Result<(), EncodeError> {
        let mut write_degrees = |degrees| -> Result<(), EncodeError> {
            let bytes = Coordinate::degrees_into_be_bytes(degrees);
            self.cursor.write_all(&bytes)?;
            Ok(())
        };

        write_degrees(coordinate.lon)?;
        write_degrees(coordinate.lat)
    }

    fn write_relative_coordinate(
        &mut self,
        coordinate: Coordinate,
        previous: Coordinate,
    ) -> Result<Coordinate, EncodeError> {
        let mut write_degrees = |degrees, previous| -> Result<(), EncodeError> {
            let bytes = Coordinate::degrees_into_be_bytes_relative(degrees, previous);
            self.cursor.write_all(&bytes)?;
            Ok(())
        };

        write_degrees(coordinate.lon, previous.lon)?;
        write_degrees(coordinate.lat, previous.lat)?;
        Ok(coordinate)
    }

    fn write_attributes(&mut self, attributes: EncodedAttributes) -> Result<(), EncodeError> {
        let fow = attributes.line.fow.into_byte();
        let frc = attributes.line.frc.into_byte();
        let bear = attributes.line.bear.try_into_byte()?;

        let first_byte = fow + (frc << 3) + (attributes.orientation_or_side << 6);
        let second_byte = bear + (attributes.lfrcnp_or_flags << 5);
        self.cursor.write_all(&[first_byte, second_byte])?;

        Ok(())
    }

    fn write_dnp(&mut self, dnp: &Length) -> Result<(), EncodeError> {
        let dnp = dnp.dnp_into_byte();
        self.cursor.write_all(&[dnp])?;
        Ok(())
    }

    fn write_radius(&mut self, radius: &Length) -> Result<(), EncodeError> {
        let radius = radius.radius_into_be_bytes();
        self.cursor.write_all(&radius)?;
        Ok(())
    }

    fn write_offset(&mut self, offset: Offset) -> Result<(), EncodeError> {
        let offset = offset.try_into_byte()?;
        self.cursor.write_all(&[offset])?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Offsets;
    use crate::{
        Bearing, Fow, Frc, LineAttributes, Orientation, PathAttributes, Point, SideOfRoad,
        decode_base64_openlr,
    };

    #[test]
    fn openlr_encode_line_location_reference_001() {
        assert_encoding_eq_decoding(LocationReference::Line(Line {
            points: vec![
                Point {
                    coordinate: Coordinate {
                        lon: 6.1268198,
                        lat: 49.608_517,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc3,
                        fow: Fow::MultipleCarriageway,
                        bear: Bearing::from_degrees(141),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc3,
                        dnp: Length::from_meters(557),
                    }),
                },
                Point {
                    coordinate: Coordinate {
                        lon: 6.128_37,
                        lat: 49.603_99,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc3,
                        fow: Fow::SingleCarriageway,
                        bear: Bearing::from_degrees(231),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc5,
                        dnp: Length::from_meters(264),
                    }),
                },
                Point {
                    coordinate: Coordinate {
                        lon: 6.128_16,
                        lat: 49.603_058,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc5,
                        fow: Fow::SingleCarriageway,
                        bear: Bearing::from_degrees(287),
                    },
                    path: None,
                },
            ],
            offsets: Offsets {
                pos: Offset::from_range(0.26757812),
                neg: Offset::default(),
            },
        }));
    }

    #[test]
    fn openlr_encode_line_location_reference_002() {
        assert_encoding_eq_decoding(LocationReference::Line(Line {
            points: vec![
                Point {
                    coordinate: Coordinate {
                        lon: -0.6752192,
                        lat: -47.365_16,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc3,
                        fow: Fow::Roundabout,
                        bear: Bearing::from_degrees(28),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc3,
                        dnp: Length::from_meters(498),
                    }),
                },
                Point {
                    coordinate: Coordinate {
                        lon: -0.6769992,
                        lat: -47.369_602,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc3,
                        fow: Fow::MultipleCarriageway,
                        bear: Bearing::from_degrees(197),
                    },
                    path: None,
                },
            ],
            offsets: Offsets {
                pos: Offset::default(),
                neg: Offset::from_range(0.45898438),
            },
        }));
    }

    #[test]
    fn openlr_encode_line_location_reference_003() {
        assert_encoding_eq_decoding(LocationReference::Line(Line {
            points: vec![
                Point {
                    coordinate: Coordinate {
                        lon: 9.975_06,
                        lat: 48.063_286,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc1,
                        fow: Fow::SingleCarriageway,
                        bear: Bearing::from_degrees(298),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc1,
                        dnp: Length::from_meters(88),
                    }),
                },
                Point {
                    coordinate: Coordinate {
                        lon: 9.975_06,
                        lat: 48.063_286,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc1,
                        fow: Fow::SingleCarriageway,
                        bear: Bearing::from_degrees(298),
                    },
                    path: None,
                },
            ],
            offsets: Offsets::default(),
        }));
    }

    #[test]
    fn openlr_encode_line_location_reference_004() {
        assert_encoding_eq_decoding(LocationReference::Line(Line {
            points: vec![
                Point {
                    coordinate: Coordinate {
                        lon: 6.1268198,
                        lat: 49.608_498,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc3,
                        fow: Fow::MultipleCarriageway,
                        bear: Bearing::from_degrees(6),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc3,
                        dnp: Length::from_meters(29),
                    }),
                },
                Point {
                    coordinate: Coordinate {
                        lon: 6.128_36,
                        lat: 49.603_966,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc3,
                        fow: Fow::SingleCarriageway,
                        bear: Bearing::from_degrees(6),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc5,
                        dnp: Length::from_meters(29),
                    }),
                },
                Point {
                    coordinate: Coordinate {
                        lon: 6.128_15,
                        lat: 49.603_046,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc5,
                        fow: Fow::SingleCarriageway,
                        bear: Bearing::from_degrees(6),
                    },
                    path: None,
                },
            ],
            offsets: Offsets::default(),
        }));
    }

    #[test]
    fn openlr_encode_line_location_reference_005() {
        assert_encoding_eq_decoding(LocationReference::Line(Line {
            points: vec![
                Point {
                    coordinate: Coordinate {
                        lon: 0.0,
                        lat: 0.00001,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc1,
                        fow: Fow::SingleCarriageway,
                        bear: Bearing::from_degrees(298),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc1,
                        dnp: Length::from_meters(88),
                    }),
                },
                Point {
                    coordinate: Coordinate {
                        lon: -0.00001,
                        lat: -0.00002,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc1,
                        fow: Fow::SingleCarriageway,
                        bear: Bearing::from_degrees(298),
                    },
                    path: None,
                },
            ],
            offsets: Offsets::default(),
        }));
    }

    #[test]
    fn openlr_encode_coordinate_location_reference_001() {
        assert_encoding_eq_decoding(LocationReference::GeoCoordinate(Coordinate {
            lon: -34.608_94,
            lat: -58.373_27,
        }));
    }

    #[test]
    fn openlr_encode_coordinate_location_reference_002() {
        assert_encoding_eq_decoding(LocationReference::GeoCoordinate(Coordinate {
            lon: 52.495_22,
            lat: 13.461_675,
        }));
    }

    #[test]
    fn openlr_encode_coordinate_location_reference_003() {
        assert_encoding_eq_decoding(LocationReference::GeoCoordinate(Coordinate {
            lon: 0.0,
            lat: 0.0,
        }));
    }

    #[test]
    fn openlr_encode_coordinate_location_reference_004() {
        assert_encoding_eq_decoding(LocationReference::GeoCoordinate(Coordinate {
            lon: 52.495_22,
            lat: -13.461_675,
        }));
    }

    #[test]
    fn openlr_encode_coordinate_location_reference_005() {
        assert_encoding_eq_decoding(LocationReference::GeoCoordinate(Coordinate {
            lon: -52.495_22,
            lat: 13.461_675,
        }));
    }

    #[test]
    fn openlr_encode_point_along_line_location_reference_001() {
        assert_encoding_eq_decoding(LocationReference::PointAlongLine(PointAlongLine {
            points: [
                Point {
                    coordinate: Coordinate {
                        lon: -2.0216238,
                        lat: 48.618_44,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc2,
                        fow: Fow::MultipleCarriageway,
                        bear: Bearing::from_degrees(73),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc2,
                        dnp: Length::from_meters(1436),
                    }),
                },
                Point {
                    coordinate: Coordinate {
                        lon: -2.0084338,
                        lat: 48.616_76,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc2,
                        fow: Fow::MultipleCarriageway,
                        bear: Bearing::from_degrees(219),
                    },
                    path: None,
                },
            ],
            offset: Offset::from_range(0.138_671_88),
            orientation: Orientation::Forward,
            side: SideOfRoad::Both,
        }));
    }

    #[test]
    fn openlr_encode_point_along_line_location_reference_002() {
        assert_encoding_eq_decoding(LocationReference::PointAlongLine(PointAlongLine {
            points: [
                Point {
                    coordinate: Coordinate {
                        lon: 0.4710495,
                        lat: 45.889_732,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc2,
                        fow: Fow::Roundabout,
                        bear: Bearing::from_degrees(264),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc4,
                        dnp: Length::from_meters(88),
                    }),
                },
                Point {
                    coordinate: Coordinate {
                        lon: 0.4707495,
                        lat: 45.889_25,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc2,
                        fow: Fow::Roundabout,
                        bear: Bearing::from_degrees(321),
                    },
                    path: None,
                },
            ],
            offset: Offset::from_range(0.9980469),
            orientation: Orientation::Backward,
            side: SideOfRoad::Left,
        }));
    }

    #[test]
    fn openlr_encode_poi_location_reference_001() {
        assert_encoding_eq_decoding(LocationReference::Poi(Poi {
            point: PointAlongLine {
                points: [
                    Point {
                        coordinate: Coordinate {
                            lon: 5.1025807,
                            lat: 52.106,
                        },
                        line: LineAttributes {
                            frc: Frc::Frc4,
                            fow: Fow::SingleCarriageway,
                            bear: Bearing::from_degrees(219),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc4,
                            dnp: Length::from_meters(147),
                        }),
                    },
                    Point {
                        coordinate: Coordinate {
                            lon: 5.1013307,
                            lat: 52.104_92,
                        },
                        line: LineAttributes {
                            frc: Frc::Frc4,
                            fow: Fow::SingleCarriageway,
                            bear: Bearing::from_degrees(39),
                        },
                        path: None,
                    },
                ],
                offset: Offset::from_range(0.34570312),
                orientation: Orientation::Unknown,
                side: SideOfRoad::OnRoadOrUnknown,
            },
            poi: Coordinate {
                lon: 5.1013007,
                lat: 52.105_79,
            },
        }));
    }

    #[test]
    fn openlr_encode_circle_location_reference_001() {
        assert_encoding_eq_decoding(LocationReference::Circle(Circle {
            center: Coordinate {
                lon: 5.101_851,
                lat: 52.105_976,
            },
            radius: Length::from_meters(300),
        }));
    }

    #[test]
    fn openlr_encode_circle_location_reference_002() {
        assert_encoding_eq_decoding(LocationReference::Circle(Circle {
            center: Coordinate {
                lon: -3.3115947,
                lat: 55.945_29,
            },
            radius: Length::from_meters(2000),
        }));
    }

    fn assert_encoding_eq_decoding(location: LocationReference) {
        let encoded = encode_base64_openlr(&location).unwrap();
        let decoded_location = decode_base64_openlr(&encoded).unwrap();
        assert_eq!(location, decoded_location);
    }
}
