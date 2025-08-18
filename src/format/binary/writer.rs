use std::io::{Cursor, Write};

use base64::Engine;
use base64::prelude::BASE64_STANDARD;

use crate::format::binary::encoding::EncodedAttributes;
use crate::model::Offsets;
use crate::{
    Circle, ClosedLine, Coordinate, Grid, GridSize, Length, Line, LocationReference, LocationType,
    Offset, Poi, PointAlongLine, Polygon, Rectangle, SerializeError,
};

/// Serializes an OpenLR Location Reference into Base64.
pub fn serialize_base64_openlr(location: &LocationReference) -> Result<String, SerializeError> {
    let data = serialize_binary_openlr(location)?;
    Ok(BASE64_STANDARD.encode(data))
}

/// Serializes an OpenLR Location Reference into binary.
pub fn serialize_binary_openlr(location: &LocationReference) -> Result<Vec<u8>, SerializeError> {
    use LocationReference::*;

    let mut writer = OpenLrBinaryWriter::default();
    writer.write_header(location.location_type())?;

    match location {
        Line(line) => writer.write_line(line)?,
        GeoCoordinate(coordinate) => writer.write_coordinate(coordinate)?,
        PointAlongLine(point) => writer.write_point_along_line(point)?,
        Poi(poi) => writer.write_poi(poi)?,
        Circle(circle) => writer.write_circle(circle)?,
        Rectangle(rectangle) => writer.write_rectangle(rectangle)?,
        Grid(grid) => writer.write_grid(grid)?,
        Polygon(polygon) => writer.write_polygon(polygon)?,
        ClosedLine(line) => writer.write_closed_line(line)?,
    };

    Ok(writer.cursor.into_inner())
}

#[derive(Debug, Default)]
struct OpenLrBinaryWriter {
    cursor: Cursor<Vec<u8>>,
}

impl OpenLrBinaryWriter {
    fn write_header(&mut self, location_type: LocationType) -> Result<(), SerializeError> {
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

    fn write_line(&mut self, line: &Line) -> Result<(), SerializeError> {
        let Line { points, offsets } = line;
        if points.len() < 2 {
            return Err(SerializeError::InvalidLine);
        }

        let first_point = points.first().ok_or(SerializeError::InvalidLine)?;
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

        let last_point = points.last().ok_or(SerializeError::InvalidLine)?;
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

    fn write_point_along_line(&mut self, point: &PointAlongLine) -> Result<(), SerializeError> {
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

    fn write_poi(&mut self, poi: &Poi) -> Result<(), SerializeError> {
        let Poi { point, poi } = poi;
        self.write_point_along_line(point)?;
        self.write_relative_coordinate(*poi, point.points[0].coordinate)?;
        Ok(())
    }

    fn write_circle(&mut self, circle: &Circle) -> Result<(), SerializeError> {
        let Circle { center, radius } = circle;
        self.write_coordinate(center)?;
        self.write_radius(radius)
    }

    fn write_rectangle(&mut self, rectangle: &Rectangle) -> Result<(), SerializeError> {
        let Rectangle {
            lower_left,
            upper_right,
        } = rectangle;

        if lower_left == upper_right {
            return Err(SerializeError::InvalidRectangle);
        }

        self.write_coordinate(lower_left)?;
        self.write_coordinate(upper_right)
    }

    fn write_grid(&mut self, grid: &Grid) -> Result<(), SerializeError> {
        let Grid { rect, size } = grid;
        self.write_rectangle(rect)?;
        self.write_grid_size(size)
    }

    fn write_polygon(&mut self, polygon: &Polygon) -> Result<(), SerializeError> {
        let Polygon { corners } = polygon;
        if corners.len() < 3 {
            return Err(SerializeError::InvalidPolygon);
        }

        let mut coordinate = *corners.first().ok_or(SerializeError::InvalidPolygon)?;
        self.write_coordinate(&coordinate)?;

        let relative_corners = corners.get(1..).into_iter().flatten();
        for relative_coordinate in relative_corners {
            coordinate = self.write_relative_coordinate(*relative_coordinate, coordinate)?;
        }

        Ok(())
    }

    fn write_closed_line(&mut self, line: &ClosedLine) -> Result<(), SerializeError> {
        let ClosedLine { points, last_line } = line;
        if points.len() < 2 {
            return Err(SerializeError::InvalidLine);
        }

        let first_point = points.first().ok_or(SerializeError::InvalidLine)?;
        let mut coordinate = first_point.coordinate;
        self.write_coordinate(&coordinate)?;

        let path = first_point.path.unwrap_or_default();
        let attributes = EncodedAttributes::from(first_point.line).with_lfrcnp(path.lfrcnp);
        self.write_attributes(attributes)?;
        self.write_dnp(&path.dnp)?;

        let relative_points = points.get(1..).into_iter().flatten();
        for point in relative_points {
            coordinate = self.write_relative_coordinate(point.coordinate, coordinate)?;
            let path = point.path.unwrap_or_default();
            let attributes = EncodedAttributes::from(point.line).with_lfrcnp(path.lfrcnp);
            self.write_attributes(attributes)?;
            self.write_dnp(&path.dnp)?;
        }

        self.write_attributes(EncodedAttributes::from(*last_line))
    }

    fn write_coordinate(&mut self, coordinate: &Coordinate) -> Result<(), SerializeError> {
        let mut write_degrees = |degrees| -> Result<(), SerializeError> {
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
    ) -> Result<Coordinate, SerializeError> {
        let mut write_degrees = |degrees, previous| -> Result<(), SerializeError> {
            let bytes = Coordinate::degrees_into_be_bytes_relative(degrees, previous);
            self.cursor.write_all(&bytes)?;
            Ok(())
        };

        write_degrees(coordinate.lon, previous.lon)?;
        write_degrees(coordinate.lat, previous.lat)?;
        Ok(coordinate)
    }

    fn write_attributes(&mut self, attributes: EncodedAttributes) -> Result<(), SerializeError> {
        let fow = attributes.line.fow.into_byte();
        let frc = attributes.line.frc.into_byte();
        let bearing = attributes.line.bearing.try_into_byte()?;

        let first_byte = fow + (frc << 3) + (attributes.orientation_or_side << 6);
        let second_byte = bearing + (attributes.lfrcnp_or_flags << 5);
        self.cursor.write_all(&[first_byte, second_byte])?;
        Ok(())
    }

    fn write_dnp(&mut self, dnp: &Length) -> Result<(), SerializeError> {
        let dnp = dnp.dnp_into_byte();
        self.cursor.write_all(&[dnp])?;
        Ok(())
    }

    fn write_radius(&mut self, radius: &Length) -> Result<(), SerializeError> {
        let radius = radius.radius_into_be_bytes();
        self.cursor.write_all(&radius)?;
        Ok(())
    }

    fn write_offset(&mut self, offset: Offset) -> Result<(), SerializeError> {
        let offset = offset.try_into_byte()?;
        self.cursor.write_all(&[offset])?;
        Ok(())
    }

    fn write_grid_size(&mut self, size: &GridSize) -> Result<(), SerializeError> {
        let size = size.try_into_be_bytes()?;
        self.cursor.write_all(&size)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::model::Offsets;
    use crate::{
        Bearing, Fow, Frc, LineAttributes, Orientation, PathAttributes, Point, SideOfRoad,
        deserialize_base64_openlr,
    };

    #[test]
    fn openlr_serialize_line_location_reference_001() {
        assert_serde_eq(LocationReference::Line(Line {
            points: vec![
                Point {
                    coordinate: Coordinate {
                        lon: 6.1268198,
                        lat: 49.6085178,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc3,
                        fow: Fow::MultipleCarriageway,
                        bearing: Bearing::from_degrees(141),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc3,
                        dnp: Length::from_meters(557.0),
                    }),
                },
                Point {
                    coordinate: Coordinate {
                        lon: 6.1283698,
                        lat: 49.6039878,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc3,
                        fow: Fow::SingleCarriageway,
                        bearing: Bearing::from_degrees(231),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc5,
                        dnp: Length::from_meters(264.0),
                    }),
                },
                Point {
                    coordinate: Coordinate {
                        lon: 6.1281598,
                        lat: 49.6030578,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc5,
                        fow: Fow::SingleCarriageway,
                        bearing: Bearing::from_degrees(287),
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
    fn openlr_serialize_line_location_reference_002() {
        assert_serde_eq(LocationReference::Line(Line {
            points: vec![
                Point {
                    coordinate: Coordinate {
                        lon: 0.6752192,
                        lat: 47.3651611,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc3,
                        fow: Fow::Roundabout,
                        bearing: Bearing::from_degrees(28),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc3,
                        dnp: Length::from_meters(498.0),
                    }),
                },
                Point {
                    coordinate: Coordinate {
                        lon: 0.6769992,
                        lat: 47.3696011,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc3,
                        fow: Fow::MultipleCarriageway,
                        bearing: Bearing::from_degrees(197),
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
    fn openlr_serialize_line_location_reference_003() {
        assert_serde_eq(LocationReference::Line(Line {
            points: vec![
                Point {
                    coordinate: Coordinate {
                        lon: 9.9750602,
                        lat: 48.0632865,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc1,
                        fow: Fow::SingleCarriageway,
                        bearing: Bearing::from_degrees(298),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc1,
                        dnp: Length::from_meters(88.0),
                    }),
                },
                Point {
                    coordinate: Coordinate {
                        lon: 9.9750602,
                        lat: 48.0632865,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc1,
                        fow: Fow::SingleCarriageway,
                        bearing: Bearing::from_degrees(298),
                    },
                    path: None,
                },
            ],
            offsets: Offsets::default(),
        }));
    }

    #[test]
    fn openlr_serialize_line_location_reference_004() {
        assert_serde_eq(LocationReference::Line(Line {
            points: vec![
                Point {
                    coordinate: Coordinate {
                        lon: 6.1268198,
                        lat: 49.6084964,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc3,
                        fow: Fow::MultipleCarriageway,
                        bearing: Bearing::from_degrees(6),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc3,
                        dnp: Length::from_meters(29.0),
                    }),
                },
                Point {
                    coordinate: Coordinate {
                        lon: 6.1283598,
                        lat: 49.6039664,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc3,
                        fow: Fow::SingleCarriageway,
                        bearing: Bearing::from_degrees(6),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc5,
                        dnp: Length::from_meters(29.0),
                    }),
                },
                Point {
                    coordinate: Coordinate {
                        lon: 6.1281498,
                        lat: 49.6030464,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc5,
                        fow: Fow::SingleCarriageway,
                        bearing: Bearing::from_degrees(6),
                    },
                    path: None,
                },
            ],
            offsets: Offsets::default(),
        }));
    }

    #[test]
    fn openlr_serialize_line_location_reference_005() {
        assert_serde_eq(LocationReference::Line(Line {
            points: vec![
                Point {
                    coordinate: Coordinate {
                        lon: 0.0,
                        lat: 0.00001,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc1,
                        fow: Fow::SingleCarriageway,
                        bearing: Bearing::from_degrees(298),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc1,
                        dnp: Length::from_meters(88.0),
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
                        bearing: Bearing::from_degrees(298),
                    },
                    path: None,
                },
            ],
            offsets: Offsets::default(),
        }));
    }

    #[test]
    fn openlr_serialize_coordinate_location_reference_001() {
        assert_serde_eq(LocationReference::GeoCoordinate(Coordinate {
            lon: -34.6089398,
            lat: -58.3732688,
        }));
    }

    #[test]
    fn openlr_serialize_coordinate_location_reference_002() {
        assert_serde_eq(LocationReference::GeoCoordinate(Coordinate {
            lon: 52.4952185,
            lat: 13.4616744,
        }));
    }

    #[test]
    fn openlr_serialize_coordinate_location_reference_003() {
        assert_serde_eq(LocationReference::GeoCoordinate(Coordinate {
            lon: 0.0,
            lat: 0.0,
        }));
    }

    #[test]
    fn openlr_serialize_coordinate_location_reference_004() {
        assert_serde_eq(LocationReference::GeoCoordinate(Coordinate {
            lon: 52.49522,
            lat: -13.461675,
        }));
    }

    #[test]
    fn openlr_serialize_coordinate_location_reference_005() {
        assert_serde_eq(LocationReference::GeoCoordinate(Coordinate {
            lon: -52.49522,
            lat: 13.461675,
        }));
    }

    #[test]
    fn openlr_serialize_point_along_line_location_reference_001() {
        assert_serde_eq(LocationReference::PointAlongLine(PointAlongLine {
            points: [
                Point {
                    coordinate: Coordinate {
                        lon: -2.0216238,
                        lat: 48.6184394,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc2,
                        fow: Fow::MultipleCarriageway,
                        bearing: Bearing::from_degrees(73),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc2,
                        dnp: Length::from_meters(1436.0),
                    }),
                },
                Point {
                    coordinate: Coordinate {
                        lon: -2.0084338,
                        lat: 48.6167594,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc2,
                        fow: Fow::MultipleCarriageway,
                        bearing: Bearing::from_degrees(219),
                    },
                    path: None,
                },
            ],
            offset: Offset::from_range(0.138671875),
            orientation: Orientation::Forward,
            side: SideOfRoad::Both,
        }));
    }

    #[test]
    fn openlr_serialize_point_along_line_location_reference_002() {
        assert_serde_eq(LocationReference::PointAlongLine(PointAlongLine {
            points: [
                Point {
                    coordinate: Coordinate {
                        lon: 0.4710495,
                        lat: 45.8897316,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc2,
                        fow: Fow::Roundabout,
                        bearing: Bearing::from_degrees(264),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc4,
                        dnp: Length::from_meters(88.0),
                    }),
                },
                Point {
                    coordinate: Coordinate {
                        lon: 0.4707495,
                        lat: 45.8892516,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc2,
                        fow: Fow::Roundabout,
                        bearing: Bearing::from_degrees(321),
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
    fn openlr_serialize_poi_location_reference_001() {
        assert_serde_eq(LocationReference::Poi(Poi {
            point: PointAlongLine {
                points: [
                    Point {
                        coordinate: Coordinate {
                            lon: 5.1025807,
                            lat: 52.1059978,
                        },
                        line: LineAttributes {
                            frc: Frc::Frc4,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(219),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc4,
                            dnp: Length::from_meters(147.0),
                        }),
                    },
                    Point {
                        coordinate: Coordinate {
                            lon: 5.1013307,
                            lat: 52.1049178,
                        },
                        line: LineAttributes {
                            frc: Frc::Frc4,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(39),
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
                lat: 52.1057878,
            },
        }));
    }

    #[test]
    fn openlr_serialize_circle_location_reference_001() {
        assert_serde_eq(LocationReference::Circle(Circle {
            center: Coordinate {
                lon: 5.1018512,
                lat: 52.1059763,
            },
            radius: Length::from_meters(300.0),
        }));
    }

    #[test]
    fn openlr_serialize_circle_location_reference_002() {
        assert_serde_eq(LocationReference::Circle(Circle {
            center: Coordinate {
                lon: -3.3115947,
                lat: 55.9452903,
            },
            radius: Length::from_meters(2000.0),
        }));
    }

    #[test]
    fn openlr_serialize_rectangle_location_reference_001() {
        assert_serde_eq(LocationReference::Rectangle(Rectangle {
            lower_left: Coordinate {
                lon: 35.8215343,
                lat: 26.0433590,
            },
            upper_right: Coordinate {
                lon: 42.1414840,
                lat: 44.7939956,
            },
        }));
    }

    #[test]
    fn openlr_serialize_rectangle_location_reference_002() {
        assert_serde_eq(LocationReference::Rectangle(Rectangle {
            lower_left: Coordinate {
                lon: 5.1000702,
                lat: 52.1032083,
            },
            upper_right: Coordinate {
                lon: 5.1039902,
                lat: 52.1070383,
            },
        }));
    }

    #[test]
    fn openlr_serialize_grid_location_reference_001() {
        assert_serde_eq(LocationReference::Grid(Grid {
            rect: Rectangle {
                lower_left: Coordinate {
                    lon: -5.0989758,
                    lat: 49.3774616,
                },
                upper_right: Coordinate {
                    lon: 15.5057108,
                    lat: 62.5224745,
                },
            },
            size: GridSize {
                columns: 523,
                rows: 296,
            },
        }));
    }

    #[test]
    fn openlr_serialize_grid_location_reference_002() {
        assert_serde_eq(LocationReference::Grid(Grid {
            rect: Rectangle {
                lower_left: Coordinate {
                    lon: 5.0988042,
                    lat: 52.1021139,
                },
                upper_right: Coordinate {
                    lon: 5.1022142,
                    lat: 52.1043039,
                },
            },
            size: GridSize {
                columns: 3,
                rows: 2,
            },
        }));
    }

    #[test]
    fn openlr_serialize_polygon_location_reference_001() {
        assert_serde_eq(LocationReference::Polygon(Polygon {
            corners: vec![
                Coordinate {
                    lon: 5.0993621,
                    lat: 52.1030580,
                },
                Coordinate {
                    lon: 5.1051721,
                    lat: 52.1043280,
                },
                Coordinate {
                    lon: 5.1046171,
                    lat: 52.1073541,
                },
                Coordinate {
                    lon: 5.1019192,
                    lat: 52.1093396,
                },
            ],
        }));
    }

    #[test]
    fn openlr_serialize_closed_line_location_reference_001() {
        assert_serde_eq(LocationReference::ClosedLine(ClosedLine {
            points: vec![
                Point {
                    coordinate: Coordinate {
                        lon: 6.1283004,
                        lat: 49.6059644,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc2,
                        fow: Fow::MultipleCarriageway,
                        bearing: Bearing::from_degrees(129),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc3,
                        dnp: Length::from_meters(264.0),
                    }),
                },
                Point {
                    coordinate: Coordinate {
                        lon: 6.1283904,
                        lat: 49.6039744,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc3,
                        fow: Fow::SingleCarriageway,
                        bearing: Bearing::from_degrees(231),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc7,
                        dnp: Length::from_meters(498.0),
                    }),
                },
            ],
            last_line: LineAttributes {
                frc: Frc::Frc2,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(242),
            },
        }));
    }

    fn assert_serde_eq(location: LocationReference) {
        let encoded = serialize_base64_openlr(&location).unwrap();
        let decoded_location = deserialize_base64_openlr(&encoded).unwrap();
        assert_eq!(location, decoded_location);
    }
}
