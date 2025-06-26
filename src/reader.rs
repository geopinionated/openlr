use std::io::{Cursor, Read};

use base64::Engine;
use base64::prelude::BASE64_STANDARD;

use crate::{
    Bearing, Coordinate, Fow, Frc, Length, LineAttributes, LineLocationReference,
    LocationReference, LocationReferencePoint, LocationType, Offset, OpenLrError, PathAttributes,
};

pub fn decode_base64_openlr(data: impl AsRef<[u8]>) -> Result<LocationReference, OpenLrError> {
    let data = BASE64_STANDARD.decode(data)?;
    decode_binary_openlr(&data)
}

pub fn decode_binary_openlr(data: &[u8]) -> Result<LocationReference, OpenLrError> {
    let mut reader = OpenLrBinaryReader::new(data);
    match reader.read_header()? {
        LocationType::Line => Ok(LocationReference::Line(reader.read_line()?)),
        _ => unimplemented!(),
    }
}

#[derive(Debug)]
struct OpenLrBinaryReader<'a> {
    cursor: Cursor<&'a [u8]>,
}

impl<'a> OpenLrBinaryReader<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self {
            cursor: Cursor::new(data),
        }
    }

    fn len(&self) -> usize {
        self.cursor.get_ref().len()
    }

    fn read_header(&mut self) -> Result<LocationType, OpenLrError> {
        let mut header = [0u8; 1];
        self.cursor.read_exact(&mut header)?;

        let version = header[0] & 0b111;
        if version != 3 {
            return Err(OpenLrError::VersionNotSupported);
        }

        let location_type = (header[0] >> 3) & 0b1111;
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
            _ => return Err(OpenLrError::BinaryParseError),
        };

        Ok(location_type)
    }

    fn read_line(&mut self) -> Result<LineLocationReference, OpenLrError> {
        let relative_points_count = (self.len() - 9) / 7;
        let mut line = LineLocationReference::with_capacity(1 + relative_points_count);

        let mut coordinate = self.read_coordinate()?;
        let (mut attributes, mut lfrcnp_or_flags) = self.read_attributes()?;

        for _ in 0..relative_points_count {
            let dnp = self.read_dnp()?;

            line.points.push(LocationReferencePoint {
                coordinate,
                line: attributes,
                path: Some(PathAttributes {
                    lfrcnp: Frc::try_from_byte(lfrcnp_or_flags)?,
                    dnp,
                }),
            });

            coordinate = self.read_relative_coordinate(coordinate)?;
            (attributes, lfrcnp_or_flags) = self.read_attributes()?;
        }

        line.points.push(LocationReferencePoint {
            coordinate,
            line: attributes,
            path: None,
        });

        let mut read_offset = |flag| {
            if flag != 0 {
                self.read_offset()
            } else {
                Ok(Offset::default())
            }
        };

        line.offsets.pos = read_offset(lfrcnp_or_flags & 0b10)?;
        line.offsets.neg = read_offset(lfrcnp_or_flags & 0b01)?;

        Ok(line)
    }

    fn read_coordinate(&mut self) -> Result<Coordinate, OpenLrError> {
        let mut parse_coordinate = || -> Result<f32, OpenLrError> {
            let mut c = [0u8; 3];
            self.cursor.read_exact(&mut c)?;
            Ok(Coordinate::degrees_from_be_bytes(c))
        };

        let lon = parse_coordinate()?;
        let lat = parse_coordinate()?;
        Ok(Coordinate { lon, lat })
    }

    fn read_relative_coordinate(
        &mut self,
        previous: Coordinate,
    ) -> Result<Coordinate, OpenLrError> {
        let mut parse_coordinate = |previous| -> Result<f32, OpenLrError> {
            let mut c = [0u8; 2];
            self.cursor.read_exact(&mut c)?;
            Ok(Coordinate::degrees_from_be_bytes_relative(c, previous))
        };

        let lon = parse_coordinate(previous.lon)?;
        let lat = parse_coordinate(previous.lat)?;
        Ok(Coordinate { lon, lat })
    }

    fn read_attributes(&mut self) -> Result<(LineAttributes, u8), OpenLrError> {
        let mut attributes = [0u8; 2];
        self.cursor.read_exact(&mut attributes)?;

        let fow = Fow::try_from_byte(attributes[0] & 0b111)?;
        let frc = Frc::try_from_byte((attributes[0] >> 3) & 0b111)?;
        let bear = Bearing::from_byte(attributes[1] & 0b11111);
        let lfrcnp_or_flags = (attributes[1] >> 5) & 0b111;

        Ok((LineAttributes { frc, fow, bear }, lfrcnp_or_flags))
    }

    fn read_dnp(&mut self) -> Result<Length, OpenLrError> {
        let mut dnp = [0u8; 1];
        self.cursor.read_exact(&mut dnp)?;
        Ok(Length::dnp_from_byte(dnp[0]))
    }

    fn read_offset(&mut self) -> Result<Offset, OpenLrError> {
        let mut offset = [0u8; 1];
        self.cursor.read_exact(&mut offset)?;
        Ok(Offset::from_byte(offset[0]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Offsets;

    #[test]
    fn openlr_line_location_reference_001() {
        let location = decode_base64_openlr("CwRbWyNG9RpsCQCb/jsbtAT/6/+jK1lE").unwrap();

        assert_eq!(
            location,
            LocationReference::Line(LineLocationReference {
                points: vec![
                    LocationReferencePoint {
                        coordinate: Coordinate {
                            lon: 6.1268198,
                            lat: 49.608_517
                        },
                        line: LineAttributes {
                            frc: Frc::Frc3,
                            fow: Fow::MultipleCarriageway,
                            bear: Bearing::from_degrees(141)
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc3,
                            dnp: Length::from_meters(557)
                        })
                    },
                    LocationReferencePoint {
                        coordinate: Coordinate {
                            lon: 6.128_37,
                            lat: 49.603_99
                        },
                        line: LineAttributes {
                            frc: Frc::Frc3,
                            fow: Fow::SingleCarriageway,
                            bear: Bearing::from_degrees(231)
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc5,
                            dnp: Length::from_meters(264)
                        })
                    },
                    LocationReferencePoint {
                        coordinate: Coordinate {
                            lon: 6.128_16,
                            lat: 49.603_058
                        },
                        line: LineAttributes {
                            frc: Frc::Frc5,
                            fow: Fow::SingleCarriageway,
                            bear: Bearing::from_degrees(287)
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
    fn openlr_line_location_reference_002() {
        let location = decode_base64_openlr("CwB67CGukRxiCACyAbwaMXU=").unwrap();

        assert_eq!(
            location,
            LocationReference::Line(LineLocationReference {
                points: vec![
                    LocationReferencePoint {
                        coordinate: Coordinate {
                            lon: 0.6752192,
                            lat: 47.365_16
                        },
                        line: LineAttributes {
                            frc: Frc::Frc3,
                            fow: Fow::Roundabout,
                            bear: Bearing::from_degrees(28)
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc3,
                            dnp: Length::from_meters(498)
                        })
                    },
                    LocationReferencePoint {
                        coordinate: Coordinate {
                            lon: 0.6769992,
                            lat: 47.369_602
                        },
                        line: LineAttributes {
                            frc: Frc::Frc3,
                            fow: Fow::MultipleCarriageway,
                            bear: Bearing::from_degrees(197)
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
    fn openlr_line_location_reference_003() {
        let location = decode_base64_openlr("CwcX6CItqAs6AQAAAAALGg==").unwrap();

        assert_eq!(
            location,
            LocationReference::Line(LineLocationReference {
                points: vec![
                    LocationReferencePoint {
                        coordinate: Coordinate {
                            lon: 9.975_06,
                            lat: 48.063_286
                        },
                        line: LineAttributes {
                            frc: Frc::Frc1,
                            fow: Fow::SingleCarriageway,
                            bear: Bearing::from_degrees(298)
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc1,
                            dnp: Length::from_meters(88)
                        })
                    },
                    LocationReferencePoint {
                        coordinate: Coordinate {
                            lon: 9.975_06,
                            lat: 48.063_286
                        },
                        line: LineAttributes {
                            frc: Frc::Frc1,
                            fow: Fow::SingleCarriageway,
                            bear: Bearing::from_degrees(298)
                        },
                        path: None
                    },
                ],
                offsets: Offsets::default()
            })
        );
    }

    #[test]
    fn openlr_line_location_reference_004() {
        let location = decode_base64_openlr("CwRbWyNG9BpgAACa/jsboAD/6/+kKwA=").unwrap();

        assert_eq!(
            location,
            LocationReference::Line(LineLocationReference {
                points: vec![
                    LocationReferencePoint {
                        coordinate: Coordinate {
                            lon: 6.1268198,
                            lat: 49.608_498
                        },
                        line: LineAttributes {
                            frc: Frc::Frc3,
                            fow: Fow::MultipleCarriageway,
                            bear: Bearing::from_degrees(6)
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc3,
                            dnp: Length::from_meters(29)
                        })
                    },
                    LocationReferencePoint {
                        coordinate: Coordinate {
                            lon: 6.128_36,
                            lat: 49.603_966
                        },
                        line: LineAttributes {
                            frc: Frc::Frc3,
                            fow: Fow::SingleCarriageway,
                            bear: Bearing::from_degrees(6)
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc5,
                            dnp: Length::from_meters(29)
                        })
                    },
                    LocationReferencePoint {
                        coordinate: Coordinate {
                            lon: 6.128_15,
                            lat: 49.603_046
                        },
                        line: LineAttributes {
                            frc: Frc::Frc5,
                            fow: Fow::SingleCarriageway,
                            bear: Bearing::from_degrees(6)
                        },
                        path: None
                    }
                ],
                offsets: Offsets::default()
            })
        );
    }
}
