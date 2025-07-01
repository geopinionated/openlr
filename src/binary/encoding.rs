use crate::model::Offsets;
use crate::{
    Bearing, Coordinate, DecodeError, EncodeError, Fow, Frc, GridSize, Length, LineAttributes,
    Offset, Orientation, SideOfRoad,
};

#[derive(Debug, Clone, Copy)]
pub(crate) struct EncodedAttributes {
    pub(crate) line: LineAttributes,
    pub(crate) lfrcnp_or_flags: u8,
    pub(crate) orientation_or_side: u8,
}

impl From<LineAttributes> for EncodedAttributes {
    fn from(line: LineAttributes) -> Self {
        Self {
            line,
            lfrcnp_or_flags: 0,
            orientation_or_side: 0,
        }
    }
}

impl EncodedAttributes {
    pub(crate) const fn with_lfrcnp(mut self, lfrcnp: Frc) -> Self {
        self.lfrcnp_or_flags = lfrcnp.into_byte();
        self
    }

    pub(crate) const fn with_offsets(mut self, offsets: &Offsets) -> Self {
        self.lfrcnp_or_flags = offsets.into_byte();
        self
    }

    pub(crate) const fn with_orientation(mut self, orientation: &Orientation) -> Self {
        self.orientation_or_side = orientation.into_byte();
        self
    }

    pub(crate) const fn with_side(mut self, side: &SideOfRoad) -> Self {
        self.orientation_or_side = side.into_byte();
        self
    }

    pub(crate) const fn lfrcnp(&self) -> Result<Frc, DecodeError> {
        Frc::try_from_byte(self.lfrcnp_or_flags)
    }

    pub(crate) const fn pos_offset_flag(&self) -> bool {
        self.lfrcnp_or_flags & 0b10 != 0
    }

    pub(crate) const fn neg_offset_flag(&self) -> bool {
        self.lfrcnp_or_flags & 0b01 != 0
    }

    pub(crate) const fn orientation(&self) -> Result<Orientation, DecodeError> {
        Orientation::try_from_byte(self.orientation_or_side)
    }

    pub(crate) const fn side(&self) -> Result<SideOfRoad, DecodeError> {
        SideOfRoad::try_from_byte(self.orientation_or_side)
    }
}

impl Frc {
    pub(crate) const fn try_from_byte(byte: u8) -> Result<Self, DecodeError> {
        match byte {
            0 => Ok(Self::Frc0),
            1 => Ok(Self::Frc1),
            2 => Ok(Self::Frc2),
            3 => Ok(Self::Frc3),
            4 => Ok(Self::Frc4),
            5 => Ok(Self::Frc5),
            6 => Ok(Self::Frc6),
            7 => Ok(Self::Frc7),
            _ => Err(DecodeError::InvalidFrc(byte)),
        }
    }

    pub(crate) const fn into_byte(self) -> u8 {
        match self {
            Self::Frc0 => 0,
            Self::Frc1 => 1,
            Self::Frc2 => 2,
            Self::Frc3 => 3,
            Self::Frc4 => 4,
            Self::Frc5 => 5,
            Self::Frc6 => 6,
            Self::Frc7 => 7,
        }
    }
}

impl Fow {
    pub(crate) const fn try_from_byte(byte: u8) -> Result<Self, DecodeError> {
        match byte {
            0 => Ok(Self::Undefined),
            1 => Ok(Self::Motorway),
            2 => Ok(Self::MultipleCarriageway),
            3 => Ok(Self::SingleCarriageway),
            4 => Ok(Self::Roundabout),
            5 => Ok(Self::TrafficSquare),
            6 => Ok(Self::SlipRoad),
            7 => Ok(Self::Other),
            _ => Err(DecodeError::InvalidFow(byte)),
        }
    }

    pub(crate) const fn into_byte(self) -> u8 {
        match self {
            Self::Undefined => 0,
            Self::Motorway => 1,
            Self::MultipleCarriageway => 2,
            Self::SingleCarriageway => 3,
            Self::Roundabout => 4,
            Self::TrafficSquare => 5,
            Self::SlipRoad => 6,
            Self::Other => 7,
        }
    }
}

impl Coordinate {
    const RESOLUTION: usize = 24;
    const DECA_MICRO_DEG_FACTOR: f64 = 100000.0;

    /// Returns degrees from a big-endian degrees representation in a 24-bit resolution.
    pub(crate) fn degrees_from_be_bytes(bytes: [u8; 3]) -> f64 {
        let is_negative = bytes[0] & 0x80 != 0;
        let sign = if is_negative { 0xFF } else { 0 };
        let degrees = i32::from_be_bytes([sign, bytes[0], bytes[1], bytes[2]]) as f64;
        ((degrees - signum(degrees) * 0.5) * 360.0) / (1 << Self::RESOLUTION) as f64
    }

    /// Returns the big-endian representation of the given degrees in a 24-bit resolution.
    pub(crate) fn degrees_into_be_bytes(degrees: f64) -> [u8; 3] {
        let degrees = signum(degrees) * 0.5 + degrees * (1 << Self::RESOLUTION) as f64 / 360.0;
        let degrees = (degrees.round() as i32).to_be_bytes();
        [degrees[1], degrees[2], degrees[3]]
    }

    /// Returns degrees from a big-endian relative degrees representation in a 16-bit resolution.
    pub(crate) fn degrees_from_be_bytes_relative(bytes: [u8; 2], previous_degrees: f64) -> f64 {
        let degrees = i16::from_be_bytes(bytes) as f64;
        previous_degrees + degrees / Self::DECA_MICRO_DEG_FACTOR
    }

    /// Returns the big-endian relative degrees representation in a 16-bit resolution.
    pub(crate) fn degrees_into_be_bytes_relative(degrees: f64, previous_degrees: f64) -> [u8; 2] {
        let degrees = (Self::DECA_MICRO_DEG_FACTOR * (degrees - previous_degrees)).round() as i16;
        i16::to_be_bytes(degrees)
    }
}

impl Length {
    /// This representation defines 256 intervals and each interval has a length of approximately 58.6 meters.
    /// Maximum length between two consecutive LR-points is limited by 15000m.
    const DISTANCE_PER_INTERVAL: f64 = 58.6;

    /// Returns the distance to next LR-point in meters from a byte.
    pub(crate) fn dnp_from_byte(byte: u8) -> Self {
        let meters = ((byte as f64 + 0.5) * Self::DISTANCE_PER_INTERVAL).round() as u32;
        Self::from_meters(meters)
    }

    /// Returns the distance to next LR-point interval.
    pub(crate) fn dnp_into_byte(self) -> u8 {
        (self.meters() as f64 / Self::DISTANCE_PER_INTERVAL - 0.5).round() as u8
    }

    /// Returns the length of a radius in meters from big-endian slice of (up to 4) bytes.
    pub(crate) fn radius_from_be_bytes(bytes: &[u8]) -> Self {
        let mut radius = [0u8; 4];
        radius[4 - bytes.len()..].copy_from_slice(bytes);
        Self::from_meters(u32::from_be_bytes(radius))
    }

    /// Returns the big-endian representation of a radius in 4 bytes.
    pub(crate) fn radius_into_be_bytes(self) -> [u8; 4] {
        u32::to_be_bytes(self.meters())
    }
}

impl Bearing {
    /// The bearing describes the angle between the true North and the road.
    /// The data format defines 32 sectors whereby each sector covers 11.25° of the circle.
    const BEAR_SECTOR: f64 = 11.25;

    pub(crate) fn from_byte(byte: u8) -> Self {
        let degrees = (byte as f64 * Self::BEAR_SECTOR + Self::BEAR_SECTOR / 2.0).round() as u16;
        Self::from_degrees(degrees)
    }

    pub(crate) fn try_into_byte(self) -> Result<u8, EncodeError> {
        let degrees = self.degrees();
        if !(0..360).contains(&degrees) {
            return Err(EncodeError::InvalidBearing(degrees));
        }

        let bear = (degrees as f64 - Self::BEAR_SECTOR / 2.0) / Self::BEAR_SECTOR;
        Ok(bear.round() as u8)
    }
}

impl Offset {
    /// The value used here is the relation of the offset length to the length of the path
    /// between the first two location reference points (last two location reference points
    /// for the negative offset). The length between these two LR-points shall be called LRP length.
    /// The relative value (or percentage) will then be equally distributed over the available
    /// 256 buckets so that every bucket covers 0.390625% of the LRP length.
    /// Returns the offset in [0, 1] range.
    pub(crate) fn from_byte(bucket: u8) -> Self {
        Self::from_range((bucket as f64 + 0.5) / 256.0)
    }

    /// Returns the bucket index corresponding to the given offset.
    pub(crate) fn try_into_byte(self) -> Result<u8, EncodeError> {
        let range = self.range();
        if !(0.0..1.0).contains(&range) {
            return Err(EncodeError::InvalidOffset(range));
        }

        let bucket = if range == 0.0 {
            0
        } else {
            (range * 256.0 - 0.5).round() as u8
        };

        Ok(bucket)
    }
}

impl Offsets {
    pub(crate) const fn into_byte(self) -> u8 {
        let pos = ((self.pos.range() > 0.0) as u8) << 1;
        let neg = (self.neg.range() > 0.0) as u8;
        pos + neg
    }
}

impl Orientation {
    pub(crate) const fn try_from_byte(byte: u8) -> Result<Self, DecodeError> {
        match byte {
            0 => Ok(Self::Unknown),
            1 => Ok(Self::Forward),
            2 => Ok(Self::Backward),
            3 => Ok(Self::Both),
            _ => Err(DecodeError::InvalidOrientation(byte)),
        }
    }

    pub(crate) const fn into_byte(self) -> u8 {
        match self {
            Self::Unknown => 0,
            Self::Forward => 1,
            Self::Backward => 2,
            Self::Both => 3,
        }
    }
}

impl SideOfRoad {
    pub(crate) const fn try_from_byte(byte: u8) -> Result<Self, DecodeError> {
        match byte {
            0 => Ok(Self::OnRoadOrUnknown),
            1 => Ok(Self::Right),
            2 => Ok(Self::Left),
            3 => Ok(Self::Both),
            _ => Err(DecodeError::InvalidSideOfRoad(byte)),
        }
    }

    pub(crate) const fn into_byte(self) -> u8 {
        match self {
            Self::OnRoadOrUnknown => 0,
            Self::Right => 1,
            Self::Left => 2,
            Self::Both => 3,
        }
    }
}

impl GridSize {
    pub(crate) fn from_be_bytes(bytes: [u8; 4]) -> Self {
        let [c1, c2, r1, r2] = bytes;
        let columns = u16::from_be_bytes([c1, c2]);
        let rows = u16::from_be_bytes([r1, r2]);
        Self { columns, rows }
    }

    pub(crate) fn try_into_be_bytes(self) -> Result<[u8; 4], EncodeError> {
        if self.columns < 2 || self.rows < 2 {
            return Err(EncodeError::InvalidGridSize);
        }

        let columns = u16::to_be_bytes(self.columns);
        let rows = u16::to_be_bytes(self.rows);
        Ok([columns[0], columns[1], rows[0], rows[1]])
    }
}

const fn signum(value: f64) -> f64 {
    if value == 0.0 { 0.0 } else { value.signum() }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn openlr_encode_decode_degrees() {
        let assert_degrees_relative_eq = |degrees| {
            let encoded = Coordinate::degrees_into_be_bytes(degrees);
            let decoded = Coordinate::degrees_from_be_bytes(encoded);
            assert_relative_eq!(degrees, decoded, epsilon = Coordinate::EPSILON);
        };

        for (lon, lat) in [
            (5.10007, 52.103207),
            (41.030143, 28.977417),
            (50.749673, 7.099048),
            (21.173398, -86.828_1),
            (43.259594, 76.940_86),
            (-27.22775, 153.11216),
            (48.068831, 12.858026),
            (-33.22979, -60.32423),
        ] {
            assert_degrees_relative_eq(lon);
            assert_degrees_relative_eq(lat);
        }
    }

    #[test]
    fn openlr_encode_decode_relative_degrees() {
        let assert_degrees_relative_eq = |degrees, previous| {
            let encoded = Coordinate::degrees_into_be_bytes_relative(degrees, previous);
            let decoded = Coordinate::degrees_from_be_bytes_relative(encoded, previous);
            assert_relative_eq!(degrees, decoded, epsilon = Coordinate::EPSILON);
            degrees
        };

        let mut coordinate = Coordinate {
            lon: 6.5954983,
            lat: 48.0714404,
        };

        for (lon, lat) in [
            (6.4856483, 48.1540304),
            (6.4849583, 48.1689504),
            (6.3911883, 48.2611404),
            (6.3875183, 48.2661004),
            (6.3873083, 48.2663904),
            (6.3128583, 48.3426604),
            (6.2923383, 48.3627404),
            (6.2804683, 48.3684204),
            (6.2734683, 48.3697604),
            (6.2329683, 48.4129304),
            (6.2428683, 48.4842204),
            (6.2398283, 48.4902004),
            (6.1870783, 48.5563704),
        ] {
            coordinate.lon = assert_degrees_relative_eq(lon, coordinate.lon);
            coordinate.lat = assert_degrees_relative_eq(lat, coordinate.lat);
        }
    }
}
