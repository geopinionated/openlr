use approx::abs_diff_eq;

use crate::OpenLrError;

/// Functional Road Class.
/// The functional road class (FRC) of a line is a road classification
/// based on the importance of the road represented by the line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum Frc {
    /// Main road, highest importance
    Frc0 = 0,
    Frc1 = 1,
    Frc2 = 2,
    Frc3 = 3,
    Frc4 = 4,
    Frc5 = 5,
    Frc6 = 6,
    /// Other class road, lowest importance
    Frc7 = 7,
}

/// Form of Way.
/// The form of way (FOW) describes the physical road type of a line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum Fow {
    /// The physical road type is unknown.
    Undefined = 0,
    /// A Motorway is defined as a road permitted for motorized vehicles
    /// only in combination with a prescribed minimum speed. It has two
    /// or more physically separated carriageways and no single level-crossings.
    Motorway = 1,
    /// A multiple carriageway is defined as a road with physically separated
    /// carriageways regardless of the number of lanes. If a road is also a
    /// motorway, it should be coded as such and not as a multiple carriageway.
    MultipleCarriageway = 2,
    /// All roads without separate carriageways are considered as roads with
    /// a single carriageway.
    SingleCarriageway = 3,
    /// A Roundabout is a road which forms a ring on which traffic traveling
    /// in only one direction is allowed.
    Roundabout = 4,
    /// A Traffic Square is an open area (partly) enclosed by roads which is
    /// used for non-traffic purposes and which is not a Roundabout.
    TrafficSquare = 5,
    /// A Slip Road is a road especially designed to enter or leave a line.
    SlipRoad = 6,
    /// The physical road type is known but does not fit into one of the
    /// other categories.
    Other = 7,
}

/// The side of road information (SOR) describes the relationship between the
/// point of interest and a referenced line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum SideOfRoad {
    /// Point is directly on (or above) the road, or determination of right/left
    /// side is not applicable.
    OnRoadOrUnknown = 0,
    /// Point is on right side of the road.
    Right = 1,
    /// Point is on left side of the road.
    Left = 2,
    /// Point is on both sides of the road.
    Both = 3,
}

/// The orientation information (ORI) describes the relationship between the
/// point of interest and the direction of a referenced line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum Orientation {
    /// Point has no sense of orientation, or determination of orientation
    /// is not applicable
    Unknown = 0,
    /// Point has orientation from first LRP towards second LRP.
    Forward = 1,
    /// Point has orientation from second LRP towards first LRP.
    Backward = 2,
    /// Point has orientation in both directions
    Both = 3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Length(u16);

/// The bearing describes the angle between the true North and the road.
/// The physical data format defines the bearing field as an integer value between 0
/// and 360 whereby “0” is included and “360” is excluded from that range.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Bearing(u16);

/// Coordinate pair stands for a pair of WGS84 longitude (lon) and latitude (lat) values.
/// This coordinate pair specifies a geometric point in a digital map.
/// The lon and lat values are stored in decamicrodegree resolution (five decimals).
#[derive(Debug, Clone, Copy)]
pub struct Coordinate {
    pub lon: f32,
    pub lat: f32,
}

impl PartialEq for Coordinate {
    fn eq(&self, other: &Self) -> bool {
        const EPSILON: f32 = 1e-5;
        abs_diff_eq!(self.lon, other.lon, epsilon = EPSILON)
            && abs_diff_eq!(self.lat, other.lat, epsilon = EPSILON)
    }
}

/// Line attributes are part of a location reference point and consist of functional road
/// class (FRC), form of way (FOW) and bearing (BEAR) data.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LineAttributes {
    pub frc: Frc,
    pub fow: Fow,
    pub bear: Bearing,
}

/// The path attributes are part of a location reference point (except for the last
/// location reference point) and consists of lowest functional road class to next point
/// (LFRCNP) and distance to next point (DNP) data.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PathAttributes {
    /// Lowest functional road class to next point.
    pub lfrcnp: Frc,
    /// Distance to next point.
    pub dnp: Length,
}

/// The basis of a location reference is a sequence of location reference points (LRPs).
/// A single LRP may be bound to the road network. In such a case all values of the LRP
/// refer to a node or line within the road network. The coordinates refer to a node of
/// a line or a point on a line and the additional attributes refer to attributes of a line.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocationReferencePoint {
    pub coordinate: Coordinate,
    pub line: LineAttributes,
    pub path: Option<PathAttributes>,
}

/// Offsets are used to locate the start and end of a location more precisely than
/// bounding to the nodes in a network. The logical format defines two offsets,
/// one at the start of the location and one at the end of the location.
/// Both offsets operate along the lines of the location and are measured in meters.
// The offset values are optional and a missing offset value means an offset of 0 meters.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Offset(f32);

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Offsets {
    pub pos: Offset,
    pub neg: Offset,
}

/// A line location reference describes a path within a map and consists of location
/// reference point(s), a last location reference point and offset data.
/// There must be at least one location reference point and exactly one last location
/// reference point. The offset field is optional.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct LineLocationReference {
    pub points: Vec<LocationReferencePoint>,
    pub offsets: Offsets,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum LocationType {
    Line = 0,
    GeoCoordinate = 1,
    PointAlongLine = 2,
    PoiWithAccessPoint = 3,
    Circle = 4,
    Rectangle = 5,
    Grid = 6,
    Polygon = 7,
    ClosedLine = 8,
}

#[derive(Debug, Clone, PartialEq)]
pub enum LocationReference {
    Line(LineLocationReference),
}

impl Frc {
    pub const fn try_from_byte(byte: u8) -> Result<Self, OpenLrError> {
        match byte {
            0 => Ok(Self::Frc0),
            1 => Ok(Self::Frc1),
            2 => Ok(Self::Frc2),
            3 => Ok(Self::Frc3),
            4 => Ok(Self::Frc4),
            5 => Ok(Self::Frc5),
            6 => Ok(Self::Frc6),
            7 => Ok(Self::Frc7),
            _ => Err(OpenLrError::BinaryParseError),
        }
    }
}

impl Fow {
    pub const fn try_from_byte(byte: u8) -> Result<Self, OpenLrError> {
        match byte {
            0 => Ok(Self::Undefined),
            1 => Ok(Self::Motorway),
            2 => Ok(Self::MultipleCarriageway),
            3 => Ok(Self::SingleCarriageway),
            4 => Ok(Self::Roundabout),
            5 => Ok(Self::TrafficSquare),
            6 => Ok(Self::SlipRoad),
            7 => Ok(Self::Other),
            _ => Err(OpenLrError::BinaryParseError),
        }
    }
}

impl LineLocationReference {
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::with_capacity(capacity),
            offsets: Offsets::default(),
        }
    }
}

impl Coordinate {
    /// Returns degrees from a big-endian coordinate representation in a 24-bit resolution.
    pub fn degrees_from_be_bytes(bytes: [u8; 3]) -> f32 {
        const RESOLUTION: usize = 24;
        let value = i32::from_be_bytes([0, bytes[0], bytes[1], bytes[2]]) as f32;
        ((value - value.signum() * 0.5) * 360.0) / (1 << RESOLUTION) as f32
    }

    /// Returns degrees from a big-endian relative coordinate representation in a 16-bit resolution.
    pub fn degrees_from_be_bytes_relative(bytes: [u8; 2], previous_value: f32) -> f32 {
        const DECA_MICRO_DEG_FACTOR: f32 = 100000.0;
        let value = i16::from_be_bytes(bytes) as f32;
        previous_value + value / DECA_MICRO_DEG_FACTOR
    }
}

impl Length {
    pub const fn from_meters(meters: u16) -> Self {
        Self(meters)
    }

    pub const fn meters(&self) -> u16 {
        self.0
    }

    /// Returns the distance to next LR-point in meters from a byte.
    /// This representation defines 256 intervals and each interval has a length of approximately 58.6 meters.
    /// Maximum length between two consecutive LR-points is limited by 15000m.
    pub fn dnp_from_byte(byte: u8) -> Length {
        const DISTANCE_PER_INTERVAL: f32 = 58.6;
        let dnp = ((byte as f32 + 0.5) * DISTANCE_PER_INTERVAL).round();
        Length(dnp as u16)
    }
}

impl Bearing {
    pub const fn from_degrees(degrees: u16) -> Self {
        Self(degrees)
    }

    pub const fn degrees(&self) -> u16 {
        self.0
    }

    /// The bearing describes the angle between the true North and the road.
    /// The data format defines 32 sectors whereby each sector covers 11.25° of the circle.
    pub fn from_byte(byte: u8) -> Self {
        const BEAR_SECTOR: f32 = 11.25;
        let bear = (byte as f32 * BEAR_SECTOR + BEAR_SECTOR / 2.0).round();
        Self(bear as u16)
    }
}

impl Offset {
    pub const fn from_range(range: f32) -> Self {
        Self(range)
    }

    /// The value used here is the relation of the offset length to the length of the path
    /// between the first two location reference points (last two location reference points
    /// for the negative offset). The length between these two LR-points shall be called LRP length.
    /// The relative value (or percentage) will then be equally distributed over the available
    /// 256 buckets so that every bucket covers 0.390625% of the LRP length.
    /// Returns the offset in [0, 1] range.
    pub fn from_byte(bucket: u8) -> Self {
        Self((bucket as f32 + 0.5) / 256.0)
    }
}
