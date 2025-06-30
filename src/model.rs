use approx::abs_diff_eq;

/// Functional Road Class.
/// The functional road class (FRC) of a line is a road classification
/// based on the importance of the road represented by the line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum Frc {
    /// Main road, highest importance
    Frc0 = 0,
    /// First class road.
    Frc1 = 1,
    /// Second class road.
    Frc2 = 2,
    /// Third class road.
    Frc3 = 3,
    /// Fourth class road.
    Frc4 = 4,
    /// Fifth class road.
    Frc5 = 5,
    /// Sixth class road.
    Frc6 = 6,
    /// Other class road, lowest importance
    Frc7 = 7,
}

impl Default for Frc {
    fn default() -> Self {
        Self::Frc7
    }
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

impl Default for Fow {
    fn default() -> Self {
        Self::Other
    }
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

impl Default for SideOfRoad {
    fn default() -> Self {
        Self::OnRoadOrUnknown
    }
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

impl Default for Orientation {
    fn default() -> Self {
        Self::Unknown
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Length(u32);

impl Length {
    pub const fn from_meters(meters: u32) -> Self {
        Self(meters)
    }

    pub const fn meters(&self) -> u32 {
        self.0
    }
}

/// The bearing describes the angle between the true North and the road.
/// The physical data format defines the bearing field as an integer value between 0
/// and 360 whereby “0” is included and “360” is excluded from that range.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Bearing(u16);

impl Bearing {
    pub const fn from_degrees(degrees: u16) -> Self {
        Self(degrees)
    }

    pub const fn degrees(&self) -> u16 {
        self.0
    }
}

/// Coordinate pair stands for a pair of WGS84 longitude (lon) and latitude (lat) values.
/// This coordinate pair specifies a geometric point in a digital map.
/// The lon and lat values are stored in decamicrodegree resolution (five decimals).
#[derive(Debug, Clone, Copy, Default)]
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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LineAttributes {
    pub frc: Frc,
    pub fow: Fow,
    pub bear: Bearing,
}

/// The path attributes are part of a location reference point (except for the last
/// location reference point) and consists of lowest functional road class to next point
/// (LFRCNP) and distance to next point (DNP) data.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
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
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point {
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

impl Offset {
    pub const fn from_range(range: f32) -> Self {
        Self(range)
    }

    pub const fn range(&self) -> f32 {
        self.0
    }
}

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
pub struct Line {
    pub points: Vec<Point>,
    pub offsets: Offsets,
}

impl Line {
    pub(crate) fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::with_capacity(capacity),
            offsets: Offsets::default(),
        }
    }
}

/// A closed line location references the area defined by a closed path (i.e. a circuit)
/// in the road network. The boundary always consists of road segments.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ClosedLine {
    pub points: Vec<Point>,
    pub last_line: LineAttributes,
}

impl ClosedLine {
    pub(crate) fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::with_capacity(capacity),
            last_line: LineAttributes::default(),
        }
    }
}

/// Point along line is a point location which is defined by a line and an offset value.
/// The line will be referenced by two location reference points and the concrete position
/// on that line is referenced using the positive offset. Additionally information about
/// the side of the road where the point is located and the orientation with respect
/// to the direction of the line can be added.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PointAlongLine {
    pub points: [Point; 2],
    pub offset: Offset,
    pub orientation: Orientation,
    pub side: SideOfRoad,
}

/// Point along line with access is a point location which is defined by a line,
/// an offset value and a coordinate. The line will be referenced by two location reference
/// points and the concrete position of the access point on that line is referenced using
/// the positive offset. The point of interest is identified by the coordinate pair.
/// Additionally information about the side of the road where the point is located and
/// the orientation with respect to the direction of the line can be added.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Poi {
    pub point: PointAlongLine,
    pub poi: Coordinate,
}

/// A circle location is given by the position of the center and the radius.
/// The center position is a geo-coordinate pair of longitude and latitude coordinate
/// values that can be everywhere on the surface. The radius is integer-valued and
/// given in meters.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Circle {
    pub center: Coordinate,
    pub radius: Length,
}

/// A rectangle location reference consists of the lower left corner point as a pair
/// of WGS84 coordinates in absolute format and the upper right corner point, given in
/// absolute format (large rectangle) or relative format (standard rectangle).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Rectangle {
    pub lower_left: Coordinate,
    pub upper_right: Coordinate,
}

/// A grid location is a special instance of a rectangle location. It is given
/// by a base rectangular shape. This base rectangle is the lower left cell of
/// the grid and can be multiplied to the North (by defining the number of rows)
/// and to the East (by defining the number of columns).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Grid {
    pub rect: Rectangle,
    pub size: GridSize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct GridSize {
    pub columns: u16,
    pub rows: u16,
}

/// A polygon location is a non-intersecting shape defined by a sequence of
/// geo-coordinate pairs. The coordinate pairs can be everywhere on the surface.
/// They define the corners of the underlying geometrical polygon. The boundary
/// of this polygon is constituted by straight lines between every pair of
/// consecutive corners in the sequence, plus the straight line between the last and
/// the first corner.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Polygon {
    pub corners: Vec<Coordinate>,
}

impl Polygon {
    pub(crate) fn with_capacity(capacity: usize) -> Self {
        Self {
            corners: Vec::with_capacity(capacity),
        }
    }
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

/// Locations are objects in a digital map, like points, paths and areas.
/// OpenLR standard can handle line locations (e.g. paths), point locations (e.g. POIs)
/// and area locations (e.g. regions) in a digital map.
/// OpenLR can handle locations which are bound to the road network but also locations
/// which can be everywhere on earth (not bound to the road network). A line location is
/// an example for a location which is bound to the road network and a simple geo-coordinate
/// is an example for a location which is not bound to the road network.
/// The main idea for locations which are bound to the road network is covering the location with a
/// concatenation of (several) shortest-paths. The concatenation of such shortest-paths shall cover the
/// location completely. Each shortest-path is specified by information about its start line and its end line.
/// This information is combined in the location reference points (LRPs). The LRPs are ordered from the
/// start of the location to the end of the location and the shortest-path between two subsequent LRPs
/// covers a part of the location. The concatenation of all these shortest-paths covers the location
/// completely and this path is called the location reference path. The location reference path may be
/// longer than the original location and offsets trim this path down to the size of the location path.
/// Offsets are also used to define a location on a line more precisely (e.g. point locations along a line)
/// than using the start and end node of that line.
#[derive(Debug, Clone, PartialEq)]
pub enum LocationReference {
    // Line Locations
    Line(Line),
    // Point Locations
    GeoCoordinate(Coordinate),
    PointAlongLine(PointAlongLine),
    Poi(Poi),
    // Area Locations
    Circle(Circle),
    Rectangle(Rectangle),
    Grid(Grid),
    Polygon(Polygon),
    ClosedLine(ClosedLine),
}

impl LocationReference {
    pub const fn location_type(&self) -> LocationType {
        match self {
            Self::Line(_) => LocationType::Line,
            Self::GeoCoordinate(_) => LocationType::GeoCoordinate,
            Self::PointAlongLine(_) => LocationType::PointAlongLine,
            Self::Poi(_) => LocationType::PoiWithAccessPoint,
            Self::Circle(_) => LocationType::Circle,
            Self::Rectangle(_) => LocationType::Rectangle,
            Self::Grid(_) => LocationType::Grid,
            Self::Polygon(_) => LocationType::Polygon,
            Self::ClosedLine(_) => LocationType::ClosedLine,
        }
    }
}
