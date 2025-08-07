use std::fmt;
use std::iter::Sum;
use std::ops::{Add, AddAssign, Mul, MulAssign, Sub, SubAssign};

use approx::abs_diff_eq;
use ordered_float::OrderedFloat;
use strum::IntoEnumIterator;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, strum::EnumIter)]
#[repr(u8)]
pub enum Rating {
    Excellent = 0,
    Good = 1,
    Average = 2,
    Poor = 3,
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RatingScore(OrderedFloat<f64>);

impl fmt::Debug for RatingScore {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.1}", self.0)
    }
}

impl From<f64> for RatingScore {
    fn from(value: f64) -> Self {
        Self(OrderedFloat(value))
    }
}

impl From<RatingScore> for f64 {
    fn from(score: RatingScore) -> Self {
        score.0.into()
    }
}

impl From<Length> for RatingScore {
    fn from(length: Length) -> Self {
        Self(length.meters().into())
    }
}

impl Add for RatingScore {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self(self.0 + other.0)
    }
}

impl Sub for RatingScore {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self(self.0 - other.0)
    }
}

impl Mul<f64> for RatingScore {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self::Output {
        Self(self.0 * rhs)
    }
}

impl Mul<RatingScore> for f64 {
    type Output = RatingScore;
    fn mul(self, rhs: RatingScore) -> Self::Output {
        RatingScore(OrderedFloat(self) * rhs.0)
    }
}

impl Mul<RatingScore> for RatingScore {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self::Output {
        Self(self.0 * rhs.0)
    }
}

impl MulAssign<f64> for RatingScore {
    fn mul_assign(&mut self, rhs: f64) {
        self.0 = self.0 * rhs;
    }
}

/// Functional Road Class.
/// The functional road class (FRC) of a line is a road classification
/// based on the importance of the road represented by the line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, strum::EnumIter)]
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

impl Frc {
    /// Gets the value of this Functional Road Class, the lower the value the higher
    /// the importance of the class.
    pub const fn value(&self) -> i8 {
        self.into_byte() as i8
    }

    pub fn from_value(value: i8) -> Option<Self> {
        Self::try_from_byte(value.try_into().ok()?).ok()
    }

    /// Variance is an estimate of how a FRC can differ from another FRC of different class.
    /// The higher the variance the more the two classes can differ and still be considered
    /// equal during the decoding process.
    pub(crate) const fn variance(&self) -> i8 {
        match self {
            Self::Frc0 | Self::Frc1 | Self::Frc2 | Self::Frc3 => 2,
            Self::Frc4 | Self::Frc5 | Self::Frc6 | Self::Frc7 => 3,
        }
    }

    pub(crate) const fn is_within_variance(&self, other: &Self) -> bool {
        self.value() <= other.value() + other.variance()
    }

    pub(crate) fn rating(&self, other: &Self) -> Rating {
        let delta = (self.value() - other.value()).abs();

        let rating_interval = |rating| match rating {
            Rating::Excellent => 0,
            Rating::Good => 1,
            Rating::Average => 2,
            Rating::Poor => 3,
        };

        Rating::iter()
            .find(|&rating| delta <= rating_interval(rating))
            .unwrap_or(Rating::Poor)
    }

    pub(crate) fn rating_score(rating: Rating) -> RatingScore {
        match rating {
            Rating::Excellent => RatingScore::from(100.0),
            Rating::Good => RatingScore::from(75.0),
            Rating::Average => RatingScore::from(50.0),
            Rating::Poor => RatingScore::from(0.0),
        }
    }
}

/// Form of Way.
/// The form of way (FOW) describes the physical road type of a line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, strum::EnumIter)]
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

impl Fow {
    pub const fn value(&self) -> i8 {
        self.into_byte() as i8
    }

    pub fn from_value(value: i8) -> Option<Self> {
        Self::try_from_byte(value.try_into().ok()?).ok()
    }

    pub(crate) const fn rating(&self, other: &Self) -> Rating {
        use Fow::*;
        match (self, other) {
            (Undefined, _) | (_, Undefined) => Rating::Average,
            (Motorway, Motorway) => Rating::Excellent,
            (Motorway, MultipleCarriageway) => Rating::Good,
            (Motorway, _) => Rating::Poor,
            (MultipleCarriageway, MultipleCarriageway) => Rating::Excellent,
            (MultipleCarriageway, Motorway) => Rating::Good,
            (MultipleCarriageway, _) => Rating::Poor,
            (SingleCarriageway, SingleCarriageway) => Rating::Excellent,
            (SingleCarriageway, MultipleCarriageway) => Rating::Good,
            (SingleCarriageway, Roundabout | TrafficSquare) => Rating::Average,
            (SingleCarriageway, _) => Rating::Poor,
            (Roundabout, Roundabout) => Rating::Excellent,
            (Roundabout, MultipleCarriageway | SingleCarriageway | TrafficSquare) => {
                Rating::Average
            }
            (Roundabout, _) => Rating::Poor,
            (TrafficSquare, TrafficSquare) => Rating::Excellent,
            (TrafficSquare, SingleCarriageway | Roundabout) => Rating::Average,
            (TrafficSquare, _) => Rating::Poor,
            (SlipRoad, SlipRoad) => Rating::Excellent,
            (SlipRoad, _) => Rating::Poor,
            (Other, Other) => Rating::Excellent,
            (Other, _) => Rating::Poor,
        }
    }

    pub(crate) fn rating_score(rating: Rating) -> RatingScore {
        match rating {
            Rating::Excellent => RatingScore::from(100.0),
            Rating::Good | Rating::Average => RatingScore::from(50.0),
            Rating::Poor => RatingScore::from(25.0),
        }
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Length(OrderedFloat<f64>);

impl fmt::Display for Length {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.1}m", self.meters())
    }
}

impl Length {
    pub const ZERO: Self = Self(OrderedFloat(0.0));
    pub const MAX: Self = Self(OrderedFloat(f64::MAX));

    /// Binary format version 3 doesn't allow LRPs distances over 15000m.
    pub const MAX_BINARY_LRP_DISTANCE: Self = Self(OrderedFloat(15000.0));

    pub const fn from_meters(meters: f64) -> Self {
        Self(OrderedFloat(meters))
    }

    pub const fn meters(&self) -> f64 {
        self.0.0
    }

    pub fn is_zero(&self) -> bool {
        *self == Self::ZERO
    }

    pub fn round(self) -> Self {
        Self(self.0.round().into())
    }

    pub fn ceil(self) -> Self {
        Self(self.0.ceil().into())
    }

    pub fn reverse(self) -> Self {
        Self(self.0 * -1.0)
    }

    pub fn clamp(self, min: Self, max: Self) -> Self {
        self.max(min).min(max)
    }
}

impl Add for Length {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self(self.0 + other.0)
    }
}

impl AddAssign for Length {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl Sub for Length {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self(self.0 - other.0)
    }
}

impl SubAssign for Length {
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0;
    }
}

impl Sum for Length {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Self::ZERO, |a, b| a + b)
    }
}

impl Mul<f64> for Length {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self::Output {
        Self(self.0 * rhs)
    }
}

impl Mul<Length> for f64 {
    type Output = Length;
    fn mul(self, rhs: Length) -> Self::Output {
        Length(OrderedFloat(self) * rhs.0)
    }
}

/// The bearing describes the angle between the true North and the road.
/// The physical data format defines the bearing field as an integer value between 0
/// and 360 whereby “0” is included and “360” is excluded from that range.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Bearing(u16);

impl Bearing {
    pub const NORTH: Self = Self(0);

    pub const fn from_degrees(degrees: u16) -> Self {
        Self(degrees % 360)
    }

    pub fn from_radians(radians: f64) -> Self {
        let degrees = radians.to_degrees().rem_euclid(360.0).round() as u16;
        Self::from_degrees(degrees)
    }

    pub const fn degrees(&self) -> u16 {
        self.0
    }

    pub const fn difference(&self, other: &Self) -> Self {
        let delta = (self.0 as i32 - other.0 as i32).unsigned_abs() as u16;
        let degrees = if delta > 180 { 360 - delta } else { delta };
        Self::from_degrees(degrees)
    }

    pub(crate) fn rating(&self, other: &Self) -> Rating {
        let difference = self.difference(other);

        let rating_interval = |rating| match rating {
            Rating::Excellent => 6,
            Rating::Good => 12,
            Rating::Average => 18,
            Rating::Poor => 24,
        };

        Rating::iter()
            .find(|&rating| difference.degrees() <= rating_interval(rating))
            .unwrap_or(Rating::Poor)
    }

    pub(crate) fn rating_score(rating: Rating) -> RatingScore {
        match rating {
            Rating::Excellent => RatingScore::from(100.0),
            Rating::Good => RatingScore::from(50.0),
            Rating::Average => RatingScore::from(25.0),
            Rating::Poor => RatingScore::from(0.0),
        }
    }
}

/// Coordinate pair stands for a pair of WGS84 longitude (lon) and latitude (lat) values.
/// This coordinate pair specifies a geometric point in a digital map.
/// The lon and lat values are stored in decamicrodegree resolution (five decimals).
#[derive(Debug, Clone, Copy, Default)]
pub struct Coordinate {
    pub lon: f64,
    pub lat: f64,
}

impl Coordinate {
    pub const EPSILON: f64 = 180.0 / (1 << 24) as f64;
}

impl PartialEq for Coordinate {
    fn eq(&self, other: &Self) -> bool {
        abs_diff_eq!(self.lon, other.lon, epsilon = Self::EPSILON)
            && abs_diff_eq!(self.lat, other.lat, epsilon = Self::EPSILON)
    }
}

/// Line attributes are part of a location reference point and consist of functional road
/// class (FRC), form of way (FOW) and bearing (BEAR) data.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LineAttributes {
    pub frc: Frc,
    pub fow: Fow,
    pub bearing: Bearing,
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

impl Point {
    /// Returns true only if this point is the last point of a Reference Location,
    /// and therefore it doesn't have Path attributes.
    pub const fn is_last(&self) -> bool {
        self.path.is_none()
    }

    /// Gets the lowest FRC to the next point.
    pub fn lfrcnp(&self) -> Frc {
        self.path.map(|path| path.lfrcnp).unwrap_or(Frc::Frc7)
    }

    /// Gets the distance to the next point.
    pub fn dnp(&self) -> Length {
        self.path.map(|path| path.dnp).unwrap_or(Length::ZERO)
    }
}

/// Offsets are used to locate the start and end of a location more precisely than
/// bounding to the nodes in a network. The logical format defines two offsets,
/// one at the start of the location and one at the end of the location.
/// Both offsets operate along the lines of the location and are measured in meters.
/// The offset values are optional and a missing offset value means an offset of 0 meters.
#[derive(Debug, Clone, Copy, Default)]
pub struct Offset(f64);

impl PartialEq for Offset {
    fn eq(&self, other: &Self) -> bool {
        abs_diff_eq!(self.0, other.0, epsilon = Self::EPSILON)
    }
}

impl Offset {
    pub const ZERO: Self = Self(0.0);
    pub const EPSILON: f64 = 0.5 / (1 << 8) as f64;
    pub const BUCKETS: f64 = 256.0;

    pub const fn from_range(range: f64) -> Self {
        Self(range)
    }

    pub const fn from_bucket(bucket_index: u8) -> Self {
        Self::from_range((bucket_index as f64 + 0.5) / Self::BUCKETS)
    }

    /// Computes the relative offset range value between offset and the location lenght.
    pub fn relative(offset: Length, dnp: Length) -> Self {
        if offset.is_zero() || dnp.is_zero() {
            return Self::ZERO;
        }

        let bucket = if offset == dnp {
            Self::BUCKETS - 1.0
        } else {
            (Self::BUCKETS * offset.meters() / dnp.meters()).floor()
        };

        Self::from_bucket(bucket as u8)
    }

    pub const fn range(&self) -> f64 {
        self.0
    }
}

/// A positive offset (POFF) is used to locate the precise start of a location.
/// The POFF defines the distance between the start of the location reference path
/// and the start of the location. The negative offset (NOFF) is used to locate the
/// precise end of the location and it defines the distance between the end of the
/// location and the end of the location reference path.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Offsets {
    pub pos: Offset,
    pub neg: Offset,
}

impl Offsets {
    pub fn positive(offset: Offset) -> Self {
        Self {
            pos: offset,
            neg: Offset::default(),
        }
    }

    pub fn distance_from_start(&self, length: Length) -> Length {
        let length = (self.pos.range() * length.meters()).round();
        Length::from_meters(length)
    }

    pub fn distance_to_end(&self, length: Length) -> Length {
        let length = (self.neg.range() * length.meters()).round();
        Length::from_meters(length)
    }
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
#[derive(Debug, Clone, PartialEq)]
pub struct Grid {
    pub rect: Rectangle,
    pub size: GridSize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
/// The minimum number of coordinate pairs is three and there exists no maximum number.
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
/// concatenation of (several) shortest-paths. The concatenation of such shortest-paths shall cover
/// the location completely. Each shortest-path is specified by information about its start line and
/// its end line. This information is combined in the location reference points (LRPs). The LRPs are
/// ordered from the start of the location to the end of the location and the shortest-path between
/// two subsequent LRPs covers a part of the location. The concatenation of all these shortest-paths
/// covers the location completely and this path is called the location reference path. The location
/// reference path may be longer than the original location and offsets trim this path down to the
/// size of the location path. Offsets are also used to define a location on a line more precisely
/// (e.g. point locations along a line) than using the start and end node of that line.
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

#[cfg(test)]
mod tests {
    use std::f64::consts::{FRAC_PI_2, PI};

    use strum::IntoEnumIterator;
    use test_log::test;

    use super::*;

    #[test]
    fn fow_rating() {
        for (fow1, fow2) in Fow::iter().zip(Fow::iter()) {
            assert_eq!(fow1.rating(&fow2), fow2.rating(&fow1));
        }
    }

    #[test]
    fn bearing_degrees() {
        assert_eq!(Bearing::from_degrees(0).degrees(), 0);
        assert_eq!(Bearing::from_degrees(90).degrees(), 90);
        assert_eq!(Bearing::from_degrees(180).degrees(), 180);
        assert_eq!(Bearing::from_degrees(270).degrees(), 270);
        assert_eq!(Bearing::from_degrees(360).degrees(), 0);
        assert_eq!(Bearing::from_degrees(360 + 90).degrees(), 90);

        assert_eq!(Bearing::from_radians(0.0).degrees(), 0);
        assert_eq!(Bearing::from_radians(FRAC_PI_2).degrees(), 90);
        assert_eq!(Bearing::from_radians(PI).degrees(), 180);
        assert_eq!(Bearing::from_radians(PI + FRAC_PI_2).degrees(), 270);
        assert_eq!(Bearing::from_radians(PI + PI).degrees(), 0);
        assert_eq!(Bearing::from_radians(PI + PI + FRAC_PI_2).degrees(), 90);

        assert_eq!(Bearing::from_radians(-PI).degrees(), 180);
        assert_eq!(Bearing::from_radians(-FRAC_PI_2).degrees(), 270);
        assert_eq!(Bearing::from_radians(-PI - FRAC_PI_2).degrees(), 90);
    }
}
