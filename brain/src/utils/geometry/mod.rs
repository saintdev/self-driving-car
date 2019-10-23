use approx::{abs_diff_eq, AbsDiffEq};
use common::prelude::*;
use euclid::{TypedPoint3D, TypedVector3D};
use nalgebra::{Isometry3, Point2, Point3, Unit, UnitQuaternion, Vector2, Vector3};
use plane_split::{Line as TypedLine, Plane as TypedPlane};
use std::f32::consts::PI;

#[cfg(target_family = "windows")]
use common::math::fractionality;

pub mod flattener;

const EPSILON: f32 = 0.001;

pub trait ExtendF32 {
    /// Normalize an angle to between -PI and PI.
    fn normalize_angle(self) -> Self;
    /// Assert that a number is almost integral, then return it as an integer.
    fn into_almost_int(self) -> i32;
}

impl ExtendF32 for f32 {
    fn normalize_angle(self) -> Self {
        let result = self % (PI * 2.0);
        if result < -PI {
            result + (PI * 2.0)
        } else if result >= PI {
            result - (PI * 2.0)
        } else {
            result
        }
    }

    fn into_almost_int(self) -> i32 {
        #[cfg(target_family = "windows")]
        assert!(fractionality(self) <= 1e-5);

        self.round() as i32
    }
}

#[derive(Copy, Clone)]
pub struct Line2 {
    p: Point2<f32>,
    q: Point2<f32>,
}

impl Line2 {
    pub fn from_points(p: Point2<f32>, q: Point2<f32>) -> Self {
        Self { p, q }
    }

    pub fn from_origin_dir(origin: Point2<f32>, dir: Unit<Vector2<f32>>) -> Self {
        Self {
            p: origin,
            q: origin + dir.as_ref(),
        }
    }

    pub fn intersect(&self, other: Self) -> Option<Point2<f32>> {
        // http://www.ambrsoft.com/MathCalc/Line/TwoLinesIntersection/TwoLinesIntersection.htm

        let (x1, y1) = (self.p.x, self.p.y);
        let (x2, y2) = (self.q.x, self.q.y);
        let (x3, y3) = (other.p.x, other.p.y);
        let (x4, y4) = (other.q.x, other.q.y);

        let d = (x2 - x1) * (y4 - y3) - (x4 - x3) * (y2 - y1);
        if d == 0.0 {
            return None;
        }
        let x = ((x2 * y1 - x1 * y2) * (x4 - x3) - (x4 * y3 - x3 * y4) * (x2 - x1)) / d;
        let y = ((x2 * y1 - x1 * y2) * (y4 - y3) - (x4 * y3 - x3 * y4) * (y2 - y1)) / d;
        Some(Point2::new(x, y))
    }
}

#[derive(Copy, Clone)]
pub struct Line {
    pub origin: Point3<f32>,
    pub dir: Unit<Vector3<f32>>,
}

impl From<TypedLine<f32, ()>> for Line {
    fn from(line: TypedLine<f32, ()>) -> Self {
        Line {
            origin: point3_from_euclid(&line.origin),
            dir: Unit::new_unchecked(vector3_from_euclid(&line.dir)),
        }
    }
}

#[derive(Copy, Clone, PartialEq)]
pub struct Plane {
    pub normal: Unit<Vector3<f32>>,
    pub offset: f32,
}

impl Plane {
    /// Constructs a plane from point-normal form.
    pub fn point_normal(p: Point3<f32>, n: Unit<Vector3<f32>>) -> Self {
        let offset = -(n.x * p.x + n.y * p.y + n.z * p.z);
        Plane { normal: n, offset }
    }

    pub fn distance_to_point(&self, point: &Point3<f32>) -> f32 {
        let plane = TypedPlane::from(*self);
        plane.signed_distance_to(&point3_to_euclid(point))
    }

    pub fn contains_point(&self, point: &Point3<f32>) -> bool {
        self.distance_to_point(point) >= 0.0
    }

    pub fn project_point(&self, point: &Point3<f32>) -> Point3<f32> {
        point - self.normal.into_inner() * self.distance_to_point(point)
    }

    pub fn project_rot(&self, quat: &UnitQuaternion<f32>) -> UnitQuaternion<f32> {
        UnitQuaternion::from_scaled_axis(self.project_vector(&quat.scaled_axis()))
    }

    pub fn project_vector(&self, vector: &Vector3<f32>) -> Vector3<f32> {
        vector - self.normal.into_inner() * self.normal.dot(&vector)
    }

    pub fn intersect(&self, other: &Self) -> Option<Line> {
        let plane = TypedPlane::from(*self);
        let other = TypedPlane::from(*other);
        plane.intersect(&other).map(Into::into)
    }

    /// Returns a transformation which "unfolds" this plane along its
    /// intersection with another plane, such that the two planes are coplanar.
    ///
    /// Returns `Err(())` if the planes are parallel.
    pub fn unfold(&self, target: &Plane) -> Result<Isometry3<f32>, ()> {
        // Special-case the identity transformation.
        if abs_diff_eq!(target, self, epsilon = EPSILON) {
            return Ok(Isometry3::identity());
        }

        let seam = self.intersect(target).ok_or(())?;
        Ok(Isometry3::rotation_wrt_point(
            self.normal.rotation_to(&target.normal),
            seam.origin,
        ))
    }

    fn origin(&self) -> Point3<f32> {
        Point3::origin() - self.offset * self.normal.into_inner()
    }

    pub fn transform(&self, m: Isometry3<f32>) -> Self {
        let point = m * self.origin();
        let normal = m * self.normal;
        Self::point_normal(point, normal)
    }
}

impl AbsDiffEq for Plane {
    type Epsilon = f32;

    fn default_epsilon() -> Self::Epsilon {
        f32::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        self.normal.abs_diff_eq(&other.normal, epsilon)
            && self.offset.abs_diff_eq(&other.offset, epsilon)
    }
}

impl From<TypedPlane<f32, ()>> for Plane {
    fn from(plane: TypedPlane<f32, ()>) -> Self {
        Self {
            normal: unit_vector3_from_euclid(&plane.normal),
            offset: plane.offset,
        }
    }
}

impl From<Plane> for TypedPlane<f32, ()> {
    fn from(plane: Plane) -> Self {
        Self {
            normal: unit_vector3_to_euclid(&plane.normal),
            offset: plane.offset,
        }
    }
}

fn vector3_from_euclid(v: &TypedVector3D<f32, ()>) -> Vector3<f32> {
    Vector3::new(v.x, v.y, v.z)
}

fn unit_vector3_from_euclid(v: &TypedVector3D<f32, ()>) -> Unit<Vector3<f32>> {
    Unit::new_unchecked(Vector3::new(v.x, v.y, v.z))
}

fn unit_vector3_to_euclid(v: &Unit<Vector3<f32>>) -> TypedVector3D<f32, ()> {
    TypedVector3D::new(v.x, v.y, v.z)
}

fn point3_from_euclid(v: &TypedPoint3D<f32, ()>) -> Point3<f32> {
    Point3::new(v.x, v.y, v.z)
}

fn point3_to_euclid(point: &Point3<f32>) -> TypedPoint3D<f32, ()> {
    TypedPoint3D::new(point.x, point.y, point.z)
}

/// Returns the two points on a circle that form a tangent with the given point.
///
/// If the point is inside the circle, returns `None`.
pub fn circle_point_tangents(
    center: Point2<f32>,
    radius: f32,
    point: Point2<f32>,
) -> Option<[Point2<f32>; 2]> {
    // I'm so glad the internet exists
    // http://www.ambrsoft.com/TrigoCalc/Circles2/CirclePoint/CirclePointDistance.htm

    let a = center.x;
    let b = center.y;
    let r = radius;
    let xp = point.x;
    let yp = point.y;

    let xpm = r * (yp - b) * ((xp - a).powi(2) + (yp - b).powi(2) - r.powi(2)).sqrt();
    let x1 = (r.powi(2) * (xp - a) + xpm) / ((xp - a).powi(2) + (yp - b).powi(2)) + a;
    let x2 = (r.powi(2) * (xp - a) - xpm) / ((xp - a).powi(2) + (yp - b).powi(2)) + a;

    let ymp = r * (xp - a) * ((xp - a).powi(2) + (yp - b).powi(2) - r.powi(2)).sqrt();
    let y1 = (r.powi(2) * (yp - b) - ymp) / ((xp - a).powi(2) + (yp - b).powi(2)) + b;
    let y2 = (r.powi(2) * (yp - b) + ymp) / ((xp - a).powi(2) + (yp - b).powi(2)) + b;

    if x1.is_nan() {
        None
    } else {
        Some([Point2::new(x1, y1), Point2::new(x2, y2)])
    }
}

pub struct RayCoordinateSystem {
    origin: Point2<f32>,
    direction: Unit<Vector2<f32>>,
}

impl RayCoordinateSystem {
    pub fn segment(p: Point2<f32>, q: Point2<f32>) -> Self {
        Self {
            origin: p,
            direction: (q - p).to_axis(),
        }
    }

    pub fn project(&self, p: Point2<f32>) -> f32 {
        (p - self.origin).dot(&self.direction)
    }
}
