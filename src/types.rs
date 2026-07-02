//! Type definitions for FEM solver

use std::ops::{Add, Sub};

/// 2D Vector type
pub type Vector2 = [f64; 2];

/// 2x2 Matrix type
pub type Matrix2 = [[f64; 2]; 2];

/// Node ID type
pub type NodeId = usize;

/// Element ID type
pub type ElementId = usize;

/// Boundary condition type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BoundaryCondition {
    /// Free boundary (no constraint)
    Free,
    /// Fixed in X direction
    FixedX,
    /// Fixed in Y direction
    FixedY,
    /// Fully fixed (both X and Y)
    Fixed,
    /// Prescribed displacement
    Prescribed(f64),
}

/// Load type for boundary conditions
#[derive(Debug, Clone, Copy)]
pub enum LoadType {
    /// Point force
    Force(Vector2),
    /// Distributed pressure
    Pressure(f64),
    /// Body force (gravity, etc.)
    BodyForce(Vector2),
}

/// Element type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ElementType {
    /// 3-node triangle
    Triangle3,
    /// 4-node quadrilateral
    Quadrilateral4,
    /// 6-node triangle
    Triangle6,
    /// 8-node quadrilateral
    Quadrilateral8,
}

impl ElementType {
    /// Get number of nodes for this element type
    pub fn num_nodes(&self) -> usize {
        match self {
            ElementType::Triangle3 => 3,
            ElementType::Quadrilateral4 => 4,
            ElementType::Triangle6 => 6,
            ElementType::Quadrilateral8 => 8,
        }
    }

    /// Get number of DOFs per node
    pub fn dofs_per_node(&self) -> usize {
        2 // X and Y displacements
    }

    /// Total DOFs for this element
    pub fn total_dofs(&self) -> usize {
        self.num_nodes() * self.dofs_per_node()
    }
}

/// 2D Point
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point2D {
    pub x: f64,
    pub y: f64,
}

impl Point2D {
    /// Create new 2D point
    pub fn new(x: f64, y: f64) -> Self {
        Point2D { x, y }
    }

    /// Get as vector array
    pub fn as_vec(&self) -> Vector2 {
        [self.x, self.y]
    }

    /// Distance to another point
    pub fn distance_to(&self, other: &Point2D) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}

impl Add<Vector2> for Point2D {
    type Output = Point2D;

    fn add(self, v: Vector2) -> Point2D {
        Point2D::new(self.x + v[0], self.y + v[1])
    }
}

impl Sub<Vector2> for Point2D {
    type Output = Point2D;

    fn sub(self, v: Vector2) -> Point2D {
        Point2D::new(self.x - v[0], self.y - v[1])
    }
}

/// Vector operations
pub fn vec_add(a: Vector2, b: Vector2) -> Vector2 {
    [a[0] + b[0], a[1] + b[1]]
}

pub fn vec_sub(a: Vector2, b: Vector2) -> Vector2 {
    [a[0] - b[0], a[1] - b[1]]
}

pub fn vec_scale(v: Vector2, s: f64) -> Vector2 {
    [v[0] * s, v[1] * s]
}

pub fn vec_dot(a: Vector2, b: Vector2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

pub fn vec_cross(a: Vector2, b: Vector2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

pub fn vec_length(v: Vector2) -> f64 {
    (v[0] * v[0] + v[1] * v[1]).sqrt()
}

pub fn vec_normalize(v: Vector2) -> Vector2 {
    let len = vec_length(v);
    if len > 1e-15 {
        [v[0] / len, v[1] / len]
    } else {
        [0.0, 0.0]
    }
}

/// Matrix operations
pub fn mat_mult(a: Matrix2, b: Matrix2) -> Matrix2 {
    [
        [
            a[0][0] * b[0][0] + a[0][1] * b[1][0],
            a[0][0] * b[0][1] + a[0][1] * b[1][1],
        ],
        [
            a[1][0] * b[0][0] + a[1][1] * b[1][0],
            a[1][0] * b[0][1] + a[1][1] * b[1][1],
        ],
    ]
}

pub fn mat_vec_mult(m: Matrix2, v: Vector2) -> Vector2 {
    [m[0][0] * v[0] + m[0][1] * v[1], m[1][0] * v[0] + m[1][1] * v[1]]
}

pub fn mat_determinant(m: Matrix2) -> f64 {
    m[0][0] * m[1][1] - m[0][1] * m[1][0]
}

pub fn mat_inverse(m: Matrix2) -> Option<Matrix2> {
    let det = mat_determinant(m);
    if det.abs() < 1e-15 {
        return None;
    }
    Some([
        [m[1][1] / det, -m[0][1] / det],
        [-m[1][0] / det, m[0][0] / det],
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vec_operations() {
        let a = [1.0, 2.0];
        let b = [3.0, 4.0];

        assert_eq!(vec_add(a, b), [4.0, 6.0]);
        assert_eq!(vec_sub(a, b), [-2.0, -2.0]);
        assert_eq!(vec_scale(a, 2.0), [2.0, 4.0]);
        assert_eq!(vec_dot(a, b), 11.0);
    }

    #[test]
    fn test_point_operations() {
        let p1 = Point2D::new(0.0, 0.0);
        let p2 = Point2D::new(3.0, 4.0);
        assert_eq!(p1.distance_to(&p2), 5.0);
    }
}
