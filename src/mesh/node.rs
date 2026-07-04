//! Node definition for FEM mesh

use crate::types::{NodeId, Point2D, Vector2};

/// Node in the FEM mesh
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Node {
    /// Unique node ID
    id: NodeId,
    /// Coordinates of the node
    coordinates: Point2D,
}

impl Node {
    /// Create a new node
    pub fn new(id: NodeId, coordinates: Point2D) -> Self {
        Node { id, coordinates }
    }

    /// Get node ID
    pub fn id(&self) -> NodeId {
        self.id
    }

    /// Get node coordinates
    pub fn coordinates(&self) -> Point2D {
        self.coordinates
    }

    /// Get coordinates as vector
    pub fn as_vec(&self) -> Vector2 {
        self.coordinates.as_vec()
    }
}
