//! Contact mechanics module
//!
//! Handles contact and collision detection between bodies
//! Supports both penalty method and Lagrange multiplier method

mod friction;
mod constraint;
mod lagrange;
mod penalty;

pub use friction::FrictionModel;
pub use constraint::ContactConstraint;
pub use lagrange::LagrangeContactSolver;
pub use penalty::PenaltyContactSolver;

use crate::types::{NodeId, Vector2};

/// Contact solving method
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ContactMethod {
    /// Lagrange multiplier method (default, more accurate)
    Lagrange,
    /// Penalty method (simpler, less accurate)
    Penalty { penalty_parameter: f64 },
}

impl Default for ContactMethod {
    fn default() -> Self {
        ContactMethod::Lagrange
    }
}

/// Contact type definition
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ContactType {
    /// Binding contact (no slip allowed, rigid connection)
    Binding,
    /// Frictional contact
    Frictional { friction_coefficient: f64 },
}

/// Contact boundary definition
#[derive(Debug, Clone)]
pub struct ContactBoundary {
    /// Master surface nodes
    pub master_surface: Vec<NodeId>,
    /// Slave surface nodes
    pub slave_surface: Vec<NodeId>,
    /// Contact type
    pub contact_type: ContactType,
    /// Contact solving method
    pub contact_method: ContactMethod,
}

impl ContactBoundary {
    /// Create new contact boundary with Lagrange method (default)
    pub fn new(
        master_surface: Vec<NodeId>,
        slave_surface: Vec<NodeId>,
        contact_type: ContactType,
    ) -> Self {
        ContactBoundary {
            master_surface,
            slave_surface,
            contact_type,
            contact_method: ContactMethod::Lagrange,
        }
    }

    /// Create new contact boundary with custom method
    pub fn with_method(
        master_surface: Vec<NodeId>,
        slave_surface: Vec<NodeId>,
        contact_type: ContactType,
        contact_method: ContactMethod,
    ) -> Self {
        ContactBoundary {
            master_surface,
            slave_surface,
            contact_type,
            contact_method,
        }
    }

    /// Check if contact type is binding
    pub fn is_binding(&self) -> bool {
        matches!(self.contact_type, ContactType::Binding)
    }

    /// Get friction coefficient if contact is frictional
    pub fn friction_coefficient(&self) -> Option<f64> {
        match self.contact_type {
            ContactType::Frictional { friction_coefficient } => Some(friction_coefficient),
            _ => None,
        }
    }

    /// Check if using Lagrange method
    pub fn is_lagrange_method(&self) -> bool {
        matches!(self.contact_method, ContactMethod::Lagrange)
    }
}

/// Collision detection result
#[derive(Debug, Clone, Copy)]
pub struct CollisionInfo {
    /// Normal vector at collision point
    pub normal: Vector2,
    /// Penetration depth (negative if separated)
    pub penetration: f64,
    /// Contact point on master surface
    pub contact_point_master: Vector2,
    /// Contact point on slave surface
    pub contact_point_slave: Vector2,
}

/// Detect collision between two surfaces using penalty method
pub fn detect_collision(
    master_nodes: &[(NodeId, Vector2)],
    slave_nodes: &[(NodeId, Vector2)],
    tolerance: f64,
) -> Option<CollisionInfo> {
    // Simple point-to-line collision detection
    // This is a basic implementation; more sophisticated algorithms exist

    if master_nodes.len() < 2 || slave_nodes.is_empty() {
        return None;
    }

    let mut min_distance = f64::INFINITY;
    let mut collision_info = None;

    // For each slave node, find closest point on master surface
    for (_, slave_pt) in slave_nodes {
        // Check against each master surface segment
        for i in 0..master_nodes.len() - 1 {
            let (_, p1) = master_nodes[i];
            let (_, p2) = master_nodes[i + 1];

            if let Some(info) = point_to_segment_distance(*slave_pt, p1, p2, tolerance) {
                if info.penetration < min_distance {
                    min_distance = info.penetration;
                    collision_info = Some(info);
                }
            }
        }
    }

    collision_info
}

/// Calculate distance from point to line segment
fn point_to_segment_distance(
    point: Vector2,
    seg_start: Vector2,
    seg_end: Vector2,
    tolerance: f64,
) -> Option<CollisionInfo> {
    let dx = seg_end[0] - seg_start[0];
    let dy = seg_end[1] - seg_start[1];
    let seg_len_sq = dx * dx + dy * dy;

    if seg_len_sq < 1e-15 {
        return None;
    }

    let px = point[0] - seg_start[0];
    let py = point[1] - seg_start[1];

    let t = ((px * dx + py * dy) / seg_len_sq).max(0.0).min(1.0);
    let closest_x = seg_start[0] + t * dx;
    let closest_y = seg_start[1] + t * dy;

    let dist_x = point[0] - closest_x;
    let dist_y = point[1] - closest_y;
    let distance = (dist_x * dist_x + dist_y * dist_y).sqrt();

    if distance < tolerance {
        let normal = if distance > 1e-15 {
            [dist_x / distance, dist_y / distance]
        } else {
            [-dy / (seg_len_sq.sqrt()), dx / (seg_len_sq.sqrt())]
        };

        Some(CollisionInfo {
            normal,
            penetration: -distance,
            contact_point_master: [closest_x, closest_y],
            contact_point_slave: point,
        })
    } else {
        None
    }
}
