//! Contact constraints

use crate::types::{NodeId, Vector2};

/// Contact constraint information
#[derive(Debug, Clone)]
pub struct ContactConstraint {
    /// Master node ID
    pub master_node: NodeId,
    /// Slave node ID
    pub slave_node: NodeId,
    /// Contact normal
    pub normal: Vector2,
    /// Penalty coefficient
    pub penalty: f64,
    /// Gap (negative if in contact)
    pub gap: f64,
}

impl ContactConstraint {
    /// Create new contact constraint
    pub fn new(
        master_node: NodeId,
        slave_node: NodeId,
        normal: Vector2,
        penalty: f64,
        gap: f64,
    ) -> Self {
        ContactConstraint {
            master_node,
            slave_node,
            normal,
            penalty,
            gap,
        }
    }

    /// Check if constraint is active (nodes in contact)
    pub fn is_active(&self) -> bool {
        self.gap < 0.0
    }

    /// Get contact force magnitude
    pub fn contact_force_magnitude(&self) -> f64 {
        if self.is_active() {
            self.penalty * (-self.gap)
        } else {
            0.0
        }
    }
}
