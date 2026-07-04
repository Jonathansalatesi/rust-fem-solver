//! Penalty method for contact constraints
//!
//! Simpler but less accurate than Lagrange multiplier method

use crate::types::{NodeId, Vector2};
use crate::{Result, Mesh};
use ndarray::{Array1, Array2};

/// Penalty contact constraint
#[derive(Debug, Clone)]
pub struct PenaltyContactPoint {
    /// Slave node ID
    pub slave_node: NodeId,
    /// Master node ID
    pub master_node: NodeId,
    /// Contact normal
    pub normal: Vector2,
    /// Penalty coefficient
    pub penalty: f64,
    /// Gap (negative if in contact)
    pub gap: f64,
}

impl PenaltyContactPoint {
    /// Create new penalty contact point
    pub fn new(
        slave_node: NodeId,
        master_node: NodeId,
        normal: Vector2,
        penalty: f64,
        gap: f64,
    ) -> Self {
        PenaltyContactPoint {
            slave_node,
            master_node,
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

/// Penalty method solver for contact
pub struct PenaltyContactSolver;

impl PenaltyContactSolver {
    /// Add penalty stiffness to global matrix
    pub fn add_penalty_stiffness(
        k: &mut Array2<f64>,
        contacts: &[PenaltyContactPoint],
    ) -> Result<()> {
        for contact in contacts {
            if contact.is_active() {
                let slave_dof = contact.slave_node * 2;
                let master_dof = contact.master_node * 2;
                let penalty_factor = contact.penalty;
                let nx = contact.normal[0];
                let ny = contact.normal[1];

                // Add penalty stiffness matrix to both slave and master
                if slave_dof + 1 < k.nrows() {
                    k[[slave_dof, slave_dof]] += penalty_factor * nx * nx;
                    k[[slave_dof, slave_dof + 1]] += penalty_factor * nx * ny;
                    k[[slave_dof + 1, slave_dof]] += penalty_factor * ny * nx;
                    k[[slave_dof + 1, slave_dof + 1]] += penalty_factor * ny * ny;
                }

                if master_dof + 1 < k.nrows() {
                    k[[master_dof, master_dof]] += penalty_factor * nx * nx;
                    k[[master_dof, master_dof + 1]] += penalty_factor * nx * ny;
                    k[[master_dof + 1, master_dof]] += penalty_factor * ny * nx;
                    k[[master_dof + 1, master_dof + 1]] += penalty_factor * ny * ny;
                }
            }
        }

        Ok(())
    }

    /// Add penalty forces to force vector
    pub fn add_penalty_forces(
        f: &mut Array1<f64>,
        contacts: &[PenaltyContactPoint],
    ) -> Result<()> {
        for contact in contacts {
            if contact.is_active() {
                let slave_dof = contact.slave_node * 2;
                let master_dof = contact.master_node * 2;
                let force_mag = contact.contact_force_magnitude();
                let fx = force_mag * contact.normal[0];
                let fy = force_mag * contact.normal[1];

                // Add force to slave node (pushing outward)
                if slave_dof + 1 < f.len() {
                    f[slave_dof] += fx;
                    f[slave_dof + 1] += fy;
                }

                // Add reaction force to master node
                if master_dof + 1 < f.len() {
                    f[master_dof] -= fx;
                    f[master_dof + 1] -= fy;
                }
            }
        }

        Ok(())
    }
}
