//! Lagrange multiplier method for contact constraints
//!
//! Implements augmented Lagrangian approach for contact handling
//! This method is more accurate than penalty method but more complex

use crate::mesh::Mesh;
use crate::types::{NodeId, Vector2};
use crate::{Result, FemError};
use ndarray::{Array1, Array2};
use std::collections::HashMap;

/// Lagrange multiplier data structure
#[derive(Debug, Clone)]
pub struct LagrangeMultiplier {
    /// Slave node ID
    pub slave_node: NodeId,
    /// Master node ID
    pub master_node: NodeId,
    /// Normal direction
    pub normal: Vector2,
    /// Current multiplier value
    pub lambda_n: f64,
    /// Tangential multiplier (for friction)
    pub lambda_t: f64,
    /// Gap at contact point
    pub gap: f64,
    /// Is binding contact
    pub is_binding: bool,
}

impl LagrangeMultiplier {
    /// Create new Lagrange multiplier
    pub fn new(
        slave_node: NodeId,
        master_node: NodeId,
        normal: Vector2,
        is_binding: bool,
    ) -> Self {
        LagrangeMultiplier {
            slave_node,
            master_node,
            normal,
            lambda_n: 0.0,
            lambda_t: 0.0,
            gap: 0.0,
            is_binding,
        }
    }

    /// Check if constraint is active
    pub fn is_active(&self) -> bool {
        if self.is_binding {
            true  // Binding contact is always active
        } else {
            self.gap < 0.0 && self.lambda_n > -1e-10  // Normal contact active if penetrating
        }
    }

    /// Update gap value
    pub fn update_gap(&mut self, new_gap: f64) {
        self.gap = new_gap;
    }

    /// Get normal contact force
    pub fn normal_force(&self) -> f64 {
        -self.lambda_n
    }

    /// Get tangential contact force
    pub fn tangential_force(&self) -> f64 {
        -self.lambda_t
    }
}

/// Lagrange contact solver
pub struct LagrangeContactSolver;

impl LagrangeContactSolver {
    /// Build contact constraint matrix (used in augmented Lagrangian method)
    pub fn build_constraint_matrix(
        multipliers: &[LagrangeMultiplier],
        mesh: &Mesh,
    ) -> Result<Array2<f64>> {
        if multipliers.is_empty() {
            return Ok(Array2::zeros((0, mesh.num_dofs())));
        }

        let num_constraints = multipliers.len();
        let num_dofs = mesh.num_dofs();
        let mut c = Array2::zeros((num_constraints, num_dofs));

        for (i, mult) in multipliers.iter().enumerate() {
            let slave_dof = mult.slave_node * 2;
            let master_dof = mult.master_node * 2;

            if slave_dof < num_dofs {
                c[[i, slave_dof]] = mult.normal[0];
                c[[i, slave_dof + 1]] = mult.normal[1];
            }

            if master_dof < num_dofs {
                c[[i, master_dof]] = -mult.normal[0];
                c[[i, master_dof + 1]] = -mult.normal[1];
            }
        }

        Ok(c)
    }

    /// Build augmented Lagrangian system matrix
    pub fn build_augmented_matrix(
        k: &Array2<f64>,
        c: &Array2<f64>,
        penalty: f64,
    ) -> Array2<f64> {
        let n_dof = k.nrows();
        let n_const = c.nrows();

        // Build augmented system:
        // [K      C^T   ] [u]
        // [C   -1/ρ*I   ] [λ]

        let mut k_aug = Array2::zeros((n_dof + n_const, n_dof + n_const));

        // K block
        for i in 0..n_dof {
            for j in 0..n_dof {
                k_aug[[i, j]] = k[[i, j]];
            }
        }

        // C^T block
        for i in 0..n_dof {
            for j in 0..n_const {
                k_aug[[i, n_dof + j]] = c[[j, i]];
            }
        }

        // C block
        for i in 0..n_const {
            for j in 0..n_dof {
                k_aug[[n_dof + i, j]] = c[[i, j]];
            }
        }

        // -1/ρ*I block (penalty term in augmented system)
        for i in 0..n_const {
            k_aug[[n_dof + i, n_dof + i]] = -1.0 / penalty;
        }

        k_aug
    }

    /// Build augmented right-hand side vector
    pub fn build_augmented_rhs(
        f: &Array1<f64>,
        multipliers: &[LagrangeMultiplier],
    ) -> Array1<f64> {
        let n_dof = f.len();
        let n_const = multipliers.len();
        let mut f_aug = Array1::zeros(n_dof + n_const);

        // Copy original forces
        for i in 0..n_dof {
            f_aug[i] = f[i];
        }

        // Add constraint right-hand side (gap constraints)
        for (i, mult) in multipliers.iter().enumerate() {
            f_aug[n_dof + i] = -mult.gap;
        }

        f_aug
    }

    /// Update Lagrange multipliers based on solution
    pub fn update_multipliers(
        multipliers: &mut [LagrangeMultiplier],
        displacement: &Array1<f64>,
        mesh: &Mesh,
        rho: f64,  // Augmentation parameter
    ) -> Result<()> {
        for mult in multipliers.iter_mut() {
            // Get node positions
            let slave_node = mesh
                .get_node(mult.slave_node)
                .ok_or_else(|| FemError::ContactError("Slave node not found".to_string()))?;
            let master_node = mesh
                .get_node(mult.master_node)
                .ok_or_else(|| FemError::ContactError("Master node not found".to_string()))?;

            let slave_dof = mult.slave_node * 2;
            let master_dof = mult.master_node * 2;

            // Get displacements
            let u_slave = if slave_dof + 1 < displacement.len() {
                [displacement[slave_dof], displacement[slave_dof + 1]]
            } else {
                [0.0, 0.0]
            };

            let u_master = if master_dof + 1 < displacement.len() {
                [displacement[master_dof], displacement[master_dof + 1]]
            } else {
                [0.0, 0.0]
            };

            // Calculate gap
            let slave_pos = slave_node.coordinates();
            let master_pos = master_node.coordinates();

            let slave_deformed = [slave_pos.x + u_slave[0], slave_pos.y + u_slave[1]];
            let master_deformed = [master_pos.x + u_master[0], master_pos.y + u_master[1]];

            let gap_vec = [
                slave_deformed[0] - master_deformed[0],
                slave_deformed[1] - master_deformed[1],
            ];
            let gap = gap_vec[0] * mult.normal[0] + gap_vec[1] * mult.normal[1];
            mult.update_gap(gap);

            // Update Lagrange multiplier (augmented Lagrangian method)
            // λ^(k+1) = λ^(k) - ρ * g(u)
            let lambda_update = -rho * mult.gap;

            if mult.is_binding {
                // For binding contact: enforce equality constraint
                mult.lambda_n = lambda_update;
            } else {
                // For frictional contact: enforce inequality constraint (KKT conditions)
                // λ ≥ 0, g ≤ 0, λ*g = 0
                let new_lambda = mult.lambda_n + lambda_update;
                mult.lambda_n = new_lambda.max(0.0);  // Enforce λ ≥ 0
            }
        }

        Ok(())
    }

    /// Apply contact forces to displacement and force vectors
    pub fn apply_contact_forces(
        multipliers: &[LagrangeMultiplier],
        f_external: &mut Array1<f64>,
    ) -> Result<()> {
        for mult in multipliers {
            if mult.is_active() {
                let slave_dof = mult.slave_node * 2;
                let master_dof = mult.master_node * 2;

                // Contact force: f_c = λ * n
                let f_n = mult.lambda_n;
                let fx = f_n * mult.normal[0];
                let fy = f_n * mult.normal[1];

                // Add to slave node (penetrating force)
                if slave_dof + 1 < f_external.len() {
                    f_external[slave_dof] -= fx;
                    f_external[slave_dof + 1] -= fy;
                }

                // Add to master node (reaction force)
                if master_dof + 1 < f_external.len() {
                    f_external[master_dof] += fx;
                    f_external[master_dof + 1] += fy;
                }
            }
        }

        Ok(())
    }

    /// Apply binding constraint forces
    pub fn apply_binding_constraint(
        multipliers: &[LagrangeMultiplier],
        f: &mut Array1<f64>,
    ) -> Result<()> {
        for mult in multipliers {
            if mult.is_binding {
                let slave_dof = mult.slave_node * 2;
                let master_dof = mult.master_node * 2;

                // For binding: enforce relative displacement = 0
                // f_c = λ * [n, -n]^T
                let f_n = mult.lambda_n;
                let fx = f_n * mult.normal[0];
                let fy = f_n * mult.normal[1];

                // Constraint force on slave
                if slave_dof + 1 < f.len() {
                    f[slave_dof] -= fx;
                    f[slave_dof + 1] -= fy;
                }

                // Reaction force on master
                if master_dof + 1 < f.len() {
                    f[master_dof] += fx;
                    f[master_dof + 1] += fy;
                }
            }
        }

        Ok(())
    }
}
