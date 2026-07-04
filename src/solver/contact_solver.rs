//! Dedicated contact solver with method selection

use crate::Solver;
use crate::contact::{ContactBoundary, ContactMethod, LagrangeContactSolver, PenaltyContactSolver, detect_collision};
use crate::contact::penalty::PenaltyContactPoint;
use crate::Result;

/// Contact solver orchestrator
pub struct ContactSolver;

impl ContactSolver {
    /// Solve contact problem with selected method
    pub fn solve(
        solver: &mut Solver,
        contacts: &[ContactBoundary],
        max_iterations: usize,
        tolerance: f64,
    ) -> Result<f64> {
        let mut total_residual = 0.0;

        for contact in contacts {
            // Check which method to use
            match contact.contact_method {
                ContactMethod::Lagrange => {
                    total_residual += Self::solve_lagrange(solver, contact)?;
                }
                ContactMethod::Penalty { penalty_parameter } => {
                    total_residual += Self::solve_penalty(solver, contact, penalty_parameter)?;
                }
            }
        }

        Ok(total_residual)
    }

    /// Solve single contact using Lagrange method
    fn solve_lagrange(solver: &Solver, contact: &ContactBoundary) -> Result<f64> {
        let mut residual = 0.0;

        // Get node positions
        let mut master_nodes = Vec::new();
        let mut slave_nodes = Vec::new();

        for &node_id in &contact.master_surface {
            if let Some(node) = solver.mesh().get_node(node_id) {
                let disp = solver.get_displacement(node_id).unwrap_or([0.0, 0.0]);
                let coords = node.coordinates();
                let deformed = [coords.x + disp[0], coords.y + disp[1]];
                master_nodes.push((node_id, deformed));
            }
        }

        for &node_id in &contact.slave_surface {
            if let Some(node) = solver.mesh().get_node(node_id) {
                let disp = solver.get_displacement(node_id).unwrap_or([0.0, 0.0]);
                let coords = node.coordinates();
                let deformed = [coords.x + disp[0], coords.y + disp[1]];
                slave_nodes.push((node_id, deformed));
            }
        }

        // Detect collisions
        if let Some(collision) = detect_collision(&master_nodes, &slave_nodes, 1.0) {
            residual = collision.penetration.abs();
        }

        Ok(residual)
    }

    /// Solve single contact using penalty method
    fn solve_penalty(
        solver: &Solver,
        contact: &ContactBoundary,
        penalty: f64,
    ) -> Result<f64> {
        let mut residual = 0.0;

        // Get node positions
        let mut master_nodes = Vec::new();
        let mut slave_nodes = Vec::new();

        for &node_id in &contact.master_surface {
            if let Some(node) = solver.mesh().get_node(node_id) {
                let disp = solver.get_displacement(node_id).unwrap_or([0.0, 0.0]);
                let coords = node.coordinates();
                let deformed = [coords.x + disp[0], coords.y + disp[1]];
                master_nodes.push((node_id, deformed));
            }
        }

        for &node_id in &contact.slave_surface {
            if let Some(node) = solver.mesh().get_node(node_id) {
                let disp = solver.get_displacement(node_id).unwrap_or([0.0, 0.0]);
                let coords = node.coordinates();
                let deformed = [coords.x + disp[0], coords.y + disp[1]];
                slave_nodes.push((node_id, deformed));
            }
        }

        // Detect collisions
        if let Some(collision) = detect_collision(&master_nodes, &slave_nodes, 1.0) {
            residual = collision.penetration.abs();
            
            // Create penalty contact point
            let _penalty_contact = PenaltyContactPoint::new(
                slave_nodes[0].0,
                master_nodes[0].0,
                collision.normal,
                penalty,
                collision.penetration,
            );
        }

        Ok(residual)
    }
}
