//! Solver module for FEM analysis
//!
//! Main solver for static and dynamic analyses with contact handling
//! Supports both Lagrange multiplier (default) and penalty methods

mod linear;
mod assembly;
mod contact_solver;

pub use linear::LinearSolver;
pub use assembly::MatrixAssembler;
pub use contact_solver::ContactSolver;

use crate::mesh::Mesh;
use crate::material::{Material, LinearElastic};
use crate::contact::{ContactBoundary, ContactMethod, LagrangeMultiplier, LagrangeContactSolver};
use crate::output::{VtkWriter, CsvWriter, TecplotWriter};
use crate::types::{NodeId, Vector2, BoundaryCondition};
use crate::Result;
use ndarray::{Array1, Array2};
use std::collections::HashMap;

/// Main FEM solver
pub struct Solver {
    mesh: Mesh,
    material: Box<dyn Material>,
    displacement: Array1<f64>,
    force: Array1<f64>,
    boundary_conditions: HashMap<NodeId, BoundaryCondition>,
    point_loads: HashMap<NodeId, Vector2>,
    contact_boundaries: Vec<ContactBoundary>,
    thickness: f64,
    /// Lagrange multipliers for active contacts
    lagrange_multipliers: Vec<LagrangeMultiplier>,
}

impl Solver {
    /// Create new solver with mesh
    pub fn new(mesh: Mesh) -> Self {
        let num_dofs = mesh.num_dofs();
        let material = Box::new(LinearElastic::new(210e9, 0.3, 7850.0));

        Solver {
            mesh,
            material,
            displacement: Array1::zeros(num_dofs),
            force: Array1::zeros(num_dofs),
            boundary_conditions: HashMap::new(),
            point_loads: HashMap::new(),
            contact_boundaries: Vec::new(),
            thickness: 1.0,
            lagrange_multipliers: Vec::new(),
        }
    }

    /// Set material properties
    pub fn set_material(&mut self, material: Box<dyn Material>) {
        self.material = material;
    }

    /// Set material from LinearElastic
    pub fn set_linear_elastic_material(
        &mut self,
        young_modulus: f64,
        poisson_ratio: f64,
        density: f64,
    ) {
        self.material = Box::new(LinearElastic::new(young_modulus, poisson_ratio, density));
    }

    /// Set element thickness (for 2D plane stress/strain)
    pub fn set_thickness(&mut self, thickness: f64) {
        self.thickness = thickness;
    }

    /// Set fixed boundary condition (fully constrained node)
    pub fn set_fixed_boundary(&mut self, node_ids: Vec<NodeId>) {
        for id in node_ids {
            self.boundary_conditions.insert(id, BoundaryCondition::Fixed);
        }
    }

    /// Set fixed in X direction
    pub fn set_fixed_x(&mut self, node_ids: Vec<NodeId>) {
        for id in node_ids {
            self.boundary_conditions.insert(id, BoundaryCondition::FixedX);
        }
    }

    /// Set fixed in Y direction
    pub fn set_fixed_y(&mut self, node_ids: Vec<NodeId>) {
        for id in node_ids {
            self.boundary_conditions.insert(id, BoundaryCondition::FixedY);
        }
    }

    /// Add point load (force) at node
    pub fn add_point_load(&mut self, node_id: NodeId, force: Vector2) {
        self.point_loads.insert(node_id, force);
    }

    /// Add contact boundary condition
    pub fn add_contact_boundary(&mut self, contact: ContactBoundary) {
        self.contact_boundaries.push(contact);
    }

    /// Get mesh reference
    pub fn mesh(&self) -> &Mesh {
        &self.mesh
    }

    /// Get displacement solution
    pub fn displacement(&self) -> &Array1<f64> {
        &self.displacement
    }

    /// Get displacement at node
    pub fn get_displacement(&self, node_id: NodeId) -> Option<Vector2> {
        let idx = node_id * 2;
        if idx + 1 < self.displacement.len() {
            Some([self.displacement[idx], self.displacement[idx + 1]])
        } else {
            None
        }
    }

    /// Get Lagrange multipliers
    pub fn lagrange_multipliers(&self) -> &[LagrangeMultiplier] {
        &self.lagrange_multipliers
    }

    /// Assemble global stiffness matrix
    fn assemble_stiffness_matrix(&self) -> Array2<f64> {
        MatrixAssembler::assemble_stiffness(&self.mesh, &self.material, self.thickness)
    }

    /// Assemble global force vector
    fn assemble_force_vector(&self) -> Array1<f64> {
        let mut force = Array1::zeros(self.mesh.num_dofs());

        // Add point loads
        for (&node_id, &load) in &self.point_loads {
            let idx = node_id * 2;
            if idx + 1 < force.len() {
                force[idx] += load[0];
                force[idx + 1] += load[1];
            }
        }

        force
    }

    /// Apply boundary conditions to system
    fn apply_boundary_conditions(&self, k: &mut Array2<f64>, f: &mut Array1<f64>) {
        let penalty_factor = 1e12;

        for (&node_id, &bc) in &self.boundary_conditions {
            let idx = node_id * 2;

            match bc {
                BoundaryCondition::Fixed => {
                    if idx < k.nrows() && idx + 1 < k.nrows() {
                        k[[idx, idx]] *= penalty_factor;
                        k[[idx + 1, idx + 1]] *= penalty_factor;
                        f[idx] = 0.0;
                        f[idx + 1] = 0.0;
                    }
                }
                BoundaryCondition::FixedX => {
                    if idx < k.nrows() {
                        k[[idx, idx]] *= penalty_factor;
                        f[idx] = 0.0;
                    }
                }
                BoundaryCondition::FixedY => {
                    if idx + 1 < k.nrows() {
                        k[[idx + 1, idx + 1]] *= penalty_factor;
                        f[idx + 1] = 0.0;
                    }
                }
                _ => {}
            }
        }
    }

    /// Solve the system (basic linear solve)
    pub fn solve(&mut self) -> Result<()> {
        // Assemble system
        let mut k = self.assemble_stiffness_matrix();
        let mut f = self.assemble_force_vector();

        // Apply boundary conditions
        self.apply_boundary_conditions(&mut k, &mut f);

        // Solve linear system
        self.displacement = LinearSolver::solve(&k, &f)?;

        Ok(())
    }

    /// Solve with contact using Lagrange multiplier method (default, more accurate)
    pub fn solve_with_contact(&mut self, max_iterations: usize, tolerance: f64) -> Result<()> {
        println!("\n=== Contact Solver (Lagrange Multiplier Method) ===");
        println!("Maximum iterations: {}", max_iterations);
        println!("Convergence tolerance: {:.2e}", tolerance);
        
        // Augmentation parameter (updated at each iteration)
        let mut rho = 1.0;

        for iteration in 0..max_iterations {
            // Assemble base system
            let mut k = self.assemble_stiffness_matrix();
            let mut f = self.assemble_force_vector();

            // Apply boundary conditions
            self.apply_boundary_conditions(&mut k, &mut f);

            // Initialize Lagrange multipliers on first iteration
            if iteration == 0 {
                self.initialize_lagrange_multipliers()?;
            }

            // Apply contact forces via Lagrange multipliers
            LagrangeContactSolver::apply_binding_constraint(&self.lagrange_multipliers, &mut f)?;

            // Solve linear system
            self.displacement = LinearSolver::solve(&k, &f)?;

            // Update Lagrange multipliers and check convergence
            LagrangeContactSolver::update_multipliers(
                &mut self.lagrange_multipliers,
                &self.displacement,
                &self.mesh,
                rho,
            )?;

            // Calculate residual
            let residual = self.calculate_contact_residual();
            
            println!("Iteration {}: residual = {:.6e}", iteration + 1, residual);

            if residual < tolerance {
                println!("✓ Contact solver converged at iteration {}\n", iteration + 1);
                break;
            }

            // Increase augmentation parameter
            rho *= 1.2;
        }

        Ok(())
    }

    /// Initialize Lagrange multipliers for all contact boundaries
    fn initialize_lagrange_multipliers(&mut self) -> Result<()> {
        self.lagrange_multipliers.clear();

        for contact in &self.contact_boundaries {
            let is_binding = contact.is_binding();
            
            // Create Lagrange multiplier for each contact pair
            for &slave_node in &contact.slave_surface {
                for &master_node in &contact.master_surface {
                    let normal = [0.0, 1.0];  // Default normal (upward)
                    let mult = LagrangeMultiplier::new(slave_node, master_node, normal, is_binding);
                    self.lagrange_multipliers.push(mult);
                }
            }
        }

        Ok(())
    }

    /// Calculate contact constraint residual
    fn calculate_contact_residual(&self) -> f64 {
        let mut residual = 0.0;

        for mult in &self.lagrange_multipliers {
            if mult.is_active() {
                // Residual is gap constraint violation
                residual += mult.gap.abs();
            }
        }

        residual
    }

    /// Export solution to VTK format (ParaView)
    pub fn export_vtk(&self, filename: &str) -> Result<()> {
        VtkWriter::write(filename, &self.mesh, &self.displacement)
    }

    /// Export solution to CSV format
    pub fn export_csv(&self, filename: &str) -> Result<()> {
        CsvWriter::write(filename, &self.mesh, &self.displacement)
    }

    /// Export solution to Tecplot format
    pub fn export_tecplot(&self, filename: &str) -> Result<()> {
        TecplotWriter::write(filename, &self.mesh, &self.displacement)
    }

    /// Export elements to CSV
    pub fn export_elements_csv(&self, filename: &str) -> Result<()> {
        CsvWriter::write_elements(filename, &self.mesh)
    }
}
