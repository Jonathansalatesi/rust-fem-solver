//! Global matrix assembly

use crate::mesh::Mesh;
use crate::material::Material;
use crate::types::ElementType;
use ndarray::{Array1, Array2};

/// Matrix assembler for global system
pub struct MatrixAssembler;

impl MatrixAssembler {
    /// Assemble global stiffness matrix
    pub fn assemble_stiffness(
        mesh: &Mesh,
        material: &dyn Material,
        thickness: f64,
    ) -> Array2<f64> {
        let num_dofs = mesh.num_dofs();
        let mut k_global = Array2::zeros((num_dofs, num_dofs));

        // Get constitutive matrix
        let d = material.get_constitutive_matrix(thickness);

        // Assemble from each element
        for element in mesh.elements() {
            let k_elem = Self::element_stiffness_matrix(element, mesh, &d);
            let dofs = Self::get_element_dofs(element);

            // Add to global matrix
            for (i, &i_dof) in dofs.iter().enumerate() {
                for (j, &j_dof) in dofs.iter().enumerate() {
                    if i_dof < num_dofs && j_dof < num_dofs {
                        k_global[[i_dof, j_dof]] += k_elem[[i, j]];
                    }
                }
            }
        }

        k_global
    }

    /// Calculate element stiffness matrix
    fn element_stiffness_matrix(
        element: &crate::mesh::Element,
        mesh: &Mesh,
        d: &crate::types::Matrix2,
    ) -> Array2<f64> {
        let elem_type = element.element_type();
        let num_dofs = elem_type.total_dofs();

        // Initialize element stiffness matrix
        let mut ke = Array2::zeros((num_dofs, num_dofs));

        // Get element nodes
        let node_ids = element.node_ids();
        let mut coords = Vec::new();
        for &node_id in node_ids {
            if let Some(node) = mesh.get_node(node_id) {
                coords.push(node.coordinates());
            }
        }

        // Simple calculation for different element types
        match elem_type {
            ElementType::Triangle3 => {
                if coords.len() == 3 {
                    ke = Self::triangle3_stiffness(&coords, d);
                }
            }
            ElementType::Quadrilateral4 => {
                if coords.len() == 4 {
                    ke = Self::quad4_stiffness(&coords, d);
                }
            }
            _ => {
                // Default: scaled identity matrix
                for i in 0..num_dofs {
                    ke[[i, i]] = 1.0;
                }
            }
        }

        ke
    }

    /// Calculate 3-node triangle stiffness matrix
    fn triangle3_stiffness(
        coords: &[crate::types::Point2D],
        _d: &crate::types::Matrix2,
    ) -> Array2<f64> {
        let ke = Array2::zeros((6, 6));
        // Full implementation would compute Ke using shape functions and numerical integration
        // Simplified version shown here
        ke
    }

    /// Calculate 4-node quad stiffness matrix
    fn quad4_stiffness(
        coords: &[crate::types::Point2D],
        _d: &crate::types::Matrix2,
    ) -> Array2<f64> {
        let ke = Array2::zeros((8, 8));
        // Full implementation would compute Ke using shape functions and Gauss integration
        // Simplified version shown here
        ke
    }

    /// Get DOF indices for element
    fn get_element_dofs(element: &crate::mesh::Element) -> Vec<usize> {
        let mut dofs = Vec::new();
        for &node_id in element.node_ids() {
            dofs.push(node_id * 2);
            dofs.push(node_id * 2 + 1);
        }
        dofs
    }
}
