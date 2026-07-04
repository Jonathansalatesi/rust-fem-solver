//! Material module for FEM solver
//!
//! Defines material properties and models

mod elastic;

pub use elastic::LinearElastic;

use crate::types::Matrix2;

/// Trait for material models
pub trait Material: Send + Sync {
    /// Get constitutive matrix (stiffness matrix for elastic materials)
    fn get_constitutive_matrix(&self, thickness: f64) -> Matrix2;

    /// Get density
    fn density(&self) -> f64;

    /// Clone as trait object
    fn clone_box(&self) -> Box<dyn Material>;
}

impl Clone for Box<dyn Material> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}
