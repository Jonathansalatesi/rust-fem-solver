//! Linear elastic material model

use super::Material;
use crate::types::Matrix2;

/// Linear elastic material
#[derive(Debug, Clone, Copy)]
pub struct LinearElastic {
    /// Young's modulus (Pa)
    pub young_modulus: f64,
    /// Poisson's ratio
    pub poisson_ratio: f64,
    /// Density (kg/m³)
    pub density: f64,
}

impl LinearElastic {
    /// Create new linear elastic material
    pub fn new(young_modulus: f64, poisson_ratio: f64, density: f64) -> Self {
        LinearElastic {
            young_modulus,
            poisson_ratio,
            density,
        }
    }

    /// Get shear modulus
    pub fn shear_modulus(&self) -> f64 {
        self.young_modulus / (2.0 * (1.0 + self.poisson_ratio))
    }

    /// Get bulk modulus
    pub fn bulk_modulus(&self) -> f64 {
        self.young_modulus / (3.0 * (1.0 - 2.0 * self.poisson_ratio))
    }

    /// Get plane stress constitutive matrix (2D)
    pub fn plane_stress_matrix(&self) -> Matrix2 {
        let factor = self.young_modulus / (1.0 - self.poisson_ratio * self.poisson_ratio);
        [
            [factor, self.poisson_ratio * factor],
            [self.poisson_ratio * factor, factor],
        ]
    }

    /// Get plane strain constitutive matrix (2D)
    pub fn plane_strain_matrix(&self) -> Matrix2 {
        let factor = self.young_modulus / ((1.0 + self.poisson_ratio) * (1.0 - 2.0 * self.poisson_ratio));
        let lambda = self.poisson_ratio * factor;
        let mu = factor * (1.0 - 2.0 * self.poisson_ratio) / 2.0;
        [
            [lambda + 2.0 * mu, lambda],
            [lambda, lambda + 2.0 * mu],
        ]
    }
}

impl Material for LinearElastic {
    fn get_constitutive_matrix(&self, _thickness: f64) -> Matrix2 {
        self.plane_stress_matrix()
    }

    fn density(&self) -> f64 {
        self.density
    }

    fn clone_box(&self) -> Box<dyn Material> {
        Box::new(*self)
    }
}
