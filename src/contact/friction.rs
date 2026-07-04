//! Friction models

/// Friction model trait
pub trait FrictionModel: Send + Sync {
    /// Calculate friction force given normal force and relative velocity
    fn friction_force(&self, normal_force: f64, relative_velocity: f64) -> f64;
}

/// Coulomb friction model
#[derive(Debug, Clone, Copy)]
pub struct CoulombFriction {
    /// Coefficient of friction
    pub coefficient: f64,
}

impl CoulombFriction {
    /// Create new Coulomb friction model
    pub fn new(coefficient: f64) -> Self {
        CoulombFriction { coefficient }
    }
}

impl FrictionModel for CoulombFriction {
    fn friction_force(&self, normal_force: f64, relative_velocity: f64) -> f64 {
        if relative_velocity.abs() < 1e-15 {
            0.0
        } else {
            self.coefficient * normal_force.abs() * relative_velocity.signum()
        }
    }
}

/// Viscous friction model
#[derive(Debug, Clone, Copy)]
pub struct ViscousFriction {
    /// Damping coefficient
    pub damping: f64,
}

impl ViscousFriction {
    /// Create new viscous friction model
    pub fn new(damping: f64) -> Self {
        ViscousFriction { damping }
    }
}

impl FrictionModel for ViscousFriction {
    fn friction_force(&self, _normal_force: f64, relative_velocity: f64) -> f64 {
        -self.damping * relative_velocity
    }
}
