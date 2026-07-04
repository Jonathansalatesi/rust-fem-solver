//! Linear system solver

use crate::{FemError, Result};
use ndarray::{Array1, Array2};
use ndarray_linalg::Solve;

/// Linear solver using direct methods
pub struct LinearSolver;

impl LinearSolver {
    /// Solve linear system K*u = f using LU decomposition
    pub fn solve(k: &Array2<f64>, f: &Array1<f64>) -> Result<Array1<f64>> {
        if k.nrows() != k.ncols() {
            return Err(FemError::AssemblyError(
                "Stiffness matrix must be square".to_string(),
            ));
        }

        if k.nrows() != f.len() {
            return Err(FemError::AssemblyError(
                "Dimension mismatch between stiffness matrix and force vector".to_string(),
            ));
        }

        // Solve using ndarray's solve method
        k.solve_into(f.clone())
            .map_err(|e| FemError::ConvergenceError(format!("Solver failed: {}", e)))
    }
}
