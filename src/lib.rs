//! Rust FEM Solver Library
//!
//! A high-performance 2D Finite Element Solver for collision analysis and contact mechanics.
//!
//! # Features
//!
//! - 2D rigid body collision detection and solving
//! - Contact type support (frictional and binding contact)
//! - Abaqus mesh file import
//! - Sparse matrix linear solver
//! - Multiple post-processing formats (VTK, XDMF, HDF5, CSV, Tecplot)

pub mod mesh;
pub mod solver;
pub mod material;
pub mod contact;
pub mod output;
pub mod types;

pub use mesh::Mesh;
pub use solver::Solver;
pub use material::{Material, LinearElastic};
pub use contact::{ContactType, ContactBoundary};
pub use types::{Vector2, Matrix2};

/// Result type for FEM solver operations
pub type Result<T> = std::result::Result<T, FemError>;

/// Error types for FEM solver
#[derive(Debug)]
pub enum FemError {
    /// File I/O errors
    IoError(std::io::Error),
    /// Parsing errors
    ParseError(String),
    /// Solver convergence issues
    ConvergenceError(String),
    /// Invalid input
    InvalidInput(String),
    /// Matrix assembly errors
    AssemblyError(String),
    /// Contact analysis errors
    ContactError(String),
}

impl std::fmt::Display for FemError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            FemError::IoError(e) => write!(f, "IO error: {}", e),
            FemError::ParseError(e) => write!(f, "Parse error: {}", e),
            FemError::ConvergenceError(e) => write!(f, "Convergence error: {}", e),
            FemError::InvalidInput(e) => write!(f, "Invalid input: {}", e),
            FemError::AssemblyError(e) => write!(f, "Assembly error: {}", e),
            FemError::ContactError(e) => write!(f, "Contact error: {}", e),
        }
    }
}

impl std::error::Error for FemError {}

impl From<std::io::Error> for FemError {
    fn from(err: std::io::Error) -> Self {
        FemError::IoError(err)
    }
}