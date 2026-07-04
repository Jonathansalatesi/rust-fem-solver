//! Output module for post-processing results
//!
//! Supports multiple output formats: VTK, XDMF, HDF5, CSV, Tecplot

pub mod vtk;
pub mod xdmf;
pub mod hdf5;
pub mod csv;
pub mod tecplot;

pub use vtk::VtkWriter;
pub use xdmf::XdmfWriter;
pub use hdf5::Hdf5Writer;
pub use csv::CsvWriter;
pub use tecplot::TecplotWriter;

use crate::{Mesh, Result};
use ndarray::Array1;

/// Trait for output writers
pub trait OutputWriter: Send + Sync {
    /// Write mesh and solution data to file
    fn write(&self, filename: &str, mesh: &Mesh, displacement: &Array1<f64>) -> Result<()>;

    /// Write stress data
    fn write_stress(&self, filename: &str, mesh: &Mesh, stress: &Array1<f64>) -> Result<()> {
        Ok(())
    }

    /// Write strain data
    fn write_strain(&self, filename: &str, mesh: &Mesh, strain: &Array1<f64>) -> Result<()> {
        Ok(())
    }
}
