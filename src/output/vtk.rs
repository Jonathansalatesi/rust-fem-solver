//! VTK/ParaView output format writer

use crate::{Mesh, Result, FemError};
use ndarray::Array1;
use std::fs::File;
use std::io::Write;

/// VTK format writer for ParaView visualization
pub struct VtkWriter;

impl VtkWriter {
    /// Write mesh and displacement field to VTK file
    pub fn write(filename: &str, mesh: &Mesh, displacement: &Array1<f64>) -> Result<()> {
        let mut file = File::create(filename)?;

        // Write VTK header
        writeln!(file, "# vtk DataFile Version 3.0")?;
        writeln!(file, "FEM Solver Results")?;
        writeln!(file, "ASCII")?;
        writeln!(file, "DATASET UNSTRUCTURED_GRID")?;

        let nodes = mesh.nodes();
        writeln!(file, "POINTS {} float", nodes.len())?;

        // Write node coordinates
        for node in nodes {
            let coords = node.coordinates();
            let disp_idx = node.id() * 2;
            let x = if disp_idx < displacement.len() {
                coords.x + displacement[disp_idx]
            } else {
                coords.x
            };
            let y = if disp_idx + 1 < displacement.len() {
                coords.y + displacement[disp_idx + 1]
            } else {
                coords.y
            };
            writeln!(file, "{:.6e} {:.6e} 0.0", x, y)?;
        }

        // Write elements
        let elements = mesh.elements();
        let mut total_connectivity = 0;
        for elem in elements {
            total_connectivity += elem.node_ids().len() + 1;
        }

        writeln!(file, "CELLS {} {}", elements.len(), total_connectivity)?;

        for elem in elements {
            let node_ids = elem.node_ids();
            write!(file, "{}" , node_ids.len())?;
            for &node_id in node_ids {
                write!(file, " {}", node_id)?;
            }
            writeln!(file)?;
        }

        // Write element types
        writeln!(file, "CELL_TYPES {}", elements.len())?;
        for elem in elements {
            let vtk_type = match elem.element_type() {
                crate::types::ElementType::Triangle3 => 5,
                crate::types::ElementType::Quadrilateral4 => 9,
                crate::types::ElementType::Triangle6 => 22,
                crate::types::ElementType::Quadrilateral8 => 23,
            };
            writeln!(file, "{}", vtk_type)?;
        }

        // Write point data (displacement field)
        writeln!(file, "POINT_DATA {}", nodes.len())?;
        writeln!(file, "VECTORS displacement float")?;

        for node in nodes {
            let disp_idx = node.id() * 2;
            let dx = if disp_idx < displacement.len() {
                displacement[disp_idx]
            } else {
                0.0
            };
            let dy = if disp_idx + 1 < displacement.len() {
                displacement[disp_idx + 1]
            } else {
                0.0
            };
            writeln!(file, "{:.6e} {:.6e} 0.0", dx, dy)?;
        }

        Ok(())
    }
}
