//! Tecplot format writer

use crate::{Mesh, Result};
use ndarray::Array1;
use std::fs::File;
use std::io::Write;

/// Tecplot 360 compatible output writer
pub struct TecplotWriter;

impl TecplotWriter {
    /// Write mesh and displacement field to Tecplot format
    pub fn write(filename: &str, mesh: &Mesh, displacement: &Array1<f64>) -> Result<()> {
        let mut file = File::create(filename)?;

        let nodes = mesh.nodes();
        let elements = mesh.elements();

        // Write Tecplot header
        writeln!(file, "TITLE = \"FEM Solver Results\"")?;
        writeln!(file, "VARIABLES = \"X\" \"Y\" \"Ux\" \"Uy\"")?;

        // Determine element type for Tecplot
        let first_elem = elements.first();
        let elem_type = if let Some(elem) = first_elem {
            match elem.element_type() {
                crate::types::ElementType::Triangle3 => "TRIANGLE",
                crate::types::ElementType::Quadrilateral4 => "QUADRILATERAL",
                crate::types::ElementType::Triangle6 => "TRIANGLE",
                crate::types::ElementType::Quadrilateral8 => "QUADRILATERAL",
            }
        } else {
            "TRIANGLE"
        };

        writeln!(file, "ZONE T=\"Solution\" N={}  E={}  ET={}  F=POINT",
            nodes.len(), elements.len(), elem_type)?;

        // Write node data
        for node in nodes {
            let coords = node.coordinates();
            let disp_idx = node.id() * 2;
            let ux = if disp_idx < displacement.len() {
                displacement[disp_idx]
            } else {
                0.0
            };
            let uy = if disp_idx + 1 < displacement.len() {
                displacement[disp_idx + 1]
            } else {
                0.0
            };

            writeln!(file, "{:.6e} {:.6e} {:.6e} {:.6e}",
                coords.x, coords.y, ux, uy)?;
        }

        // Write connectivity
        for elem in elements {
            for &node_id in elem.node_ids() {
                write!(file, "{} ", node_id + 1)?;  // Tecplot uses 1-based indexing
            }
            writeln!(file)?;
        }

        Ok(())
    }
}
