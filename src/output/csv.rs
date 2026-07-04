//! CSV format writer for spreadsheet analysis

use crate::{Mesh, Result};
use ndarray::Array1;
use std::fs::File;
use std::io::Write;

/// CSV format writer for tabular data export
pub struct CsvWriter;

impl CsvWriter {
    /// Write mesh and displacement field to CSV file
    pub fn write(filename: &str, mesh: &Mesh, displacement: &Array1<f64>) -> Result<()> {
        let mut file = File::create(filename)?;

        // Write header
        writeln!(file, "node_id,x,y,ux,uy")?;

        // Write node data
        for node in mesh.nodes() {
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

            writeln!(file, "{},{:.6e},{:.6e},{:.6e},{:.6e}",
                node.id(), coords.x, coords.y, ux, uy)?;
        }

        Ok(())
    }

    /// Write element data to CSV
    pub fn write_elements(filename: &str, mesh: &Mesh) -> Result<()> {
        let mut file = File::create(filename)?;

        // Write header
        writeln!(file, "element_id,type,node1,node2,node3,node4,node5,node6,node7,node8")?;

        // Write element data
        for elem in mesh.elements() {
            write!(file, "{},{:?}", elem.id(), elem.element_type())?;
            for &node_id in elem.node_ids() {
                write!(file, ",{}", node_id)?;
            }
            // Pad with empty values if element has fewer nodes
            let max_nodes = 8;
            for _ in elem.node_ids().len()..max_nodes {
                write!(file, ",")?;
            }
            writeln!(file)?;
        }

        Ok(())
    }
}
