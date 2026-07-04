//! XDMF format writer for scientific visualization

use crate::{Mesh, Result};
use ndarray::Array1;
use std::fs::File;
use std::io::Write;

/// XDMF (Extensible Data Model and Format) writer
pub struct XdmfWriter;

impl XdmfWriter {
    /// Write mesh and displacement field to XDMF file
    pub fn write(filename: &str, mesh: &Mesh, displacement: &Array1<f64>) -> Result<()> {
        let mut file = File::create(filename)?;

        // Write XDMF header
        writeln!(file, "<?xml version=\"1.0\"?>")?;
        writeln!(file, "<Xdmf Version=\"3.0\">")?;
        writeln!(file, "  <Domain>")?;
        writeln!(file, "    <Grid GridType=\"Uniform\">")?;
        writeln!(file, "      <Topology TopologyType=\"Mixed\" NumberOfElements=\"{}\">")?;
        writeln!(file, "        <DataItem Format=\"HDF\" Dimensions=\"{}\" NumberType=\"Int\">")?;
        writeln!(file, "          connectivity.h5:/connectivity")?;
        writeln!(file, "        </DataItem>")?;
        writeln!(file, "      </Topology>")?;

        // Write geometry (node coordinates)
        writeln!(file, "      <Geometry GeometryType=\"XYZ\">")?;
        writeln!(file, "        <DataItem Format=\"HDF\" Dimensions=\"{} 3\" NumberType=\"Double\">")?;
        writeln!(file, "          coordinates.h5:/coordinates")?;
        writeln!(file, "        </DataItem>")?;
        writeln!(file, "      </Geometry>")?;

        // Write displacement field
        writeln!(file, "      <Attribute Name=\"Displacement\" AttributeType=\"Vector\" Center=\"Node\">")?;
        writeln!(file, "        <DataItem Format=\"HDF\" Dimensions=\"{} 3\" NumberType=\"Double\">")?;
        writeln!(file, "          displacement.h5:/displacement")?;
        writeln!(file, "        </DataItem>")?;
        writeln!(file, "      </Attribute>")?;

        writeln!(file, "    </Grid>")?;
        writeln!(file, "  </Domain>")?;
        writeln!(file, "</Xdmf>")?;

        Ok(())
    }
}
