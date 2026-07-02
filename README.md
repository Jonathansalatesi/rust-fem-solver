# Rust FEM Solver

A high-performance 2D Finite Element Solver written in Rust for collision analysis and contact mechanics.

## Features

### Core Functionality
- **2D Rigid Body Collision Detection & Solving** - Efficient collision detection and response
- **Contact Types** - Support for both frictional and binding contact
- **Abaqus Mesh Import** - Read `.inp` mesh files from Abaqus
- **Sparse Matrix Solver** - Fast linear system solver using sparse matrices

### Post-processing Output Formats
- **VTK/ParaView** - Visualization in ParaView, Visit, and other VTK-compatible software
- **XDMF** - Extensible Data Model and Format for HPC visualization
- **HDF5** - High-performance I/O for large-scale data
- **CSV** - Comma-separated values for spreadsheet analysis
- **Tecplot** - Compatible with Tecplot 360 and other commercial visualization tools

## Project Structure

```
src/
├── lib.rs              # Main library entry point
├── mesh/
│   ├── mod.rs          # Mesh module
│   ├── loader.rs       # Abaqus .inp file parser
│   ├── element.rs      # Element definitions (Quad4, Tri3, etc.)
│   └── node.rs         # Node definitions
├── solver/
│   ├── mod.rs          # Solver module
│   ├── linear.rs       # Linear system solver
│   ├── contact.rs      # Contact analysis solver
│   └── assembly.rs     # Global matrix assembly
├── material/
│   ├── mod.rs          # Material models
│   └── elastic.rs      # Linear elastic material
├── contact/
│   ├── mod.rs          # Contact mechanics
│   ├── friction.rs     # Friction models
│   └── constraint.rs   # Contact constraints
└── output/
    ├── mod.rs          # Output module
    ├── vtk.rs          # VTK format writer
    ├── xdmf.rs         # XDMF format writer
    ├── hdf5.rs         # HDF5 format writer
    ├── csv.rs          # CSV format writer
    └── tecplot.rs      # Tecplot format writer
```

## Quick Start

### Basic Usage

```rust
use rust_fem_solver::mesh::Mesh;
use rust_fem_solver::solver::Solver;
use rust_fem_solver::material::LinearElastic;

fn main() {
    // Load mesh from Abaqus file
    let mesh = Mesh::from_abaqus("model.inp").expect("Failed to load mesh");
    
    // Create solver
    let mut solver = Solver::new(mesh);
    
    // Define material properties
    let material = LinearElastic {
        young_modulus: 210e9,  // 210 GPa
        poisson_ratio: 0.3,
        density: 7850.0,       // kg/m³
    };
    
    // Set boundary conditions
    solver.set_fixed_boundary(vec![0, 1, 2]); // Fix nodes 0, 1, 2
    
    // Apply loads
    solver.add_point_load(100, [0.0, -1000.0]); // 1000 N downward on node 100
    
    // Solve
    solver.solve().expect("Solver failed");
    
    // Export results
    solver.export_vtk("results.vtk").expect("Export failed");
    solver.export_csv("results.csv").expect("Export failed");
}
```

### Contact Analysis

```rust
use rust_fem_solver::contact::{ContactType, ContactBoundary};

// Define contact between two bodies
let contact = ContactBoundary {
    master_surface: vec![10, 11, 12, 13],    // Element nodes
    slave_surface: vec![20, 21, 22, 23],
    contact_type: ContactType::Frictional {
        friction_coefficient: 0.3,
    },
    penalty_parameter: 1e8,
};

solver.add_contact_boundary(contact);
```

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
rust-fem-solver = "0.1"
```

## Building

```bash
# Development build
cargo build

# Release build with optimizations
cargo build --release

# Run tests
cargo test

# Run examples
cargo run --example simple_collision --release
cargo run --example abaqus_import --release
```

## Dependencies

- **ndarray** - N-dimensional array operations
- **nalgebra** - Linear algebra operations
- **sprs** - Sparse matrix support
- **serde** - Serialization/deserialization
- **hdf5** - HDF5 file format support
- **csv** - CSV file handling
- **xml-rs** - XML parsing for XDMF

## Examples

See the `examples/` directory for:
- `simple_collision.rs` - Basic collision analysis between two bodies
- `abaqus_import.rs` - Import and solve Abaqus mesh files

## Performance

The solver is optimized for:
- Large-scale problems (millions of DOFs)
- Sparse matrix operations
- Parallel computation
- Memory efficiency

Release build includes LTO (Link Time Optimization) and O3 optimization level.

## Contributing

Contributions are welcome! Please ensure:
- Code follows Rust conventions
- All tests pass: `cargo test`
- Add tests for new features
- Update documentation

## License

MIT License - See LICENSE file for details

## Authors

Jonathan Salatesi

## References

- Belytschko, T., Liu, W. K., & Moran, B. (2000). Nonlinear Finite Elements for Continua and Structures.
- Wriggers, P. (2006). Computational Contact Mechanics (2nd ed.).
- Abaqus Documentation: https://help.3ds.com/2021/English/DSSIMULIA_Established/SIMACAEusrTOC.htm