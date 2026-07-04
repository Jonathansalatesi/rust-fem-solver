# Rust FEM Solver Development Guide

## Project Overview

This is a professional-grade 2D Finite Element Solver written in Rust for collision analysis and contact mechanics simulations.

## Architecture

### Core Modules

#### 1. **types** (`src/types.rs`)
- Fundamental type definitions and utilities
- 2D vector and matrix operations
- Boundary condition and element type enums
- Point geometry handling

#### 2. **mesh** (`src/mesh/`)
- Mesh data structures and operations
- Node and element definitions
- Abaqus `.inp` file parser and loader
- Mesh statistics and validation

#### 3. **material** (`src/material/`)
- Material property management
- Linear elastic material model (main implementation)
- Trait-based architecture for extensible material models
- Constitutive matrix calculations

#### 4. **solver** (`src/solver/`)
- Main FEM solver orchestration
- Global matrix assembly
- Linear system solver (LU decomposition)
- Contact solver with penalty method
- Boundary condition application

#### 5. **contact** (`src/contact/`)
- Contact mechanics algorithms
- Collision detection (point-to-segment)
- Friction models (Coulomb, viscous)
- Contact constraint management

#### 6. **output** (`src/output/`)
- Multiple post-processing format writers:
  - **VTK** - ParaView, VisIt compatible
  - **XDMF** - Scientific data format
  - **HDF5** - Binary high-performance I/O
  - **CSV** - Spreadsheet compatible
  - **Tecplot** - Engineering visualization

## Mathematical Foundation

### Finite Element Method

1. **Mesh Discretization**: Domain divided into elements (triangles, quads)
2. **Weak Form**: Integral equations formulated from PDEs
3. **Stiffness Matrix Assembly**: 
   ```
   K_global = Σ K_element
   ```
4. **Force Vector Assembly**:
   ```
   F_global = Σ F_point_loads + F_distributed
   ```
5. **System Solve**:
   ```
   K * u = F  →  u = K^(-1) * F
   ```

### Contact Mechanics

**Penalty Method**:
- Normal contact force: `f_n = penalty * max(0, -gap)`
- Friction: Coulomb model `f_f = μ * f_n * sign(v_rel)`

**Binding Contact**:
- Rigid constraint: slave nodes move with master
- Implemented via Lagrange multipliers or high penalty

## Usage Examples

### Basic Static Analysis

```rust
use rust_fem_solver::mesh::Mesh;
use rust_fem_solver::solver::Solver;

// Load or create mesh
let mesh = Mesh::from_abaqus("model.inp")?;

// Setup solver
let mut solver = Solver::new(mesh);
solver.set_linear_elastic_material(210e9, 0.3, 7850.0);

// Boundary conditions
solver.set_fixed_boundary(vec![0, 1, 2]);

// Loads
solver.add_point_load(100, [0.0, -1000.0]);

// Solve
solver.solve()?;

// Export
solver.export_vtk("results.vtk")?;
```

### Contact Analysis

```rust
use rust_fem_solver::contact::{ContactBoundary, ContactType};

let contact = ContactBoundary::new(
    vec![10, 11, 12],  // Master nodes
    vec![20, 21, 22],  // Slave nodes
    ContactType::Frictional {
        friction_coefficient: 0.3,
    },
    1e8,  // Penalty parameter
);

solver.add_contact_boundary(contact);
solver.solve_with_contact(100, 1e-6)?;  // Max 100 iterations, tolerance 1e-6
```

## Building and Testing

```bash
# Build library
cargo build --release

# Run tests
cargo test

# Run examples
cargo run --example simple_collision --release
cargo run --example abaqus_import --release

# Build documentation
cargo doc --open
```

## Performance Characteristics

### Computational Complexity
- **Matrix Assembly**: O(n_elements)
- **Direct Solver**: O(n_dof³) for dense, O(n_dof^1.5) for sparse
- **Contact Detection**: O(n_master × n_slave)

### Optimization Tips
1. Use sparse matrix solvers for large problems
2. Parallelize element assembly
3. Use iterative solvers for very large systems
4. Batch contact detection queries

## Future Enhancements

### Phase 2 Features
- [ ] 3D support (brick, tetrahedral elements)
- [ ] Nonlinear material models (plasticity, hyperelasticity)
- [ ] Dynamic analysis (time integration)
- [ ] Sparse matrix solver integration
- [ ] Parallel processing with Rayon
- [ ] Mesh refinement strategies

### Phase 3 Features
- [ ] Large deformation kinematics
- [ ] Self-contact detection
- [ ] Advanced friction models
- [ ] Thermal analysis coupling
- [ ] Direct implicit dynamic solver

## Contributing

Contribution guidelines:
1. Follow Rust style conventions
2. Add tests for new features
3. Update documentation
4. Ensure all tests pass: `cargo test`
5. Check code with clippy: `cargo clippy`

## References and Resources

### Textbooks
- Zienkiewicz, O. C., Taylor, R. L., & Zhu, J. Z. (2013). *The Finite Element Method: Its Basis and Fundamentals*.
- Belytschko, T., Liu, W. K., & Moran, B. (2000). *Nonlinear Finite Elements for Continua and Structures*.
- Wriggers, P. (2006). *Computational Contact Mechanics*.

### Documentation
- [Rust Book](https://doc.rust-lang.org/book/)
- [Abaqus Manual](https://help.3ds.com/)
- [VTK Format](https://vtk.org/)
- [XDMF Specification](http://www.xdmf.org/)

### Related Tools
- **ParaView**: Visualization
- **Visit**: Scientific visualization
- **Gmsh**: Mesh generation
- **Salome**: CAD/mesh preprocessing

## License

MIT License - See LICENSE file

## Authors

Jonathan Salatesi

## Troubleshooting

### Solver Divergence
- Check boundary conditions are properly applied
- Verify load magnitudes are reasonable
- Increase penalty parameter for contact
- Reduce time step (for dynamic problems)

### Memory Issues
- Use sparse matrix storage for large meshes
- Process large result files in chunks
- Stream VTK output instead of buffering

### File Format Issues
- Ensure Abaqus file uses supported element types (CPS3, CPS4, CPE3, CPE4)
- Check file encoding is UTF-8
- Verify node numbering is consecutive
