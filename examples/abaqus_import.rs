//! Example: Import and solve Abaqus mesh file
//!
//! Demonstrates loading mesh from Abaqus .inp file and solving

use rust_fem_solver::mesh::Mesh;
use rust_fem_solver::solver::Solver;

fn main() -> rust_fem_solver::Result<()> {
    println!("=== Abaqus Mesh Import Example ===");
    println!();

    // Create a sample Abaqus input file for demonstration
    let abaqus_content = r#"*Node
1, 0.0, 0.0
2, 1.0, 0.0
3, 1.0, 1.0
4, 0.0, 1.0
5, 2.0, 0.0
6, 2.0, 1.0
*Element, type=CPS4
1, 1, 2, 3, 4
2, 2, 5, 6, 3
"#;

    // Write example file
    std::fs::write("example_mesh.inp", abaqus_content)?;
    println!("Created example Abaqus file: example_mesh.inp");
    println!();

    // Load mesh
    println!("Loading mesh from Abaqus file...");
    let mesh = Mesh::from_abaqus("example_mesh.inp")?;
    println!("Mesh loaded successfully!");
    println!();

    // Print mesh statistics
    let stats = mesh.statistics();
    println!("{}", stats);
    println!();

    // Create solver
    let mut solver = Solver::new(mesh);

    // Configure material
    println!("Setting material properties...");
    solver.set_linear_elastic_material(
        210e9,  // E = 210 GPa (steel)
        0.3,    // ν = 0.3
        7850.0, // ρ = 7850 kg/m³
    );
    println!("  Young's modulus: 210 GPa");
    println!("  Poisson's ratio: 0.3");
    println!("  Density: 7850 kg/m³");
    println!();

    // Apply boundary conditions
    println!("Setting boundary conditions...");
    solver.set_fixed_boundary(vec![1, 4]);  // Fix left edge
    println!("  Fixed nodes: 1, 4");
    println!();

    // Apply loads
    println!("Applying loads...");
    solver.add_point_load(3, [0.0, -1000.0]);  // Load at top-right
    solver.add_point_load(6, [0.0, -1000.0]);
    println!("  Applied -1000 N (downward) at nodes 3 and 6");
    println!();

    // Solve
    println!("Solving linear system...");
    solver.solve()?;
    println!("Solution complete!");
    println!();

    // Display results
    println!("Nodal Displacements:");
    println!("  Node |      ux (m)     |      uy (m)    ");
    println!("  -----|-----------------|---------------");
    for node_id in 1..=6 {
        if let Some(disp) = solver.get_displacement(node_id) {
            println!("    {}  | {:.6e} | {:.6e}", node_id, disp[0], disp[1]);
        }
    }
    println!();

    // Export results in multiple formats
    println!("Exporting results to post-processing formats...");
    solver.export_vtk("abaqus_results.vtk")?;
    println!("  ✓ VTK format:    abaqus_results.vtk (for ParaView/Visit)");

    solver.export_csv("abaqus_results.csv")?;
    println!("  ✓ CSV format:    abaqus_results.csv (for spreadsheets)");

    solver.export_tecplot("abaqus_results.dat")?;
    println!("  ✓ Tecplot format: abaqus_results.dat (for Tecplot 360)");

    solver.export_elements_csv("abaqus_elements.csv")?;
    println!("  ✓ Elements:      abaqus_elements.csv");

    println!();
    println!("Example complete!");
    println!("Recommendation: Open abaqus_results.vtk in ParaView for visualization.");

    Ok(())
}
