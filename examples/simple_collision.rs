//! Example: Simple collision analysis
//!
//! Demonstrates basic collision detection and solving between two bodies

use rust_fem_solver::mesh::Mesh;
use rust_fem_solver::solver::Solver;
use rust_fem_solver::contact::{ContactBoundary, ContactType};
use rust_fem_solver::types::Point2D;

fn main() -> rust_fem_solver::Result<()> {
    println!("=== Simple Collision Analysis ===");
    println!();

    // Create a simple mesh with two bodies
    let mut mesh = Mesh::new();

    // Body 1: Square (0,0) to (1,1)
    println!("Creating Body 1 (fixed square)...");
    mesh.add_node(0, Point2D::new(0.0, 0.0));
    mesh.add_node(1, Point2D::new(1.0, 0.0));
    mesh.add_node(2, Point2D::new(1.0, 1.0));
    mesh.add_node(3, Point2D::new(0.0, 1.0));

    mesh.add_element(1, rust_fem_solver::types::ElementType::Quadrilateral4, vec![0, 1, 2, 3])?;

    // Body 2: Square (0.5, 1.5) to (1.5, 2.5)
    println!("Creating Body 2 (falling square)...");
    mesh.add_node(4, Point2D::new(0.5, 1.5));
    mesh.add_node(5, Point2D::new(1.5, 1.5));
    mesh.add_node(6, Point2D::new(1.5, 2.5));
    mesh.add_node(7, Point2D::new(0.5, 2.5));

    mesh.add_element(2, rust_fem_solver::types::ElementType::Quadrilateral4, vec![4, 5, 6, 7])?;

    println!("Mesh created successfully!");
    println!();

    // Print mesh statistics
    let stats = mesh.statistics();
    println!("{}", stats);
    println!();

    // Create solver
    let mut solver = Solver::new(mesh);

    // Set material properties (steel)
    solver.set_linear_elastic_material(
        210e9,  // Young's modulus: 210 GPa
        0.3,    // Poisson's ratio
        7850.0, // Density: 7850 kg/m³
    );

    // Set boundary conditions
    println!("Setting boundary conditions...");
    solver.set_fixed_boundary(vec![0, 1, 2, 3]); // Fix body 1
    println!("Body 1 is fixed.");
    println!();

    // Apply load (gravity on body 2)
    println!("Applying loads...");
    let gravity_force = [0.0, -1000.0]; // 1000 N downward
    solver.add_point_load(4, gravity_force);
    solver.add_point_load(5, gravity_force);
    solver.add_point_load(6, gravity_force);
    solver.add_point_load(7, gravity_force);
    println!("Applied gravity force: {:?}", gravity_force);
    println!();

    // Add contact boundary condition
    println!("Setting up contact between bodies...");
    let contact = ContactBoundary::new(
        vec![2, 3],          // Master surface (top of body 1)
        vec![4, 7],          // Slave surface (bottom of body 2)
        ContactType::Frictional {
            friction_coefficient: 0.3,
        },
        1e8,                 // Penalty parameter
    );
    solver.add_contact_boundary(contact);
    println!("Contact defined: Frictional with μ = 0.3");
    println!();

    // Solve
    println!("Solving...");
    solver.solve()?;
    println!("Solution complete!");
    println!();

    // Print displacement results
    println!("Displacements:");
    for node_id in 0..8 {
        if let Some(disp) = solver.get_displacement(node_id) {
            println!("  Node {}: ux = {:.6e}, uy = {:.6e}", node_id, disp[0], disp[1]);
        }
    }
    println!();

    // Export results
    println!("Exporting results...");
    solver.export_vtk("collision_results.vtk")?;
    println!("  ✓ VTK file: collision_results.vtk");

    solver.export_csv("collision_results.csv")?;
    println!("  ✓ CSV file: collision_results.csv");

    solver.export_tecplot("collision_results.dat")?;
    println!("  ✓ Tecplot file: collision_results.dat");

    solver.export_elements_csv("collision_elements.csv")?;
    println!("  ✓ Elements file: collision_elements.csv");

    println!();
    println!("Analysis complete! Open collision_results.vtk in ParaView to visualize.");

    Ok(())
}
