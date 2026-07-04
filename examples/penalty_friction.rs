//! Example: Frictional contact using penalty method
//!
//! Demonstrates frictional contact with penalty method for comparison

use rust_fem_solver::mesh::Mesh;
use rust_fem_solver::solver::Solver;
use rust_fem_solver::contact::{ContactBoundary, ContactType, ContactMethod};
use rust_fem_solver::types::Point2D;

fn main() -> rust_fem_solver::Result<()> {
    println!("\n╔═══════════════════════════════════════════════════════════════╗");
    println!("║  Penalty Method Contact Analysis - Frictional Contact        ║");
    println!("╚═══════════════════════════════════════════════════════════════╝\n");

    // Create mesh
    let mut mesh = Mesh::new();

    // Body 1: Fixed inclined surface
    println!("📐 Creating Body 1 (fixed inclined plane)...");
    mesh.add_node(0, Point2D::new(0.0, 0.0));
    mesh.add_node(1, Point2D::new(2.0, 0.0));
    mesh.add_node(2, Point2D::new(2.0, 0.5));
    mesh.add_node(3, Point2D::new(0.0, 0.5));
    mesh.add_element(1, rust_fem_solver::types::ElementType::Quadrilateral4, vec![0, 1, 2, 3])?;

    // Body 2: Sliding block
    println!("📐 Creating Body 2 (sliding block)...");
    mesh.add_node(4, Point2D::new(0.5, 0.7));
    mesh.add_node(5, Point2D::new(1.0, 0.7));
    mesh.add_node(6, Point2D::new(1.0, 1.2));
    mesh.add_node(7, Point2D::new(0.5, 1.2));
    mesh.add_element(2, rust_fem_solver::types::ElementType::Quadrilateral4, vec![4, 5, 6, 7])?;

    let stats = mesh.statistics();
    println!("\n{}\n", stats);

    // Setup solver
    let mut solver = Solver::new(mesh);
    solver.set_linear_elastic_material(210e9, 0.3, 7850.0);

    // Boundary conditions
    println!("🔧 Applying boundary conditions...");
    solver.set_fixed_boundary(vec![0, 1, 2, 3]);
    println!("   ✓ Body 1 (inclined plane) is fixed\n");

    // Loads
    println!("⚡ Applying loads...");
    let gravity = [0.0, -1000.0];
    solver.add_point_load(4, gravity);
    solver.add_point_load(5, gravity);
    solver.add_point_load(6, gravity);
    solver.add_point_load(7, gravity);
    println!("   ✓ Applied gravity: {} N (downward)\n", gravity[1]);

    // Add frictional contact using PENALTY method
    println!("📌 Setting up FRICTIONAL contact (Penalty method)...");
    let contact = ContactBoundary::with_method(
        vec![2, 3],              // Master: top of Body 1
        vec![4, 5],              // Slave: bottom of Body 2
        ContactType::Frictional {
            friction_coefficient: 0.5,
        },
        ContactMethod::Penalty {
            penalty_parameter: 1e10,  // Higher penalty = more accurate but less stable
        },
    );
    println!("   ✓ Contact method: Penalty method");
    println!("   ✓ Contact type: Frictional (μ = 0.5)");
    println!("   ✓ Penalty parameter: 1e10\n");
    
    solver.add_contact_boundary(contact);

    // Solve
    println!("🔄 Solving...");
    solver.solve()?;
    println!("✅ Solution complete!\n");

    // Display results
    println!("📊 SOLUTION RESULTS");
    println!("─────────────────────────────────────────────────────");
    println!("\nNodeal Displacements:");
    println!("  Node |        ux (m)      |        uy (m)");
    println!("  ─────┼──────────────────┼──────────────────");
    for node_id in 0..8 {
        if let Some(disp) = solver.get_displacement(node_id) {
            println!("   {:2}  | {:17.6e} | {:17.6e}", node_id, disp[0], disp[1]);
        }
    }

    // Export results
    println!("\n💾 Exporting results...");
    solver.export_vtk("frictional_contact_penalty.vtk")?;
    println!("   ✓ VTK:    frictional_contact_penalty.vtk");
    
    solver.export_csv("frictional_contact_penalty.csv")?;
    println!("   ✓ CSV:    frictional_contact_penalty.csv");

    println!("\n✅ Analysis complete!");
    println!("   Comparison: Penalty method is simpler but less accurate than Lagrange.");
    println!("   Open frictional_contact_penalty.vtk in ParaView for visualization.\n");

    Ok(())
}
