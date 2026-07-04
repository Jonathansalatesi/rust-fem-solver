//! Example: Contact analysis with Lagrange multiplier method (default)
//!
//! Demonstrates binding contact using Lagrange multiplier method

use rust_fem_solver::mesh::Mesh;
use rust_fem_solver::solver::Solver;
use rust_fem_solver::contact::{ContactBoundary, ContactType, ContactMethod};
use rust_fem_solver::types::Point2D;

fn main() -> rust_fem_solver::Result<()> {
    println!("\n╔═══════════════════════════════════════════════════════════════╗");
    println!("║  Lagrange Multiplier Contact Analysis - Binding Contact      ║");
    println!("╚═══════════════════════════════════════════════════════════════╝\n");

    // Create mesh with two bodies
    let mut mesh = Mesh::new();

    // Body 1: Fixed square (0,0) to (1,1)
    println!("📐 Creating Body 1 (fixed square from (0,0) to (1,1))...");
    mesh.add_node(0, Point2D::new(0.0, 0.0));
    mesh.add_node(1, Point2D::new(1.0, 0.0));
    mesh.add_node(2, Point2D::new(1.0, 1.0));
    mesh.add_node(3, Point2D::new(0.0, 1.0));
    mesh.add_element(1, rust_fem_solver::types::ElementType::Quadrilateral4, vec![0, 1, 2, 3])?;

    // Body 2: Movable square (0.25, 1.2) to (0.75, 1.7)
    println!("📐 Creating Body 2 (movable square from (0.25,1.2) to (0.75,1.7))...");
    mesh.add_node(4, Point2D::new(0.25, 1.2));
    mesh.add_node(5, Point2D::new(0.75, 1.2));
    mesh.add_node(6, Point2D::new(0.75, 1.7));
    mesh.add_node(7, Point2D::new(0.25, 1.7));
    mesh.add_element(2, rust_fem_solver::types::ElementType::Quadrilateral4, vec![4, 5, 6, 7])?;

    let stats = mesh.statistics();
    println!("\n{}\n", stats);

    // Setup solver
    let mut solver = Solver::new(mesh);
    solver.set_linear_elastic_material(210e9, 0.3, 7850.0);
    solver.set_thickness(1.0);

    // Boundary conditions
    println!("🔧 Applying boundary conditions...");
    solver.set_fixed_boundary(vec![0, 1, 2, 3]);
    println!("   ✓ Body 1 is fully fixed\n");

    // Loads on Body 2 (pressing down onto Body 1)
    println!("⚡ Applying loads...");
    let load = [0.0, -5000.0];
    solver.add_point_load(4, load);
    solver.add_point_load(5, load);
    solver.add_point_load(6, load);
    solver.add_point_load(7, load);
    println!("   ✓ Applied load on Body 2: {} N (downward)\n", load[1]);

    // Add binding contact using Lagrange method (default)
    println!("📌 Setting up BINDING contact (Lagrange multiplier method)...");
    let contact = ContactBoundary::new(
        vec![2, 3],              // Master: top edge of Body 1
        vec![4, 5],              // Slave: bottom edge of Body 2
        ContactType::Binding,    // Binding (rigid connection)
    );
    println!("   ✓ Contact method: Lagrange multiplier (default)");
    println!("   ✓ Contact type: Binding (no slip)");
    println!("   ✓ Master nodes: [2, 3], Slave nodes: [4, 5]\n");
    
    solver.add_contact_boundary(contact);

    // Solve with contact
    println!("🔄 Solving with contact constraints...");
    solver.solve_with_contact(50, 1e-4)?;

    // Display results
    println!("\n📊 SOLUTION RESULTS");
    println!("─────────────────────────────────────────────────────");
    println!("\nNodeal Displacements:");
    println!("  Node |        ux (m)      |        uy (m)");
    println!("  ─────┼──────────────────┼──────────────────");
    for node_id in 0..8 {
        if let Some(disp) = solver.get_displacement(node_id) {
            println!("   {:2}  | {:17.6e} | {:17.6e}", node_id, disp[0], disp[1]);
        }
    }

    // Print Lagrange multipliers (contact forces)
    println!("\nContact Forces (via Lagrange Multipliers):");
    println!("  Slave | Master | Normal Force (N) | Tangent Force (N)");
    println!("  ──────┼────────┼──────────────────┼──────────────────");
    let multipliers = solver.lagrange_multipliers();
    for (i, mult) in multipliers.iter().enumerate() {
        println!("   {:2}   |  {:2}    | {:16.4} | {:16.4}",
            mult.slave_node, mult.master_node, mult.normal_force(), mult.tangential_force());
    }

    // Export results
    println!("\n💾 Exporting results...");
    solver.export_vtk("binding_contact_lagrange.vtk")?;
    println!("   ✓ VTK:    binding_contact_lagrange.vtk");
    
    solver.export_csv("binding_contact_lagrange.csv")?;
    println!("   ✓ CSV:    binding_contact_lagrange.csv");
    
    solver.export_tecplot("binding_contact_lagrange.dat")?;
    println!("   ✓ Tecplot: binding_contact_lagrange.dat");

    println!("\n✅ Analysis complete!");
    println!("   Open binding_contact_lagrange.vtk in ParaView for visualization.\n");

    Ok(())
}
