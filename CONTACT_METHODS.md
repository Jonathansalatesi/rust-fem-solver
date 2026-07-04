# Contact Mechanics Methods Comparison

## Overview

This FEM solver now supports **two complementary contact mechanics methods**:

1. **Lagrange Multiplier Method** (Default) - More accurate, handles binding
2. **Penalty Method** - Simpler, good for initial studies

## Method Comparison

### Lagrange Multiplier Method ✅ (Default)

**Characteristics:**
- More mathematically rigorous
- Enforces exact constraints (no penetration)
- Supports **binding contacts** (rigid connections)
- Slightly higher computational cost
- Better for production analyses

**Advantages:**
- Exact constraint enforcement
- No penalty parameter tuning needed
- Stable for large deformations
- Binding contact support (new!)
- Convergence guaranteed (under conditions)

**Disadvantages:**
- Larger system matrix (includes multipliers)
- Requires iterative refinement (augmented Lagrangian)
- More complex implementation

**Mathematical Basis:**
```
Augmented Lagrangian:
  L(u, λ, ρ) = J(u) + λ^T·g(u) + (ρ/2)·||g(u)||²

Iterative update:
  λ^(k+1) = λ^(k) - ρ·g(u^(k+1))
  ρ^(k+1) = β·ρ^(k)    (penalty parameter increase)
```

**When to use:**
- Production simulations
- Large deformations expected
- Binding contacts needed
- High accuracy required

### Penalty Method

**Characteristics:**
- Simpler, faster implementation
- Approximate constraint enforcement
- Only frictional contacts supported
- Lower computational overhead
- Requires parameter tuning

**Advantages:**
- Simple to implement
- Smaller system size
- Faster per iteration
- Good for quick studies
- No multiplier DOFs

**Disadvantages:**
- Penetration allowed (tuning dependent)
- Parameter sensitivity
- Can become ill-conditioned
- Not suitable for binding contact

**Mathematical Basis:**
```
Penalty method:
  f_contact = ε·g⁺(u)·n  where g⁺ = max(0, g)
  K_modified = K + ε·C^T·C

Too small ε  → Poor constraint enforcement
Too large ε  → Ill-conditioned system
```

**When to use:**
- Quick feasibility studies
- Small deformations
- Frictional contacts only
- Mesh generation testing
- Preliminary designs

## Contact Types

### Binding Contact (Lagrange Only)

Rigid connection between surfaces - no slip and no separation allowed.

**Use cases:**
- Glued/welded interfaces
- Bolt connections
- Composite interfaces
- Structure-to-foundation connections

**Example:**
```rust
let contact = ContactBoundary::new(
    vec![10, 11, 12],           // Master nodes
    vec![20, 21, 22],           // Slave nodes
    ContactType::Binding,       // Binding type
);
// Automatically uses Lagrange method
```

### Frictional Contact (Both Methods)

Surfaces can slip with friction, but cannot penetrate.

**Coulomb Friction Model:**
```
f_t ≤ μ·f_n  (slip condition)
f_t = μ·f_n  (sticking)
```

**Lagrange Example:**
```rust
let contact = ContactBoundary::new(
    vec![10, 11, 12],
    vec![20, 21, 22],
    ContactType::Frictional { friction_coefficient: 0.3 },
    // Uses Lagrange method by default
);
```

**Penalty Example:**
```rust
let contact = ContactBoundary::with_method(
    vec![10, 11, 12],
    vec![20, 21, 22],
    ContactType::Frictional { friction_coefficient: 0.3 },
    ContactMethod::Penalty { penalty_parameter: 1e10 },
);
```

## Usage Guide

### Default (Lagrange Multiplier)

```rust
use rust_fem_solver::contact::{ContactBoundary, ContactType};

// Create contact - uses Lagrange by default
let contact = ContactBoundary::new(
    master_nodes,
    slave_nodes,
    ContactType::Binding,  // or Frictional { ... }
);

solver.add_contact_boundary(contact);
solver.solve_with_contact(100, 1e-6)?;  // Iterative solver
```

### Explicit Lagrange Method

```rust
use rust_fem_solver::contact::{ContactBoundary, ContactType, ContactMethod};

let contact = ContactBoundary::with_method(
    master_nodes,
    slave_nodes,
    ContactType::Binding,
    ContactMethod::Lagrange,  // Explicit (same as default)
);
```

### Penalty Method

```rust
let contact = ContactBoundary::with_method(
    master_nodes,
    slave_nodes,
    ContactType::Frictional { friction_coefficient: 0.3 },
    ContactMethod::Penalty { penalty_parameter: 1e8 },
);

solver.add_contact_boundary(contact);
solver.solve()?;  // Single solve with penalty
```

## Convergence Behavior

### Lagrange Method

```
Iteration |  Residual
    1     |  1.234e-2
    2     |  5.432e-4
    3     |  2.105e-5  ← Quadratic convergence
    4     |  1.234e-7
```

Exponential convergence with augmented Lagrangian approach.

### Penalty Method

```
Iteration |  Gap/Penetration
    1     |  0.845        ← No iteration
    2     |  0.123        (penalty applied once)
    3     |  0.012
```

No iteration - single solve adjusts stiffness matrix.

## Parameter Selection

### Lagrange Multiplier Method

- **Augmentation parameter (ρ)**: Start with 1.0, increase by factor 1.2-2.0 each iteration
- **Tolerance**: 1e-4 to 1e-6 (typical)
- **Max iterations**: 20-100 (usually <50)

### Penalty Method

- **Penalty coefficient (ε)**: Critical parameter!
  - Too small: Poor contact enforcement
  - Too large: Ill-conditioned matrix
  - Typical range: 1e8 - 1e12
  
- **Rule of thumb**: ε ≈ K·Characteristic_length²
  where K is modulus, length is typical element size

## Numerical Examples

See example files:
- `examples/lagrange_binding.rs` - Binding contact (Lagrange)
- `examples/penalty_friction.rs` - Frictional contact (Penalty)
- `examples/simple_collision.rs` - Basic collision
- `examples/abaqus_import.rs` - Import Abaqus mesh

## Performance Tips

1. **Lagrange Method**
   - Start with coarse mesh to verify setup
   - Use reasonable tolerance (1e-4) initially
   - Increase augmentation parameter gradually
   - Save converged solution for refinement

2. **Penalty Method**
   - Start with smaller penalty, increase if needed
   - Monitor constraint violations
   - Use only for frictional contacts
   - Good for feasibility studies

## Theoretical References

1. **Lagrange Multiplier Method:**
   - Wriggers, P. (2006). *Computational Contact Mechanics*
   - Laursen, T. A. (2002). *Computational Contact and Impact Mechanics*
   - Augmented Lagrangian: Fortin & Glowinski

2. **Penalty Method:**
   - Belytschko, T., Liu, W. K., & Moran, B. (2000). *Nonlinear FEM*
   - Classical penalty methods review

## Future Enhancements

- [ ] Active set strategy (improved efficiency)
- [ ] Mortar method (higher accuracy)
- [ ] Nitsche method (no multipliers, exact constraints)
- [ ] Contact smoothing algorithms
- [ ] Self-contact detection
- [ ] 3D contact handling
