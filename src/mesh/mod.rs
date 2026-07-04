//! Mesh module for FEM solver
//!
//! Handles mesh data structures and operations

mod loader;
mod element;
mod node;

pub use loader::AbaqusLoader;
pub use element::{Element, ElementConnectivity};
pub use node::Node;

use crate::types::{NodeId, ElementId, ElementType, Point2D, Vector2};
use crate::Result;
use std::collections::HashMap;

/// Mesh structure containing all geometric and topological data
#[derive(Debug, Clone)]
pub struct Mesh {
    /// Nodes in the mesh
    nodes: Vec<Node>,
    /// Elements in the mesh
    elements: Vec<Element>,
    /// Node ID to index mapping
    node_map: HashMap<NodeId, usize>,
    /// Element ID to index mapping
    element_map: HashMap<ElementId, usize>,
}

impl Mesh {
    /// Create a new empty mesh
    pub fn new() -> Self {
        Mesh {
            nodes: Vec::new(),
            elements: Vec::new(),
            node_map: HashMap::new(),
            element_map: HashMap::new(),
        }
    }

    /// Create mesh from Abaqus .inp file
    pub fn from_abaqus(filename: &str) -> Result<Self> {
        AbaqusLoader::load(filename)
    }

    /// Add a node to the mesh
    pub fn add_node(&mut self, id: NodeId, coords: Point2D) {
        let index = self.nodes.len();
        self.node_map.insert(id, index);
        self.nodes.push(Node::new(id, coords));
    }

    /// Add an element to the mesh
    pub fn add_element(
        &mut self,
        id: ElementId,
        elem_type: ElementType,
        connectivity: Vec<NodeId>,
    ) -> Result<()> {
        // Validate connectivity
        if connectivity.len() != elem_type.num_nodes() {
            return Err(crate::FemError::InvalidInput(
                format!(
                    "Element {} has {} nodes, expected {}",
                    id,
                    connectivity.len(),
                    elem_type.num_nodes()
                ),
            ));
        }

        let index = self.elements.len();
        self.element_map.insert(id, index);
        self.elements.push(Element::new(
            id,
            elem_type,
            ElementConnectivity::new(connectivity),
        ));

        Ok(())
    }

    /// Get number of nodes
    pub fn num_nodes(&self) -> usize {
        self.nodes.len()
    }

    /// Get number of elements
    pub fn num_elements(&self) -> usize {
        self.elements.len()
    }

    /// Get total number of DOFs (2 per node for 2D)
    pub fn num_dofs(&self) -> usize {
        self.nodes.len() * 2
    }

    /// Get node by ID
    pub fn get_node(&self, id: NodeId) -> Option<&Node> {
        self.node_map.get(&id).and_then(|&idx| self.nodes.get(idx))
    }

    /// Get mutable node by ID
    pub fn get_node_mut(&mut self, id: NodeId) -> Option<&mut Node> {
        if let Some(&idx) = self.node_map.get(&id) {
            return self.nodes.get_mut(idx);
        }
        None
    }

    /// Get element by ID
    pub fn get_element(&self, id: ElementId) -> Option<&Element> {
        self.element_map
            .get(&id)
            .and_then(|&idx| self.elements.get(idx))
    }

    /// Get node index from ID
    pub fn get_node_index(&self, id: NodeId) -> Option<usize> {
        self.node_map.get(&id).copied()
    }

    /// Get all nodes
    pub fn nodes(&self) -> &[Node] {
        &self.nodes
    }

    /// Get all elements
    pub fn elements(&self) -> &[Element] {
        &self.elements
    }

    /// Get mutable access to all elements
    pub fn elements_mut(&mut self) -> &mut [Element] {
        &mut self.elements
    }

    /// Get node coordinates as array
    pub fn get_coordinates(&self, id: NodeId) -> Option<Vector2> {
        self.get_node(id).map(|n| n.coordinates().as_vec())
    }

    /// Get bounding box of the mesh
    pub fn bounding_box(&self) -> Option<(Point2D, Point2D)> {
        if self.nodes.is_empty() {
            return None;
        }

        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;

        for node in &self.nodes {
            let coords = node.coordinates();
            min_x = min_x.min(coords.x);
            min_y = min_y.min(coords.y);
            max_x = max_x.max(coords.x);
            max_y = max_y.max(coords.y);
        }

        Some((Point2D::new(min_x, min_y), Point2D::new(max_x, max_y)))
    }

    /// Calculate mesh statistics
    pub fn statistics(&self) -> MeshStats {
        let mut stats = MeshStats::default();
        stats.num_nodes = self.num_nodes();
        stats.num_elements = self.num_elements();
        stats.num_dofs = self.num_dofs();

        // Count element types
        for elem in &self.elements {
            match elem.element_type() {
                ElementType::Triangle3 => stats.num_tri3 += 1,
                ElementType::Quadrilateral4 => stats.num_quad4 += 1,
                ElementType::Triangle6 => stats.num_tri6 += 1,
                ElementType::Quadrilateral8 => stats.num_quad8 += 1,
            }
        }

        if let Some((min, max)) = self.bounding_box() {
            stats.bounding_box = Some((min, max));
        }

        stats
    }
}

impl Default for Mesh {
    fn default() -> Self {
        Self::new()
    }
}

/// Mesh statistics
#[derive(Debug, Default, Clone)]
pub struct MeshStats {
    pub num_nodes: usize,
    pub num_elements: usize,
    pub num_dofs: usize,
    pub num_tri3: usize,
    pub num_quad4: usize,
    pub num_tri6: usize,
    pub num_quad8: usize,
    pub bounding_box: Option<(Point2D, Point2D)>,
}

impl std::fmt::Display for MeshStats {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        writeln!(f, "Mesh Statistics:")?;
        writeln!(f, "  Nodes: {}", self.num_nodes)?;
        writeln!(f, "  Elements: {}", self.num_elements)?;
        writeln!(f, "  DOFs: {}", self.num_dofs)?;
        writeln!(f, "  Triangle3: {}", self.num_tri3)?;
        writeln!(f, "  Quadrilateral4: {}", self.num_quad4)?;
        writeln!(f, "  Triangle6: {}", self.num_tri6)?;
        writeln!(f, "  Quadrilateral8: {}", self.num_quad8)?;

        if let Some((min, max)) = self.bounding_box {
            writeln!(f, "  Bounding Box: ({:.3}, {:.3}) to ({:.3}, {:.3})",
                min.x, min.y, max.x, max.y)?;
        }
        Ok(())
    }
}
