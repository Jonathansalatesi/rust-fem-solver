//! Element definition for FEM mesh

use crate::types::{ElementId, ElementType, NodeId};

/// Element connectivity information
#[derive(Debug, Clone, PartialEq)]
pub struct ElementConnectivity {
    node_ids: Vec<NodeId>,
}

impl ElementConnectivity {
    /// Create new element connectivity
    pub fn new(node_ids: Vec<NodeId>) -> Self {
        ElementConnectivity { node_ids }
    }

    /// Get node IDs
    pub fn node_ids(&self) -> &[NodeId] {
        &self.node_ids
    }

    /// Get node ID at index
    pub fn node_id(&self, idx: usize) -> Option<NodeId> {
        self.node_ids.get(idx).copied()
    }

    /// Number of nodes
    pub fn num_nodes(&self) -> usize {
        self.node_ids.len()
    }
}

/// Element in the FEM mesh
#[derive(Debug, Clone, PartialEq)]
pub struct Element {
    /// Element ID
    id: ElementId,
    /// Element type
    element_type: ElementType,
    /// Node connectivity
    connectivity: ElementConnectivity,
}

impl Element {
    /// Create new element
    pub fn new(
        id: ElementId,
        element_type: ElementType,
        connectivity: ElementConnectivity,
    ) -> Self {
        Element {
            id,
            element_type,
            connectivity,
        }
    }

    /// Get element ID
    pub fn id(&self) -> ElementId {
        self.id
    }

    /// Get element type
    pub fn element_type(&self) -> ElementType {
        self.element_type
    }

    /// Get connectivity
    pub fn connectivity(&self) -> &ElementConnectivity {
        &self.connectivity
    }

    /// Get node IDs in this element
    pub fn node_ids(&self) -> &[NodeId] {
        self.connectivity.node_ids()
    }

    /// Total DOFs in element
    pub fn num_dofs(&self) -> usize {
        self.element_type.total_dofs()
    }
}
