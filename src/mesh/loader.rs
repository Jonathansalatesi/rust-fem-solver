//! Abaqus .inp file loader
//!
//! Parses Abaqus input files to extract mesh data

use crate::types::{ElementType, Point2D};
use crate::{FemError, Mesh, Result};
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::str::FromStr;

/// Abaqus file loader
pub struct AbaqusLoader;

impl AbaqusLoader {
    /// Load mesh from Abaqus .inp file
    pub fn load(filename: &str) -> Result<Mesh> {
        let file = File::open(filename)?;
        let reader = BufReader::new(file);
        let lines: Vec<String> = reader.lines().collect::<std::io::Result<Vec<_>>>()?;

        let mut mesh = Mesh::new();
        let mut i = 0;

        while i < lines.len() {
            let line = lines[i].trim();

            if line.starts_with("*Node") {
                i = Self::parse_nodes(&lines, i + 1, &mut mesh)?;
            } else if line.starts_with("*Element") {
                let elem_type = Self::parse_element_header(line)?;
                i = Self::parse_elements(&lines, i + 1, &mut mesh, elem_type)?;
            } else {
                i += 1;
            }
        }

        if mesh.num_nodes() == 0 || mesh.num_elements() == 0 {
            return Err(FemError::ParseError(
                "No nodes or elements found in mesh file".to_string(),
            ));
        }

        Ok(mesh)
    }

    /// Parse *Node section
    fn parse_nodes(lines: &[String], start: usize, mesh: &mut Mesh) -> Result<usize> {
        let mut i = start;

        while i < lines.len() {
            let line = lines[i].trim();

            if line.is_empty() || line.starts_with('*') {
                return Ok(i);
            }

            let parts: Vec<&str> = line.split(',').map(|s| s.trim()).collect();

            if parts.len() >= 3 {
                let node_id = parts[0].parse::<usize>()
                    .map_err(|_| FemError::ParseError(format!("Invalid node ID: {}", parts[0])))?;
                
                let x = f64::from_str(parts[1])
                    .map_err(|_| FemError::ParseError(format!("Invalid X coordinate: {}", parts[1])))?;
                
                let y = f64::from_str(parts[2])
                    .map_err(|_| FemError::ParseError(format!("Invalid Y coordinate: {}", parts[2])))?;

                mesh.add_node(node_id, Point2D::new(x, y));
            }

            i += 1;
        }

        Ok(i)
    }

    /// Parse element header to determine element type
    fn parse_element_header(header: &str) -> Result<ElementType> {
        let upper = header.to_uppercase();
        
        if upper.contains("TYPE=CPS3") || upper.contains("TYPE=CPE3") {
            Ok(ElementType::Triangle3)
        } else if upper.contains("TYPE=CPS4") || upper.contains("TYPE=CPE4") {
            Ok(ElementType::Quadrilateral4)
        } else if upper.contains("TYPE=CPS6") || upper.contains("TYPE=CPE6") {
            Ok(ElementType::Triangle6)
        } else if upper.contains("TYPE=CPS8") || upper.contains("TYPE=CPE8") {
            Ok(ElementType::Quadrilateral8)
        } else {
            Err(FemError::ParseError(
                "Unsupported element type in Abaqus file".to_string(),
            ))
        }
    }

    /// Parse *Element section
    fn parse_elements(
        lines: &[String],
        start: usize,
        mesh: &mut Mesh,
        elem_type: ElementType,
    ) -> Result<usize> {
        let mut i = start;

        while i < lines.len() {
            let line = lines[i].trim();

            if line.is_empty() || line.starts_with('*') {
                return Ok(i);
            }

            let parts: Vec<&str> = line.split(',').map(|s| s.trim()).collect();

            if parts.len() >= elem_type.num_nodes() + 1 {
                let elem_id = parts[0].parse::<usize>()
                    .map_err(|_| FemError::ParseError(format!("Invalid element ID: {}", parts[0])))?;
                
                let mut connectivity = Vec::new();
                for j in 1..=elem_type.num_nodes() {
                    let node_id = parts[j].parse::<usize>()
                        .map_err(|_| FemError::ParseError(format!("Invalid node ID: {}", parts[j])))?;
                    connectivity.push(node_id);
                }

                mesh.add_element(elem_id, elem_type, connectivity)?;
            }

            i += 1;
        }

        Ok(i)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_element_header() {
        assert_eq!(
            AbaqusLoader::parse_element_header("*Element, type=CPS3").unwrap(),
            ElementType::Triangle3
        );
        assert_eq!(
            AbaqusLoader::parse_element_header("*Element, type=CPS4").unwrap(),
            ElementType::Quadrilateral4
        );
    }
}
