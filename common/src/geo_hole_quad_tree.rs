use geo::{
    relate::{
        Relate,
    },
    algorithm::{
        intersects::{
            Intersects,
        },
    },
};

use crate::{
    problem,
};

pub struct GeoHoleQuadTree {
    root: Node,
    geo_hole: geo::Polygon<f64>,
}

#[derive(Debug)]
pub struct Node {
    pub min: problem::Point,
    pub max: problem::Point,
    pub kind: NodeKind,
}

#[derive(Debug)]
pub enum NodeKind {
    Inside,
    Outside,
    Branch { children: Vec<Node>, },
}

#[derive(Debug)]
pub enum CreateError {
    NoPointsInHole,
    NoPointsInFigure,
    FieldIsTooSmall,
}

impl GeoHoleQuadTree {
    pub fn new(problem: &problem::Problem) -> Result<GeoHoleQuadTree, CreateError> {
        if problem.hole.is_empty() {
            return Err(CreateError::NoPointsInHole);
        }
        if problem.figure.vertices.is_empty() {
            return Err(CreateError::NoPointsInFigure);
        }

        let field_min = problem::Point(
            problem.hole.iter()
                .chain(problem.figure.vertices.iter())
                .map(|p| p.0)
                .min()
                .unwrap(),
            problem.hole.iter()
                .chain(problem.figure.vertices.iter())
                .map(|p| p.1)
                .min()
                .unwrap(),
        );
        let field_max = problem::Point(
            problem.hole.iter()
                .chain(problem.figure.vertices.iter())
                .map(|p| p.0)
                .max()
                .unwrap(),
            problem.hole.iter()
                .chain(problem.figure.vertices.iter())
                .map(|p| p.1)
                .max()
                .unwrap(),
        );

        // actually build a tree
        let root = quad_tree_build(
            &problem.hole_polygon_f64(),
            problem::Point(field_min.0 - 2, field_min.1 - 2),
            problem::Point(field_max.0 + 2, field_max.1 + 2),
        ).ok_or(CreateError::FieldIsTooSmall)?;

        Ok(GeoHoleQuadTree {
            root,
            geo_hole: problem.hole_polygon_f64(),
        })
    }

    pub fn iter(&self) -> NodesIterator {
        NodesIterator { queue: vec![&self.root], }
    }

}

impl problem::InvalidEdge for GeoHoleQuadTree {
    fn is_edge_invalid(&self, edge_from: problem::Point, edge_to: problem::Point) -> bool {
        // let geo_start = geo::Coordinate {
        //     x: if edge_from.0 < edge_to.0 { edge_from.0 as f64 } else { (edge_from.0 + 1) as f64 },
        //     y: if edge_from.1 < edge_to.1 { edge_from.1 as f64 } else { (edge_from.1 + 1) as f64 },
        // };
        // let geo_end = geo::Coordinate {
        //     x: if edge_to.0 < edge_from.0 { edge_to.0 as f64 } else { (edge_to.0 + 1) as f64 },
        //     y: if edge_to.1 < edge_from.1 { edge_to.1 as f64 } else { (edge_to.1 + 1) as f64 },
        // };
        let geo_start = geo::Coordinate::from(edge_from);
        let geo_end = geo::Coordinate::from(edge_to);
        let geo_edge = geo::Line { start: geo_start, end: geo_end, };
        match quad_tree_edge_node_intersection(&self.root, &geo_edge) {
            IntersectsNode::Outside =>
                true,
            IntersectsNode::Inside =>
                false,
            IntersectsNode::DoesNot =>
            // slow path: actually check polygon
                self.geo_hole.is_edge_invalid(edge_from, edge_to),
        }
    }
}

pub struct NodesIterator<'a> {
    queue: Vec<&'a Node>,
}

impl<'a> Iterator for NodesIterator<'a> {
    type Item = &'a Node;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let node = self.queue.pop()?;

            match &node.kind {
                NodeKind::Inside | NodeKind::Outside =>
                    return Some(node),
                NodeKind::Branch { children, } =>
                    self.queue.extend(children.iter()),
            }
        }
    }
}

fn quad_tree_build(hole: &geo::Polygon<f64>, min: problem::Point, max: problem::Point) -> Option<Node> {
    if min.0 > max.0 || min.1 > max.1 {
        return None;
    }

    let rect = geo::Rect::new(
        geo::Coordinate { x: min.0 as f64, y: min.1 as f64, },
        geo::Coordinate { x: max.0 as f64, y: max.1 as f64, },
    );
    let intersection_matrix = hole.relate(&rect);

    log::debug!(
        "quad_tree_build({:?}, {:?}) | matrix = {:?}|{:?}|{:?}|{:?}",
        min,
        max,
        intersection_matrix.is_intersects(),
        intersection_matrix.is_disjoint(),
        intersection_matrix.is_contains(),
        intersection_matrix.is_within(),
    );

    if intersection_matrix.is_disjoint() {
        // log::debug!(" > NodeKind::Outside");
        Some(Node { min, max, kind: NodeKind::Outside, })
    } else if intersection_matrix.is_contains() {
        // log::debug!(" > NodeKind::Inside");
        Some(Node { min, max, kind: NodeKind::Inside, })
    } else if min == max {
        // assert!(intersection_matrix.is_intersects());
        // Some(Node { min, max, kind: NodeKind::Inside, })
        None
    } else {
        let center = problem::Point((max.0 + min.0) / 2, (max.1 + min.1) / 2);
        // log::debug!(" > NodeKind::Branch @ {:?} | matrix = {:?}", center, intersection_matrix);
        let children: Vec<_> = quad_tree_build(hole, min, center).into_iter()
            .chain(quad_tree_build(hole, problem::Point(center.0 + 1, min.1), problem::Point(max.0, center.1)))
            .chain(quad_tree_build(hole, problem::Point(min.0, center.1 + 1), problem::Point(center.0, max.1)))
            .chain(quad_tree_build(hole, problem::Point(center.0 + 1, center.1 + 1), max))
            .collect();
        if children.is_empty() {
            None
        } else {
            Some(Node { min, max, kind: NodeKind::Branch { children, }, })
        }
    }
}

enum IntersectsNode {
    DoesNot,
    Inside,
    Outside,
}

fn quad_tree_edge_node_intersection(node: &Node, edge: &geo::Line<f64>) -> IntersectsNode {
    let rect = geo::Rect::new(
        geo::Coordinate { x: node.min.0 as f64, y: node.min.1 as f64, },
        geo::Coordinate { x: (node.max.0 + 1) as f64, y: (node.max.1 + 1) as f64, },
    );
    if !edge.intersects(&rect) {
        return IntersectsNode::DoesNot;
    }
    match &node.kind {
        NodeKind::Inside =>
            IntersectsNode::Inside,
        NodeKind::Outside =>
            IntersectsNode::Outside,
        NodeKind::Branch { children, } => {
            for child in children {
                match quad_tree_edge_node_intersection(child, edge) {
                    IntersectsNode::DoesNot =>
                        return IntersectsNode::DoesNot,
                    IntersectsNode::Inside =>
                        (),
                    IntersectsNode::Outside =>
                        return IntersectsNode::Outside,
                }
            }
            IntersectsNode::Inside
        }
    }
}

#[cfg(test)]
mod tests {
use rand::Rng;
    use crate::{
        problem::{
            self,
            InvalidEdge,
        },
    };
    use super::{
        GeoHoleQuadTree,
    };

    #[test]
    fn autocheck_on_problem_3() {
        let problem_data = r#"{"bonuses":[{"bonus":"GLOBALIST","problem":60,"position":[45,110]},{"bonus":"GLOBALIST","problem":81,"position":[39,38]},{"bonus":"WALLHACK","problem":59,"position":[90,59]}],"hole":[[50,70],[35,75],[35,65],[15,55],[30,45],[25,30],[30,30],[30,15],[45,25],[55,35],[55,15],[65,20],[80,5],[85,25],[90,25],[80,45],[95,45],[105,50],[100,65],[85,70],[90,85],[65,80],[60,85],[55,70],[50,110],[45,110]],"epsilon":180000,"figure":{"edges":[[9,17],[17,22],[22,27],[27,19],[19,14],[14,8],[8,9],[22,28],[28,30],[9,6],[6,4],[19,23],[23,24],[24,20],[20,21],[14,10],[10,11],[11,15],[15,16],[23,29],[29,32],[10,7],[7,2],[24,33],[33,35],[11,3],[3,0],[21,25],[25,26],[26,18],[18,13],[13,12],[12,16],[15,5],[5,1],[20,31],[31,34],[16,21]],"vertices":[[15,70],[25,100],[30,35],[30,55],[35,10],[35,75],[40,25],[40,40],[45,35],[50,25],[50,50],[50,60],[50,75],[50,95],[55,45],[55,65],[55,70],[60,20],[60,105],[65,45],[65,65],[65,70],[70,25],[70,50],[70,60],[70,75],[70,95],[75,35],[80,25],[80,40],[85,10],[85,75],[90,35],[90,55],[95,100],[105,70]]}}"#;
        let problem: problem::Problem = serde_json::from_str(problem_data).unwrap();

        let geo_hole_quad_tree = GeoHoleQuadTree::new(&problem).unwrap();

        let geo_hole = problem.hole_polygon_f64();

        let field_min = problem::Point(
            problem.hole.iter()
                .chain(problem.figure.vertices.iter())
                .map(|p| p.0)
                .min()
                .unwrap(),
            problem.hole.iter()
                .chain(problem.figure.vertices.iter())
                .map(|p| p.1)
                .min()
                .unwrap(),
        );
        let field_max = problem::Point(
            problem.hole.iter()
                .chain(problem.figure.vertices.iter())
                .map(|p| p.0)
                .max()
                .unwrap(),
            problem.hole.iter()
                .chain(problem.figure.vertices.iter())
                .map(|p| p.1)
                .max()
                .unwrap(),
        );

        let mut rng = rand::thread_rng();
        for _ in 0 .. 32768 {
            let pa = problem::Point(
                rng.gen_range(field_min.0 - 10 ..= field_max.0 + 10),
                rng.gen_range(field_min.1 - 10 ..= field_max.1 + 10),
            );
            let pb = problem::Point(
                rng.gen_range(field_min.0 - 10 ..= field_max.0 + 10),
                rng.gen_range(field_min.1 - 10 ..= field_max.1 + 10),
            );
            let orig = geo_hole.is_edge_invalid(pa, pb);
            let test = geo_hole_quad_tree.is_edge_invalid(pa, pb);
            assert_eq!(orig, test);
        }
    }

    #[test]
    fn autocheck_on_problem_11() {
        let problem_data = r#"{"bonuses":[{"bonus":"BREAK_A_LEG","problem":31,"position":[5,5]},{"bonus":"GLOBALIST","problem":20,"position":[9,6]},{"bonus":"GLOBALIST","problem":49,"position":[6,9]}],"hole":[[10,0],[10,10],[0,10]],"epsilon":0,"figure":{"edges":[[0,1],[1,2],[2,0]],"vertices":[[0,0],[10,0],[10,10]]}}"#;
        let problem: problem::Problem = serde_json::from_str(problem_data).unwrap();

        let geo_hole_quad_tree = GeoHoleQuadTree::new(&problem).unwrap();

        let geo_hole = problem.hole_polygon_f64();

        let field_min = problem::Point(
            problem.hole.iter()
                .chain(problem.figure.vertices.iter())
                .map(|p| p.0)
                .min()
                .unwrap(),
            problem.hole.iter()
                .chain(problem.figure.vertices.iter())
                .map(|p| p.1)
                .min()
                .unwrap(),
        );
        let field_max = problem::Point(
            problem.hole.iter()
                .chain(problem.figure.vertices.iter())
                .map(|p| p.0)
                .max()
                .unwrap(),
            problem.hole.iter()
                .chain(problem.figure.vertices.iter())
                .map(|p| p.1)
                .max()
                .unwrap(),
        );

        let mut rng = rand::thread_rng();
        for _ in 0 .. 32768 {
            let pa = problem::Point(
                rng.gen_range(field_min.0 - 10 ..= field_max.0 + 10),
                rng.gen_range(field_min.1 - 10 ..= field_max.1 + 10),
            );
            let pb = problem::Point(
                rng.gen_range(field_min.0 - 10 ..= field_max.0 + 10),
                rng.gen_range(field_min.1 - 10 ..= field_max.1 + 10),
            );
            let orig = geo_hole.is_edge_invalid(pa, pb);
            let test = geo_hole_quad_tree.is_edge_invalid(pa, pb);
            assert_eq!(orig, test);
        }
    }
}
