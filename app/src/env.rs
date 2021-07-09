use std::{
    mem,
};

use geo::{
    algorithm::{
        rotate::{
            RotatePoint,
        },
    },
};

use piston_window::{
    Viewport,
};

use common::{
    problem,
};

use crate::{
    draw,
};

#[derive(Debug)]
pub struct Env {
    screen_width: u32,
    screen_height: u32,
    console_height: u32,
    border_width: u32,
    problem: problem::Problem,
    initial_problem: problem::Problem,
    original_pose: problem::Pose,
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
    mouse_cursor: Option<[f64; 2]>,
    score_state: ScoringState,
    drag_state: DragState,
}

#[derive(Debug)]
enum DragState {
    WantVertex,
    WantVertexHighlight { vertex_index: usize, },
    WantTarget { vertex_index: usize, allowed: Vec<problem::Point>, },
    WantTargetHighlight { vertex_index: usize, allowed: Vec<problem::Point>, candidate: problem::Point, },
}

pub struct ViewportTranslator {
    console_height: u32,
    border_width: u32,
    scale_x: f64,
    scale_y: f64,
    min_x: f64,
    min_y: f64,
}

#[derive(Debug)]
pub enum CreateError {
    NoPointsInHole,
    NoPointsInFigure,
}

#[derive(Debug)]
pub enum DrawError {
    NoPointsInHole,
    NoPointsInFigure,
    InvalidEdgeSourceIndex { edge: problem::Edge, index: usize, },
    InvalidEdgeTargetIndex { edge: problem::Edge, index: usize, },
}

#[derive(Debug)]
pub enum RotateError {
    GeoExport(problem::GeoExportError),
    GeoImport(problem::GeoImportError),
    NoCentroidBuilt,
}

#[derive(Debug)]
enum ScoringState {
    Unscored,
    Ok(i64),
    VerticeCountMismatch,
    BrokenEdgesFound(Vec<problem::Edge>),
    EdgesNotFitHole(Vec<problem::Edge>),
}


impl Env {
    pub fn new(
        problem: problem::Problem,
        screen_width: u32,
        screen_height: u32,
        console_height: u32,
        border_width: u32,
    )
        -> Result<Env, CreateError>
    {
        let min_x_hole = problem
            .hole
            .iter()
            .map(|p| p.0)
            .min()
            .ok_or(CreateError::NoPointsInHole)?;
        let min_x_figure = problem
            .figure
            .vertices
            .iter()
            .map(|p| p.0)
            .min()
            .ok_or(CreateError::NoPointsInFigure)?;
        let min_y_hole = problem
            .hole
            .iter()
            .map(|p| p.1)
            .min()
            .ok_or(CreateError::NoPointsInHole)?;
        let min_y_figure = problem
            .figure
            .vertices
            .iter()
            .map(|p| p.1)
            .min()
            .ok_or(CreateError::NoPointsInFigure)?;
        let max_x_hole = problem
            .hole
            .iter()
            .map(|p| p.0)
            .max()
            .ok_or(CreateError::NoPointsInHole)?;
        let max_x_figure = problem
            .figure
            .vertices
            .iter()
            .map(|p| p.0)
            .max()
            .ok_or(CreateError::NoPointsInFigure)?;
        let max_y_hole = problem
            .hole
            .iter()
            .map(|p| p.1)
            .max()
            .ok_or(CreateError::NoPointsInHole)?;
        let max_y_figure = problem
            .figure
            .vertices
            .iter()
            .map(|p| p.1)
            .max()
            .ok_or(CreateError::NoPointsInFigure)?;

        Ok(Env {
            screen_width,
            screen_height,
            console_height,
            border_width,
            original_pose: problem.export_pose(),
            initial_problem: problem.clone(),
            problem,
            min_x: if min_x_hole > min_x_figure { min_x_figure } else { min_x_hole } as f64,
            min_y: if min_y_hole > min_y_figure { min_y_figure } else { min_y_hole } as f64,
            max_x: if max_x_hole < max_x_figure { max_x_figure } else { max_x_hole } as f64,
            max_y: if max_y_hole < max_y_figure { max_y_figure } else { max_y_hole } as f64,
            mouse_cursor: None,
            score_state: ScoringState::Unscored,
            drag_state: DragState::WantVertex,
        })
    }

    pub fn translator(&self, viewport: &Option<Viewport>) -> Option<ViewportTranslator> {
        let (w, h) = viewport
            .map(|v| (v.draw_size[0], v.draw_size[1]))
            .unwrap_or((self.screen_width, self.screen_height));

        if (w <= 2 * self.border_width) || (h <= self.border_width + self.console_height) {
            None
        } else {
            Some(ViewportTranslator {
                console_height: self.console_height,
                border_width: self.border_width,
                scale_x: (w - (self.border_width * 2)) as f64 / (self.max_x - self.min_x),
                scale_y: (h - (self.border_width + self.console_height)) as f64 / (self.max_y - self.min_y),
                min_x: self.min_x,
                min_y: self.min_y,
            })
        }
    }

    pub fn console_text(&self) -> String {
        format!(
            "move: W/A/S/D, rotate: Z/X, export pose: E, drag: {}, {}",
            match self.drag_state {
                DragState::WantVertex |
                DragState::WantVertexHighlight { .. } =>
                    "choose vertex".to_string(),
                DragState::WantTarget { .. } |
                DragState::WantTargetHighlight { .. } =>
                    "choose new vertex position (M to reset)".to_string(),
            },
            match &self.score_state {
                ScoringState::Unscored => "<unscored>".to_string(),
                ScoringState::Ok(score) => format!("score: {}", score),
                ScoringState::VerticeCountMismatch => "score err: vertice count mismatch".to_string(),
                ScoringState::BrokenEdgesFound(edges) => format!("score err: {} broken edges found", edges.len()),
                ScoringState::EdgesNotFitHole(edges) => format!("score err: {} edges does fit hole", edges.len()),
            },
        )
    }

    pub fn draw<DF>(&mut self, tr: &ViewportTranslator, mut draw_element: DF) -> Result<(), DrawError> where DF: FnMut(draw::DrawElement) {
        let mut points_iter = self.problem.hole.iter();
        let mut prev_point = points_iter.next()
            .ok_or(DrawError::NoPointsInHole)?;
        for point in points_iter.chain(Some(prev_point)) {
            draw_element(draw::DrawElement::Line {
                color: [0., 0., 1., 1.,],
                radius: 1.0,
                source_x: prev_point.0 as f64,
                source_y: prev_point.1 as f64,
                target_x: point.0 as f64,
                target_y: point.1 as f64,
            });
            prev_point = point;
        }

        for &edge in &self.problem.figure.edges {
            let source_point = self.problem.figure.vertices.get(edge.0)
                .ok_or(DrawError::InvalidEdgeSourceIndex { edge, index: edge.0, })?;
            let target_point = self.problem.figure.vertices.get(edge.1)
                .ok_or(DrawError::InvalidEdgeTargetIndex { edge, index: edge.1, })?;
            draw_element(draw::DrawElement::Line {
                color: [1., 1., 0., 1.,],
                radius: 0.5,
                source_x: source_point.0 as f64,
                source_y: source_point.1 as f64,
                target_x: target_point.0 as f64,
                target_y: target_point.1 as f64,
            });
        }

        if let Some(mouse_cursor) = self.mouse_cursor {
            match mem::replace(&mut self.drag_state, DragState::WantVertex) {
                DragState::WantVertex |
                DragState::WantVertexHighlight { .. } => {
                    self.drag_state = DragState::WantVertex;
                    for (vertex_index, vertex) in self.problem.figure.vertices.iter().enumerate() {
                        let vertex_x = tr.x(vertex.0 as f64);
                        let vertex_y = tr.y(vertex.1 as f64);
                        let sq_dist =
                            (mouse_cursor[0] - vertex_x) * (mouse_cursor[0] - vertex_x) +
                            (mouse_cursor[1] - vertex_y) * (mouse_cursor[1] - vertex_y);
                        if sq_dist < 32.0 {
                            draw_element(draw::DrawElement::Ellipse {
                                color: [0.0, 1.0, 0.0, 1.0],
                                x: vertex.0 as f64,
                                y: vertex.1 as f64,
                                width: 16.0,
                                height: 16.0,
                            });
                            self.drag_state = DragState::WantVertexHighlight { vertex_index, };
                            break;
                        }
                    }
                },
                DragState::WantTarget { vertex_index, allowed, } |
                DragState::WantTargetHighlight { vertex_index, allowed, .. } => {
                    let vertex = self.problem.figure.vertices[vertex_index];
                    draw_element(draw::DrawElement::Ellipse {
                        color: [0.0, 1.0, 0.0, 1.0],
                        x: vertex.0 as f64,
                        y: vertex.1 as f64,
                        width: 16.0,
                        height: 16.0,
                    });
                    let mut highlight = None;
                    for &vertex in &allowed {
                        let vertex_x = tr.x(vertex.0 as f64);
                        let vertex_y = tr.y(vertex.1 as f64);
                        let sq_dist =
                            (mouse_cursor[0] - vertex_x) * (mouse_cursor[0] - vertex_x) +
                            (mouse_cursor[1] - vertex_y) * (mouse_cursor[1] - vertex_y);
                        if sq_dist < 32.0 {
                            draw_element(draw::DrawElement::Ellipse {
                                color: [1.0, 0.0, 0.0, 1.0],
                                x: vertex.0 as f64,
                                y: vertex.1 as f64,
                                width: 16.0,
                                height: 16.0,
                            });
                            highlight = Some(vertex);
                        } else {
                            draw_element(draw::DrawElement::Ellipse {
                                color: [1.0, 0.0, 0.0, 1.0],
                                x: vertex.0 as f64,
                                y: vertex.1 as f64,
                                width: 16.0,
                                height: 16.0,
                            });
                        }
                    }
                    if let Some(candidate) = highlight {
                        self.drag_state = DragState::WantTargetHighlight { vertex_index, allowed, candidate, };
                    } else {
                        self.drag_state = DragState::WantTarget { vertex_index, allowed, };
                    }
                },
            }
        }

        Ok(())
    }

    pub fn update_mouse_cursor(&mut self, position: [f64; 2]) {
        self.mouse_cursor = Some(position);
    }

    pub fn mouse_cursor_left(&mut self) {
        self.mouse_cursor = None;
    }

    pub fn mouse_click(&mut self) {
        match mem::replace(&mut self.drag_state, DragState::WantVertex) {
            DragState::WantVertex =>
                (),
            DragState::WantVertexHighlight { vertex_index, } => {
                let vertex = self.problem.figure.vertices[vertex_index];
                let connected_edges: Vec<_> = self.problem
                    .figure
                    .edges
                    .iter()
                    .filter(|e| e.0 == vertex_index || e.1 == vertex_index)
                    .collect();
                let max_edge_len = connected_edges
                    .iter()
                    .flat_map(|e| {
                        Some(dist(self.problem.figure.vertices[e.0], vertex))
                            .into_iter()
                            .chain(Some(dist(self.problem.figure.vertices[e.1], vertex)))
                    })
                    .max()
                    .unwrap();
                let max_edge_len = (max_edge_len as f64 * 1.25) as i64;

                let mut allowed = Vec::new();
                for try_x in vertex.0 - max_edge_len .. vertex.0 + max_edge_len {
                    for try_y in vertex.1 - max_edge_len .. vertex.1 + max_edge_len {
                        if (try_x as f64) < self.min_x || (try_x as f64) > self.max_x || (try_y as f64) < self.min_y || (try_y as f64) > self.max_y {
                            continue;
                        }
                        let try_vertex = problem::Point(try_x, try_y);
                        let mut is_ok = true;
                        for edge in &connected_edges {
                            let sample_vertex_a = self.original_pose.vertices[edge.0];
                            let sample_vertex_b = self.original_pose.vertices[edge.1];

                            let other_vertex_index = if edge.0 == vertex_index { edge.1 } else { edge.0 };
                            let other_vertex = self.problem.figure.vertices[other_vertex_index];

                            let orig_sq_dist = (sample_vertex_a.0 - sample_vertex_b.0) * (sample_vertex_a.0 - sample_vertex_b.0)
                                + (sample_vertex_a.1 - sample_vertex_b.1) * (sample_vertex_a.1 - sample_vertex_b.1);
                            let try_sq_dist = (try_vertex.0 - other_vertex.0) * (try_vertex.0 - other_vertex.0)
                                + (try_vertex.1 - other_vertex.1) * (try_vertex.1 - other_vertex.1);

                            let ratio = ((try_sq_dist as f64 / orig_sq_dist as f64) - 1.0).abs();
                            if ratio > self.problem.epsilon as f64 / 1000000.0 {
                                is_ok = false;
                                break;
                            }
                        }
                        if is_ok {
                            allowed.push(try_vertex);
                        }
                    }
                }
                self.drag_state = DragState::WantTarget { vertex_index, allowed, };
                self.import_solution(self.export_solution());
            },
            DragState::WantTarget { vertex_index, allowed, } =>
                self.drag_state = DragState::WantTarget { vertex_index, allowed, },
            DragState::WantTargetHighlight { vertex_index, candidate, .. } => {
                self.problem.figure.vertices[vertex_index] = candidate;
                self.import_solution(self.export_solution());
            },
        }
    }

    pub fn reset_drag(&mut self) {
        self.drag_state = DragState::WantVertex;
    }

    pub fn move_figure_left(&mut self) {
        for point in &self.problem.figure.vertices {
            if point.0 < 1 {
                return;
            }
        }
        for point in &mut self.problem.figure.vertices {
            point.0 -= 1;
        }

        self.import_solution(self.export_solution())
    }

    pub fn move_figure_right(&mut self) {
        for point in &self.problem.figure.vertices {
            if point.0 + 1 > self.max_x as i64 {
                return;
            }
        }
        for point in &mut self.problem.figure.vertices {
            point.0 += 1;
        }

        self.import_solution(self.export_solution())
    }

    pub fn move_figure_upper(&mut self) {
        for point in &self.problem.figure.vertices {
            if point.1 < 1 {
                return;
            }
        }
        for point in &mut self.problem.figure.vertices {
            point.1 -= 1;
        }

        self.import_solution(self.export_solution())
    }

    pub fn move_figure_lower(&mut self) {
        for point in &self.problem.figure.vertices {
            if point.1 + 1 > self.max_y as i64 {
                return;
            }
        }
        for point in &mut self.problem.figure.vertices {
            point.1 += 1;
        }

        self.import_solution(self.export_solution())
    }

    pub fn rotate_figure_left(&mut self) -> Result<(), RotateError> {
        let geo_figure = self.problem.figure.export_to_geo()
            .map_err(RotateError::GeoExport)?;

        let rotated_points: Vec<_> = geo_figure
            .points
            .iter()
            .map(|p| p.rotate_around_point(-15.0, geo_figure.centroid))
            .collect();

        for point in &rotated_points {
            if point.x() < self.min_x || point.x() > self.max_x || point.y() < self.min_y || point.y() > self.max_y {
                return Ok(());
            }
        }

        self.problem.figure.import_from_geo(rotated_points)
            .map_err(RotateError::GeoImport)?;

        self.import_solution(self.export_solution());
        Ok(())
    }

    pub fn rotate_figure_right(&mut self) -> Result<(), RotateError> {
        let geo_figure = self.problem.figure.export_to_geo()
            .map_err(RotateError::GeoExport)?;

        let rotated_points: Vec<_> = geo_figure
            .points
            .iter()
            .map(|p| p.rotate_around_point(15.0, geo_figure.centroid))
            .collect();

        for point in &rotated_points {
            if point.x() < self.min_x || point.x() > self.max_x || point.y() < self.min_y || point.y() > self.max_y {
                return Ok(());
            }
        }

        self.problem.figure.import_from_geo(rotated_points)
            .map_err(RotateError::GeoImport)?;
        self.import_solution(self.export_solution());
        Ok(())
    }

    pub fn import_solution(&mut self, pose: problem::Pose) {
        self.problem = self.initial_problem.clone();
        let score = self.problem.import_pose(pose);
        match score {
            Ok(score_value) => {
                log::debug!(" ;; pose loaded successfully, score: {:?}", score);
                self.score_state = ScoringState::Ok(score_value);
            },
            Err(problem::PoseValidationError::VerticeCountMismatch) => {
                log::debug!(" ;; pose load failure, vertice mismatch");
                self.score_state = ScoringState::VerticeCountMismatch;
            },
            Err(problem::PoseValidationError::BrokenEdgesFound(edges)) => {
                log::debug!(" ;; pose load failure, broken edges found");
                self.score_state = ScoringState::BrokenEdgesFound(edges);
            },
            Err(problem::PoseValidationError::EdgesNotFitHole(edges)) => {
                log::debug!(" ;; pose load failure, edges not fitting hole found");
                self.score_state = ScoringState::EdgesNotFitHole(edges);
            },
        }
    }

    pub fn export_solution(&self) -> problem::Pose {
        self.problem.export_pose()
    }

    pub fn figure_reset(&mut self) {
        self.problem.figure.vertices = self.original_pose.vertices.clone();
    }
}

impl ViewportTranslator {
    pub fn x(&self, x: f64) -> f64 {
        (x - self.min_x) * self.scale_x + self.border_width as f64
    }

    pub fn y(&self, y: f64) -> f64 {
        (y - self.min_y) * self.scale_y + self.console_height as f64
    }

    // pub fn back_x(&self, viewport_x: f64) -> f64 {
    //     (viewport_x - self.border_width as f64) / self.scale_x + self.min_x
    // }

    // pub fn back_y(&self, viewport_y: f64) -> f64 {
    //     (viewport_y - self.console_height as f64) / self.scale_y + self.min_y
    // }
}

fn dist(point_a: problem::Point, point_b: problem::Point) -> i64 {
    let sq = (point_a.0 - point_b.0) * (point_a.0 - point_b.0) +
        (point_a.1 - point_b.1) * (point_a.1 - point_b.1);
    (sq as f64).sqrt() as i64
}
