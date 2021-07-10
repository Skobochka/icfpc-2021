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
    solver,
};

use crate::{
    draw,
};

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
    allowed_angles: Vec<f64>,
    selected_angle: Option<f64>,
    solver_mode: SolverMode,
}

enum SolverMode {
    None,
    SimulatedAnnealing {
        solver: solver::simulated_annealing::SimulatedAnnealingSolver,
    },
}

#[derive(Debug)]
enum DragState {
    WantVertex,
    WantVertexHighlight { vertex_index: usize, },
    WantTarget { vertex_index: usize, allowed: Vec<AllowedMove>, },
    WantTargetHighlight { vertex_index: usize, allowed: Vec<AllowedMove>, candidate: AllowedMove, },
    WantEdgeTarget { edge: problem::Edge, allowed: Vec<(problem::Point, problem::Point)>, },
    WantEdgeTargetHighlight { edge: problem::Edge, allowed: Vec<(problem::Point, problem::Point)>, candidate: (problem::Point, problem::Point), },
}

#[derive(Clone, Copy, Debug)]
enum AllowedMove {
    FoldVertex { target: problem::Point, },
    ChooseEdge { other_index: usize, },
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
pub enum SimulatedAnnealingSolverError {
    SolverCreate(solver::CreateError),
    SolverStep(solver::simulated_annealing::StepError),
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

        let min_x = if min_x_hole > min_x_figure { min_x_figure } else { min_x_hole } as f64;
        let min_y = if min_y_hole > min_y_figure { min_y_figure } else { min_y_hole } as f64;
        let max_x = if max_x_hole < max_x_figure { max_x_figure } else { max_x_hole } as f64;
        let max_y = if max_y_hole < max_y_figure { max_y_figure } else { max_y_hole } as f64;

        Ok(Env {
            screen_width,
            screen_height,
            console_height,
            border_width,
            original_pose: problem.export_pose(),
            initial_problem: problem.clone(),
            allowed_angles: problem.possible_rotations(),
            selected_angle: None,
            problem,
            min_x: min_x - ((max_x - min_x) / 2.0),
            min_y: min_y - ((max_y - min_y) / 2.0),
            max_x: max_x + ((max_x - min_x) / 2.0),
            max_y: max_y + ((max_x - min_x) / 2.0),
            mouse_cursor: None,
            score_state: ScoringState::Unscored,
            drag_state: DragState::WantVertex,
            solver_mode: SolverMode::None,
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
        match &self.solver_mode {
            SolverMode::None =>
                format!(
                    "move: W/A/S/D, rotate: Z/X, next/prev angle: C/V, export pose: E, drag: {}, {}, sel.angle: {}, angles: {:?}",
                    match self.drag_state {
                        DragState::WantVertex |
                        DragState::WantVertexHighlight { .. } =>
                            "choose vertex".to_string(),
                        DragState::WantTarget { .. } |
                        DragState::WantTargetHighlight { .. } =>
                            "choose new vertex position or edge (M to reset)".to_string(),
                        DragState::WantEdgeTarget { .. } |
                        DragState::WantEdgeTargetHighlight { .. } =>
                            "choose new edge position (M to reset)".to_string(),
                    },
                    match &self.score_state {
                        ScoringState::Unscored => "<unscored>".to_string(),
                        ScoringState::Ok(score) => format!("score: {}", score),
                        ScoringState::VerticeCountMismatch => "score err: vertice count mismatch".to_string(),
                        ScoringState::BrokenEdgesFound(edges) => format!("score err: {} broken edges found", edges.len()),
                        ScoringState::EdgesNotFitHole(edges) => format!("score err: {} edges does fit hole", edges.len()),
                    },
                    match self.selected_angle {
                        None => "<n/a>".to_string(),
                        Some(a) => format!("{}", a),
                    },
                    self.allowed_angles,
                ),
            SolverMode::SimulatedAnnealing { solver, } =>
                format!("exit solver: Y, step: I, temp: {}, fitness: {:?}, energy: {}", solver.temp(), solver.fitness(), solver.fitness().energy()),
        }
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

        match &self.solver_mode {
            SolverMode::None => {
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
            },
            SolverMode::SimulatedAnnealing { solver, } => {
                let solver_vertices = solver.vertices();
                for &edge in &self.problem.figure.edges {
                    let source_point = solver_vertices.get(edge.0)
                        .ok_or(DrawError::InvalidEdgeSourceIndex { edge, index: edge.0, })?;
                    let target_point = solver_vertices.get(edge.1)
                        .ok_or(DrawError::InvalidEdgeTargetIndex { edge, index: edge.1, })?;
                    draw_element(draw::DrawElement::Line {
                        color: [1., 1., 0., 1.,],
                        radius: 0.5,
                        source_x: source_point.0 as f64,
                        source_y: source_point.1 as f64,
                        target_x: target_point.0 as f64,
                        target_y: target_point.1 as f64,
                    });
                    draw_element(draw::DrawElement::Ellipse {
                        color: [1.0, 0.0, 0.0, 1.0],
                        x: source_point.0 as f64,
                        y: source_point.1 as f64,
                        width: 16.0,
                        height: 16.0,
                    });
                }

            },
        }

        if let Some(bonuses) = self.problem.bonuses.as_ref() {
            for bonus in bonuses {
                draw_element(draw::DrawElement::Ellipse {
                    color: [0.4, 0.7, 0.3, 0.85],
                    x: bonus.position.0 as f64,
                    y: bonus.position.1 as f64,
                    width: 16.0,
                    height: 16.0,
                });
                draw_element(draw::DrawElement::Text {
                    color: [1.0, 1.0, 1.0, 1.0],
                    size: 16,
                    text: match bonus.bonus {
                        problem::ProblemBonusType::BreakALeg =>
                            "break_a_leg".to_string(),
                        problem::ProblemBonusType::Globalist =>
                            "globalist".to_string(),
                    },
                    x: bonus.position.0 as f64,
                    y: bonus.position.1 as f64,
                });
            }
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
                    for &allowed_move in &allowed {
                        let vertex = match allowed_move {
                            AllowedMove::FoldVertex { target, } =>
                                target,
                            AllowedMove::ChooseEdge { other_index, } =>
                                self.problem.figure.vertices[other_index],
                        };
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
                            highlight = Some(allowed_move);
                        } else {
                            draw_element(draw::DrawElement::Ellipse {
                                color: match allowed_move {
                                    AllowedMove::FoldVertex { .. } =>
                                        [1.0, 0.0, 0.0, 1.0],
                                    AllowedMove::ChooseEdge { .. } =>
                                        [1.0, 1.0, 0.0, 1.0],
                                },
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
                DragState::WantEdgeTarget { edge, allowed, } |
                DragState::WantEdgeTargetHighlight { edge, allowed, .. } => {
                    draw_element(draw::DrawElement::Line {
                        color: [1., 0., 0., 1.,],
                        radius: 2.0,
                        source_x: self.problem.figure.vertices[edge.0].0 as f64,
                        source_y: self.problem.figure.vertices[edge.0].1 as f64,
                        target_x: self.problem.figure.vertices[edge.1].0 as f64,
                        target_y: self.problem.figure.vertices[edge.1].1 as f64,
                    });
                    let mut highlight = None;
                    for &(p, q) in &allowed {

                        draw_element(draw::DrawElement::Line {
                            color: [1., 0., 0., 1.,],
                            radius: 2.0,
                            source_x: p.0 as f64,
                            source_y: p.1 as f64,
                            target_x: q.0 as f64,
                            target_y: q.1 as f64,
                        });

                        let vertex = p;
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
                            highlight = Some((p, q));
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
                        self.drag_state = DragState::WantEdgeTargetHighlight { edge, allowed, candidate, };
                    } else {
                        self.drag_state = DragState::WantEdgeTarget { edge, allowed, };
                    }
                },

            }
        }

        Ok(())
    }

    pub fn enter_solver_simulated_annealing(&mut self) -> Result<(), SimulatedAnnealingSolverError> {
        let solver = solver::simulated_annealing::SimulatedAnnealingSolver::new(
            solver::Solver::new(&self.problem)
                .map_err(SimulatedAnnealingSolverError::SolverCreate)?,
            solver::simulated_annealing::Params {
                max_temp: 100.0,
                cooling_step_temp: 1.0,
                minimum_temp: 2.0,
                iterations_per_cooling_step: 100,
            },
        );
        self.solver_mode = SolverMode::SimulatedAnnealing { solver, };
        Ok(())
    }

    pub fn step_solver_simulated_annealing(&mut self) -> Result<(), SimulatedAnnealingSolverError> {
        match &mut self.solver_mode {
            SolverMode::None =>
                Ok(()),
            SolverMode::SimulatedAnnealing { solver, } =>
                solver.step().map_err(SimulatedAnnealingSolverError::SolverStep)
        }
    }

    pub fn exit_solver(&mut self) {
        self.solver_mode = SolverMode::None;
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
                let mut allowed = Vec::new();
                let connected_edges: Vec<_> = self.problem
                    .figure
                    .edges
                    .iter()
                    .filter(|e| e.0 == vertex_index || e.1 == vertex_index)
                    .collect();
                for edge in &connected_edges {
                    if edge.0 != vertex_index {
                        allowed.push(AllowedMove::ChooseEdge { other_index: edge.0, });
                    }
                    if edge.1 != vertex_index {
                        allowed.push(AllowedMove::ChooseEdge { other_index: edge.1, });
                    }
                }

                for try_x in self.min_x as i64 ..= self.max_x as i64 {
                    for try_y in self.min_y as i64 .. self.max_y as i64 {
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
                            allowed.push(AllowedMove::FoldVertex { target: try_vertex, });
                        }
                    }
                }
                self.drag_state = DragState::WantTarget { vertex_index, allowed, };
                self.rescore_solution();
                self.update_angles();
            },
            DragState::WantTarget { vertex_index, allowed, } =>
                self.drag_state = DragState::WantTarget { vertex_index, allowed, },
            DragState::WantTargetHighlight { vertex_index, candidate: AllowedMove::FoldVertex { target, }, .. } => {
                self.problem.figure.vertices[vertex_index] = target;
                self.rescore_solution();
                self.update_angles();
            },
            DragState::WantTargetHighlight { vertex_index, candidate: AllowedMove::ChooseEdge { other_index, }, .. } => {
                let mut allowed = Vec::new();
                let connected_edges: Vec<_> = self.problem
                    .figure
                    .edges
                    .iter()
                    .filter(|e| e.0 == vertex_index || e.1 == vertex_index || e.0 == other_index || e.1 == other_index)
                    .collect();
                let vp = self.problem.figure.vertices[vertex_index];
                let vq = self.problem.figure.vertices[other_index];

                for try_x in self.min_x as i64 ..= self.max_x as i64 {
                    for try_y in self.min_y as i64 .. self.max_y as i64 {
                        if (try_x as f64) < self.min_x || (try_x as f64) > self.max_x || (try_y as f64) < self.min_y || (try_y as f64) > self.max_y {
                            continue;
                        }
                        let oth_x = try_x + (vq.0 - vp.0);
                        let oth_y = try_y + (vq.1 - vp.1);
                        if (oth_x as f64) < self.min_x || (oth_x as f64) > self.max_x || (oth_y as f64) < self.min_y || (oth_y as f64) > self.max_y {
                            continue;
                        }
                        let try_vertex = problem::Point(try_x, try_y);
                        let oth_vertex = problem::Point(oth_x, oth_y);

                        let mut is_ok = true;
                        for edge in &connected_edges {
                            let sample_vertex_a = self.original_pose.vertices[edge.0];
                            let sample_vertex_b = self.original_pose.vertices[edge.1];

                            let px = if edge.0 == vertex_index {
                                try_x
                            } else if edge.0 == other_index {
                                oth_x
                            } else {
                                self.problem.figure.vertices[edge.0].0
                            };
                            let py = if edge.0 == vertex_index {
                                try_y
                            } else if edge.0 == other_index {
                                oth_y
                            } else {
                                self.problem.figure.vertices[edge.0].1
                            };
                            let qx = if edge.1 == vertex_index {
                                try_x
                            } else if edge.1 == other_index {
                                oth_x
                            } else {
                                self.problem.figure.vertices[edge.1].0
                            };
                            let qy = if edge.1 == vertex_index {
                                try_y
                            } else if edge.1 == other_index {
                                oth_y
                            } else {
                                self.problem.figure.vertices[edge.1].1
                            };

                            let orig_sq_dist = (sample_vertex_a.0 - sample_vertex_b.0) * (sample_vertex_a.0 - sample_vertex_b.0)
                                + (sample_vertex_a.1 - sample_vertex_b.1) * (sample_vertex_a.1 - sample_vertex_b.1);
                            let try_sq_dist = (px - qx) * (px - qx) + (py - qy) * (py - qy);

                            let ratio = ((try_sq_dist as f64 / orig_sq_dist as f64) - 1.0).abs();
                            if ratio > self.problem.epsilon as f64 / 1000000.0 {
                                is_ok = false;
                                break;
                            }
                        }
                        if is_ok {
                            allowed.push((try_vertex, oth_vertex));
                        }
                    }
                }

                self.drag_state = DragState::WantEdgeTarget {
                    edge: problem::Edge(vertex_index, other_index),
                    allowed,
                };
            },
            DragState::WantEdgeTarget { edge, allowed, } =>
                self.drag_state = DragState::WantEdgeTarget { edge, allowed, },
            DragState::WantEdgeTargetHighlight { edge, candidate: (p, q), .. } => {
                self.problem.figure.vertices[edge.0] = p;
                self.problem.figure.vertices[edge.1] = q;
                self.rescore_solution();
                self.update_angles();
            },
        }
    }

    pub fn reset_drag(&mut self) {
        self.drag_state = DragState::WantVertex;
    }

    pub fn move_figure_left(&mut self) {
        for point in &self.problem.figure.vertices {
            if point.0 - 1 < self.min_x as i64  {
                return;
            }
        }
        for point in &mut self.problem.figure.vertices {
            point.0 -= 1;
        }

        self.rescore_solution();
        self.update_angles();
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

        self.rescore_solution();
        self.update_angles();
    }

    pub fn move_figure_upper(&mut self) {
        for point in &self.problem.figure.vertices {
            if point.1 - 1 < self.min_x as i64 {
                return;
            }
        }
        for point in &mut self.problem.figure.vertices {
            point.1 -= 1;
        }

        self.rescore_solution();
        self.update_angles();
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

        self.rescore_solution();
        self.update_angles();
    }

    pub fn rotate_figure_left(&mut self) -> Result<(), RotateError> {
        if self.selected_angle.is_none() {
            return Ok(());
        }

        let geo_figure = self.problem.figure.export_to_geo()
            .map_err(RotateError::GeoExport)?;

        let rotated_points: Vec<_> = geo_figure
            .points
            .iter()
            .map(|p| p.rotate_around_point(-self.selected_angle.unwrap(), geo_figure.centroid))
            .collect();

        for point in &rotated_points {
            if point.x() < self.min_x || point.x() > self.max_x || point.y() < self.min_y || point.y() > self.max_y {
                return Ok(());
            }
        }

        self.problem.figure.import_from_geo(rotated_points)
            .map_err(RotateError::GeoImport)?;

        self.rescore_solution();
        self.update_angles();
        self.select_next_angle();
        Ok(())
    }

    pub fn rotate_figure_right(&mut self) -> Result<(), RotateError> {
        if self.selected_angle.is_none() {
            return Ok(());
        }

        let geo_figure = self.problem.figure.export_to_geo()
            .map_err(RotateError::GeoExport)?;

        let rotated_points: Vec<_> = geo_figure
            .points
            .iter()
            .map(|p| p.rotate_around_point(self.selected_angle.unwrap(), geo_figure.centroid))
            .collect();

        for point in &rotated_points {
            if point.x() < self.min_x || point.x() > self.max_x || point.y() < self.min_y || point.y() > self.max_y {
                return Ok(());
            }
        }

        self.problem.figure.import_from_geo(rotated_points)
            .map_err(RotateError::GeoImport)?;
        self.rescore_solution();
        self.update_angles();
        self.select_next_angle();
        Ok(())
    }

    pub fn import_solution(&mut self, pose: problem::Pose) {
        self.problem = self.initial_problem.clone();
        let score = self.problem.import_pose(pose);
        self.update_score_state(score);
    }

    pub fn rescore_solution(&mut self) {
        let score = self.initial_problem.score_vertices(&self.problem.figure.vertices);
        self.update_score_state(score);
    }

    pub fn update_angles(&mut self) {
        self.allowed_angles = self.initial_problem.possible_rotations_for_vertices(&self.problem.figure.vertices);
        log::debug!("possible rotations around centroid: {:?}", self.allowed_angles);
    }

    pub fn select_next_angle(&mut self) {
        if self.allowed_angles.len() == 0 {
            self.selected_angle = None;
            return;
        }

        match self.selected_angle {
            None => {
                self.selected_angle = Some(self.allowed_angles[0]);
            },
            Some(selected_angle) => {
                match self.allowed_angles.iter().position(|&angle| angle == selected_angle) {
                    None => {
                        self.selected_angle = Some(self.allowed_angles[0]);
                    },
                    Some(angle_idx) => {
                        let mut next_idx = angle_idx + 1;
                        if next_idx >= self.allowed_angles.len() {
                            next_idx = 0;
                        }
                        self.selected_angle = Some(self.allowed_angles[next_idx]);
                    }

                }
            },
        }
    }

    pub fn select_prev_angle(&mut self) {
        if self.allowed_angles.len() == 0 {
            self.selected_angle = None;
            return;
        }

        match self.selected_angle {
            None => {
                self.selected_angle = Some(self.allowed_angles[0]);
            },
            Some(selected_angle) => {
                match self.allowed_angles.iter().position(|&angle| angle == selected_angle) {
                    None => {
                        self.selected_angle = Some(self.allowed_angles[0]);
                    },
                    Some(angle_idx) => {
                        let mut next_idx = angle_idx as isize - 1;
                        if next_idx < 0 {
                            next_idx = self.allowed_angles.len() as isize - 1;
                        }
                        self.selected_angle = Some(self.allowed_angles[next_idx as usize]);
                    }

                }
            },
        }
    }

    pub fn update_score_state(&mut self, score: Result<i64, problem::PoseValidationError>) {
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
                log::debug!(" ;; pose load failure, broken edges found: {:?}", edges);
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

// fn dist(point_a: problem::Point, point_b: problem::Point) -> i64 {
//     let sq = (point_a.0 - point_b.0) * (point_a.0 - point_b.0) +
//         (point_a.1 - point_b.1) * (point_a.1 - point_b.1);
//     (sq as f64).sqrt() as i64
// }
