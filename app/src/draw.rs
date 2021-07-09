
#[derive(Debug)]
pub enum DrawElement {
    Line {
        color: [f32; 4],
        radius: f64,
        source_x: f64,
        source_y: f64,
        target_x: f64,
        target_y: f64,
    },
    Ellipse {
        color: [f32; 4],
        x: f64,
        y: f64,
        width: f64,
        height: f64,
    },
}
