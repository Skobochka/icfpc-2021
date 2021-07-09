use std::{
    io,
    path::PathBuf,
};

use structopt::{
    clap::{
        AppSettings,
        crate_name,
    },
    StructOpt,
};

use piston_window::{
    OpenGL,
    PistonWindow,
    WindowSettings,
    PressEvent,
    Button,
    Key
};

use common::{
    cli,
    problem,
};

mod env;
mod draw;

#[derive(Clone, StructOpt, Debug)]
#[structopt(setting = AppSettings::DeriveDisplayOrder)]
#[structopt(setting = AppSettings::AllowLeadingHyphen)]
pub struct CliArgs {
    #[structopt(flatten)]
    pub common: cli::CommonCliArgs,

    /// asserts directory
    #[structopt(long = "assets-directory", default_value = "./assets")]
    pub assets_directory: PathBuf,
    /// window console height in pixels
    #[structopt(long = "console-height", default_value = "32")]
    pub console_height: u32,
    /// window border width in pixels
    #[structopt(long = "border-width", default_value = "16")]
    pub border_width: u32,
    /// window initial screen width in pixels
    #[structopt(long = "screen-width", default_value = "640")]
    pub screen_width: u32,
    /// window initial screen height in pixels
    #[structopt(long = "screen-height", default_value = "320")]
    pub screen_height: u32,
}

#[derive(Debug)]
pub enum Error {
    ProblemLoad(problem::FromFileError),
    GlyphsCreate(io::Error),
    EnvCreate(env::CreateError),
    EnvDraw(env::DrawError),
    PistonWindowCreate(Box<dyn std::error::Error>),
    PistonDraw2d(Box<dyn std::error::Error>),
    PoseExport(problem::WriteFileError),
}

fn main() -> Result<(), Error> {
    pretty_env_logger::init();
    let cli_args = CliArgs::from_args();
    log::info!("program starts as: {:?}", cli_args);

    let problem = problem::Problem::from_file(&cli_args.common.problem_file)
        .map_err(Error::ProblemLoad)?;
    log::debug!(" ;; problem loaded: {:?}", problem);

    let opengl = OpenGL::V3_2;
    let mut window: PistonWindow =
        WindowSettings::new(
            crate_name!(),
            [cli_args.screen_width, cli_args.screen_height],
        )
        .exit_on_esc(true)
        .graphics_api(opengl)
        .build()
        .map_err(Error::PistonWindowCreate)?;

    let mut font_path = cli_args.assets_directory;
    font_path.push("FiraSans-Regular.ttf");
    let mut glyphs = window.load_font(&font_path)
        .map_err(Error::GlyphsCreate)?;

    let mut env =
        env::Env::new(
            problem,
            cli_args.screen_width,
            cli_args.screen_height,
            cli_args.console_height,
            cli_args.border_width,
        )
        .map_err(Error::EnvCreate)?;

    while let Some(event) = window.next() {
        let maybe_result = window.draw_2d(&event, |context, g2d, device| {
            use piston_window::{clear, text, line, Transformed};
            clear([0.0, 0.0, 0.0, 1.0], g2d);

            text::Text::new_color([0.0, 1.0, 0.0, 1.0], 16)
                .draw(
                    &env.console_text(),
                    &mut glyphs,
                    &context.draw_state,
                    context.transform.trans_pos([5.0, 20.0]),
                    g2d,
                )
                .map_err(From::from)
                .map_err(Error::PistonDraw2d)?;

            if let Some(tr) = env.translator(&context.viewport) {
                env.draw(
                    |element| {
                        match element {
                            draw::DrawElement::Line { color, radius, source_x, source_y, target_x, target_y } => {
                                line(color, radius, [tr.x(source_x), tr.y(source_y), tr.x(target_x), tr.y(target_y)], context.transform, g2d);
                            },
                        }
                    })
                    .map_err(Error::EnvDraw)?;
            }

            // Update glyphs before rendering.
            glyphs.factory.encoder.flush(device);

            Ok(())
        });
        if let Some(result) = maybe_result {
            let () = result?;
        }

        match event.press_args() {
            Some(Button::Keyboard(Key::Q)) =>
                break,
            Some(Button::Keyboard(Key::A)) =>
                env.move_figure_left(),
            Some(Button::Keyboard(Key::D)) =>
                env.move_figure_right(),
            Some(Button::Keyboard(Key::W)) =>
                env.move_figure_upper(),
            Some(Button::Keyboard(Key::S)) =>
                env.move_figure_lower(),
            Some(Button::Keyboard(Key::E)) => {
                let pose = env.export_solution();
                pose.write_to_file(&cli_args.common.pose_file)
                    .map_err(Error::PoseExport)?;
                log::info!("pose {:?} has been written to {:?}", pose, cli_args.common.pose_file);
            },
            _ =>
                (),
        }
    }

    Ok(())
}
