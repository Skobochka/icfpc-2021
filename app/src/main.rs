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
    TextureSettings,
    Viewport,
    Glyphs,
    PressEvent,
    Button,
    Key
};

use common::{
    cli,
    problem,
};

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
    LoadProblem(problem::FromFileError),
    PistonWindowCreate(Box<dyn std::error::Error>),
    GlyphsCreate(io::Error),
}

fn main() -> Result<(), Error> {
    pretty_env_logger::init();
    let cli_args = CliArgs::from_args();
    log::info!("program starts as: {:?}", cli_args);

    let problem = problem::Problem::from_file(&cli_args.common.problem_file)
        .map_err(Error::LoadProblem)?;
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
    let mut glyphs = Glyphs::new(&font_path, window.create_texture_context(), TextureSettings::new())
        .map_err(Error::GlyphsCreate)?;

    while let Some(event) = window.next() {
        window.draw_2d(&event, |context, g2d, _device| {

            // todo!()
        });

        match event.press_args() {
            Some(Button::Keyboard(Key::Q)) =>
                break,
            _ =>
                (),
        }
    }

    Ok(())
}
