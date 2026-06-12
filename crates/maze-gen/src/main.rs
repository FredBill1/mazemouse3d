mod draw;
mod maze;

use draw::{draw_maze, draw_score_history, DrawOptions};
use maze::{AnnealedMicromouseMaze, MazeConfig};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let maze = AnnealedMicromouseMaze::new(MazeConfig {
        size: 16,
        seed: 514,
        iterations: 6000,
        initial_temp: 20.0,
        final_temp: 0.12,
    })?;

    println!("Maze metrics:\n{}", maze.metrics);

    draw_maze(
        &maze,
        "annealed_micromouse_maze.png",
        DrawOptions {
            show_solution: true,
            show_diagonal_runs: true,
            ..Default::default()
        },
    )?;
    draw_score_history(&maze.score_history, "score_history.png")?;
    Ok(())
}
