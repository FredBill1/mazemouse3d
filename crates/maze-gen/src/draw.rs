use crate::maze::{AnnealedMicromouseMaze, EAST, NORTH, SOUTH, WEST};
use plotters::prelude::*;
use std::error::Error;

#[derive(Clone, Copy, Debug)]
pub struct DrawOptions {
    pub show_solution: bool,
    pub show_diagonal_runs: bool,
    pub cell_px: i32,
    pub margin_px: i32,
    pub wall_width: u32,
}

impl Default for DrawOptions {
    fn default() -> Self {
        Self {
            show_solution: true,
            show_diagonal_runs: true,
            cell_px: 42,
            margin_px: 28,
            wall_width: 3,
        }
    }
}

pub fn draw_maze(
    maze: &AnnealedMicromouseMaze,
    save_path: &str,
    opt: DrawOptions,
) -> Result<(), Box<dyn Error>> {
    let n = maze.size as i32;
    let side = (n * opt.cell_px + opt.margin_px * 2) as u32;
    let root = BitMapBackend::new(save_path, (side, side)).into_drawing_area();
    root.fill(&WHITE)?;

    let to_px = |x: f64, y: f64| -> (i32, i32) {
        (
            opt.margin_px + (x * opt.cell_px as f64).round() as i32,
            opt.margin_px + ((n as f64 - y) * opt.cell_px as f64).round() as i32,
        )
    };
    let cell_rect = |r: usize, c: usize| -> [(i32, i32); 2] {
        let x0 = opt.margin_px + c as i32 * opt.cell_px;
        let y0 = opt.margin_px + (n - r as i32 - 1) * opt.cell_px;
        [(x0, y0), (x0 + opt.cell_px, y0 + opt.cell_px)]
    };

    let grid_style = ShapeStyle::from(&RGBColor(215, 215, 215)).stroke_width(1);
    for i in 0..=maze.size {
        root.draw(&PathElement::new(
            vec![to_px(i as f64, 0.0), to_px(i as f64, n as f64)],
            grid_style.clone(),
        ))?;
        root.draw(&PathElement::new(
            vec![to_px(0.0, i as f64), to_px(n as f64, i as f64)],
            grid_style.clone(),
        ))?;
    }

    let (sr, sc) = maze.rc(maze.start);
    root.draw(&Rectangle::new(
        cell_rect(sr, sc),
        RGBColor(50, 205, 50).mix(0.35).filled(),
    ))?;
    root.draw(&Text::new(
        "S",
        to_px(sc as f64 + 0.5, sr as f64 + 0.5),
        ("sans-serif", 18).into_font().style(FontStyle::Bold),
    ))?;

    for &goal in &maze.goals {
        let (r, c) = maze.rc(goal);
        root.draw(&Rectangle::new(
            cell_rect(r, c),
            RGBColor(255, 215, 0).mix(0.45).filled(),
        ))?;
    }
    root.draw(&Text::new(
        "GOAL",
        to_px(maze.size as f64 / 2.0, maze.size as f64 / 2.0),
        ("sans-serif", 14).into_font().style(FontStyle::Bold),
    ))?;

    if opt.show_diagonal_runs {
        let degrees = maze.degree_grid(&maze.walls);
        let style = ShapeStyle::from(&RGBColor(255, 165, 0).mix(0.70)).stroke_width(3);
        for run in maze.collect_diagonal_runs(&maze.walls, 3, &degrees) {
            let pts = run
                .into_iter()
                .map(|(r, c)| to_px(c + 0.5, r + 0.5))
                .collect::<Vec<_>>();
            root.draw(&PathElement::new(pts, style.clone()))?;
        }
    }

    if opt.show_solution && !maze.solution.is_empty() {
        let pts = maze
            .solution
            .iter()
            .map(|&id| {
                let (r, c) = maze.rc(id);
                to_px(c as f64 + 0.5, r as f64 + 0.5)
            })
            .collect::<Vec<_>>();
        root.draw(&PathElement::new(
            pts,
            ShapeStyle::from(&RGBColor(30, 144, 255).mix(0.80)).stroke_width(3),
        ))?;
    }

    let wall_style = ShapeStyle::from(&BLACK).stroke_width(opt.wall_width);
    for r in 0..maze.size {
        for c in 0..maze.size {
            let w = maze.walls[r * maze.size + c];
            if w & SOUTH != 0 {
                root.draw(&PathElement::new(
                    vec![to_px(c as f64, r as f64), to_px(c as f64 + 1.0, r as f64)],
                    wall_style.clone(),
                ))?;
            }
            if w & WEST != 0 {
                root.draw(&PathElement::new(
                    vec![to_px(c as f64, r as f64), to_px(c as f64, r as f64 + 1.0)],
                    wall_style.clone(),
                ))?;
            }
            if r == maze.size - 1 && w & NORTH != 0 {
                root.draw(&PathElement::new(
                    vec![
                        to_px(c as f64, r as f64 + 1.0),
                        to_px(c as f64 + 1.0, r as f64 + 1.0),
                    ],
                    wall_style.clone(),
                ))?;
            }
            if c == maze.size - 1 && w & EAST != 0 {
                root.draw(&PathElement::new(
                    vec![
                        to_px(c as f64 + 1.0, r as f64),
                        to_px(c as f64 + 1.0, r as f64 + 1.0),
                    ],
                    wall_style.clone(),
                ))?;
            }
        }
    }

    root.present()?;
    Ok(())
}

pub fn draw_score_history(history: &[f64], save_path: &str) -> Result<(), Box<dyn Error>> {
    let root = BitMapBackend::new(save_path, (900, 320)).into_drawing_area();
    root.fill(&WHITE)?;
    let (mut min_y, mut max_y) = history
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &v| {
            (lo.min(v), hi.max(v))
        });
    if !min_y.is_finite() || !max_y.is_finite() {
        min_y = 0.0;
        max_y = 1.0;
    }
    if (max_y - min_y).abs() < 1e-9 {
        min_y -= 1.0;
        max_y += 1.0;
    }

    let mut chart = ChartBuilder::on(&root)
        .caption("Simulated annealing progress", ("sans-serif", 22))
        .margin(16)
        .x_label_area_size(36)
        .y_label_area_size(52)
        .build_cartesian_2d(0usize..history.len().max(1), min_y..max_y)?;

    chart
        .configure_mesh()
        .x_desc("Iteration")
        .y_desc("Best score")
        .draw()?;
    chart.draw_series(LineSeries::new(
        history.iter().enumerate().map(|(i, &y)| (i, y)),
        &BLUE,
    ))?;
    root.present()?;
    Ok(())
}
