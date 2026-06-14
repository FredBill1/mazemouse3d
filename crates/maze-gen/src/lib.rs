mod maze;

use maze::{AnnealedMicromouseMaze, MazeConfig};
use serde::{Deserialize, Serialize};
use wasm_bindgen::prelude::*;

pub use maze::{EAST, Metrics, NORTH, SOUTH, WEST};

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct MazeGenConfig {
    pub size: usize,
    pub seed: u64,
    pub iterations: usize,
    pub initial_temp: f64,
    pub final_temp: f64,
    pub include_score_history: bool,
}

impl Default for MazeGenConfig {
    fn default() -> Self {
        Self {
            size: 16,
            seed: 514,
            iterations: 6000,
            initial_temp: 20.0,
            final_temp: 0.12,
            include_score_history: false,
        }
    }
}

impl From<MazeGenConfig> for MazeConfig {
    fn from(value: MazeGenConfig) -> Self {
        Self {
            size: value.size,
            seed: value.seed,
            iterations: value.iterations,
            initial_temp: value.initial_temp,
            final_temp: value.final_temp,
            include_score_history: value.include_score_history,
        }
    }
}

#[derive(Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct MazeGenOutput {
    pub size: usize,
    pub seed: u64,
    pub iterations: usize,
    pub initial_temp: f64,
    pub final_temp: f64,
    pub start: usize,
    pub goals: [usize; 4],
    pub walls: Vec<u8>,
    pub solution: Vec<usize>,
    pub metrics: Metrics,
    pub score_history: Vec<f64>,
}

impl From<AnnealedMicromouseMaze> for MazeGenOutput {
    fn from(value: AnnealedMicromouseMaze) -> Self {
        Self {
            size: value.size,
            seed: value.seed,
            iterations: value.iterations,
            initial_temp: value.initial_temp,
            final_temp: value.final_temp,
            start: value.start,
            goals: value.goals,
            walls: value.walls,
            solution: value.solution,
            metrics: value.metrics,
            score_history: value.score_history,
        }
    }
}

#[wasm_bindgen]
pub fn generate_maze(config: JsValue) -> Result<JsValue, JsValue> {
    let config = if config.is_undefined() || config.is_null() {
        MazeGenConfig::default()
    } else {
        serde_wasm_bindgen::from_value(config)
            .map_err(|error| JsValue::from_str(&format!("invalid maze config: {error}")))?
    };

    let maze =
        AnnealedMicromouseMaze::new(config.into()).map_err(|error| JsValue::from_str(&error))?;
    serde_wasm_bindgen::to_value(&MazeGenOutput::from(maze))
        .map_err(|error| JsValue::from_str(&format!("failed to serialize maze: {error}")))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_config() -> MazeConfig {
        MazeConfig {
            size: 16,
            seed: 514,
            iterations: 120,
            initial_temp: 20.0,
            final_temp: 0.12,
            include_score_history: false,
        }
    }

    #[test]
    fn generation_is_deterministic_for_same_config() {
        let first = AnnealedMicromouseMaze::new(test_config()).expect("maze should generate");
        let second = AnnealedMicromouseMaze::new(test_config()).expect("maze should generate");

        assert_eq!(first.walls, second.walls);
        assert_eq!(first.solution, second.solution);
    }

    #[test]
    fn invalid_size_returns_error() {
        let result = AnnealedMicromouseMaze::new(MazeConfig {
            size: 5,
            ..test_config()
        });

        assert!(result.is_err());
    }

    #[test]
    fn generated_maze_has_solution_to_goal() {
        let maze = AnnealedMicromouseMaze::new(test_config()).expect("maze should generate");

        assert!(!maze.solution.is_empty());
        assert_eq!(maze.solution.first(), Some(&maze.start));
        assert!(
            maze.solution
                .last()
                .is_some_and(|cell| maze.goals.contains(cell))
        );
    }

    #[test]
    fn score_history_is_empty_by_default() {
        let maze = AnnealedMicromouseMaze::new(test_config()).expect("maze should generate");
        let output = MazeGenOutput::from(maze);

        assert!(output.score_history.is_empty());
    }

    #[test]
    fn score_history_can_be_included() {
        let config = MazeConfig {
            iterations: 24,
            include_score_history: true,
            ..test_config()
        };
        let maze = AnnealedMicromouseMaze::new(config).expect("maze should generate");
        let output = MazeGenOutput::from(maze);

        assert_eq!(output.score_history.len(), config.iterations + 1);
    }
}
