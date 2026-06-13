use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use wasm_bindgen::prelude::*;

const NORTH: u8 = 1;
const EAST: u8 = 2;
const HEADING_COUNT: usize = 8;
const TURN_COST: f64 = 0.02;
const CARDINAL_STEP_COST: f64 = 0.5;
const DIAGONAL_STEP_COST: f64 = std::f64::consts::SQRT_2 * 0.5;

const HEADING_DELTAS: [(i32, i32); HEADING_COUNT] = [
    (0, 1),
    (1, 1),
    (1, 0),
    (1, -1),
    (0, -1),
    (-1, -1),
    (-1, 0),
    (-1, 1),
];

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PlanPathRequest {
    pub size: usize,
    pub walls: Vec<u8>,
    pub start_x2: i32,
    pub start_z2: i32,
    pub start_heading: u8,
    pub goal_cell: usize,
}

#[derive(Clone, Copy, Debug, Serialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub struct GridPoint {
    pub x2: i32,
    pub z2: i32,
}

#[derive(Clone, Copy, Debug, Serialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub struct PathStep {
    pub x2: i32,
    pub z2: i32,
    pub heading: u8,
}

#[derive(Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct PlanPathOutput {
    pub cost: f64,
    pub steps: Vec<PathStep>,
    pub waypoints: Vec<GridPoint>,
}

#[wasm_bindgen]
pub fn plan_path(request: JsValue) -> Result<JsValue, JsValue> {
    let request: PlanPathRequest = serde_wasm_bindgen::from_value(request)
        .map_err(|error| JsValue::from_str(&format!("invalid path request: {error}")))?;
    let output = plan_path_impl(&request).map_err(|error| JsValue::from_str(&error))?;

    serde_wasm_bindgen::to_value(&output)
        .map_err(|error| JsValue::from_str(&format!("failed to serialize path: {error}")))
}

fn plan_path_impl(request: &PlanPathRequest) -> Result<PlanPathOutput, String> {
    let maze = HalfGridMaze::new(request.size, &request.walls)?;

    if request.start_heading as usize >= HEADING_COUNT {
        return Err(format!("invalid start heading: {}", request.start_heading));
    }

    if request.goal_cell >= request.size * request.size {
        return Err(format!("invalid goal cell: {}", request.goal_cell));
    }

    if !maze.is_passable(request.start_x2, request.start_z2) {
        return Err(format!(
            "start half-grid node is not passable: ({}, {})",
            request.start_x2, request.start_z2
        ));
    }

    let goal = maze.cell_center(request.goal_cell);
    let start = State {
        x2: request.start_x2,
        z2: request.start_z2,
        heading: request.start_heading,
    };
    let start_id = maze.state_id(start);
    let total_states = maze.total_states();
    let mut g_score = vec![f64::INFINITY; total_states];
    let mut previous = vec![None; total_states];
    let mut open = BinaryHeap::new();

    g_score[start_id] = 0.0;
    open.push(HeapEntry {
        state: start,
        cost: 0.0,
        priority: heuristic(start.x2, start.z2, goal),
        sequence: 0,
    });

    let mut sequence = 1usize;
    let mut goal_state = None;

    while let Some(entry) = open.pop() {
        let state_id = maze.state_id(entry.state);

        if entry.cost > g_score[state_id] + f64::EPSILON {
            continue;
        }

        if entry.state.x2 == goal.x2 && entry.state.z2 == goal.z2 {
            goal_state = Some(entry.state);
            break;
        }

        for (next, step_cost) in maze.neighbors(entry.state) {
            let next_id = maze.state_id(next);
            let tentative = entry.cost + step_cost;

            if tentative >= g_score[next_id] {
                continue;
            }

            g_score[next_id] = tentative;
            previous[next_id] = Some(state_id);
            open.push(HeapEntry {
                state: next,
                cost: tentative,
                priority: tentative + heuristic(next.x2, next.z2, goal),
                sequence,
            });
            sequence += 1;
        }
    }

    let Some(goal_state) = goal_state else {
        return Err("no path found".into());
    };

    let goal_id = maze.state_id(goal_state);
    let steps = reconstruct_path(&maze, &previous, goal_id);
    let waypoints = compress_waypoints(&steps);

    Ok(PlanPathOutput {
        cost: g_score[goal_id],
        steps,
        waypoints,
    })
}

fn reconstruct_path(
    maze: &HalfGridMaze<'_>,
    previous: &[Option<usize>],
    mut state_id: usize,
) -> Vec<PathStep> {
    let mut states = Vec::new();

    loop {
        let state = maze.state_from_id(state_id);
        states.push(PathStep {
            x2: state.x2,
            z2: state.z2,
            heading: state.heading,
        });

        let Some(prev_id) = previous[state_id] else {
            break;
        };
        state_id = prev_id;
    }

    states.reverse();
    states
}

fn compress_waypoints(steps: &[PathStep]) -> Vec<GridPoint> {
    let mut waypoints = Vec::new();

    for step in steps {
        let point = GridPoint {
            x2: step.x2,
            z2: step.z2,
        };

        if waypoints.last() != Some(&point) {
            waypoints.push(point);
        }
    }

    waypoints
}

fn heuristic(x2: i32, z2: i32, goal: GridPoint) -> f64 {
    let dx = (goal.x2 - x2) as f64 / 2.0;
    let dz = (goal.z2 - z2) as f64 / 2.0;
    dx.hypot(dz)
}

#[derive(Clone, Copy, Debug)]
struct State {
    x2: i32,
    z2: i32,
    heading: u8,
}

#[derive(Clone, Copy, Debug)]
struct HeapEntry {
    state: State,
    cost: f64,
    priority: f64,
    sequence: usize,
}

impl Eq for HeapEntry {}

impl PartialEq for HeapEntry {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority && self.sequence == other.sequence
    }
}

impl Ord for HeapEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .priority
            .total_cmp(&self.priority)
            .then_with(|| other.sequence.cmp(&self.sequence))
    }
}

impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

struct HalfGridMaze<'a> {
    size: usize,
    walls: &'a [u8],
    width: usize,
}

impl<'a> HalfGridMaze<'a> {
    fn new(size: usize, walls: &'a [u8]) -> Result<Self, String> {
        if size == 0 {
            return Err("maze size must be positive".into());
        }

        if walls.len() != size * size {
            return Err(format!(
                "wall array length {} does not match maze size {}",
                walls.len(),
                size
            ));
        }

        Ok(Self {
            size,
            walls,
            width: size * 2 + 1,
        })
    }

    fn total_states(&self) -> usize {
        self.width * self.width * HEADING_COUNT
    }

    fn state_id(&self, state: State) -> usize {
        ((state.z2 as usize * self.width + state.x2 as usize) * HEADING_COUNT)
            + state.heading as usize
    }

    fn state_from_id(&self, id: usize) -> State {
        let heading = (id % HEADING_COUNT) as u8;
        let node = id / HEADING_COUNT;
        State {
            x2: (node % self.width) as i32,
            z2: (node / self.width) as i32,
            heading,
        }
    }

    fn cell_center(&self, cell: usize) -> GridPoint {
        GridPoint {
            x2: ((cell % self.size) * 2 + 1) as i32,
            z2: ((cell / self.size) * 2 + 1) as i32,
        }
    }

    fn neighbors(&self, state: State) -> Vec<(State, f64)> {
        let mut out = Vec::with_capacity(3);
        out.push((
            State {
                heading: (state.heading + HEADING_COUNT as u8 - 1) % HEADING_COUNT as u8,
                ..state
            },
            TURN_COST,
        ));
        out.push((
            State {
                heading: (state.heading + 1) % HEADING_COUNT as u8,
                ..state
            },
            TURN_COST,
        ));

        let (dx, dz) = HEADING_DELTAS[state.heading as usize];
        let next = State {
            x2: state.x2 + dx,
            z2: state.z2 + dz,
            ..state
        };

        if self.is_passable(next.x2, next.z2) {
            let step_cost = if dx != 0 && dz != 0 {
                DIAGONAL_STEP_COST
            } else {
                CARDINAL_STEP_COST
            };
            out.push((next, step_cost));
        }

        out
    }

    fn is_passable(&self, x2: i32, z2: i32) -> bool {
        let limit = (self.size * 2) as i32;

        if x2 <= 0 || z2 <= 0 || x2 >= limit || z2 >= limit {
            return false;
        }

        let odd_x = x2 % 2 != 0;
        let odd_z = z2 % 2 != 0;

        match (odd_x, odd_z) {
            (true, true) => true,
            (false, false) => false,
            (false, true) => self.is_open_vertical_passage(x2, z2),
            (true, false) => self.is_open_horizontal_passage(x2, z2),
        }
    }

    fn is_open_vertical_passage(&self, x2: i32, z2: i32) -> bool {
        let row = ((z2 - 1) / 2) as usize;
        let col = (x2 / 2 - 1) as usize;

        if row >= self.size || col + 1 >= self.size {
            return false;
        }

        self.walls[row * self.size + col] & EAST == 0
    }

    fn is_open_horizontal_passage(&self, x2: i32, z2: i32) -> bool {
        let row = (z2 / 2 - 1) as usize;
        let col = ((x2 - 1) / 2) as usize;

        if row + 1 >= self.size || col >= self.size {
            return false;
        }

        self.walls[row * self.size + col] & NORTH == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SOUTH: u8 = 4;
    const WEST: u8 = 8;
    const ALL_WALLS: u8 = NORTH | EAST | SOUTH | WEST;

    fn open_between(walls: &mut [u8], size: usize, a: usize, b: usize) {
        let ar = a / size;
        let ac = a % size;
        let br = b / size;
        let bc = b % size;

        match (br as isize - ar as isize, bc as isize - ac as isize) {
            (1, 0) => {
                walls[a] &= !NORTH;
                walls[b] &= !SOUTH;
            }
            (-1, 0) => {
                walls[a] &= !SOUTH;
                walls[b] &= !NORTH;
            }
            (0, 1) => {
                walls[a] &= !EAST;
                walls[b] &= !WEST;
            }
            (0, -1) => {
                walls[a] &= !WEST;
                walls[b] &= !EAST;
            }
            _ => panic!("cells are not adjacent"),
        }
    }

    #[test]
    fn half_grid_passability_respects_walls_and_blocks_intersections() {
        let size = 2;
        let mut walls = vec![ALL_WALLS; size * size];
        open_between(&mut walls, size, 0, 1);
        let maze = HalfGridMaze::new(size, &walls).expect("maze should be valid");

        assert!(maze.is_passable(1, 1));
        assert!(maze.is_passable(2, 1));
        assert!(!maze.is_passable(1, 2));
        assert!(!maze.is_passable(2, 2));
    }

    #[test]
    fn astar_routes_around_a_wall() {
        let size = 2;
        let mut walls = vec![ALL_WALLS; size * size];
        open_between(&mut walls, size, 0, 1);
        open_between(&mut walls, size, 1, 3);

        let output = plan_path_impl(&PlanPathRequest {
            size,
            walls,
            start_x2: 1,
            start_z2: 1,
            start_heading: 2,
            goal_cell: 3,
        })
        .expect("path should exist");

        assert_eq!(output.waypoints.first(), Some(&GridPoint { x2: 1, z2: 1 }));
        assert_eq!(output.waypoints.last(), Some(&GridPoint { x2: 3, z2: 3 }));
        assert!(output.waypoints.contains(&GridPoint { x2: 2, z2: 1 }));
        assert!(output.waypoints.contains(&GridPoint { x2: 3, z2: 2 }));
    }

    #[test]
    fn astar_uses_diagonal_half_grid_steps_when_open() {
        let size = 2;
        let mut walls = vec![ALL_WALLS; size * size];
        open_between(&mut walls, size, 0, 1);
        open_between(&mut walls, size, 0, 2);
        open_between(&mut walls, size, 1, 3);
        open_between(&mut walls, size, 2, 3);

        let output = plan_path_impl(&PlanPathRequest {
            size,
            walls,
            start_x2: 1,
            start_z2: 1,
            start_heading: 2,
            goal_cell: 3,
        })
        .expect("path should exist");

        assert!(output.waypoints.windows(2).any(|points| {
            (points[0].x2 - points[1].x2).abs() == 1 && (points[0].z2 - points[1].z2).abs() == 1
        }));
    }

    #[test]
    fn astar_represents_required_in_place_turns() {
        let size = 2;
        let mut walls = vec![ALL_WALLS; size * size];
        open_between(&mut walls, size, 0, 1);

        let output = plan_path_impl(&PlanPathRequest {
            size,
            walls,
            start_x2: 1,
            start_z2: 1,
            start_heading: 0,
            goal_cell: 1,
        })
        .expect("path should exist");

        assert!(output.steps.windows(2).any(|steps| {
            steps[0].x2 == steps[1].x2
                && steps[0].z2 == steps[1].z2
                && steps[0].heading != steps[1].heading
        }));
    }
}
