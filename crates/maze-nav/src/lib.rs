use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use wasm_bindgen::prelude::*;

const NORTH: u8 = 1;
const EAST: u8 = 2;
const SOUTH: u8 = 4;
const WEST: u8 = 8;
const HEADING_COUNT: usize = 8;
const TURN_COST: f64 = 0.02;
const CARDINAL_STEP_COST: f64 = 0.5;
const DIAGONAL_STEP_COST: f64 = std::f64::consts::SQRT_2 * 0.5;
const DEFAULT_WALL_THICKNESS: f64 = 0.08;
const DEFAULT_WHEEL_RADIUS: f64 = 0.09;
const DEFAULT_TRACK_WIDTH: f64 = 0.41;
const DEFAULT_ROBOT_RADIUS: f64 = 0.22;
const DEFAULT_ROBOT_HALF_WIDTH: f64 = 0.235;
const DEFAULT_ROBOT_FRONT_EXTENT: f64 = 0.37;
const DEFAULT_ROBOT_REAR_EXTENT: f64 = 0.17;
const OFF_PATH_REPLAN_DISTANCE: f64 = 0.7;
const CANDIDATE_MAX_PATH_DISTANCE: f64 = 0.9;
const BROAD_PHASE_CLEARANCE: f64 = 0.85;
const WALL_BIN_SEARCH_PADDING: f64 = 0.58;
const MIN_ROLLOUT_CLEARANCE: f64 = 0.035;
const RECOVERY_GRACE_SECONDS: f64 = 0.16;
const RECOVERY_CLEARANCE: f64 = 0.03;
const RECOVERY_REVERSE_SPEED: f64 = -1.45;

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

#[derive(Debug, Deserialize, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DwaControllerConfig {
    pub size: usize,
    pub walls: Vec<u8>,
    pub seed: u32,
    #[serde(default)]
    pub solution: Vec<usize>,
    #[serde(default)]
    pub options: DwaOptionsInput,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DwaOptionsInput {
    pub max_linear_speed: Option<f64>,
    pub max_wheel_rad_per_sec: Option<f64>,
    pub max_angular_speed: Option<f64>,
    pub max_linear_acceleration: Option<f64>,
    pub max_angular_acceleration: Option<f64>,
    pub prediction_horizon: Option<f64>,
    pub rollout_step: Option<f64>,
    pub control_period: Option<f64>,
    pub linear_samples: Option<usize>,
    pub angular_samples: Option<usize>,
    pub path_lookahead: Option<f64>,
    pub safety_margin: Option<f64>,
    pub arrival_distance: Option<f64>,
    pub wall_thickness: Option<f64>,
    pub wheel_radius: Option<f64>,
    pub track_width: Option<f64>,
    pub robot_radius: Option<f64>,
    pub robot_half_width: Option<f64>,
    pub robot_front_extent: Option<f64>,
    pub robot_rear_extent: Option<f64>,
}

#[derive(Clone, Copy, Debug)]
struct DwaOptions {
    max_linear_speed: f64,
    max_wheel_rad_per_sec: f64,
    max_angular_speed: f64,
    max_linear_acceleration: f64,
    max_angular_acceleration: f64,
    prediction_horizon: f64,
    rollout_step: f64,
    control_period: f64,
    linear_samples: usize,
    angular_samples: usize,
    path_lookahead: f64,
    safety_margin: f64,
    arrival_distance: f64,
    wall_thickness: f64,
    wheel_radius: f64,
    track_width: f64,
    robot_radius: f64,
    robot_half_width: f64,
    robot_front_extent: f64,
    robot_rear_extent: f64,
}

impl DwaOptions {
    fn from_input(input: DwaOptionsInput) -> Self {
        let defaults = Self::default();

        Self {
            max_linear_speed: input.max_linear_speed.unwrap_or(defaults.max_linear_speed),
            max_wheel_rad_per_sec: input
                .max_wheel_rad_per_sec
                .unwrap_or(defaults.max_wheel_rad_per_sec),
            max_angular_speed: input
                .max_angular_speed
                .unwrap_or(defaults.max_angular_speed),
            max_linear_acceleration: input
                .max_linear_acceleration
                .unwrap_or(defaults.max_linear_acceleration),
            max_angular_acceleration: input
                .max_angular_acceleration
                .unwrap_or(defaults.max_angular_acceleration),
            prediction_horizon: input
                .prediction_horizon
                .unwrap_or(defaults.prediction_horizon),
            rollout_step: input.rollout_step.unwrap_or(defaults.rollout_step),
            control_period: input.control_period.unwrap_or(defaults.control_period),
            linear_samples: input
                .linear_samples
                .unwrap_or(defaults.linear_samples)
                .max(2),
            angular_samples: input
                .angular_samples
                .unwrap_or(defaults.angular_samples)
                .max(3),
            path_lookahead: input.path_lookahead.unwrap_or(defaults.path_lookahead),
            safety_margin: input.safety_margin.unwrap_or(defaults.safety_margin),
            arrival_distance: input.arrival_distance.unwrap_or(defaults.arrival_distance),
            wall_thickness: input.wall_thickness.unwrap_or(defaults.wall_thickness),
            wheel_radius: input.wheel_radius.unwrap_or(defaults.wheel_radius),
            track_width: input.track_width.unwrap_or(defaults.track_width),
            robot_radius: input.robot_radius.unwrap_or(defaults.robot_radius),
            robot_half_width: input.robot_half_width.unwrap_or(defaults.robot_half_width),
            robot_front_extent: input
                .robot_front_extent
                .unwrap_or(defaults.robot_front_extent),
            robot_rear_extent: input
                .robot_rear_extent
                .unwrap_or(defaults.robot_rear_extent),
        }
    }
}

impl Default for DwaOptions {
    fn default() -> Self {
        Self {
            max_linear_speed: 5.8,
            max_wheel_rad_per_sec: 105.0,
            max_angular_speed: 14.0,
            max_linear_acceleration: 80.0,
            max_angular_acceleration: 92.0,
            prediction_horizon: 0.45,
            rollout_step: 1.0 / 30.0,
            control_period: 1.0 / 60.0,
            linear_samples: 5,
            angular_samples: 11,
            path_lookahead: 0.75,
            safety_margin: 0.06,
            arrival_distance: 0.2,
            wall_thickness: DEFAULT_WALL_THICKNESS,
            wheel_radius: DEFAULT_WHEEL_RADIUS,
            track_width: DEFAULT_TRACK_WIDTH,
            robot_radius: DEFAULT_ROBOT_RADIUS,
            robot_half_width: DEFAULT_ROBOT_HALF_WIDTH,
            robot_front_extent: DEFAULT_ROBOT_FRONT_EXTENT,
            robot_rear_extent: DEFAULT_ROBOT_REAR_EXTENT,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct DwaTelemetry {
    pub x: f64,
    pub z: f64,
    pub yaw: f64,
    pub velocity_x: f64,
    pub velocity_z: f64,
    pub angular_velocity_y: f64,
}

#[derive(Clone, Copy, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DwaCommandOutput {
    pub left_rad_per_sec: f64,
    pub right_rad_per_sec: f64,
    pub linear_speed: f64,
    pub angular_speed: f64,
    pub debug: DwaDebugOutput,
}

#[derive(Clone, Copy, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DwaDebugOutput {
    pub target_cell: usize,
    pub path_progress: f64,
    pub path_length: f64,
    pub target_x: f64,
    pub target_z: f64,
    pub clearance: f64,
    pub score: f64,
    pub sampled_candidates: usize,
    pub valid_candidates: usize,
    pub replan_count: u32,
}

#[derive(Clone, Copy, Debug)]
struct Point2 {
    x: f64,
    z: f64,
}

#[derive(Clone, Copy, Debug)]
struct WallRect {
    min_x: f64,
    max_x: f64,
    min_z: f64,
    max_z: f64,
}

#[derive(Clone, Copy, Debug)]
struct PathProjection {
    progress: f64,
    distance_squared: f64,
}

#[derive(Clone, Copy, Debug)]
struct Candidate {
    v: f64,
    w: f64,
    score: f64,
    clearance: f64,
    progress: f64,
    target: Point2,
    sampled: usize,
    valid: usize,
}

#[wasm_bindgen]
pub struct DwaController {
    size: usize,
    walls: Vec<u8>,
    options: DwaOptions,
    wall_rects: Vec<WallRect>,
    wall_bins: Vec<Vec<usize>>,
    solution: Vec<usize>,
    random_state: u32,
    target_cell: usize,
    path: Vec<Point2>,
    path_distances: Vec<f64>,
    path_progress: f64,
    replan_count: u32,
    last_v: f64,
    last_w: f64,
}

#[wasm_bindgen]
impl DwaController {
    #[wasm_bindgen(constructor)]
    pub fn new(config: JsValue) -> Result<DwaController, JsValue> {
        let config: DwaControllerConfig = serde_wasm_bindgen::from_value(config)
            .map_err(|error| JsValue::from_str(&format!("invalid DWA config: {error}")))?;
        DwaController::from_config(config)
            .map_err(|error| JsValue::from_str(&format!("invalid DWA maze: {error}")))
    }

    pub fn next_command(&mut self, telemetry: JsValue) -> Result<JsValue, JsValue> {
        let telemetry: DwaTelemetry = serde_wasm_bindgen::from_value(telemetry)
            .map_err(|error| JsValue::from_str(&format!("invalid DWA telemetry: {error}")))?;
        let output = self
            .next_command_impl(telemetry)
            .map_err(|error| JsValue::from_str(&error))?;

        serde_wasm_bindgen::to_value(&output).map_err(|error| {
            JsValue::from_str(&format!("failed to serialize DWA command: {error}"))
        })
    }
}

impl DwaController {
    fn from_config(config: DwaControllerConfig) -> Result<Self, String> {
        HalfGridMaze::new(config.size, &config.walls)?;

        let options = DwaOptions::from_input(config.options);
        let wall_rects = build_wall_rects(config.size, &config.walls, options.wall_thickness);
        let wall_bins = build_wall_bins(config.size, &wall_rects);
        let solution = sprint_solution(config.size, &config.walls, config.solution);
        let fallback_target = solution
            .last()
            .copied()
            .unwrap_or(config.size * config.size - 1);

        Ok(Self {
            size: config.size,
            walls: config.walls,
            options,
            wall_rects,
            wall_bins,
            solution,
            random_state: config.seed ^ 0x9e37_79b9,
            target_cell: fallback_target,
            path: Vec::new(),
            path_distances: Vec::new(),
            path_progress: 0.0,
            replan_count: 0,
            last_v: 0.0,
            last_w: 0.0,
        })
    }

    fn next_command_impl(&mut self, telemetry: DwaTelemetry) -> Result<DwaCommandOutput, String> {
        self.ensure_path(telemetry.x, telemetry.z, telemetry.yaw)?;
        let projection = self.update_path_progress(Point2 {
            x: telemetry.x,
            z: telemetry.z,
        });

        if projection
            .map(|value| value.distance_squared.sqrt() > OFF_PATH_REPLAN_DISTANCE)
            .unwrap_or(false)
        {
            self.plan_to_cell(telemetry.x, telemetry.z, telemetry.yaw, self.target_cell)?;
            self.update_path_progress(Point2 {
                x: telemetry.x,
                z: telemetry.z,
            });
        }

        let current_v = (telemetry.velocity_x * telemetry.yaw.sin()
            + telemetry.velocity_z * telemetry.yaw.cos())
        .clamp(0.0, self.options.max_linear_speed);
        let current_w = telemetry.angular_velocity_y.clamp(
            -self.options.max_angular_speed,
            self.options.max_angular_speed,
        );
        let adaptive_lookahead = (self.options.path_lookahead + current_v * 0.22).min(2.0);
        let target = self
            .sample_path(self.path_progress + adaptive_lookahead)
            .unwrap_or_else(|| self.cell_center_world(self.target_cell));
        let window_v = current_v.max(
            (self.last_v - self.options.max_linear_acceleration * self.options.control_period)
                .max(0.0),
        );
        let window_w = if current_w.abs() > self.last_w.abs() {
            current_w
        } else {
            self.last_w
        };
        let candidate = self.select_candidate(
            telemetry.x,
            telemetry.z,
            telemetry.yaw,
            window_v,
            window_w,
            target,
        );
        let candidate = candidate
            .filter(|candidate| candidate.v.abs() > 0.45 || candidate.score > 0.0)
            .unwrap_or_else(|| self.progress_fallback_candidate(telemetry, target));
        let (left_rad_per_sec, right_rad_per_sec) =
            wheel_speeds(candidate.v, candidate.w, self.options);

        self.last_v = candidate.v;
        self.last_w = candidate.w;

        Ok(DwaCommandOutput {
            left_rad_per_sec,
            right_rad_per_sec,
            linear_speed: candidate.v,
            angular_speed: candidate.w,
            debug: DwaDebugOutput {
                target_cell: self.target_cell,
                path_progress: round_debug(candidate.progress),
                path_length: round_debug(*self.path_distances.last().unwrap_or(&0.0)),
                target_x: round_debug(candidate.target.x),
                target_z: round_debug(candidate.target.z),
                clearance: round_debug(candidate.clearance),
                score: round_debug(candidate.score),
                sampled_candidates: candidate.sampled,
                valid_candidates: candidate.valid,
                replan_count: self.replan_count,
            },
        })
    }

    fn ensure_path(&mut self, x: f64, z: f64, yaw: f64) -> Result<(), String> {
        let current_cell = self.cell_from_world(x, z);
        let target_reached = distance2d(Point2 { x, z }, self.cell_center_world(self.target_cell))
            <= self.options.arrival_distance;

        if self.path.is_empty() || target_reached {
            if target_reached || self.target_cell == current_cell {
                self.target_cell = self.choose_target_cell(current_cell);
            }

            self.plan_to_cell(x, z, yaw, self.target_cell)?;
        }

        Ok(())
    }

    fn plan_to_cell(&mut self, x: f64, z: f64, yaw: f64, target_cell: usize) -> Result<(), String> {
        let maze = HalfGridMaze::new(self.size, &self.walls)?;
        let Some(start) = maze.nearest_passable(x, z) else {
            return Err("no passable half-grid point near current pose".into());
        };

        let output = plan_path_impl(&PlanPathRequest {
            size: self.size,
            walls: self.walls.clone(),
            start_x2: start.x2,
            start_z2: start.z2,
            start_heading: quantize_heading(yaw),
            goal_cell: target_cell,
        })?;

        let path = output
            .waypoints
            .into_iter()
            .map(|point| Point2 {
                x: point.x2 as f64 / 2.0,
                z: point.z2 as f64 / 2.0,
            })
            .collect();
        self.set_path(path);

        Ok(())
    }

    fn set_path(&mut self, path: Vec<Point2>) {
        self.path = path;
        self.path_distances = cumulative_path_distances(&self.path);
        self.path_progress = 0.0;
        self.replan_count += 1;
    }

    fn choose_target_cell(&mut self, current_cell: usize) -> usize {
        if self.solution.len() > 1 {
            let first = self.solution[0];
            let last = *self.solution.last().unwrap_or(&first);

            if current_cell == last {
                return first;
            }

            if current_cell == first {
                return last;
            }
        }

        let total_cells = self.size * self.size;

        for _ in 0..(total_cells * 2).max(1) {
            let candidate = (self.next_random() * total_cells as f64).floor() as usize;

            if candidate != current_cell && candidate != self.target_cell {
                return candidate.min(total_cells - 1);
            }
        }

        (current_cell + total_cells / 2).max(1) % total_cells
    }

    fn next_random(&mut self) -> f64 {
        self.random_state = self.random_state.wrapping_add(0x6d2b_79f5);
        let mut value = self.random_state;
        value = (value ^ (value >> 15)).wrapping_mul(value | 1);
        value ^= value.wrapping_add((value ^ (value >> 7)).wrapping_mul(value | 61));
        ((value ^ (value >> 14)) as f64) / 4_294_967_296.0
    }

    fn select_candidate(
        &self,
        x: f64,
        z: f64,
        yaw: f64,
        current_v: f64,
        current_w: f64,
        target: Point2,
    ) -> Option<Candidate> {
        let current_clearance = self.clearance_at_pose(Point2 { x, z }, yaw);
        let recovery_min_v = if current_clearance < RECOVERY_CLEARANCE {
            RECOVERY_REVERSE_SPEED
        } else {
            0.0
        };
        let heading_to_target = (target.x - x).atan2(target.z - z);
        let heading_error = normalize_angle(heading_to_target - yaw).abs();
        let heading_alignment = heading_error.cos().max(0.0);
        let heading_speed_cap = if heading_error > std::f64::consts::PI * 0.62 {
            0.75
        } else if heading_error > std::f64::consts::PI * 0.42 {
            1.55
        } else {
            self.options.max_linear_speed * (0.42 + 0.58 * heading_alignment.sqrt())
        };
        let clearance_speed_cap = if current_clearance < 0.05 {
            1.6
        } else if current_clearance < 0.1 {
            3.8
        } else {
            self.options.max_linear_speed
        };
        let nominal_min_v =
            current_v - self.options.max_linear_acceleration * self.options.control_period;
        let min_v = if current_clearance < RECOVERY_CLEARANCE {
            recovery_min_v
        } else {
            nominal_min_v.max(0.0)
        };
        let max_v = (current_v
            + self.options.max_linear_acceleration * self.options.control_period)
            .min(self.options.max_linear_speed)
            .min(heading_speed_cap)
            .min(clearance_speed_cap);
        let min_v = min_v.min(max_v);
        let min_w = (current_w
            - self.options.max_angular_acceleration * self.options.control_period)
            .max(-self.options.max_angular_speed);
        let max_w = (current_w
            + self.options.max_angular_acceleration * self.options.control_period)
            .min(self.options.max_angular_speed);
        let mut best = None;
        let mut sampled = 0usize;
        let mut valid = 0usize;

        let mut linear_samples = (0..self.options.linear_samples)
            .map(|index| sample_range(min_v, max_v, index, self.options.linear_samples))
            .collect::<Vec<_>>();

        if min_v <= 0.0 && max_v >= 0.0 {
            linear_samples.push(0.0);
        }

        if current_clearance < RECOVERY_CLEARANCE {
            linear_samples.push(RECOVERY_REVERSE_SPEED.max(min_v));
        }

        linear_samples.sort_by(|left, right| left.total_cmp(right));
        linear_samples.dedup_by(|left, right| (*left - *right).abs() < 1e-6);

        for v in linear_samples {
            for w_index in 0..self.options.angular_samples {
                let w = sample_range(min_w, max_w, w_index, self.options.angular_samples);
                sampled += 1;

                if !wheel_speeds_within_limits(v, w, self.options) {
                    continue;
                }

                let Some(mut candidate) = self.rollout_candidate(x, z, yaw, v, w, target) else {
                    continue;
                };

                valid += 1;
                candidate.sampled = sampled;
                candidate.valid = valid;

                if best
                    .map(|best_candidate: Candidate| candidate.score > best_candidate.score)
                    .unwrap_or(true)
                {
                    best = Some(candidate);
                }
            }
        }

        best.map(|mut candidate| {
            candidate.sampled = sampled;
            candidate.valid = valid;
            candidate
        })
    }

    fn rollout_candidate(
        &self,
        start_x: f64,
        start_z: f64,
        start_yaw: f64,
        v: f64,
        w: f64,
        target: Point2,
    ) -> Option<Candidate> {
        let mut x = start_x;
        let mut z = start_z;
        let mut yaw = start_yaw;
        let mut elapsed = 0.0;
        let initial_clearance = self.clearance_at_pose(Point2 { x, z }, yaw);
        let mut min_clearance = initial_clearance.max(0.0);

        while elapsed < self.options.prediction_horizon {
            let step = self
                .options
                .rollout_step
                .min(self.options.prediction_horizon - elapsed);
            x += yaw.sin() * v * step;
            z += yaw.cos() * v * step;
            yaw = normalize_angle(yaw + w * step);
            elapsed += step;

            let clearance = self.clearance_at_pose(Point2 { x, z }, yaw);

            if clearance <= 0.0 {
                if initial_clearance > 0.0 && elapsed > RECOVERY_GRACE_SECONDS {
                    return None;
                }

                min_clearance = min_clearance.min(0.0);
                continue;
            }

            if initial_clearance > MIN_ROLLOUT_CLEARANCE && clearance < MIN_ROLLOUT_CLEARANCE {
                return None;
            }

            min_clearance = min_clearance.min(clearance);
        }

        let final_point = Point2 { x, z };
        let final_clearance = self.clearance_at_pose(final_point, yaw);

        if min_clearance <= 0.0 && final_clearance <= initial_clearance.max(0.0) + 0.01 {
            return None;
        }

        if initial_clearance > MIN_ROLLOUT_CLEARANCE && min_clearance < MIN_ROLLOUT_CLEARANCE {
            return None;
        }

        let projection = self
            .project_onto_path(final_point, (self.path_progress - 0.25).max(0.0))
            .unwrap_or(PathProjection {
                progress: self.path_progress,
                distance_squared: 0.0,
            });
        let path_distance = projection.distance_squared.sqrt();

        if path_distance > CANDIDATE_MAX_PATH_DISTANCE {
            return None;
        }

        let progress = projection.progress;
        let max_progress_delta = v.max(0.0) * self.options.prediction_horizon + 0.18;
        let progress_delta = (progress - self.path_progress)
            .max(0.0)
            .min(max_progress_delta);
        let target_distance = distance2d(final_point, target);
        let target_yaw = (target.x - x).atan2(target.z - z);
        let heading_error = normalize_angle(target_yaw - yaw).abs();
        let smooth_v = (v - self.last_v).abs();
        let smooth_w = (w - self.last_w).abs();
        let score = progress_delta * 8.0
            + v.max(0.0) * 1.65
            + v.min(0.0) * 0.15
            + min_clearance.min(0.7) * 2.6
            - path_distance * 6.4
            - target_distance * 2.15
            - heading_error * 0.85
            - w.abs() * 0.035
            - smooth_v * 0.04
            - smooth_w * 0.012;

        Some(Candidate {
            v,
            w,
            score,
            clearance: min_clearance,
            progress,
            target,
            sampled: 0,
            valid: 0,
        })
    }

    fn progress_fallback_candidate(&self, telemetry: DwaTelemetry, target: Point2) -> Candidate {
        let target_yaw = (target.x - telemetry.x).atan2(target.z - telemetry.z);
        let heading_error = normalize_angle(target_yaw - telemetry.yaw);
        let alignment = heading_error.cos().max(0.0);
        let mut v = if heading_error.abs() > 1.35 {
            1.1
        } else {
            self.options.max_linear_speed * (0.38 + 0.62 * alignment)
        };
        let mut w = (heading_error * 5.4).clamp(
            -self.options.max_angular_speed,
            self.options.max_angular_speed,
        );

        while !wheel_speeds_within_limits(v, w, self.options) && v > 0.4 {
            v *= 0.88;
        }

        if !wheel_speeds_within_limits(v, w, self.options) {
            w *= 0.75;
        }

        Candidate {
            v,
            w,
            score: -500_000.0,
            clearance: self.clearance_at_pose(
                Point2 {
                    x: telemetry.x,
                    z: telemetry.z,
                },
                telemetry.yaw,
            ),
            progress: self.path_progress,
            target,
            sampled: 0,
            valid: 0,
        }
    }

    fn update_path_progress(&mut self, position: Point2) -> Option<PathProjection> {
        let projection = self.project_onto_path(position, (self.path_progress - 0.18).max(0.0));

        if let Some(projection) = projection {
            self.path_progress = self.path_progress.max(projection.progress);
        }

        projection
    }

    fn project_onto_path(&self, position: Point2, minimum_progress: f64) -> Option<PathProjection> {
        project_onto_path(&self.path, &self.path_distances, position, minimum_progress)
    }

    fn sample_path(&self, progress: f64) -> Option<Point2> {
        sample_path_at(&self.path, &self.path_distances, progress)
    }

    fn clearance_at_pose(&self, point: Point2, yaw: f64) -> f64 {
        let center_offset =
            (self.options.robot_front_extent - self.options.robot_rear_extent) / 2.0;
        let footprint_center = Point2 {
            x: point.x + yaw.sin() * center_offset,
            z: point.z + yaw.cos() * center_offset,
        };
        let clearance_radius = self.options.robot_radius + self.options.safety_margin;

        if footprint_center.x < -clearance_radius
            || footprint_center.z < -clearance_radius
            || footprint_center.x > self.size as f64 + clearance_radius
            || footprint_center.z > self.size as f64 + clearance_radius
        {
            return -1.0;
        }

        let mut clearance = f64::INFINITY;

        let broad_phase_radius = self
            .options
            .robot_front_extent
            .max(self.options.robot_rear_extent)
            + self.options.robot_half_width
            + self.options.safety_margin
            + BROAD_PHASE_CLEARANCE;
        let collision_radius = self
            .options
            .robot_front_extent
            .max(self.options.robot_rear_extent)
            .hypot(self.options.robot_half_width)
            + self.options.safety_margin;

        let search_radius = broad_phase_radius + WALL_BIN_SEARCH_PADDING;
        let min_col = ((point.x - search_radius).floor() as i32)
            .clamp(0, self.size.saturating_sub(1) as i32) as usize;
        let max_col = ((point.x + search_radius).floor() as i32)
            .clamp(0, self.size.saturating_sub(1) as i32) as usize;
        let min_row = ((point.z - search_radius).floor() as i32)
            .clamp(0, self.size.saturating_sub(1) as i32) as usize;
        let max_row = ((point.z + search_radius).floor() as i32)
            .clamp(0, self.size.saturating_sub(1) as i32) as usize;

        for row in min_row..=max_row {
            for col in min_col..=max_col {
                for wall_index in &self.wall_bins[row * self.size + col] {
                    let wall = self.wall_rects[*wall_index];

                    if !rect_near_point(wall, point, broad_phase_radius) {
                        continue;
                    }

                    if rect_near_point(wall, point, collision_radius)
                        && footprint_intersects_rect(point, yaw, self.options, wall)
                    {
                        return -1.0;
                    }

                    clearance =
                        clearance.min(distance_to_rect(footprint_center, wall) - clearance_radius);
                }
            }
        }

        clearance.max(0.001)
    }

    fn cell_from_world(&self, x: f64, z: f64) -> usize {
        let col = x.floor().clamp(0.0, self.size.saturating_sub(1) as f64) as usize;
        let row = z.floor().clamp(0.0, self.size.saturating_sub(1) as f64) as usize;

        row * self.size + col
    }

    fn cell_center_world(&self, cell: usize) -> Point2 {
        Point2 {
            x: (cell % self.size) as f64 + 0.5,
            z: (cell / self.size) as f64 + 0.5,
        }
    }
}

fn build_wall_bins(size: usize, wall_rects: &[WallRect]) -> Vec<Vec<usize>> {
    let mut bins = vec![Vec::new(); size * size];

    for (index, rect) in wall_rects.iter().enumerate() {
        let center_x = (rect.min_x + rect.max_x) / 2.0;
        let center_z = (rect.min_z + rect.max_z) / 2.0;
        let col = center_x.floor().clamp(0.0, size.saturating_sub(1) as f64) as usize;
        let row = center_z.floor().clamp(0.0, size.saturating_sub(1) as f64) as usize;

        bins[row * size + col].push(index);
    }

    bins
}

fn sprint_solution(size: usize, walls: &[u8], solution: Vec<usize>) -> Vec<usize> {
    if solution.len() < 2 {
        return solution;
    }

    let start = solution[0];
    let next = solution[1];
    let start_row = start / size;
    let start_col = start % size;
    let next_row = next / size;
    let next_col = next % size;
    let delta_row = next_row as isize - start_row as isize;
    let delta_col = next_col as isize - start_col as isize;

    if delta_row.abs() + delta_col.abs() != 1 {
        return solution;
    }

    let mut current = start;
    let mut run = vec![start];

    loop {
        let row = current / size;
        let col = current % size;
        let Some(next_row) = row.checked_add_signed(delta_row) else {
            break;
        };
        let Some(next_col) = col.checked_add_signed(delta_col) else {
            break;
        };

        if next_row >= size || next_col >= size {
            break;
        }

        let next_cell = next_row * size + next_col;

        if !cells_connected(size, walls, current, next_cell) {
            break;
        }

        run.push(next_cell);
        current = next_cell;
    }

    if run.len() >= 6 {
        vec![start, *run.last().unwrap_or(&start)]
    } else {
        solution
    }
}

fn cells_connected(size: usize, walls: &[u8], a: usize, b: usize) -> bool {
    let ar = a / size;
    let ac = a % size;
    let br = b / size;
    let bc = b % size;

    match (br as isize - ar as isize, bc as isize - ac as isize) {
        (1, 0) => walls[a] & NORTH == 0 && walls[b] & SOUTH == 0,
        (-1, 0) => walls[a] & SOUTH == 0 && walls[b] & NORTH == 0,
        (0, 1) => walls[a] & EAST == 0 && walls[b] & WEST == 0,
        (0, -1) => walls[a] & WEST == 0 && walls[b] & EAST == 0,
        _ => false,
    }
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

    fn nearest_passable(&self, x: f64, z: f64) -> Option<GridPoint> {
        let limit = (self.size * 2) as i32;
        let target_x2 = ((x * 2.0).round() as i32).clamp(1, limit - 1);
        let target_z2 = ((z * 2.0).round() as i32).clamp(1, limit - 1);
        let mut best = None;

        for z2 in 1..limit {
            for x2 in 1..limit {
                if !self.is_passable(x2, z2) {
                    continue;
                }

                let distance_squared = (x2 - target_x2).pow(2) + (z2 - target_z2).pow(2);

                if best
                    .map(|(_, best_distance): (GridPoint, i32)| distance_squared < best_distance)
                    .unwrap_or(true)
                {
                    best = Some((GridPoint { x2, z2 }, distance_squared));
                }
            }
        }

        best.map(|(point, _)| point)
    }
}

fn build_wall_rects(size: usize, walls: &[u8], wall_thickness: f64) -> Vec<WallRect> {
    let mut rects = Vec::new();

    for row in 0..size {
        for col in 0..size {
            let cell = row * size + col;

            if walls[cell] & SOUTH != 0 {
                rects.push(horizontal_wall(row, col, wall_thickness));
            }

            if walls[cell] & WEST != 0 {
                rects.push(vertical_wall(row, col, wall_thickness));
            }

            if row == size - 1 && walls[cell] & NORTH != 0 {
                rects.push(horizontal_wall(row + 1, col, wall_thickness));
            }

            if col == size - 1 && walls[cell] & EAST != 0 {
                rects.push(vertical_wall(row, col + 1, wall_thickness));
            }
        }
    }

    rects
}

fn horizontal_wall(row_line: usize, col: usize, wall_thickness: f64) -> WallRect {
    let center_x = col as f64 + 0.5;
    let center_z = row_line as f64;
    let half_x = (1.0 + wall_thickness) / 2.0;
    let half_z = wall_thickness / 2.0;

    WallRect {
        min_x: center_x - half_x,
        max_x: center_x + half_x,
        min_z: center_z - half_z,
        max_z: center_z + half_z,
    }
}

fn vertical_wall(row: usize, col_line: usize, wall_thickness: f64) -> WallRect {
    let center_x = col_line as f64;
    let center_z = row as f64 + 0.5;
    let half_x = wall_thickness / 2.0;
    let half_z = (1.0 + wall_thickness) / 2.0;

    WallRect {
        min_x: center_x - half_x,
        max_x: center_x + half_x,
        min_z: center_z - half_z,
        max_z: center_z + half_z,
    }
}

fn distance_to_rect(point: Point2, rect: WallRect) -> f64 {
    let dx = if point.x < rect.min_x {
        rect.min_x - point.x
    } else if point.x > rect.max_x {
        point.x - rect.max_x
    } else {
        0.0
    };
    let dz = if point.z < rect.min_z {
        rect.min_z - point.z
    } else if point.z > rect.max_z {
        point.z - rect.max_z
    } else {
        0.0
    };

    dx.hypot(dz)
}

fn rect_near_point(rect: WallRect, point: Point2, radius: f64) -> bool {
    rect.max_x >= point.x - radius
        && rect.min_x <= point.x + radius
        && rect.max_z >= point.z - radius
        && rect.min_z <= point.z + radius
}

fn footprint_intersects_rect(point: Point2, yaw: f64, options: DwaOptions, rect: WallRect) -> bool {
    let forward = Point2 {
        x: yaw.sin(),
        z: yaw.cos(),
    };
    let right = Point2 {
        x: yaw.cos(),
        z: -yaw.sin(),
    };
    let center_offset = (options.robot_front_extent - options.robot_rear_extent) / 2.0;
    let half_depth = (options.robot_front_extent + options.robot_rear_extent) / 2.0;
    let center = Point2 {
        x: point.x + forward.x * center_offset,
        z: point.z + forward.z * center_offset,
    };
    let robot_corners = [
        add2(
            add2(center, scale2(right, -options.robot_half_width)),
            scale2(forward, -half_depth),
        ),
        add2(
            add2(center, scale2(right, options.robot_half_width)),
            scale2(forward, -half_depth),
        ),
        add2(
            add2(center, scale2(right, options.robot_half_width)),
            scale2(forward, half_depth),
        ),
        add2(
            add2(center, scale2(right, -options.robot_half_width)),
            scale2(forward, half_depth),
        ),
    ];
    let inflated = WallRect {
        min_x: rect.min_x - options.safety_margin,
        max_x: rect.max_x + options.safety_margin,
        min_z: rect.min_z - options.safety_margin,
        max_z: rect.max_z + options.safety_margin,
    };
    let rect_corners = [
        Point2 {
            x: inflated.min_x,
            z: inflated.min_z,
        },
        Point2 {
            x: inflated.max_x,
            z: inflated.min_z,
        },
        Point2 {
            x: inflated.max_x,
            z: inflated.max_z,
        },
        Point2 {
            x: inflated.min_x,
            z: inflated.max_z,
        },
    ];

    for axis in [
        Point2 { x: 1.0, z: 0.0 },
        Point2 { x: 0.0, z: 1.0 },
        right,
        forward,
    ] {
        let (robot_min, robot_max) = project_points(&robot_corners, axis);
        let (rect_min, rect_max) = project_points(&rect_corners, axis);

        if robot_max < rect_min || rect_max < robot_min {
            return false;
        }
    }

    true
}

fn project_points(points: &[Point2; 4], axis: Point2) -> (f64, f64) {
    let mut min = dot2(points[0], axis);
    let mut max = min;

    for point in points.iter().skip(1) {
        let projected = dot2(*point, axis);
        min = min.min(projected);
        max = max.max(projected);
    }

    (min, max)
}

fn add2(a: Point2, b: Point2) -> Point2 {
    Point2 {
        x: a.x + b.x,
        z: a.z + b.z,
    }
}

fn scale2(point: Point2, scale: f64) -> Point2 {
    Point2 {
        x: point.x * scale,
        z: point.z * scale,
    }
}

fn dot2(a: Point2, b: Point2) -> f64 {
    a.x * b.x + a.z * b.z
}

fn cumulative_path_distances(path: &[Point2]) -> Vec<f64> {
    let mut distances = vec![0.0];

    for index in 1..path.len() {
        distances.push(distances[index - 1] + distance2d(path[index - 1], path[index]));
    }

    distances
}

fn project_onto_path(
    path: &[Point2],
    distances: &[f64],
    position: Point2,
    minimum_progress: f64,
) -> Option<PathProjection> {
    let mut best = None;

    for index in 0..path.len().saturating_sub(1) {
        let start = path[index];
        let end = path[index + 1];
        let dx = end.x - start.x;
        let dz = end.z - start.z;
        let length_squared = dx * dx + dz * dz;

        if length_squared <= f64::EPSILON {
            continue;
        }

        let raw_t = ((position.x - start.x) * dx + (position.z - start.z) * dz) / length_squared;
        let t = raw_t.clamp(0.0, 1.0);
        let progress = distances[index] + length_squared.sqrt() * t;

        if progress < minimum_progress {
            continue;
        }

        let projection_x = start.x + dx * t;
        let projection_z = start.z + dz * t;
        let distance_squared =
            (position.x - projection_x).powi(2) + (position.z - projection_z).powi(2);

        if best
            .map(|candidate: PathProjection| distance_squared < candidate.distance_squared)
            .unwrap_or(true)
        {
            best = Some(PathProjection {
                progress,
                distance_squared,
            });
        }
    }

    best
}

fn sample_path_at(path: &[Point2], distances: &[f64], progress: f64) -> Option<Point2> {
    if path.is_empty() {
        return None;
    }

    if path.len() == 1 {
        return path.first().copied();
    }

    let total_length = *distances.last().unwrap_or(&0.0);
    let clamped_progress = progress.clamp(0.0, total_length);

    for index in 0..path.len().saturating_sub(1) {
        let segment_start = distances[index];
        let segment_end = distances[index + 1];

        if clamped_progress > segment_end && index + 2 < path.len() {
            continue;
        }

        let segment_length = (segment_end - segment_start).max(f64::EPSILON);
        let t = ((clamped_progress - segment_start) / segment_length).clamp(0.0, 1.0);

        return Some(Point2 {
            x: path[index].x + (path[index + 1].x - path[index].x) * t,
            z: path[index].z + (path[index + 1].z - path[index].z) * t,
        });
    }

    path.last().copied()
}

fn wheel_speeds(v: f64, w: f64, options: DwaOptions) -> (f64, f64) {
    let left_linear = v + (w * options.track_width) / 2.0;
    let right_linear = v - (w * options.track_width) / 2.0;

    (
        (left_linear / options.wheel_radius).clamp(
            -options.max_wheel_rad_per_sec,
            options.max_wheel_rad_per_sec,
        ),
        (right_linear / options.wheel_radius).clamp(
            -options.max_wheel_rad_per_sec,
            options.max_wheel_rad_per_sec,
        ),
    )
}

fn wheel_speeds_within_limits(v: f64, w: f64, options: DwaOptions) -> bool {
    let left_linear = v + (w * options.track_width) / 2.0;
    let right_linear = v - (w * options.track_width) / 2.0;
    let left = left_linear / options.wheel_radius;
    let right = right_linear / options.wheel_radius;

    left.abs() <= options.max_wheel_rad_per_sec + 1e-9
        && right.abs() <= options.max_wheel_rad_per_sec + 1e-9
}

fn sample_range(min: f64, max: f64, index: usize, samples: usize) -> f64 {
    if samples <= 1 {
        return max;
    }

    min + (max - min) * (index as f64 / (samples - 1) as f64)
}

fn distance2d(a: Point2, b: Point2) -> f64 {
    (b.x - a.x).hypot(b.z - a.z)
}

fn quantize_heading(yaw: f64) -> u8 {
    modulo(
        (yaw / (std::f64::consts::PI / 4.0)).round() as i32,
        HEADING_COUNT as i32,
    ) as u8
}

fn normalize_angle(angle: f64) -> f64 {
    let mut normalized =
        modulo_f64(angle + std::f64::consts::PI, std::f64::consts::PI * 2.0) - std::f64::consts::PI;

    if normalized <= -std::f64::consts::PI {
        normalized += std::f64::consts::PI * 2.0;
    }

    normalized
}

fn modulo(value: i32, divisor: i32) -> i32 {
    ((value % divisor) + divisor) % divisor
}

fn modulo_f64(value: f64, divisor: f64) -> f64 {
    ((value % divisor) + divisor) % divisor
}

fn round_debug(value: f64) -> f64 {
    (value * 1000.0).round() / 1000.0
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn dwa_path_preserves_half_grid_diagonal_waypoints() {
        let size = 2;
        let mut walls = vec![ALL_WALLS; size * size];
        open_between(&mut walls, size, 0, 1);
        open_between(&mut walls, size, 0, 2);
        open_between(&mut walls, size, 1, 3);
        open_between(&mut walls, size, 2, 3);
        let mut controller = test_controller(size, walls, vec![0, 3]);

        controller
            .next_command_impl(DwaTelemetry {
                x: 0.5,
                z: 0.5,
                yaw: std::f64::consts::FRAC_PI_4,
                velocity_x: 0.0,
                velocity_z: 0.0,
                angular_velocity_y: 0.0,
            })
            .expect("DWA command should be produced");

        assert!(controller.path.windows(2).any(|points| {
            (points[0].x - points[1].x).abs() == 0.5 && (points[0].z - points[1].z).abs() == 0.5
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

    #[test]
    fn wall_rects_match_maze_wall_bits() {
        let size = 1;
        let walls = vec![ALL_WALLS];
        let rects = build_wall_rects(size, &walls, DEFAULT_WALL_THICKNESS);

        assert_eq!(rects.len(), 4);
        assert!(rects.iter().any(|rect| rect.max_z < 0.05));
        assert!(rects.iter().any(|rect| rect.min_z > 0.95));
        assert!(rects.iter().any(|rect| rect.max_x < 0.05));
        assert!(rects.iter().any(|rect| rect.min_x > 0.95));
    }

    #[test]
    fn dwa_candidate_respects_speed_windows_and_wheel_limits() {
        let mut walls = vec![ALL_WALLS; 9];
        open_between(&mut walls, 3, 0, 1);
        open_between(&mut walls, 3, 1, 2);
        let mut controller = test_controller(3, walls, vec![0, 1, 2]);
        let output = controller
            .next_command_impl(DwaTelemetry {
                x: 0.5,
                z: 0.5,
                yaw: std::f64::consts::PI / 2.0,
                velocity_x: 0.0,
                velocity_z: 0.0,
                angular_velocity_y: 0.0,
            })
            .expect("DWA command should be produced");

        assert!(output.linear_speed >= 0.0);
        assert!(output.linear_speed <= controller.options.max_linear_speed);
        assert!(output.angular_speed.abs() <= controller.options.max_angular_speed);
        assert!(output.left_rad_per_sec.abs() <= controller.options.max_wheel_rad_per_sec);
        assert!(output.right_rad_per_sec.abs() <= controller.options.max_wheel_rad_per_sec);
        assert!(output.debug.valid_candidates > 0);
    }

    #[test]
    fn dwa_turn_candidate_keeps_clearance_in_corner() {
        let size = 2;
        let mut walls = vec![ALL_WALLS; size * size];
        open_between(&mut walls, size, 0, 1);
        open_between(&mut walls, size, 1, 3);
        let mut controller = test_controller(size, walls, vec![0, 1, 3]);
        let output = controller
            .next_command_impl(DwaTelemetry {
                x: 0.5,
                z: 0.5,
                yaw: std::f64::consts::PI / 2.0,
                velocity_x: 2.0,
                velocity_z: 0.0,
                angular_velocity_y: 0.0,
            })
            .expect("DWA command should be produced");

        assert!(output.debug.valid_candidates > 0);
        assert!(output.debug.clearance > 0.0);
    }

    #[test]
    fn dwa_selects_collision_free_candidate_in_diagonal_passage() {
        let size = 2;
        let mut walls = vec![ALL_WALLS; size * size];
        open_between(&mut walls, size, 0, 1);
        open_between(&mut walls, size, 0, 2);
        open_between(&mut walls, size, 1, 3);
        open_between(&mut walls, size, 2, 3);
        let mut controller = test_controller(size, walls, vec![0, 3]);
        let output = controller
            .next_command_impl(DwaTelemetry {
                x: 0.5,
                z: 0.5,
                yaw: std::f64::consts::FRAC_PI_4,
                velocity_x: 1.5 * std::f64::consts::FRAC_1_SQRT_2,
                velocity_z: 1.5 * std::f64::consts::FRAC_1_SQRT_2,
                angular_velocity_y: 0.0,
            })
            .expect("DWA command should be produced");

        assert!(output.debug.valid_candidates > 0);
        assert!(output.debug.clearance > 0.0);
    }

    #[test]
    fn dwa_replans_after_arriving_at_target() {
        let mut walls = vec![ALL_WALLS; 9];
        open_between(&mut walls, 3, 0, 1);
        open_between(&mut walls, 3, 1, 2);
        let mut controller = test_controller(3, walls, vec![0, 1, 2]);
        controller
            .next_command_impl(DwaTelemetry {
                x: 2.5,
                z: 0.5,
                yaw: -std::f64::consts::PI / 2.0,
                velocity_x: 0.0,
                velocity_z: 0.0,
                angular_velocity_y: 0.0,
            })
            .expect("DWA command should be produced");

        assert_eq!(controller.target_cell, 0);
        assert_eq!(controller.replan_count, 1);
    }

    fn test_controller(size: usize, walls: Vec<u8>, solution: Vec<usize>) -> DwaController {
        DwaController::from_config(DwaControllerConfig {
            size,
            walls,
            seed: 7,
            solution,
            options: DwaOptionsInput {
                prediction_horizon: Some(0.5),
                linear_samples: Some(9),
                angular_samples: Some(17),
                ..DwaOptionsInput::default()
            },
        })
        .expect("controller should initialize")
    }
}
