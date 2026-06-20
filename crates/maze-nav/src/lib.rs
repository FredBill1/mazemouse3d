use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use wasm_bindgen::prelude::*;

const NORTH: u8 = 1;
const EAST: u8 = 2;
const SOUTH: u8 = 4;
const WEST: u8 = 8;
const HEADING_COUNT: usize = 8;
const TURN_COST: f64 = 0.18;
const CARDINAL_STEP_COST: f64 = 0.5;
const DIAGONAL_STEP_COST: f64 = std::f64::consts::SQRT_2 * 0.5;
const REPLAN_ATTEMPT_MULTIPLIER: usize = 2;
const MAX_TRACKABLE_LATERAL_ACCEL: f64 = 4.0;
const MIN_ROLLOUT_CLEARANCE: f64 = 0.13;
const RELAXED_ROLLOUT_CLEARANCE: f64 = 0.08;
const MAX_REVERSE_SPEED_RATIO: f64 = 0.45;
const MAX_SHORTCUT_SPAN_POINTS: usize = 8;

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

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct WorldPoint {
    pub x: f64,
    pub z: f64,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
struct Pose2 {
    x: f64,
    z: f64,
    yaw: f64,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
struct Velocity2 {
    vx: f64,
    omega: f64,
}

#[derive(Clone, Copy, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct WheelCommand {
    left_rad_per_sec: f64,
    right_rad_per_sec: f64,
}

#[derive(Clone, Copy, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct TwistCommand {
    linear_speed: f64,
    angular_speed: f64,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
struct NavigationConfig {
    dwb_frequency: f64,
    max_wheel_rad_per_sec: f64,
    max_linear_speed: f64,
    max_angular_speed: f64,
    max_linear_accel: f64,
    max_linear_decel: f64,
    max_angular_accel: f64,
    max_angular_decel: f64,
    track_width: f64,
    wheel_radius: f64,
    sim_time: f64,
    sim_step: f64,
    vx_samples: usize,
    omega_samples: usize,
    waypoint_tolerance: f64,
    arrival_distance: f64,
    robot_half_width: f64,
    robot_front_length: f64,
    robot_rear_length: f64,
    robot_footprint: Vec<WorldPoint>,
    #[serde(default)]
    robot_footprints: Vec<Vec<WorldPoint>>,
    safety_margin: f64,
    wall_thickness: f64,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
struct NavigationInitRequest {
    size: usize,
    walls: Vec<u8>,
    goals: Vec<usize>,
    seed: u32,
    config: NavigationConfig,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
struct NavigationTickRequest {
    sequence: u32,
    delta_seconds: f64,
    pose: Pose2,
    velocity: Velocity2,
}

#[derive(Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct NavigationTickOutput {
    sequence: u32,
    command: WheelCommand,
    twist: TwistCommand,
    path: Vec<WorldPoint>,
    path_version: u32,
    target_cell: Option<usize>,
    debug: NavigationDebugOutput,
}

#[derive(Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct NavigationDebugOutput {
    dwb_hz: f64,
    smoother_hz: f64,
    status: String,
    valid_trajectories: usize,
    sampled_trajectories: usize,
    rejected_trajectories: DwbRejectCounts,
    current_linear_speed: f64,
    current_angular_speed: f64,
    target_linear_speed: f64,
    target_angular_speed: f64,
    smoothed_linear_speed: f64,
    smoothed_angular_speed: f64,
    dynamic_window: DwbWindowDebug,
    best: Option<DwbBestDebug>,
    current_clearance: f64,
    current_pose_collides: bool,
    path_progress: f64,
    path_length: f64,
    remaining_distance: f64,
    path_tracking_error: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Twist {
    v: f64,
    w: f64,
}

#[derive(Clone, Copy, Debug)]
struct WallRect {
    min_x: f64,
    max_x: f64,
    min_z: f64,
    max_z: f64,
}

#[derive(Clone, Debug)]
struct WallSpatialIndex {
    size: usize,
    bins: Vec<Vec<usize>>,
}

impl WallSpatialIndex {
    fn new(size: usize, walls: &[WallRect]) -> Self {
        let mut bins = vec![Vec::new(); size * size];

        for (index, wall) in walls.iter().enumerate() {
            let min_col = wall.min_x.floor().clamp(0.0, size.saturating_sub(1) as f64) as usize;
            let max_col = wall.max_x.floor().clamp(0.0, size.saturating_sub(1) as f64) as usize;
            let min_row = wall.min_z.floor().clamp(0.0, size.saturating_sub(1) as f64) as usize;
            let max_row = wall.max_z.floor().clamp(0.0, size.saturating_sub(1) as f64) as usize;

            for row in min_row..=max_row {
                for col in min_col..=max_col {
                    bins[row * size + col].push(index);
                }
            }
        }

        Self { size, bins }
    }

    fn query(&self, min_x: f64, max_x: f64, min_z: f64, max_z: f64) -> Vec<usize> {
        if self.size == 0 {
            return Vec::new();
        }

        let min_col = min_x.floor().clamp(0.0, self.size.saturating_sub(1) as f64) as usize;
        let max_col = max_x.floor().clamp(0.0, self.size.saturating_sub(1) as f64) as usize;
        let min_row = min_z.floor().clamp(0.0, self.size.saturating_sub(1) as f64) as usize;
        let max_row = max_z.floor().clamp(0.0, self.size.saturating_sub(1) as f64) as usize;
        let mut out = Vec::new();

        for row in min_row..=max_row {
            for col in min_col..=max_col {
                out.extend(self.bins[row * self.size + col].iter().copied());
            }
        }

        out.sort_unstable();
        out.dedup();
        out
    }
}

#[derive(Clone, Copy, Debug)]
struct PathProjection {
    progress: f64,
    distance_squared: f64,
}

#[derive(Clone, Copy, Debug)]
struct TrajectoryScore {
    score: f64,
    progress: f64,
    breakdown: DwbScoreBreakdown,
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
#[serde(rename_all = "camelCase")]
struct DwbRejectCounts {
    current_pose_collision: usize,
    rollout_collision: usize,
    braking_collision: usize,
    wheel_speed: usize,
    trackability: usize,
    low_clearance: usize,
    no_progress: usize,
    no_path_projection: usize,
    non_finite_score: usize,
}

#[derive(Clone, Copy, Debug)]
enum DwbRejectReason {
    CurrentPoseCollision,
    RolloutCollision,
    WheelSpeed,
    Trackability,
    LowClearance,
    NoProgress,
    NoPathProjection,
    NonFiniteScore,
}

#[derive(Clone, Copy, Debug)]
enum DwbClearanceMode {
    Nominal,
    Relaxed,
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
#[serde(rename_all = "camelCase")]
struct DwbWindowDebug {
    current_v: f64,
    current_w: f64,
    min_v: f64,
    max_v: f64,
    min_w: f64,
    max_w: f64,
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
#[serde(rename_all = "camelCase")]
struct DwbScoreBreakdown {
    total: f64,
    path_distance: f64,
    target_distance: f64,
    heading_error: f64,
    obstacle_cost: f64,
    progress_reward: f64,
    speed_reward: f64,
    angular_cost: f64,
    acceleration_cost: f64,
    low_speed_turn_cost: f64,
    reverse_cost: f64,
    min_clearance: f64,
    progress: f64,
    end_x: f64,
    end_z: f64,
}

#[derive(Clone, Copy, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct DwbBestDebug {
    linear_speed: f64,
    angular_speed: f64,
    score: DwbScoreBreakdown,
}

#[derive(Clone, Copy, Debug)]
struct DwbStats {
    sampled: usize,
    valid: usize,
    rejected: DwbRejectCounts,
    window: DwbWindowDebug,
    best: Option<DwbBestDebug>,
    current_clearance: f64,
    current_pose_collides: bool,
    path_progress: f64,
    path_length: f64,
    path_tracking_error: f64,
}

impl DwbRejectCounts {
    fn record(&mut self, reason: DwbRejectReason) {
        match reason {
            DwbRejectReason::CurrentPoseCollision => self.current_pose_collision += 1,
            DwbRejectReason::RolloutCollision => self.rollout_collision += 1,
            DwbRejectReason::WheelSpeed => self.wheel_speed += 1,
            DwbRejectReason::Trackability => self.trackability += 1,
            DwbRejectReason::LowClearance => self.low_clearance += 1,
            DwbRejectReason::NoProgress => self.no_progress += 1,
            DwbRejectReason::NoPathProjection => self.no_path_projection += 1,
            DwbRejectReason::NonFiniteScore => self.non_finite_score += 1,
        }
    }
}

impl DwbStats {
    fn empty(
        current_clearance: f64,
        current_pose_collides: bool,
        path_progress: f64,
        path_length: f64,
        path_tracking_error: f64,
    ) -> Self {
        Self {
            sampled: 0,
            valid: 0,
            rejected: DwbRejectCounts::default(),
            window: DwbWindowDebug::default(),
            best: None,
            current_clearance,
            current_pose_collides,
            path_progress,
            path_length,
            path_tracking_error,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct DwbSearchResult {
    best_twist: Twist,
    best_score: Option<TrajectoryScore>,
    stats: DwbStats,
}

#[derive(Clone, Copy, Debug)]
struct DwbWindow {
    current_v: f64,
    current_w: f64,
    min_v: f64,
    max_v: f64,
    min_w: f64,
    max_w: f64,
}

#[wasm_bindgen]
pub fn plan_path(request: JsValue) -> Result<JsValue, JsValue> {
    let request: PlanPathRequest = serde_wasm_bindgen::from_value(request)
        .map_err(|error| JsValue::from_str(&format!("invalid path request: {error}")))?;
    let output = plan_path_impl(&request).map_err(|error| JsValue::from_str(&error))?;

    serde_wasm_bindgen::to_value(&output)
        .map_err(|error| JsValue::from_str(&format!("failed to serialize path: {error}")))
}

#[wasm_bindgen]
pub struct NavigationController {
    size: usize,
    walls: Vec<u8>,
    goals: Vec<usize>,
    wall_rects: Vec<WallRect>,
    wall_index: WallSpatialIndex,
    config: NavigationConfig,
    random_state: u32,
    target_cell: Option<usize>,
    blocked_target_cell: Option<usize>,
    path: Vec<WorldPoint>,
    path_distances: Vec<f64>,
    path_progress: f64,
    path_version: u32,
    plan_retry_at: f64,
    target_twist: Twist,
    smoothed_twist: Twist,
    elapsed_seconds: f64,
    dwb_accumulator: f64,
    dwb_ticks: usize,
    smoother_ticks: usize,
    last_status: String,
    last_stats: DwbStats,
}

#[wasm_bindgen]
impl NavigationController {
    #[wasm_bindgen(constructor)]
    pub fn new(request: JsValue) -> Result<NavigationController, JsValue> {
        let request: NavigationInitRequest = serde_wasm_bindgen::from_value(request)
            .map_err(|error| JsValue::from_str(&format!("invalid navigation init: {error}")))?;

        HalfGridMaze::new(request.size, &request.walls)
            .map_err(|error| JsValue::from_str(&format!("invalid navigation maze: {error}")))?;

        let wall_rects =
            build_wall_rects(request.size, &request.walls, request.config.wall_thickness);
        let wall_index = WallSpatialIndex::new(request.size, &wall_rects);

        Ok(NavigationController {
            size: request.size,
            goals: request
                .goals
                .into_iter()
                .filter(|goal| *goal < request.size * request.size)
                .collect(),
            wall_rects,
            wall_index,
            walls: request.walls,
            config: request.config,
            random_state: request.seed ^ 0x9e37_79b9,
            target_cell: None,
            blocked_target_cell: None,
            path: Vec::new(),
            path_distances: Vec::new(),
            path_progress: 0.0,
            path_version: 0,
            plan_retry_at: 0.0,
            target_twist: Twist { v: 0.0, w: 0.0 },
            smoothed_twist: Twist { v: 0.0, w: 0.0 },
            elapsed_seconds: 0.0,
            dwb_accumulator: f64::INFINITY,
            dwb_ticks: 0,
            smoother_ticks: 0,
            last_status: "initializing".to_string(),
            last_stats: DwbStats::empty(f64::INFINITY, false, 0.0, 0.0, 0.0),
        })
    }

    pub fn tick(&mut self, request: JsValue) -> Result<JsValue, JsValue> {
        let request: NavigationTickRequest = serde_wasm_bindgen::from_value(request)
            .map_err(|error| JsValue::from_str(&format!("invalid navigation tick: {error}")))?;
        let delta_seconds = request.delta_seconds.clamp(0.0, 0.1);

        self.elapsed_seconds += delta_seconds;
        self.dwb_accumulator += delta_seconds;

        let dwb_period = 1.0 / self.config.dwb_frequency.max(1.0);
        if self.dwb_accumulator >= dwb_period {
            let (twist, stats, status) = self.compute_dwb_command(request.pose, request.velocity);
            if let Some(twist) = twist {
                self.target_twist = twist;
            }
            self.last_stats = stats;
            self.last_status = status;
            self.dwb_accumulator = 0.0;
            self.dwb_ticks += 1;
        }

        self.smoothed_twist =
            self.smooth_velocity(self.smoothed_twist, self.target_twist, delta_seconds);
        self.smoother_ticks += 1;

        let output = NavigationTickOutput {
            sequence: request.sequence,
            command: self.twist_to_wheels(self.smoothed_twist),
            twist: TwistCommand {
                linear_speed: self.smoothed_twist.v,
                angular_speed: self.smoothed_twist.w,
            },
            path: self.path.clone(),
            path_version: self.path_version,
            target_cell: self.target_cell,
            debug: NavigationDebugOutput {
                dwb_hz: frequency(self.dwb_ticks, self.elapsed_seconds),
                smoother_hz: frequency(self.smoother_ticks, self.elapsed_seconds),
                status: self.last_status.clone(),
                valid_trajectories: self.last_stats.valid,
                sampled_trajectories: self.last_stats.sampled,
                rejected_trajectories: self.last_stats.rejected,
                current_linear_speed: request.velocity.vx,
                current_angular_speed: request.velocity.omega,
                target_linear_speed: self.target_twist.v,
                target_angular_speed: self.target_twist.w,
                smoothed_linear_speed: self.smoothed_twist.v,
                smoothed_angular_speed: self.smoothed_twist.w,
                dynamic_window: self.last_stats.window,
                best: self.last_stats.best,
                current_clearance: self.last_stats.current_clearance,
                current_pose_collides: self.last_stats.current_pose_collides,
                path_progress: self.last_stats.path_progress,
                path_length: self.last_stats.path_length,
                remaining_distance: (self.last_stats.path_length - self.last_stats.path_progress)
                    .max(0.0),
                path_tracking_error: self.last_stats.path_tracking_error,
            },
        };

        serde_wasm_bindgen::to_value(&output).map_err(|error| {
            JsValue::from_str(&format!("failed to serialize navigation tick: {error}"))
        })
    }
}

impl NavigationController {
    fn compute_dwb_command(
        &mut self,
        pose: Pose2,
        velocity: Velocity2,
    ) -> (Option<Twist>, DwbStats, String) {
        let current_clearance = self.clearance_at_pose(pose);
        let current_pose_collides = self.footprint_collides(pose);

        if let Some(target_cell) = self.target_cell
            && distance2d(
                WorldPoint {
                    x: pose.x,
                    z: pose.z,
                },
                cell_center_world(self.size, target_cell),
            ) <= self.config.arrival_distance
        {
            self.clear_path();
            self.blocked_target_cell = None;
        }

        if self.path.is_empty() {
            if self.elapsed_seconds >= self.plan_retry_at && self.plan_new_path(pose) {
                self.plan_retry_at = 0.0;
            } else {
                if self.elapsed_seconds >= self.plan_retry_at {
                    self.plan_retry_at = self.elapsed_seconds + 1.0;
                }

                return (
                    None,
                    DwbStats::empty(current_clearance, current_pose_collides, 0.0, 0.0, 0.0),
                    "planner-error:no-path".to_string(),
                );
            }
        }

        let position = WorldPoint {
            x: pose.x,
            z: pose.z,
        };
        let mut path_projection = self.update_path_progress(position);
        let mut path_tracking_error = path_projection
            .map(|projection| projection.distance_squared.sqrt())
            .unwrap_or(f64::INFINITY);

        if path_tracking_error > self.path_replan_distance() && self.plan_new_path(pose) {
            path_projection = self.update_path_progress(position);
            path_tracking_error = path_projection
                .map(|projection| projection.distance_squared.sqrt())
                .unwrap_or(f64::INFINITY);
        }

        let mut path_length = self.path_length();
        let max_reverse_speed = self.config.max_linear_speed * MAX_REVERSE_SPEED_RATIO;
        let nominal_clearance = self.config.safety_margin.max(MIN_ROLLOUT_CLEARANCE);
        let reverse_is_available = current_clearance < nominal_clearance || velocity.vx < -0.02;
        let min_window_speed = if reverse_is_available {
            -max_reverse_speed
        } else {
            0.0
        };
        let current_v = velocity
            .vx
            .clamp(min_window_speed, self.config.max_linear_speed);
        let current_w = velocity.omega.clamp(
            -self.config.max_angular_speed,
            self.config.max_angular_speed,
        );
        let dwb_period = 1.0 / self.config.dwb_frequency.max(1.0);
        let min_v = (current_v - self.config.max_linear_decel * dwb_period)
            .max(min_window_speed)
            .min(self.config.max_linear_speed);
        let max_v = (current_v + self.config.max_linear_accel * dwb_period)
            .max(min_v)
            .min(self.config.max_linear_speed);
        let min_w = (current_w - self.config.max_angular_decel * dwb_period)
            .max(-self.config.max_angular_speed);
        let max_w = (current_w + self.config.max_angular_accel * dwb_period)
            .min(self.config.max_angular_speed);

        let mut search = self.search_dwb_window(
            pose,
            DwbWindow {
                current_v,
                current_w,
                min_v,
                max_v,
                min_w,
                max_w,
            },
            current_clearance,
            current_pose_collides,
            path_length,
            path_tracking_error,
        );

        if search.best_score.is_none() && self.plan_new_path(pose) {
            path_projection = self.update_path_progress(position);
            path_tracking_error = path_projection
                .map(|projection| projection.distance_squared.sqrt())
                .unwrap_or(f64::INFINITY);
            path_length = self.path_length();
            search = self.search_dwb_window(
                pose,
                DwbWindow {
                    current_v,
                    current_w,
                    min_v,
                    max_v,
                    min_w,
                    max_w,
                },
                current_clearance,
                current_pose_collides,
                path_length,
                path_tracking_error,
            );
        }

        match search.best_score {
            Some(_) => (
                Some(search.best_twist),
                search.stats,
                "tracking".to_string(),
            ),
            None => (
                None,
                search.stats,
                "planner-error:no-valid-trajectory".to_string(),
            ),
        }
    }

    fn search_dwb_window(
        &self,
        pose: Pose2,
        window: DwbWindow,
        current_clearance: f64,
        current_pose_collides: bool,
        path_length: f64,
        path_tracking_error: f64,
    ) -> DwbSearchResult {
        let mut result = DwbSearchResult {
            best_twist: Twist { v: 0.0, w: 0.0 },
            best_score: None,
            stats: DwbStats::empty(
                current_clearance,
                current_pose_collides,
                self.path_progress,
                path_length,
                path_tracking_error,
            ),
        };
        result.stats.window = DwbWindowDebug {
            current_v: window.current_v,
            current_w: window.current_w,
            min_v: window.min_v,
            max_v: window.max_v,
            min_w: window.min_w,
            max_w: window.max_w,
        };

        for v in velocity_samples(window.min_v, window.max_v, self.config.vx_samples) {
            for w in velocity_samples(window.min_w, window.max_w, self.config.omega_samples) {
                let candidate = Twist { v, w };
                result.stats.sampled += 1;

                let score = match self.evaluate_candidate(
                    pose,
                    candidate,
                    window.current_v,
                    window.current_w,
                    current_clearance,
                    DwbClearanceMode::Nominal,
                ) {
                    Ok(score) => score,
                    Err(reason) => {
                        result.stats.rejected.record(reason);
                        continue;
                    }
                };

                result.stats.valid += 1;

                if result
                    .best_score
                    .map(|best| is_better_score(score, best))
                    .unwrap_or(true)
                {
                    result.best_twist = candidate;
                    result.stats.best = Some(DwbBestDebug {
                        linear_speed: candidate.v,
                        angular_speed: candidate.w,
                        score: score.breakdown,
                    });
                    result.best_score = Some(score);
                }
            }
        }

        if result.best_score.is_none() && result.stats.rejected.low_clearance > 0 {
            for v in velocity_samples(window.min_v, window.max_v, self.config.vx_samples) {
                for w in velocity_samples(window.min_w, window.max_w, self.config.omega_samples) {
                    let candidate = Twist { v, w };
                    let score = match self.evaluate_candidate(
                        pose,
                        candidate,
                        window.current_v,
                        window.current_w,
                        current_clearance,
                        DwbClearanceMode::Relaxed,
                    ) {
                        Ok(score) => score,
                        Err(_) => continue,
                    };

                    result.stats.valid += 1;

                    if result
                        .best_score
                        .map(|best| is_better_score(score, best))
                        .unwrap_or(true)
                    {
                        result.best_twist = candidate;
                        result.stats.best = Some(DwbBestDebug {
                            linear_speed: candidate.v,
                            angular_speed: candidate.w,
                            score: score.breakdown,
                        });
                        result.best_score = Some(score);
                    }
                }
            }
        }

        result
    }

    fn evaluate_candidate(
        &self,
        start: Pose2,
        twist: Twist,
        current_v: f64,
        current_w: f64,
        current_clearance: f64,
        clearance_mode: DwbClearanceMode,
    ) -> Result<TrajectoryScore, DwbRejectReason> {
        if !self.wheel_speed_is_feasible(twist) {
            return Err(DwbRejectReason::WheelSpeed);
        }

        if twist.v.abs() * twist.w.abs() > MAX_TRACKABLE_LATERAL_ACCEL {
            return Err(DwbRejectReason::Trackability);
        }

        if twist.v.abs() < 0.55 && twist.w.abs() > 2.4 {
            return Err(DwbRejectReason::Trackability);
        }

        let mut pose = start;
        let mut rollout_twist = Twist {
            v: current_v,
            w: current_w,
        };
        let mut max_progress = self.path_progress;
        let mut forward_motion = 0.0;
        let steps = (self.config.sim_time / self.config.sim_step)
            .ceil()
            .max(1.0) as usize;
        let projection_min = (self.path_progress - self.config.waypoint_tolerance).max(0.0);
        let projection_max =
            self.path_progress + self.config.max_linear_speed * self.config.sim_time + 0.75;
        let mut min_rollout_clearance = f64::INFINITY;
        let clearance_stride = (0.08 / self.config.sim_step).ceil().max(1.0) as usize;
        let required_clearance = self.rollout_clearance_floor(current_clearance, clearance_mode);

        if self.footprint_collides(pose) {
            return Err(DwbRejectReason::CurrentPoseCollision);
        }

        for step_index in 0..steps {
            rollout_twist = self.smooth_velocity(rollout_twist, twist, self.config.sim_step);
            pose = integrate_pose(pose, rollout_twist, self.config.sim_step);
            forward_motion += rollout_twist.v.max(0.0) * self.config.sim_step;

            if self.footprint_collides(pose) {
                return Err(DwbRejectReason::RolloutCollision);
            }

            if (step_index + 1) % clearance_stride == 0 || step_index + 1 == steps {
                let clearance = self.clearance_at_pose(pose);

                if clearance < required_clearance {
                    return Err(DwbRejectReason::LowClearance);
                }

                min_rollout_clearance = min_rollout_clearance.min(clearance);
            }

            if let Some(projection) = project_onto_path(
                &self.path,
                &self.path_distances,
                WorldPoint {
                    x: pose.x,
                    z: pose.z,
                },
                projection_min,
                projection_max,
            ) {
                max_progress = max_progress.max(projection.progress);
            }
        }

        let end = WorldPoint {
            x: pose.x,
            z: pose.z,
        };
        let projection = project_onto_path(
            &self.path,
            &self.path_distances,
            end,
            projection_min,
            projection_max,
        )
        .or_else(|| {
            project_onto_path(
                &self.path,
                &self.path_distances,
                end,
                projection_min,
                projection_max + 0.75,
            )
        })
        .ok_or(DwbRejectReason::NoPathProjection)?;
        let path_distance = projection.distance_squared.sqrt();
        let progress_gain = (max_progress - self.path_progress).max(0.0);
        let remaining_distance = (self.path_length() - self.path_progress).max(0.0);
        if matches!(clearance_mode, DwbClearanceMode::Nominal)
            && remaining_distance > self.config.arrival_distance
            && progress_gain < 0.03
            && (twist.v >= 0.0
                || current_clearance >= self.config.safety_margin.max(MIN_ROLLOUT_CLEARANCE))
            && twist.v < 0.2
        {
            return Err(DwbRejectReason::NoProgress);
        }

        let lookahead_progress = self.path_progress + 1.05 + twist.v.max(0.0) * 0.55;
        let target = sample_path_at(&self.path, &self.path_distances, lookahead_progress)
            .or_else(|| self.path.last().copied())
            .ok_or(DwbRejectReason::NoPathProjection)?;
        let target_distance = distance2d(end, target);
        let target_yaw = (target.x - pose.x).atan2(target.z - pose.z);
        let heading_error = normalize_angle(target_yaw - pose.yaw).abs();
        let low_speed_turn_cost = if twist.v < 0.25 && twist.w.abs() > 0.5 {
            (0.25 - twist.v.max(0.0)) * twist.w.abs() * 8.0
        } else {
            0.0
        };
        let stall_cost = if progress_gain < 0.02 && twist.v.abs() < 0.05 {
            14.0
        } else {
            0.0
        };
        let reverse_cost = twist.v.min(0.0).abs() * 30.0;
        let end_clearance = self.clearance_at_pose(pose);
        if end_clearance < required_clearance {
            return Err(DwbRejectReason::LowClearance);
        }
        min_rollout_clearance = min_rollout_clearance.min(end_clearance);
        let average_forward_speed =
            forward_motion / (steps as f64 * self.config.sim_step).max(f64::EPSILON);
        let obstacle_cost = if min_rollout_clearance.is_finite() {
            let hard_clearance = self.config.safety_margin.max(0.16);
            let hard_deficit = (hard_clearance - min_rollout_clearance).max(0.0) / hard_clearance;
            let comfort_clearance = self.config.safety_margin.max(0.24);
            let comfort_deficit =
                (comfort_clearance - min_rollout_clearance).max(0.0) / comfort_clearance;
            let tight_speed_cost = average_forward_speed * hard_deficit.powi(2) * 22.0;
            let clearance_loss = (current_clearance - min_rollout_clearance).max(0.0);
            let clearance_loss_cost = (clearance_loss / hard_clearance).powi(2) * 90.0;
            hard_deficit.powi(2) * 85.0
                + comfort_deficit.powi(2) * 12.0
                + tight_speed_cost
                + clearance_loss_cost
        } else {
            0.0
        };
        let path_cost = path_distance * 5.0;
        let target_cost = target_distance * 1.2;
        let heading_cost = heading_error * 0.55;
        let angular_cost = twist.w.abs() * 0.025;
        let acceleration_cost =
            (twist.w - current_w).abs() * 0.035 + (twist.v - current_v).abs() * 0.035;
        let progress_rate = progress_gain / self.config.sim_time.max(f64::EPSILON);
        let progress_reward = progress_rate * 22.0;
        let speed_reward = average_forward_speed * 5.5;
        let score = path_cost
            + target_cost
            + heading_cost
            + obstacle_cost
            + angular_cost
            + acceleration_cost
            + low_speed_turn_cost
            + stall_cost
            + reverse_cost
            - progress_reward
            - speed_reward;

        if !score.is_finite() {
            return Err(DwbRejectReason::NonFiniteScore);
        }

        Ok(TrajectoryScore {
            score,
            progress: progress_gain,
            breakdown: DwbScoreBreakdown {
                total: score,
                path_distance,
                target_distance,
                heading_error,
                obstacle_cost,
                progress_reward,
                speed_reward,
                angular_cost,
                acceleration_cost,
                low_speed_turn_cost: low_speed_turn_cost + stall_cost,
                reverse_cost,
                min_clearance: min_rollout_clearance,
                progress: progress_gain,
                end_x: end.x,
                end_z: end.z,
            },
        })
    }

    fn smooth_velocity(&self, current: Twist, target: Twist, dt: f64) -> Twist {
        if dt <= 0.0 {
            return current;
        }

        Twist {
            v: approach_axis(
                current.v,
                target.v,
                self.config.max_linear_accel,
                self.config.max_linear_decel,
                dt,
            )
            .clamp(
                -self.config.max_linear_speed * 0.45,
                self.config.max_linear_speed,
            ),
            w: approach_axis(
                current.w,
                target.w,
                self.config.max_angular_accel,
                self.config.max_angular_decel,
                dt,
            )
            .clamp(
                -self.config.max_angular_speed,
                self.config.max_angular_speed,
            ),
        }
    }

    fn twist_to_wheels(&self, twist: Twist) -> WheelCommand {
        let left_linear = twist.v + twist.w * self.config.track_width / 2.0;
        let right_linear = twist.v - twist.w * self.config.track_width / 2.0;

        WheelCommand {
            left_rad_per_sec: (left_linear / self.config.wheel_radius).clamp(
                -self.config.max_wheel_rad_per_sec,
                self.config.max_wheel_rad_per_sec,
            ),
            right_rad_per_sec: (right_linear / self.config.wheel_radius).clamp(
                -self.config.max_wheel_rad_per_sec,
                self.config.max_wheel_rad_per_sec,
            ),
        }
    }

    fn clear_path(&mut self) {
        self.path.clear();
        self.path_distances.clear();
        self.path_progress = 0.0;
        self.target_cell = None;
        self.path_version = self.path_version.wrapping_add(1);
    }

    fn plan_new_path(&mut self, pose: Pose2) -> bool {
        let Some(start) = nearest_passable_half_grid(self.size, &self.walls, pose.x, pose.z) else {
            return false;
        };
        let current_cell = cell_from_world(self.size, pose.x, pose.z);
        let total_cells = self.size * self.size;
        let max_attempts = total_cells.max(1) * REPLAN_ATTEMPT_MULTIPLIER;

        let mut target_candidates: Vec<usize> = self
            .goals
            .iter()
            .copied()
            .filter(|goal| *goal != current_cell && Some(*goal) != self.blocked_target_cell)
            .collect();

        let mut best_path =
            self.best_drivable_path(start, quantize_heading(pose.yaw), &target_candidates);

        if best_path.is_none() {
            target_candidates.clear();

            for _ in 0..max_attempts {
                target_candidates.push(self.choose_target_cell(current_cell));
            }

            best_path =
                self.best_drivable_path(start, quantize_heading(pose.yaw), &target_candidates);
        }

        let Some((target_cell, mut path, _, _)) = best_path else {
            return false;
        };

        let current_position = WorldPoint {
            x: pose.x,
            z: pose.z,
        };
        if let Some(first) = path.first().copied()
            && distance2d(current_position, first) > self.config.waypoint_tolerance
            && self.path_segment_is_clear(current_position, first)
        {
            path.insert(0, current_position);
        }
        let path_distances = cumulative_path_distances(&path);

        self.target_cell = Some(target_cell);
        self.path = path;
        self.path_distances = path_distances;
        self.path_progress = 0.0;
        self.path_version = self.path_version.wrapping_add(1);
        self.update_path_progress(WorldPoint {
            x: pose.x,
            z: pose.z,
        });
        true
    }

    fn best_drivable_path(
        &self,
        start: GridPoint,
        start_heading: u8,
        target_candidates: &[usize],
    ) -> Option<(usize, Vec<WorldPoint>, Vec<f64>, f64)> {
        let mut best_path: Option<(usize, Vec<WorldPoint>, Vec<f64>, f64)> = None;

        for target_cell in target_candidates {
            let request = PlanPathRequest {
                size: self.size,
                walls: self.walls.clone(),
                start_x2: start.x2,
                start_z2: start.z2,
                start_heading,
                goal_cell: *target_cell,
            };

            let Ok(result) = plan_path_impl(&request) else {
                continue;
            };

            if result.waypoints.is_empty() {
                continue;
            }

            let raw_path: Vec<WorldPoint> = result
                .waypoints
                .iter()
                .map(|point| WorldPoint {
                    x: point.x2 as f64 / 2.0,
                    z: point.z2 as f64 / 2.0,
                })
                .collect();
            let path = self.shortcut_path(&raw_path);
            let path_distances = cumulative_path_distances(&path);
            let score =
                result.cost + path_turn_cost(&path) * 0.15 + path_distance_score(&path) * 0.02;

            if best_path
                .as_ref()
                .map(|(_, _, _, best_score)| score < *best_score)
                .unwrap_or(true)
            {
                best_path = Some((*target_cell, path, path_distances, score));
            }
        }

        best_path
    }

    fn choose_target_cell(&mut self, current_cell: usize) -> usize {
        let total_cells = self.size * self.size;

        if total_cells <= 1 {
            return 0;
        }

        for _ in 0..(total_cells * REPLAN_ATTEMPT_MULTIPLIER) {
            let candidate = (self.next_random() * total_cells as f64).floor() as usize;

            if candidate != current_cell
                && Some(candidate) != self.target_cell
                && Some(candidate) != self.blocked_target_cell
            {
                return candidate;
            }
        }

        for candidate in 0..total_cells {
            if candidate != current_cell
                && Some(candidate) != self.target_cell
                && Some(candidate) != self.blocked_target_cell
            {
                return candidate;
            }
        }

        current_cell
    }

    fn next_random(&mut self) -> f64 {
        self.random_state = self.random_state.wrapping_add(0x6d2b_79f5);
        let mut value = self.random_state;
        value = (value ^ (value >> 15)).wrapping_mul(value | 1);
        value ^= value.wrapping_add((value ^ (value >> 7)).wrapping_mul(value | 61));
        ((value ^ (value >> 14)) as f64) / 4_294_967_296.0
    }

    fn update_path_progress(&mut self, position: WorldPoint) -> Option<PathProjection> {
        if self.path.len() < 2 {
            self.path_progress = 0.0;
            return None;
        }

        let projection = project_onto_path(
            &self.path,
            &self.path_distances,
            position,
            (self.path_progress - self.config.waypoint_tolerance).max(0.0),
            self.path_progress + self.config.max_linear_speed * 4.0,
        );

        if let Some(projection) = projection {
            self.path_progress = self.path_progress.max(projection.progress);
        }

        projection
    }

    fn path_length(&self) -> f64 {
        self.path_distances.last().copied().unwrap_or(0.0)
    }

    fn path_replan_distance(&self) -> f64 {
        (self.config.robot_half_width + self.config.safety_margin).max(0.32)
    }

    fn rollout_clearance_floor(&self, current_clearance: f64, mode: DwbClearanceMode) -> f64 {
        let nominal = match mode {
            DwbClearanceMode::Nominal => self.config.safety_margin.max(MIN_ROLLOUT_CLEARANCE),
            DwbClearanceMode::Relaxed => self.config.safety_margin.max(RELAXED_ROLLOUT_CLEARANCE),
        };

        if current_clearance.is_finite() && current_clearance < nominal {
            if matches!(mode, DwbClearanceMode::Relaxed) {
                return self.config.safety_margin * 0.5;
            }

            return current_clearance.max(self.config.safety_margin * 0.5);
        }

        nominal
    }

    fn shortcut_path(&self, path: &[WorldPoint]) -> Vec<WorldPoint> {
        if path.len() <= 2 {
            return path.to_vec();
        }

        let mut out = vec![path[0]];
        let mut index = 0usize;

        while index + 1 < path.len() {
            let mut best_next = index + 1;
            let max_candidate = (index + MAX_SHORTCUT_SPAN_POINTS).min(path.len() - 1);

            for candidate in ((index + 2)..=max_candidate).rev() {
                if !is_axis_or_diagonal_segment(path[index], path[candidate]) {
                    continue;
                }

                if self.path_segment_is_clear(path[index], path[candidate]) {
                    best_next = candidate;
                    break;
                }
            }

            out.push(path[best_next]);
            index = best_next;
        }

        out
    }

    fn path_segment_is_clear(&self, start: WorldPoint, end: WorldPoint) -> bool {
        let length = distance2d(start, end);

        if length <= f64::EPSILON {
            return true;
        }

        let yaw = (end.x - start.x).atan2(end.z - start.z);
        let steps = (length / 0.08).ceil().max(1.0) as usize;
        let required_clearance = self.config.safety_margin.max(MIN_ROLLOUT_CLEARANCE);

        for step in 0..=steps {
            let t = step as f64 / steps as f64;
            let pose = Pose2 {
                x: start.x + (end.x - start.x) * t,
                z: start.z + (end.z - start.z) * t,
                yaw,
            };

            if self.footprint_collides(pose) || self.clearance_at_pose(pose) < required_clearance {
                return false;
            }
        }

        true
    }

    fn wheel_speed_is_feasible(&self, twist: Twist) -> bool {
        let left_linear = twist.v + twist.w * self.config.track_width / 2.0;
        let right_linear = twist.v - twist.w * self.config.track_width / 2.0;
        let max_linear = self.config.max_wheel_rad_per_sec * self.config.wheel_radius;

        left_linear.abs() <= max_linear + 0.000_001 && right_linear.abs() <= max_linear + 0.000_001
    }

    fn footprint_collides(&self, pose: Pose2) -> bool {
        self.footprint_polygons(pose).into_iter().any(|footprint| {
            if footprint.iter().any(|point| {
                point.x < -0.02
                    || point.x > self.size as f64 + 0.02
                    || point.z < -0.02
                    || point.z > self.size as f64 + 0.02
            }) {
                return true;
            }

            let (min_x, max_x, min_z, max_z) = footprint_bounds(&footprint);

            self.wall_index
                .query(min_x, max_x, min_z, max_z)
                .into_iter()
                .map(|index| self.wall_rects[index])
                .any(|wall| {
                    wall.max_x >= min_x
                        && wall.min_x <= max_x
                        && wall.max_z >= min_z
                        && wall.min_z <= max_z
                        && polygon_intersects_aabb(&footprint, wall)
                })
        })
    }

    fn footprint_polygons(&self, pose: Pose2) -> Vec<Vec<WorldPoint>> {
        let forward = WorldPoint {
            x: pose.yaw.sin(),
            z: pose.yaw.cos(),
        };
        let right = WorldPoint {
            x: pose.yaw.cos(),
            z: -pose.yaw.sin(),
        };

        if !self.config.robot_footprints.is_empty() {
            return self
                .config
                .robot_footprints
                .iter()
                .map(|footprint| transform_footprint(pose, right, forward, footprint))
                .collect();
        }

        if !self.config.robot_footprint.is_empty() {
            return vec![transform_footprint(
                pose,
                right,
                forward,
                &self.config.robot_footprint,
            )];
        }

        vec![transform_footprint(
            pose,
            right,
            forward,
            &fallback_robot_footprint(&self.config),
        )]
    }

    fn clearance_at_pose(&self, pose: Pose2) -> f64 {
        let footprints = self.footprint_polygons(pose);

        if footprints.iter().all(Vec::is_empty) {
            return f64::INFINITY;
        }

        footprints
            .iter()
            .filter(|footprint| !footprint.is_empty())
            .map(|footprint| {
                if footprint.iter().any(|point| {
                    point.x < 0.0
                        || point.x > self.size as f64
                        || point.z < 0.0
                        || point.z > self.size as f64
                }) {
                    return 0.0;
                }

                let (min_x, max_x, min_z, max_z) = footprint_bounds(footprint);
                let clearance_window = self.config.safety_margin.max(0.025) + 0.28;
                let wall_clearance = self
                    .wall_index
                    .query(
                        min_x - clearance_window,
                        max_x + clearance_window,
                        min_z - clearance_window,
                        max_z + clearance_window,
                    )
                    .into_iter()
                    .map(|index| self.wall_rects[index])
                    .filter(|wall| {
                        wall.max_x >= min_x - clearance_window
                            && wall.min_x <= max_x + clearance_window
                            && wall.max_z >= min_z - clearance_window
                            && wall.min_z <= max_z + clearance_window
                    })
                    .map(|wall| polygon_aabb_distance(footprint, wall))
                    .fold(f64::INFINITY, f64::min);
                let boundary_clearance = footprint
                    .iter()
                    .map(|point| {
                        point
                            .x
                            .min(self.size as f64 - point.x)
                            .min(point.z)
                            .min(self.size as f64 - point.z)
                    })
                    .fold(f64::INFINITY, f64::min);

                wall_clearance.min(boundary_clearance)
            })
            .fold(f64::INFINITY, f64::min)
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
        is_passable_half_grid(self.size, self.walls, x2, z2)
    }
}

fn is_passable_half_grid(size: usize, walls: &[u8], x2: i32, z2: i32) -> bool {
    let limit = (size * 2) as i32;

    if x2 <= 0 || z2 <= 0 || x2 >= limit || z2 >= limit {
        return false;
    }

    let odd_x = x2 % 2 != 0;
    let odd_z = z2 % 2 != 0;

    match (odd_x, odd_z) {
        (true, true) => true,
        (false, false) => false,
        (false, true) => {
            let row = ((z2 - 1) / 2) as usize;
            let col = (x2 / 2 - 1) as usize;

            row < size && col + 1 < size && walls[row * size + col] & EAST == 0
        }
        (true, false) => {
            let row = (z2 / 2 - 1) as usize;
            let col = ((x2 - 1) / 2) as usize;

            row + 1 < size && col < size && walls[row * size + col] & NORTH == 0
        }
    }
}

fn nearest_passable_half_grid(size: usize, walls: &[u8], x: f64, z: f64) -> Option<GridPoint> {
    let limit = (size * 2) as i32;
    let target_x2 = (x * 2.0).round().clamp(1.0, (limit - 1) as f64) as i32;
    let target_z2 = (z * 2.0).round().clamp(1.0, (limit - 1) as f64) as i32;
    let mut best: Option<(GridPoint, i32)> = None;

    for z2 in 1..limit {
        for x2 in 1..limit {
            if !is_passable_half_grid(size, walls, x2, z2) {
                continue;
            }

            let distance_squared = (x2 - target_x2).pow(2) + (z2 - target_z2).pow(2);

            if best
                .map(|(_, best_distance)| distance_squared < best_distance)
                .unwrap_or(true)
            {
                best = Some((GridPoint { x2, z2 }, distance_squared));
            }
        }
    }

    best.map(|(point, _)| point)
}

fn quantize_heading(yaw: f64) -> u8 {
    modulo(
        (yaw / (std::f64::consts::PI / 4.0)).round() as i32,
        HEADING_COUNT as i32,
    ) as u8
}

fn cell_from_world(size: usize, x: f64, z: f64) -> usize {
    let col = x.floor().clamp(0.0, size.saturating_sub(1) as f64) as usize;
    let row = z.floor().clamp(0.0, size.saturating_sub(1) as f64) as usize;

    row * size + col
}

fn cell_center_world(size: usize, cell: usize) -> WorldPoint {
    WorldPoint {
        x: (cell % size) as f64 + 0.5,
        z: (cell / size) as f64 + 0.5,
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

fn local_footprint_point(
    pose: Pose2,
    right: WorldPoint,
    forward: WorldPoint,
    local_x: f64,
    local_z: f64,
) -> WorldPoint {
    WorldPoint {
        x: pose.x + right.x * local_x + forward.x * local_z,
        z: pose.z + right.z * local_x + forward.z * local_z,
    }
}

fn transform_footprint(
    pose: Pose2,
    right: WorldPoint,
    forward: WorldPoint,
    local_points: &[WorldPoint],
) -> Vec<WorldPoint> {
    local_points
        .iter()
        .map(|point| local_footprint_point(pose, right, forward, point.x, point.z))
        .collect()
}

fn fallback_robot_footprint(config: &NavigationConfig) -> Vec<WorldPoint> {
    let half_width = config.robot_half_width;
    let front = config.robot_front_length;
    let rear = config.robot_rear_length;
    let side_front = front * 0.62;
    let shoulder_width = half_width * 0.7;
    let shoulder_front = front * 0.88;

    vec![
        WorldPoint {
            x: half_width,
            z: -rear,
        },
        WorldPoint {
            x: half_width,
            z: side_front,
        },
        WorldPoint {
            x: shoulder_width,
            z: shoulder_front,
        },
        WorldPoint { x: 0.0, z: front },
        WorldPoint {
            x: -shoulder_width,
            z: shoulder_front,
        },
        WorldPoint {
            x: -half_width,
            z: side_front,
        },
        WorldPoint {
            x: -half_width,
            z: -rear,
        },
    ]
}

fn polygon_intersects_aabb(poly: &[WorldPoint], rect: WallRect) -> bool {
    let rect_points = [
        WorldPoint {
            x: rect.min_x,
            z: rect.min_z,
        },
        WorldPoint {
            x: rect.max_x,
            z: rect.min_z,
        },
        WorldPoint {
            x: rect.max_x,
            z: rect.max_z,
        },
        WorldPoint {
            x: rect.min_x,
            z: rect.max_z,
        },
    ];

    for index in 0..poly.len() {
        let next = (index + 1) % poly.len();
        let edge = WorldPoint {
            x: poly[next].x - poly[index].x,
            z: poly[next].z - poly[index].z,
        };
        let axis = normalize_axis(WorldPoint {
            x: edge.z,
            z: -edge.x,
        });
        let (poly_min, poly_max) = project_points(poly, axis);
        let (rect_min, rect_max) = project_points(&rect_points, axis);

        if poly_max < rect_min || rect_max < poly_min {
            return false;
        }
    }

    for axis in [WorldPoint { x: 1.0, z: 0.0 }, WorldPoint { x: 0.0, z: 1.0 }] {
        let (poly_min, poly_max) = project_points(poly, axis);
        let (rect_min, rect_max) = project_points(&rect_points, axis);

        if poly_max < rect_min || rect_max < poly_min {
            return false;
        }
    }

    true
}

fn polygon_aabb_distance(poly: &[WorldPoint], rect: WallRect) -> f64 {
    if polygon_intersects_aabb(poly, rect) {
        return 0.0;
    }

    let rect_points = rect_corners(rect);
    let rect_edges = rect_edges(rect);
    let mut best = f64::INFINITY;

    for point in poly {
        best = best.min(point_aabb_distance(point.x, point.z, rect));
    }

    for point in rect_points {
        best = best.min(point_polygon_edge_distance(point, poly));
    }

    for index in 0..poly.len() {
        let next = (index + 1) % poly.len();

        for (rect_start, rect_end) in rect_edges {
            best = best.min(segment_segment_distance(
                poly[index],
                poly[next],
                rect_start,
                rect_end,
            ));
        }
    }

    best
}

fn rect_corners(rect: WallRect) -> [WorldPoint; 4] {
    [
        WorldPoint {
            x: rect.min_x,
            z: rect.min_z,
        },
        WorldPoint {
            x: rect.max_x,
            z: rect.min_z,
        },
        WorldPoint {
            x: rect.max_x,
            z: rect.max_z,
        },
        WorldPoint {
            x: rect.min_x,
            z: rect.max_z,
        },
    ]
}

fn rect_edges(rect: WallRect) -> [(WorldPoint, WorldPoint); 4] {
    let corners = rect_corners(rect);

    [
        (corners[0], corners[1]),
        (corners[1], corners[2]),
        (corners[2], corners[3]),
        (corners[3], corners[0]),
    ]
}

fn point_polygon_edge_distance(point: WorldPoint, poly: &[WorldPoint]) -> f64 {
    let mut best = f64::INFINITY;

    for index in 0..poly.len() {
        let next = (index + 1) % poly.len();
        best = best.min(point_segment_distance(point, poly[index], poly[next]));
    }

    best
}

fn segment_segment_distance(a: WorldPoint, b: WorldPoint, c: WorldPoint, d: WorldPoint) -> f64 {
    if segments_intersect(a, b, c, d) {
        return 0.0;
    }

    point_segment_distance(a, c, d)
        .min(point_segment_distance(b, c, d))
        .min(point_segment_distance(c, a, b))
        .min(point_segment_distance(d, a, b))
}

fn point_segment_distance(point: WorldPoint, start: WorldPoint, end: WorldPoint) -> f64 {
    let dx = end.x - start.x;
    let dz = end.z - start.z;
    let length_squared = dx * dx + dz * dz;

    if length_squared <= f64::EPSILON {
        return distance2d(point, start);
    }

    let t =
        (((point.x - start.x) * dx + (point.z - start.z) * dz) / length_squared).clamp(0.0, 1.0);
    let projection = WorldPoint {
        x: start.x + dx * t,
        z: start.z + dz * t,
    };

    distance2d(point, projection)
}

fn segments_intersect(a: WorldPoint, b: WorldPoint, c: WorldPoint, d: WorldPoint) -> bool {
    let ab_c = cross(a, b, c);
    let ab_d = cross(a, b, d);
    let cd_a = cross(c, d, a);
    let cd_b = cross(c, d, b);

    if ab_c.abs() <= f64::EPSILON && point_on_segment(c, a, b) {
        return true;
    }

    if ab_d.abs() <= f64::EPSILON && point_on_segment(d, a, b) {
        return true;
    }

    if cd_a.abs() <= f64::EPSILON && point_on_segment(a, c, d) {
        return true;
    }

    if cd_b.abs() <= f64::EPSILON && point_on_segment(b, c, d) {
        return true;
    }

    (ab_c > 0.0) != (ab_d > 0.0) && (cd_a > 0.0) != (cd_b > 0.0)
}

fn cross(a: WorldPoint, b: WorldPoint, c: WorldPoint) -> f64 {
    (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x)
}

fn point_on_segment(point: WorldPoint, start: WorldPoint, end: WorldPoint) -> bool {
    point.x >= start.x.min(end.x) - f64::EPSILON
        && point.x <= start.x.max(end.x) + f64::EPSILON
        && point.z >= start.z.min(end.z) - f64::EPSILON
        && point.z <= start.z.max(end.z) + f64::EPSILON
}

fn footprint_bounds(footprint: &[WorldPoint]) -> (f64, f64, f64, f64) {
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_z = f64::INFINITY;
    let mut max_z = f64::NEG_INFINITY;

    for point in footprint {
        min_x = min_x.min(point.x);
        max_x = max_x.max(point.x);
        min_z = min_z.min(point.z);
        max_z = max_z.max(point.z);
    }

    (min_x, max_x, min_z, max_z)
}

fn normalize_axis(axis: WorldPoint) -> WorldPoint {
    let length = axis.x.hypot(axis.z);

    if length <= f64::EPSILON {
        WorldPoint { x: 1.0, z: 0.0 }
    } else {
        WorldPoint {
            x: axis.x / length,
            z: axis.z / length,
        }
    }
}

fn project_points(points: &[WorldPoint], axis: WorldPoint) -> (f64, f64) {
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;

    for point in points {
        let projected = point.x * axis.x + point.z * axis.z;
        min = min.min(projected);
        max = max.max(projected);
    }

    (min, max)
}

fn point_aabb_distance(x: f64, z: f64, rect: WallRect) -> f64 {
    let dx = if x < rect.min_x {
        rect.min_x - x
    } else if x > rect.max_x {
        x - rect.max_x
    } else {
        0.0
    };
    let dz = if z < rect.min_z {
        rect.min_z - z
    } else if z > rect.max_z {
        z - rect.max_z
    } else {
        0.0
    };

    dx.hypot(dz)
}

fn integrate_pose(pose: Pose2, twist: Twist, dt: f64) -> Pose2 {
    Pose2 {
        x: pose.x + twist.v * pose.yaw.sin() * dt,
        z: pose.z + twist.v * pose.yaw.cos() * dt,
        yaw: normalize_angle(pose.yaw + twist.w * dt),
    }
}

fn velocity_samples(min: f64, max: f64, count: usize) -> Vec<f64> {
    if count <= 1 || (max - min).abs() <= f64::EPSILON {
        return vec![min.clamp(min, max)];
    }

    let mut samples = Vec::with_capacity(count + 1);

    for index in 0..count {
        let t = index as f64 / (count - 1) as f64;
        samples.push(min + (max - min) * t);
    }

    if min <= 0.0 && max >= 0.0 && !samples.iter().any(|value| value.abs() < 0.000_001) {
        samples.push(0.0);
    }

    samples
}

fn is_better_score(candidate: TrajectoryScore, best: TrajectoryScore) -> bool {
    let candidate_makes_progress = candidate.progress > 0.03;
    let best_makes_progress = best.progress > 0.03;

    if candidate_makes_progress != best_makes_progress {
        return candidate_makes_progress;
    }

    candidate.score < best.score
        || (candidate.score == best.score && candidate.progress > best.progress)
}

fn cumulative_path_distances(path: &[WorldPoint]) -> Vec<f64> {
    let mut distances = vec![0.0];

    for index in 1..path.len() {
        distances.push(distances[index - 1] + distance2d(path[index - 1], path[index]));
    }

    distances
}

fn path_distance_score(path: &[WorldPoint]) -> f64 {
    path.windows(2)
        .map(|segment| distance2d(segment[0], segment[1]))
        .sum()
}

fn path_turn_cost(path: &[WorldPoint]) -> f64 {
    path.windows(3)
        .map(|window| {
            let first_yaw = (window[1].x - window[0].x).atan2(window[1].z - window[0].z);
            let second_yaw = (window[2].x - window[1].x).atan2(window[2].z - window[1].z);

            normalize_angle(second_yaw - first_yaw).abs()
        })
        .sum()
}

fn is_axis_or_diagonal_segment(start: WorldPoint, end: WorldPoint) -> bool {
    let dx = (end.x - start.x).abs();
    let dz = (end.z - start.z).abs();
    let epsilon = 0.000_001;

    dx <= epsilon || dz <= epsilon || (dx - dz).abs() <= epsilon
}

fn project_onto_path(
    path: &[WorldPoint],
    distances: &[f64],
    position: WorldPoint,
    minimum_progress: f64,
    maximum_progress: f64,
) -> Option<PathProjection> {
    let mut best: Option<PathProjection> = None;

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

        if progress < minimum_progress || progress > maximum_progress {
            continue;
        }

        let projection = WorldPoint {
            x: start.x + dx * t,
            z: start.z + dz * t,
        };
        let distance_squared =
            (position.x - projection.x).powi(2) + (position.z - projection.z).powi(2);

        if best
            .map(|candidate| distance_squared < candidate.distance_squared)
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

fn sample_path_at(path: &[WorldPoint], distances: &[f64], progress: f64) -> Option<WorldPoint> {
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

        return Some(WorldPoint {
            x: path[index].x + (path[index + 1].x - path[index].x) * t,
            z: path[index].z + (path[index + 1].z - path[index].z) * t,
        });
    }

    path.last().copied()
}

fn approach_axis(current: f64, target: f64, accel: f64, decel: f64, dt: f64) -> f64 {
    let moving_away_from_zero = target.abs() > current.abs();
    let limit = if moving_away_from_zero { accel } else { decel };

    approach_scalar(current, target, limit, dt)
}

fn approach_scalar(current: f64, target: f64, rate: f64, dt: f64) -> f64 {
    let max_delta = rate.max(0.0) * dt.max(0.0);

    if (target - current).abs() <= max_delta {
        target
    } else {
        current + (target - current).signum() * max_delta
    }
}

fn distance2d(from: WorldPoint, to: WorldPoint) -> f64 {
    (to.x - from.x).hypot(to.z - from.z)
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

fn frequency(ticks: usize, elapsed_seconds: f64) -> f64 {
    if elapsed_seconds <= 0.0 {
        0.0
    } else {
        ticks as f64 / elapsed_seconds
    }
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

    fn test_config() -> NavigationConfig {
        NavigationConfig {
            dwb_frequency: 10.0,
            max_wheel_rad_per_sec: 24.0,
            max_linear_speed: 1.8,
            max_angular_speed: 7.0,
            max_linear_accel: 8.0,
            max_linear_decel: 8.0,
            max_angular_accel: 28.0,
            max_angular_decel: 28.0,
            track_width: 0.41,
            wheel_radius: 0.09,
            sim_time: 0.8,
            sim_step: 0.025,
            vx_samples: 9,
            omega_samples: 13,
            waypoint_tolerance: 0.13,
            arrival_distance: 0.2,
            robot_half_width: 0.235,
            robot_front_length: 0.365,
            robot_rear_length: 0.18,
            robot_footprint: vec![
                WorldPoint { x: 0.235, z: -0.18 },
                WorldPoint { x: 0.235, z: 0.23 },
                WorldPoint { x: 0.0, z: 0.365 },
                WorldPoint { x: -0.235, z: 0.23 },
                WorldPoint {
                    x: -0.235,
                    z: -0.18,
                },
            ],
            robot_footprints: Vec::new(),
            safety_margin: 0.03,
            wall_thickness: 0.08,
        }
    }

    fn controller(size: usize, walls: Vec<u8>) -> NavigationController {
        let wall_rects = build_wall_rects(size, &walls, 0.08);
        let wall_index = WallSpatialIndex::new(size, &wall_rects);

        NavigationController {
            size,
            goals: Vec::new(),
            wall_rects,
            wall_index,
            walls,
            config: test_config(),
            random_state: 1,
            target_cell: None,
            blocked_target_cell: None,
            path: Vec::new(),
            path_distances: Vec::new(),
            path_progress: 0.0,
            path_version: 0,
            plan_retry_at: 0.0,
            target_twist: Twist { v: 0.0, w: 0.0 },
            smoothed_twist: Twist { v: 0.0, w: 0.0 },
            elapsed_seconds: 0.0,
            dwb_accumulator: f64::INFINITY,
            dwb_ticks: 0,
            smoother_ticks: 0,
            last_status: String::new(),
            last_stats: DwbStats::empty(f64::INFINITY, false, 0.0, 0.0, 0.0),
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

    #[test]
    fn footprint_collision_rejects_wall_overlap() {
        let size = 1;
        let walls = vec![ALL_WALLS];
        let nav = controller(size, walls);

        assert!(!nav.footprint_collides(Pose2 {
            x: 0.5,
            z: 0.5,
            yaw: 0.0,
        }));
        assert!(nav.footprint_collides(Pose2 {
            x: 0.05,
            z: 0.5,
            yaw: 0.0,
        }));
    }

    #[test]
    fn dwb_rejects_unsafe_trajectories_and_moves_when_safe() {
        let size = 3;
        let mut walls = vec![ALL_WALLS; size * size];
        open_between(&mut walls, size, 0, 1);
        open_between(&mut walls, size, 1, 2);
        let mut nav = controller(size, walls);
        nav.target_cell = Some(2);
        nav.path = vec![
            WorldPoint { x: 0.5, z: 0.5 },
            WorldPoint { x: 1.0, z: 0.5 },
            WorldPoint { x: 1.5, z: 0.5 },
            WorldPoint { x: 2.0, z: 0.5 },
            WorldPoint { x: 2.5, z: 0.5 },
        ];
        nav.path_distances = cumulative_path_distances(&nav.path);

        let (twist, stats, status) = nav.compute_dwb_command(
            Pose2 {
                x: 0.5,
                z: 0.5,
                yaw: std::f64::consts::PI / 2.0,
            },
            Velocity2 {
                vx: 0.0,
                omega: 0.0,
            },
        );

        assert_eq!(status, "tracking");
        assert!(stats.valid > 0);
        assert!(twist.expect("DWB should produce a command").v > 0.0);
    }

    #[test]
    fn velocity_smoother_limits_acceleration() {
        let nav = controller(1, vec![ALL_WALLS]);
        let smoothed =
            nav.smooth_velocity(Twist { v: 0.0, w: 0.0 }, Twist { v: 1.8, w: 7.0 }, 0.05);

        assert!(smoothed.v <= nav.config.max_linear_accel * 0.05 + f64::EPSILON);
        assert!(smoothed.w <= nav.config.max_angular_accel * 0.05 + f64::EPSILON);
    }
}
