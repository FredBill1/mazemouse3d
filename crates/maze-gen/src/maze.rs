use rand::{RngExt, SeedableRng, rngs::StdRng};
use serde::Serialize;
use std::collections::{HashSet, VecDeque};
use std::fmt;

pub const NORTH: u8 = 1;
pub const EAST: u8 = 2;
pub const SOUTH: u8 = 4;
pub const WEST: u8 = 8;
const ALL_WALLS: u8 = NORTH | EAST | SOUTH | WEST;

type Edge = (usize, usize);

struct BridgeSearch {
    tin: Vec<usize>,
    low: Vec<usize>,
    timer: usize,
    bridges: HashSet<Edge>,
}

impl BridgeSearch {
    fn new(total: usize) -> Self {
        Self {
            tin: vec![0; total],
            low: vec![0; total],
            timer: 0,
            bridges: HashSet::new(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct MazeConfig {
    pub size: usize,
    pub seed: u64,
    pub iterations: usize,
    pub initial_temp: f64,
    pub final_temp: f64,
}

#[derive(Clone, Debug, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Metrics {
    pub score: f64,
    pub shortest_path_steps: usize,
    pub turns_on_shortest_path: usize,
    pub longest_straight_on_shortest_path: usize,
    pub longest_straight_anywhere: usize,
    pub diagonal_run_count: usize,
    pub longest_diagonal_run: usize,
    pub dead_ends: usize,
    pub junctions: usize,
    pub extra_loops: isize,
    pub avg_degree: f64,
    pub path_junctions: usize,
    pub side_exits_from_shortest_path: usize,
    pub bridge_count: usize,
    pub path_bridge_count: usize,
    pub non_bridge_path_edges: usize,
    pub path_bridge_ratio: f64,
    pub full_2x2_open_blocks: usize,
    pub almost_2x2_open_blocks: usize,
    pub dense_3x3_penalty_units: usize,
    pub degree4_cells: usize,
    pub adjacent_junction_pairs: usize,
}

impl fmt::Display for Metrics {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  score: {:.2}", self.score)?;
        writeln!(f, "  shortest_path_steps: {}", self.shortest_path_steps)?;
        writeln!(
            f,
            "  turns_on_shortest_path: {}",
            self.turns_on_shortest_path
        )?;
        writeln!(
            f,
            "  longest_straight_on_shortest_path: {}",
            self.longest_straight_on_shortest_path
        )?;
        writeln!(
            f,
            "  longest_straight_anywhere: {}",
            self.longest_straight_anywhere
        )?;
        writeln!(f, "  diagonal_run_count: {}", self.diagonal_run_count)?;
        writeln!(f, "  longest_diagonal_run: {}", self.longest_diagonal_run)?;
        writeln!(f, "  dead_ends: {}", self.dead_ends)?;
        writeln!(f, "  junctions: {}", self.junctions)?;
        writeln!(f, "  extra_loops: {}", self.extra_loops)?;
        writeln!(f, "  avg_degree: {:.3}", self.avg_degree)?;
        writeln!(f, "  path_junctions: {}", self.path_junctions)?;
        writeln!(
            f,
            "  side_exits_from_shortest_path: {}",
            self.side_exits_from_shortest_path
        )?;
        writeln!(f, "  bridge_count: {}", self.bridge_count)?;
        writeln!(f, "  path_bridge_count: {}", self.path_bridge_count)?;
        writeln!(f, "  non_bridge_path_edges: {}", self.non_bridge_path_edges)?;
        writeln!(f, "  path_bridge_ratio: {:.3}", self.path_bridge_ratio)?;
        writeln!(f, "  full_2x2_open_blocks: {}", self.full_2x2_open_blocks)?;
        writeln!(
            f,
            "  almost_2x2_open_blocks: {}",
            self.almost_2x2_open_blocks
        )?;
        writeln!(
            f,
            "  dense_3x3_penalty_units: {}",
            self.dense_3x3_penalty_units
        )?;
        writeln!(f, "  degree4_cells: {}", self.degree4_cells)?;
        writeln!(
            f,
            "  adjacent_junction_pairs: {}",
            self.adjacent_junction_pairs
        )
    }
}

pub struct AnnealedMicromouseMaze {
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
    protected_edges: HashSet<Edge>,
}

impl AnnealedMicromouseMaze {
    pub fn new(cfg: MazeConfig) -> Result<Self, String> {
        if cfg.size < 6 || !cfg.size.is_multiple_of(2) {
            return Err("size 建议使用偶数且不小于 6，例如 16".into());
        }

        let n = cfg.size;
        let mid = n / 2;
        let goals = [
            (mid - 1) * n + mid - 1,
            (mid - 1) * n + mid,
            mid * n + mid - 1,
            mid * n + mid,
        ];

        let mut maze = Self {
            size: n,
            seed: cfg.seed,
            iterations: cfg.iterations,
            initial_temp: cfg.initial_temp,
            final_temp: cfg.final_temp,
            start: 0,
            goals,
            walls: vec![ALL_WALLS; n * n],
            solution: Vec::new(),
            metrics: Metrics::default(),
            score_history: Vec::with_capacity(cfg.iterations + 1),
            protected_edges: HashSet::new(),
        };
        maze.protected_edges = maze.goal_room_edges();
        maze.generate();
        Ok(maze)
    }

    #[inline]
    pub fn rc(&self, id: usize) -> (usize, usize) {
        (id / self.size, id % self.size)
    }

    #[inline]
    fn idx(&self, r: usize, c: usize) -> usize {
        r * self.size + c
    }

    #[inline]
    fn idx_i(&self, r: isize, c: isize) -> Option<usize> {
        (r >= 0 && c >= 0 && (r as usize) < self.size && (c as usize) < self.size)
            .then(|| self.idx(r as usize, c as usize))
    }

    #[inline]
    fn edge_key(a: usize, b: usize) -> Edge {
        if a < b { (a, b) } else { (b, a) }
    }

    fn goal_room_edges(&self) -> HashSet<Edge> {
        let mut edges = HashSet::new();
        for (i, &a) in self.goals.iter().enumerate() {
            let (ar, ac) = self.rc(a);
            for &b in &self.goals[i + 1..] {
                let (br, bc) = self.rc(b);
                if ar.abs_diff(br) + ac.abs_diff(bc) == 1 {
                    edges.insert(Self::edge_key(a, b));
                }
            }
        }
        edges
    }

    fn edge_bits(&self, a: usize, b: usize) -> (u8, u8) {
        let (ar, ac) = self.rc(a);
        let (br, bc) = self.rc(b);
        match (br as isize - ar as isize, bc as isize - ac as isize) {
            (1, 0) => (NORTH, SOUTH),
            (-1, 0) => (SOUTH, NORTH),
            (0, 1) => (EAST, WEST),
            (0, -1) => (WEST, EAST),
            _ => panic!("两个格子不相邻: {a}, {b}"),
        }
    }

    #[inline]
    fn wall_between(&self, walls: &[u8], a: usize, b: usize) -> bool {
        let (ba, _) = self.edge_bits(a, b);
        walls[a] & ba != 0
    }

    #[inline]
    fn remove_wall(&self, walls: &mut [u8], a: usize, b: usize) {
        let (ba, bb) = self.edge_bits(a, b);
        walls[a] &= !ba;
        walls[b] &= !bb;
    }

    #[inline]
    fn add_wall(&self, walls: &mut [u8], a: usize, b: usize) {
        let (ba, bb) = self.edge_bits(a, b);
        walls[a] |= ba;
        walls[b] |= bb;
    }

    fn internal_edges(&self) -> Vec<Edge> {
        let mut edges = Vec::with_capacity(2 * self.size * (self.size - 1));
        for r in 0..self.size {
            for c in 0..self.size {
                let a = self.idx(r, c);
                if r + 1 < self.size {
                    edges.push((a, self.idx(r + 1, c)));
                }
                if c + 1 < self.size {
                    edges.push((a, self.idx(r, c + 1)));
                }
            }
        }
        edges
    }

    pub fn degree_grid(&self, walls: &[u8]) -> Vec<u8> {
        walls.iter().map(|w| 4 - w.count_ones() as u8).collect()
    }

    fn neighbors_into(&self, walls: &[u8], id: usize, out: &mut Vec<usize>) {
        out.clear();
        let (r, c) = self.rc(id);
        let w = walls[id];
        if r + 1 < self.size && w & NORTH == 0 {
            out.push(self.idx(r + 1, c));
        }
        if c + 1 < self.size && w & EAST == 0 {
            out.push(self.idx(r, c + 1));
        }
        if r > 0 && w & SOUTH == 0 {
            out.push(self.idx(r - 1, c));
        }
        if c > 0 && w & WEST == 0 {
            out.push(self.idx(r, c - 1));
        }
    }

    pub fn shortest_path(&self, walls: &[u8]) -> Option<Vec<usize>> {
        let total = self.size * self.size;
        let mut prev = vec![usize::MAX; total];
        let mut q = VecDeque::from([self.start]);
        let mut neigh = Vec::with_capacity(4);
        prev[self.start] = self.start;

        while let Some(cur) = q.pop_front() {
            if self.goals.contains(&cur) {
                let mut p = vec![cur];
                let mut x = cur;
                while x != self.start {
                    x = prev[x];
                    p.push(x);
                }
                p.reverse();
                return Some(p);
            }
            self.neighbors_into(walls, cur, &mut neigh);
            for &nx in &neigh {
                if prev[nx] == usize::MAX {
                    prev[nx] = cur;
                    q.push_back(nx);
                }
            }
        }
        None
    }

    fn all_connected(&self, walls: &[u8]) -> bool {
        let total = self.size * self.size;
        let mut seen = vec![false; total];
        let mut q = VecDeque::from([self.start]);
        let mut count = 1;
        let mut neigh = Vec::with_capacity(4);
        seen[self.start] = true;

        while let Some(cur) = q.pop_front() {
            self.neighbors_into(walls, cur, &mut neigh);
            for &nx in &neigh {
                if !seen[nx] {
                    seen[nx] = true;
                    count += 1;
                    q.push_back(nx);
                }
            }
        }
        count == total
    }

    fn initial_maze(&self, rng: &mut StdRng) -> Vec<u8> {
        let total = self.size * self.size;
        let mut walls = vec![ALL_WALLS; total];
        let mut visited = vec![false; total];
        let mut stack = vec![self.start];
        let mut choices = Vec::with_capacity(4);
        visited[self.start] = true;

        while let Some(&cur) = stack.last() {
            choices.clear();
            let (r, c) = self.rc(cur);
            if r + 1 < self.size && !visited[self.idx(r + 1, c)] {
                choices.push(self.idx(r + 1, c));
            }
            if c + 1 < self.size && !visited[self.idx(r, c + 1)] {
                choices.push(self.idx(r, c + 1));
            }
            if r > 0 && !visited[self.idx(r - 1, c)] {
                choices.push(self.idx(r - 1, c));
            }
            if c > 0 && !visited[self.idx(r, c - 1)] {
                choices.push(self.idx(r, c - 1));
            }

            if choices.is_empty() {
                stack.pop();
            } else {
                let nx = choices[rng.random_range(0..choices.len())];
                self.remove_wall(&mut walls, cur, nx);
                visited[nx] = true;
                stack.push(nx);
            }
        }

        for &(a, b) in &self.protected_edges {
            self.remove_wall(&mut walls, a, b);
        }
        walls
    }

    fn is_open_rc(&self, walls: &[u8], r1: usize, c1: usize, r2: usize, c2: usize) -> bool {
        let w = walls[self.idx(r1, c1)];
        if r2 == r1 + 1 && c1 == c2 {
            w & NORTH == 0
        } else if r1 == r2 + 1 && c1 == c2 {
            w & SOUTH == 0
        } else if c2 == c1 + 1 && r1 == r2 {
            w & EAST == 0
        } else if c1 == c2 + 1 && r1 == r2 {
            w & WEST == 0
        } else {
            false
        }
    }

    fn open_between_cells(&self, walls: &[u8], a: (isize, isize), b: (isize, isize)) -> bool {
        let Some(_) = self.idx_i(a.0, a.1) else {
            return false;
        };
        let Some(_) = self.idx_i(b.0, b.1) else {
            return false;
        };
        if (a.0 - b.0).abs() + (a.1 - b.1).abs() != 1 {
            return false;
        }
        self.is_open_rc(
            walls,
            a.0 as usize,
            a.1 as usize,
            b.0 as usize,
            b.1 as usize,
        )
    }

    fn can_diagonal_strip_step(
        &self,
        walls: &[u8],
        a: (isize, isize),
        dr: isize,
        dc: isize,
        offset: (isize, isize),
        degrees: &[u8],
    ) -> bool {
        let b = (a.0 + dr, a.1 + dc);
        let a2 = (a.0 + offset.0, a.1 + offset.1);
        let b2 = (b.0 + offset.0, b.1 + offset.1);
        let cells = [a, a2, b, b2];
        let mut deg = [0u8; 4];

        for (i, &(r, c)) in cells.iter().enumerate() {
            let Some(id) = self.idx_i(r, c) else {
                return false;
            };
            deg[i] = degrees[id];
        }
        if !self.open_between_cells(walls, a, a2) {
            return false;
        }
        if !self.open_between_cells(walls, a2, b) {
            return false;
        }
        if !self.open_between_cells(walls, b, b2) {
            return false;
        }
        deg.iter().copied().max().unwrap_or(0) < 4 && deg.iter().filter(|&&d| d >= 3).count() <= 2
    }

    pub fn collect_diagonal_runs(
        &self,
        walls: &[u8],
        min_len: usize,
        degrees: &[u8],
    ) -> Vec<Vec<(f64, f64)>> {
        let n = self.size as isize;
        let mut runs = Vec::new();
        for (dr, dc) in [(1, 1), (1, -1)] {
            for offset in [(0, dc), (dr, 0)] {
                for r in 0..n {
                    for c in 0..n {
                        if self.can_diagonal_strip_step(
                            walls,
                            (r - dr, c - dc),
                            dr,
                            dc,
                            offset,
                            degrees,
                        ) {
                            continue;
                        }
                        if !self.can_diagonal_strip_step(walls, (r, c), dr, dc, offset, degrees) {
                            continue;
                        }
                        let (mut cr, mut cc) = (r, c);
                        let mut line_a = vec![(r, c)];
                        let mut line_b = vec![(r + offset.0, c + offset.1)];
                        while self.can_diagonal_strip_step(walls, (cr, cc), dr, dc, offset, degrees)
                        {
                            cr += dr;
                            cc += dc;
                            line_a.push((cr, cc));
                            line_b.push((cr + offset.0, cc + offset.1));
                        }
                        if line_a.len() >= min_len {
                            runs.push(
                                line_a
                                    .iter()
                                    .zip(line_b.iter())
                                    .map(|(&(ar, ac), &(br, bc))| {
                                        ((ar + br) as f64 / 2.0, (ac + bc) as f64 / 2.0)
                                    })
                                    .collect(),
                            );
                        }
                    }
                }
            }
        }
        runs
    }

    fn path_turn_metrics(&self, path: &[usize]) -> (usize, usize) {
        if path.len() < 2 {
            return (0, 0);
        }
        let mut turns = 0;
        let mut longest = 1;
        let mut cur_len = 1;
        let mut last = None;
        for w in path.windows(2) {
            let (ar, ac) = self.rc(w[0]);
            let (br, bc) = self.rc(w[1]);
            let d = (br as isize - ar as isize, bc as isize - ac as isize);
            if Some(d) == last {
                cur_len += 1;
                longest = longest.max(cur_len);
            } else if last.replace(d).is_some() {
                turns += 1;
                cur_len = 1;
            }
        }
        (turns, longest)
    }

    fn longest_straight_anywhere(&self, walls: &[u8]) -> usize {
        let mut best = 1;
        for r in 0..self.size {
            let mut run = 1;
            for c in 0..self.size - 1 {
                if walls[self.idx(r, c)] & EAST == 0 {
                    run += 1;
                    best = best.max(run);
                } else {
                    run = 1;
                }
            }
        }
        for c in 0..self.size {
            let mut run = 1;
            for r in 0..self.size - 1 {
                if walls[self.idx(r, c)] & NORTH == 0 {
                    run += 1;
                    best = best.max(run);
                } else {
                    run = 1;
                }
            }
        }
        best
    }

    fn open_area_penalty(&self, walls: &[u8], degrees: &[u8]) -> (f64, Metrics) {
        let n = self.size;
        let mut m = Metrics::default();

        for r in 0..n - 1 {
            for c in 0..n - 1 {
                let mut open = 0;
                if walls[self.idx(r, c)] & EAST == 0 {
                    open += 1;
                }
                if walls[self.idx(r + 1, c)] & EAST == 0 {
                    open += 1;
                }
                if walls[self.idx(r, c)] & NORTH == 0 {
                    open += 1;
                }
                if walls[self.idx(r, c + 1)] & NORTH == 0 {
                    open += 1;
                }
                if open == 4 {
                    m.full_2x2_open_blocks += 1;
                } else if open == 3 {
                    m.almost_2x2_open_blocks += 1;
                }
            }
        }

        for r in 0..n - 2 {
            for c in 0..n - 2 {
                let mut open_edges = 0usize;
                for rr in r..r + 3 {
                    for cc in c..c + 2 {
                        if walls[self.idx(rr, cc)] & EAST == 0 {
                            open_edges += 1;
                        }
                    }
                }
                for rr in r..r + 2 {
                    for cc in c..c + 3 {
                        if walls[self.idx(rr, cc)] & NORTH == 0 {
                            open_edges += 1;
                        }
                    }
                }
                if open_edges >= 9 {
                    m.dense_3x3_penalty_units += (open_edges - 8).pow(2);
                }
            }
        }

        for r in 0..n {
            for c in 0..n {
                let id = self.idx(r, c);
                let deg = degrees[id];
                if deg >= 4 {
                    m.degree4_cells += 1;
                }
                if deg >= 3 {
                    if r + 1 < n && walls[id] & NORTH == 0 && degrees[self.idx(r + 1, c)] >= 3 {
                        m.adjacent_junction_pairs += 1;
                    }
                    if c + 1 < n && walls[id] & EAST == 0 && degrees[self.idx(r, c + 1)] >= 3 {
                        m.adjacent_junction_pairs += 1;
                    }
                }
            }
        }

        let penalty = m.full_2x2_open_blocks as f64 * 85.0
            + m.almost_2x2_open_blocks as f64 * 2.0
            + m.dense_3x3_penalty_units as f64 * 30.0
            + m.degree4_cells as f64 * 40.0
            + m.adjacent_junction_pairs as f64 * 3.0;
        (penalty, m)
    }

    fn bridge_dfs(&self, walls: &[u8], v: usize, parent: usize, search: &mut BridgeSearch) {
        search.timer += 1;
        search.tin[v] = search.timer;
        search.low[v] = search.timer;
        let mut neigh = Vec::with_capacity(4);
        self.neighbors_into(walls, v, &mut neigh);
        for nx in neigh {
            if nx == parent {
                continue;
            }
            if search.tin[nx] != 0 {
                search.low[v] = search.low[v].min(search.tin[nx]);
            } else {
                self.bridge_dfs(walls, nx, v, search);
                search.low[v] = search.low[v].min(search.low[nx]);
                if search.low[nx] > search.tin[v] {
                    search.bridges.insert(Self::edge_key(v, nx));
                }
            }
        }
    }

    fn bridge_stats(&self, walls: &[u8], path: &[usize]) -> (usize, usize, usize, f64) {
        let total = self.size * self.size;
        let mut search = BridgeSearch::new(total);
        self.bridge_dfs(walls, self.start, usize::MAX, &mut search);

        let path_bridge_count = path
            .windows(2)
            .filter(|w| search.bridges.contains(&Self::edge_key(w[0], w[1])))
            .count();
        let path_edges = path.len().saturating_sub(1);
        let non_bridge_path_edges = path_edges - path_bridge_count;
        let ratio = path_bridge_count as f64 / path_edges.max(1) as f64;
        (
            search.bridges.len(),
            path_bridge_count,
            non_bridge_path_edges,
            ratio,
        )
    }

    fn score(&self, walls: &[u8]) -> (f64, Metrics) {
        let Some(path) = self.shortest_path(walls) else {
            return (-1e9, Metrics::default());
        };
        let n = self.size;
        let path_len = path.len() - 1;
        let (turns, longest_path_straight) = self.path_turn_metrics(&path);
        let longest_straight = self.longest_straight_anywhere(walls);
        let degrees = self.degree_grid(walls);

        let mut dead_ends = 0usize;
        let mut junctions = 0usize;
        let mut degree_sum = 0usize;
        for (id, &deg) in degrees.iter().enumerate() {
            degree_sum += deg as usize;
            if !self.goals.contains(&id) && id != self.start && deg == 1 {
                dead_ends += 1;
            }
            if deg >= 3 {
                junctions += 1;
            }
        }

        let edges = degree_sum / 2;
        let extra_loops = edges as isize - (n * n - 1) as isize;
        let avg_degree = degree_sum as f64 / (n * n) as f64;

        let diagonal_runs = self.collect_diagonal_runs(walls, 3, &degrees);
        let diagonal_run_count = diagonal_runs.len();
        let longest_diagonal = diagonal_runs.iter().map(Vec::len).max().unwrap_or(0);

        let mut path_index = vec![None; n * n];
        for (i, &cell) in path.iter().enumerate() {
            path_index[cell] = Some(i);
        }

        let mut neigh = Vec::with_capacity(4);
        let mut path_junctions = 0usize;
        let mut side_exits = 0usize;
        for (i, &cell) in path.iter().enumerate() {
            if degrees[cell] >= 3 {
                path_junctions += 1;
            }
            self.neighbors_into(walls, cell, &mut neigh);
            side_exits += neigh
                .iter()
                .filter(|&&nb| match path_index[nb] {
                    Some(j) => j.abs_diff(i) != 1,
                    None => true,
                })
                .count();
        }

        let (bridge_count, path_bridge_count, non_bridge_path_edges, path_bridge_ratio_raw) =
            self.bridge_stats(walls, &path);
        let (room_penalty, room) = self.open_area_penalty(walls, &degrees);

        let preferred_min_path = (n * n) as f64 * 0.28;
        let too_short_penalty = (preferred_min_path as isize - path_len as isize)
            .max(0)
            .pow(2) as f64
            * 0.10;
        let open_penalty = (avg_degree - 2.42).max(0.0).powi(2) * 260.0;
        let start_open_penalty = (degrees[self.start] as isize - 2).max(0) as f64 * 10.0;
        let linearity_penalty = path_bridge_ratio_raw.powf(1.65) * 300.0;
        let excessive_loop_penalty =
            (extra_loops - ((n * n) as f64 * 0.18) as isize).max(0) as f64 * 8.0;

        let score = path_len as f64 * 1.55
            + turns.min(55) as f64 * 2.2
            + longest_path_straight.min(10) as f64 * 4.0
            + longest_straight.min(12) as f64 * 2.5
            + longest_diagonal.min(9) as f64 * 9.0
            + (diagonal_run_count.min(12) as f64).ln_1p() * 12.0
            + (dead_ends as f64).ln_1p() * 5.5
            + (junctions as f64).ln_1p() * 9.0
            + (extra_loops.max(0) as f64).ln_1p() * 22.0
            + path_junctions.min(35) as f64 * 1.7
            + (side_exits.min(50) as f64).ln_1p() * 24.0
            + non_bridge_path_edges as f64 * 1.15
            - too_short_penalty
            - open_penalty
            - room_penalty
            - linearity_penalty
            - excessive_loop_penalty
            - start_open_penalty;

        (
            score,
            Metrics {
                score: round(score, 2),
                shortest_path_steps: path_len,
                turns_on_shortest_path: turns,
                longest_straight_on_shortest_path: longest_path_straight,
                longest_straight_anywhere: longest_straight,
                diagonal_run_count,
                longest_diagonal_run: longest_diagonal,
                dead_ends,
                junctions,
                extra_loops,
                avg_degree: round(avg_degree, 3),
                path_junctions,
                side_exits_from_shortest_path: side_exits,
                bridge_count,
                path_bridge_count,
                non_bridge_path_edges,
                path_bridge_ratio: round(path_bridge_ratio_raw, 3),
                full_2x2_open_blocks: room.full_2x2_open_blocks,
                almost_2x2_open_blocks: room.almost_2x2_open_blocks,
                dense_3x3_penalty_units: room.dense_3x3_penalty_units,
                degree4_cells: room.degree4_cells,
                adjacent_junction_pairs: room.adjacent_junction_pairs,
            },
        )
    }

    fn mutate(&self, walls: &[u8], rng: &mut StdRng, all_edges: &[Edge]) -> Option<Vec<u8>> {
        let (a, b) = all_edges[rng.random_range(0..all_edges.len())];
        if self.protected_edges.contains(&Self::edge_key(a, b)) {
            return None;
        }

        let mut candidate = walls.to_vec();
        if self.wall_between(&candidate, a, b) {
            self.remove_wall(&mut candidate, a, b);
            return Some(candidate);
        }
        self.add_wall(&mut candidate, a, b);
        self.all_connected(&candidate).then_some(candidate)
    }

    pub fn generate(&mut self) {
        let mut rng = StdRng::seed_from_u64(self.seed);
        let all_edges = self.internal_edges();
        let mut current = self.initial_maze(&mut rng);
        let (mut current_score, _) = self.score(&current);
        let mut best = current.clone();
        let mut best_score = current_score;
        self.score_history.clear();
        self.score_history.push(best_score);

        for i in 1..=self.iterations {
            let t = i as f64 / self.iterations as f64;
            let temp = self.initial_temp * (self.final_temp / self.initial_temp).powf(t);

            if let Some(candidate) = self.mutate(&current, &mut rng, &all_edges) {
                let (candidate_score, _) = self.score(&candidate);
                let delta = candidate_score - current_score;
                if delta >= 0.0 || rng.random::<f64>() < (delta / temp.max(1e-9)).exp() {
                    current = candidate;
                    current_score = candidate_score;
                    if current_score > best_score {
                        best = current.clone();
                        best_score = current_score;
                    }
                }
            }
            self.score_history.push(best_score);
        }

        self.walls = best;
        self.solution = self.shortest_path(&self.walls).unwrap_or_default();
        self.metrics = self.score(&self.walls).1;
    }
}

fn round(x: f64, digits: i32) -> f64 {
    let p = 10f64.powi(digits);
    (x * p).round() / p
}
