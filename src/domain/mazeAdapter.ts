import {
  DEFAULT_MAZE_CONFIG,
  type MazeConfig,
  type MazeSnapshot,
  type RawMazeGenOutput,
  type ResolvedMazeConfig,
} from "./maze";

export function resolveMazeConfig(config: MazeConfig = {}): ResolvedMazeConfig {
  return {
    size: config.size ?? DEFAULT_MAZE_CONFIG.size,
    seed: config.seed ?? DEFAULT_MAZE_CONFIG.seed,
    iterations: config.iterations ?? DEFAULT_MAZE_CONFIG.iterations,
    initialTemp: config.initialTemp ?? DEFAULT_MAZE_CONFIG.initialTemp,
    finalTemp: config.finalTemp ?? DEFAULT_MAZE_CONFIG.finalTemp,
  };
}

export function normalizeMazeOutput(raw: RawMazeGenOutput): MazeSnapshot {
  if (raw.goals.length !== 4) {
    throw new Error(`expected four goal cells, received ${raw.goals.length}`);
  }

  const walls =
    raw.walls instanceof Uint8Array ? new Uint8Array(raw.walls) : Uint8Array.from(raw.walls);

  return {
    ...raw,
    config: resolveMazeConfig({
      size: raw.size,
      seed: raw.seed,
      iterations: raw.iterations,
      initialTemp: raw.initialTemp,
      finalTemp: raw.finalTemp,
    }),
    goals: [raw.goals[0], raw.goals[1], raw.goals[2], raw.goals[3]],
    solution: [...raw.solution],
    walls,
  };
}
