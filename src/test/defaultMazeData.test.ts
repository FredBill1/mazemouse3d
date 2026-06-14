import { describe, expect, it } from "vitest";
import {
  createDefaultMazeSnapshot,
  isPrecomputedDefaultMazeConfig,
} from "../domain/defaultMazeData";
import { DEFAULT_MAZE_CONFIG } from "../domain/maze";

describe("default maze data", () => {
  it("creates a snapshot for the default config", () => {
    const snapshot = createDefaultMazeSnapshot();

    expect(snapshot.config).toEqual(DEFAULT_MAZE_CONFIG);
    expect(snapshot.size).toBe(DEFAULT_MAZE_CONFIG.size);
    expect(snapshot.seed).toBe(DEFAULT_MAZE_CONFIG.seed);
    expect(snapshot.goals).toHaveLength(4);
    expect(snapshot.solution.length).toBeGreaterThan(0);
    expect(snapshot.walls).toBeInstanceOf(Uint8Array);
    expect(snapshot.walls).toHaveLength(DEFAULT_MAZE_CONFIG.size * DEFAULT_MAZE_CONFIG.size);
    expect(snapshot.scoreHistory).toEqual([]);
  });

  it("returns independent wall buffers", () => {
    const first = createDefaultMazeSnapshot();
    const second = createDefaultMazeSnapshot();
    const original = second.walls[0];

    first.walls[0] = first.walls[0] ^ 1;

    expect(second.walls[0]).toBe(original);
    expect(first.walls).not.toBe(second.walls);
  });

  it("matches only the checked-in default config", () => {
    expect(isPrecomputedDefaultMazeConfig(DEFAULT_MAZE_CONFIG)).toBe(true);
    expect(isPrecomputedDefaultMazeConfig({ ...DEFAULT_MAZE_CONFIG, seed: 515 })).toBe(false);
  });
});
