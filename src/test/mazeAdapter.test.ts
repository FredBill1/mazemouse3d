import { describe, expect, it } from "vitest";
import { DEFAULT_MAZE_CONFIG, type RawMazeGenOutput } from "../domain/maze";
import { normalizeMazeOutput, resolveMazeConfig } from "../domain/mazeAdapter";

const rawMaze: RawMazeGenOutput = {
  size: 2,
  seed: 7,
  iterations: 10,
  initialTemp: 3,
  finalTemp: 0.5,
  start: 0,
  goals: [0, 1, 2, 3],
  walls: [15, 15, 15, 15],
  solution: [0, 1, 3],
  scoreHistory: [1, 2, 3],
  metrics: {
    score: 12,
    shortestPathSteps: 2,
    turnsOnShortestPath: 1,
    longestStraightOnShortestPath: 1,
    longestStraightAnywhere: 2,
    diagonalRunCount: 0,
    longestDiagonalRun: 0,
    deadEnds: 0,
    junctions: 0,
    extraLoops: 0,
    avgDegree: 1,
    pathJunctions: 0,
    sideExitsFromShortestPath: 0,
    bridgeCount: 0,
    pathBridgeCount: 0,
    nonBridgePathEdges: 2,
    pathBridgeRatio: 0,
    full2x2OpenBlocks: 0,
    almost2x2OpenBlocks: 0,
    dense3x3PenaltyUnits: 0,
    degree4Cells: 0,
    adjacentJunctionPairs: 0,
  },
};

describe("maze adapter", () => {
  it("fills missing config values with defaults", () => {
    expect(resolveMazeConfig({ seed: 8 })).toEqual({
      ...DEFAULT_MAZE_CONFIG,
      seed: 8,
    });
  });

  it("normalizes wasm output into a transferable snapshot", () => {
    const snapshot = normalizeMazeOutput(rawMaze);

    expect(snapshot.config).toEqual({
      size: 2,
      seed: 7,
      iterations: 10,
      initialTemp: 3,
      finalTemp: 0.5,
    });
    expect(snapshot.goals).toEqual([0, 1, 2, 3]);
    expect(snapshot.solution).toEqual([0, 1, 3]);
    expect(snapshot.walls).toBeInstanceOf(Uint8Array);
    expect([...snapshot.walls]).toEqual([15, 15, 15, 15]);
  });

  it("rejects malformed goal output", () => {
    expect(() => normalizeMazeOutput({ ...rawMaze, goals: [0, 1, 2] })).toThrow(
      "expected four goal cells",
    );
  });
});
