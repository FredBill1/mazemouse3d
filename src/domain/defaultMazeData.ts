import {
  DEFAULT_MAZE_CONFIG,
  type MazeMetrics,
  type MazeSnapshot,
  type ResolvedMazeConfig,
} from "./maze";

const GENERATED_CONFIG = {
  size: 16,
  seed: 514,
  iterations: 6000,
  initialTemp: 20,
  finalTemp: 0.12,
} as const satisfies ResolvedMazeConfig;

const START = 0 as const;
const GOALS = [119, 120, 135, 136] as const;
const WALLS_BASE64 =
  "DgwFBAUFBQUFBQUEBAcMBgoJBQINBQYMBgwFAwgHCgoIBQYJBAcIAgsIBw4JBQIKCg0BBgkFAwoOCgwDDAcLCgoMBQMMBwwDCgsIBQINBQIKCQQFAgwDDgkFAQYJBQYKCQYIBgkCDAMNBQYJBAcJAgwDCgoMAggEBAcJBgkGDgoIBQMJAwoKCQEHDAIMAgkDCwwFBQQDCg4MBAMJAwkFBgwBBQcLDgkBAgoMBQQFBwoKDAUEBQMMBgoLCg4JBgwCCQMMAwwFAwoJBQEDDgkCCgwFAwwBBQUBBgwFBgkGCwoIBwwBBQQFBgsKDgoNAQUCCQUBBQUDDQEFAwkBBQUFAw==" as const;
const SOLUTION = [
  0, 16, 32, 33, 34, 50, 51, 67, 66, 65, 81, 82, 83, 84, 100, 101, 85, 86, 70, 71, 55, 39, 38, 54,
  53, 52, 36, 35, 19, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 28, 44, 45, 46, 30, 14, 15, 31, 47, 63, 79,
  95, 111, 110, 94, 93, 92, 76, 75, 74, 90, 91, 107, 108, 124, 125, 141, 140, 156, 155, 139, 138,
  154, 153, 152, 168, 167, 166, 150, 134, 118, 119,
] as const;
const METRICS = {
  score: 438.81,
  shortestPathSteps: 80,
  turnsOnShortestPath: 49,
  longestStraightOnShortestPath: 9,
  longestStraightAnywhere: 13,
  diagonalRunCount: 13,
  longestDiagonalRun: 4,
  deadEnds: 35,
  junctions: 50,
  extraLoops: 8,
  avgDegree: 2.055,
  pathJunctions: 27,
  sideExitsFromShortestPath: 28,
  bridgeCount: 83,
  pathBridgeCount: 8,
  nonBridgePathEdges: 72,
  pathBridgeRatio: 0.1,
  full2x2OpenBlocks: 1,
  almost2x2OpenBlocks: 57,
  dense3x3PenaltyUnits: 0,
  degree4Cells: 0,
  adjacentJunctionPairs: 11,
} as const satisfies MazeMetrics;

export function isPrecomputedDefaultMazeConfig(config: ResolvedMazeConfig): boolean {
  return sameConfig(config, GENERATED_CONFIG) && sameConfig(DEFAULT_MAZE_CONFIG, GENERATED_CONFIG);
}

export function createDefaultMazeSnapshot(): MazeSnapshot {
  return {
    ...GENERATED_CONFIG,
    config: GENERATED_CONFIG,
    start: START,
    goals: [GOALS[0], GOALS[1], GOALS[2], GOALS[3]],
    walls: decodeBase64Bytes(WALLS_BASE64),
    solution: [...SOLUTION],
    metrics: { ...METRICS },
    scoreHistory: [],
  };
}

function sameConfig(left: ResolvedMazeConfig, right: ResolvedMazeConfig): boolean {
  return (
    left.size === right.size &&
    left.seed === right.seed &&
    left.iterations === right.iterations &&
    left.initialTemp === right.initialTemp &&
    left.finalTemp === right.finalTemp
  );
}

function decodeBase64Bytes(value: string): Uint8Array {
  const binary = atob(value);
  const bytes = new Uint8Array(binary.length);

  for (let index = 0; index < binary.length; index += 1) {
    bytes[index] = binary.charCodeAt(index);
  }

  return bytes;
}
