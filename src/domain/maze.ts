export const WALL_BITS = {
  NORTH: 1,
  EAST: 2,
  SOUTH: 4,
  WEST: 8,
} as const;

export type WallBit = (typeof WALL_BITS)[keyof typeof WALL_BITS];

export interface MazeConfig {
  readonly size?: number;
  readonly seed?: number;
  readonly iterations?: number;
  readonly initialTemp?: number;
  readonly finalTemp?: number;
}

export interface ResolvedMazeConfig {
  readonly size: number;
  readonly seed: number;
  readonly iterations: number;
  readonly initialTemp: number;
  readonly finalTemp: number;
}

export const DEFAULT_MAZE_CONFIG: ResolvedMazeConfig = {
  size: 16,
  seed: 514,
  iterations: 6000,
  initialTemp: 20.0,
  finalTemp: 0.12,
};

export interface MazeMetrics {
  readonly score: number;
  readonly shortestPathSteps: number;
  readonly turnsOnShortestPath: number;
  readonly longestStraightOnShortestPath: number;
  readonly longestStraightAnywhere: number;
  readonly diagonalRunCount: number;
  readonly longestDiagonalRun: number;
  readonly deadEnds: number;
  readonly junctions: number;
  readonly extraLoops: number;
  readonly avgDegree: number;
  readonly pathJunctions: number;
  readonly sideExitsFromShortestPath: number;
  readonly bridgeCount: number;
  readonly pathBridgeCount: number;
  readonly nonBridgePathEdges: number;
  readonly pathBridgeRatio: number;
  readonly full2x2OpenBlocks: number;
  readonly almost2x2OpenBlocks: number;
  readonly dense3x3PenaltyUnits: number;
  readonly degree4Cells: number;
  readonly adjacentJunctionPairs: number;
}

export interface RawMazeGenOutput {
  readonly size: number;
  readonly seed: number;
  readonly iterations: number;
  readonly initialTemp: number;
  readonly finalTemp: number;
  readonly start: number;
  readonly goals: readonly number[];
  readonly walls: readonly number[] | Uint8Array;
  readonly solution: readonly number[];
  readonly metrics: MazeMetrics;
  readonly scoreHistory: readonly number[];
}

export interface MazeSnapshot extends Omit<RawMazeGenOutput, "walls" | "goals" | "solution"> {
  readonly config: ResolvedMazeConfig;
  readonly goals: readonly [number, number, number, number];
  readonly walls: Uint8Array;
  readonly solution: readonly number[];
}

export type MazeWorkerRequest = {
  readonly type: "generateMaze";
  readonly requestId: number;
  readonly config?: MazeConfig;
};

export type MazeGeneratedResponse = {
  readonly type: "mazeGenerated";
  readonly requestId: number;
  readonly snapshot: MazeSnapshot;
};

export type MazeWorkerErrorResponse = {
  readonly type: "error";
  readonly requestId: number;
  readonly message: string;
};

export type MazeWorkerResponse = MazeGeneratedResponse | MazeWorkerErrorResponse;
