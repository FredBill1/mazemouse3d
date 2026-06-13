import { WALL_BITS, type MazeSnapshot, type WallBit } from "../domain/maze";

export type WallOrientation = "horizontal" | "vertical";

export interface WallSegment {
  readonly orientation: WallOrientation;
  readonly row: number;
  readonly col: number;
  readonly centerX: number;
  readonly centerZ: number;
  readonly sizeX: number;
  readonly sizeZ: number;
}

export interface CellPoint {
  readonly x: number;
  readonly z: number;
}

export function cellIndexToRowCol(cell: number, size: number): { row: number; col: number } {
  return {
    row: Math.floor(cell / size),
    col: cell % size,
  };
}

export function cellCenter(cell: number, size: number): CellPoint {
  const { row, col } = cellIndexToRowCol(cell, size);

  return {
    x: col + 0.5,
    z: row + 0.5,
  };
}

export function hasWall(walls: Uint8Array, cell: number, bit: WallBit): boolean {
  return (walls[cell] & bit) !== 0;
}

export function buildWallSegments(
  maze: Pick<MazeSnapshot, "size" | "walls">,
  wallThickness = 0.08,
): WallSegment[] {
  const segments: WallSegment[] = [];

  for (let row = 0; row < maze.size; row += 1) {
    for (let col = 0; col < maze.size; col += 1) {
      const cell = row * maze.size + col;

      if (hasWall(maze.walls, cell, WALL_BITS.SOUTH)) {
        segments.push(horizontalWall(row, col, wallThickness));
      }

      if (hasWall(maze.walls, cell, WALL_BITS.WEST)) {
        segments.push(verticalWall(row, col, wallThickness));
      }

      if (row === maze.size - 1 && hasWall(maze.walls, cell, WALL_BITS.NORTH)) {
        segments.push(horizontalWall(row + 1, col, wallThickness));
      }

      if (col === maze.size - 1 && hasWall(maze.walls, cell, WALL_BITS.EAST)) {
        segments.push(verticalWall(row, col + 1, wallThickness));
      }
    }
  }

  return segments;
}

function horizontalWall(rowLine: number, col: number, wallThickness: number): WallSegment {
  return {
    orientation: "horizontal",
    row: rowLine,
    col,
    centerX: col + 0.5,
    centerZ: rowLine,
    sizeX: 1 + wallThickness,
    sizeZ: wallThickness,
  };
}

function verticalWall(row: number, colLine: number, wallThickness: number): WallSegment {
  return {
    orientation: "vertical",
    row,
    col: colLine,
    centerX: colLine,
    centerZ: row + 0.5,
    sizeX: wallThickness,
    sizeZ: 1 + wallThickness,
  };
}
