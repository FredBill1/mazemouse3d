import { describe, expect, it } from "vitest";
import { WALL_BITS, type MazeSnapshot } from "../domain/maze";
import {
  buildWallSegments,
  cellCenter,
  cellIndexToRowCol,
  hasWall,
} from "../rendering/mazeGeometry";

function snapshotWithWalls(walls: number[]): Pick<MazeSnapshot, "size" | "walls"> {
  return {
    size: 2,
    walls: Uint8Array.from(walls),
  };
}

describe("maze geometry", () => {
  it("maps cell indexes to grid coordinates", () => {
    expect(cellIndexToRowCol(3, 2)).toEqual({ row: 1, col: 1 });
    expect(cellCenter(2, 2)).toEqual({ x: 0.5, z: 1.5 });
  });

  it("checks wall bits", () => {
    const walls = Uint8Array.from([WALL_BITS.NORTH | WALL_BITS.WEST]);

    expect(hasWall(walls, 0, WALL_BITS.NORTH)).toBe(true);
    expect(hasWall(walls, 0, WALL_BITS.EAST)).toBe(false);
  });

  it("builds one segment per visible wall edge", () => {
    const segments = buildWallSegments(snapshotWithWalls([15, 15, 15, 15]));

    expect(segments).toHaveLength(12);
    expect(segments.filter((segment) => segment.orientation === "horizontal")).toHaveLength(6);
    expect(segments.filter((segment) => segment.orientation === "vertical")).toHaveLength(6);
  });

  it("does not duplicate opened internal passages", () => {
    const segments = buildWallSegments(snapshotWithWalls([15 & ~WALL_BITS.EAST, 15, 15, 15]));

    expect(segments).toHaveLength(12);
  });
});
