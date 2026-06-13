import { describe, expect, it } from "vitest";
import {
  MICROMOUSE_BLUEPRINT,
  initialMouseYaw,
  yawBetweenCells,
} from "../rendering/micromouseModel";

describe("micromouse model", () => {
  it("describes the expected visible hardware", () => {
    expect(MICROMOUSE_BLUEPRINT.wheels).toHaveLength(4);
    expect(MICROMOUSE_BLUEPRINT.sensors).toHaveLength(6);
    expect(MICROMOUSE_BLUEPRINT.motorCount).toBe(2);
    expect(MICROMOUSE_BLUEPRINT.gearCount).toBe(2);
    expect(MICROMOUSE_BLUEPRINT.chipCount).toBe(1);
    expect(MICROMOUSE_BLUEPRINT.batteryCount).toBe(1);
  });

  it("maps neighboring cells to local +Z based yaw", () => {
    expect(yawBetweenCells(0, 1, 4)).toBeCloseTo(Math.PI / 2);
    expect(yawBetweenCells(1, 0, 4)).toBeCloseTo(-Math.PI / 2);
    expect(yawBetweenCells(4, 0, 4)).toBeCloseTo(Math.PI);
    expect(yawBetweenCells(0, 4, 4)).toBeCloseTo(0);
  });

  it("uses the first solution step for initial heading", () => {
    expect(initialMouseYaw({ size: 4, start: 0, solution: [0, 1] })).toBeCloseTo(Math.PI / 2);
    expect(initialMouseYaw({ size: 4, start: 0, solution: [0] })).toBe(0);
  });
});
