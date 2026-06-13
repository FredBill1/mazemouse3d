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
    expect(MICROMOUSE_BLUEPRINT.gearCount).toBe(4);
    expect(MICROMOUSE_BLUEPRINT.chipCount).toBe(1);
    expect(MICROMOUSE_BLUEPRINT.batteryCount).toBe(1);
  });

  it("keeps the wheel cluster and mass in the rear half of the mouse", () => {
    const wheelZValues = MICROMOUSE_BLUEPRINT.wheels.map((wheel) => wheel.localZ);
    const rearWheelZ = Math.min(...wheelZValues);
    const frontWheelZ = Math.max(...wheelZValues);
    const outerWheelX =
      Math.max(...MICROMOUSE_BLUEPRINT.wheels.map((wheel) => Math.abs(wheel.localX))) +
      MICROMOUSE_BLUEPRINT.wheel.width / 2;
    const innerWheelX =
      Math.min(...MICROMOUSE_BLUEPRINT.wheels.map((wheel) => Math.abs(wheel.localX))) -
      MICROMOUSE_BLUEPRINT.wheel.width / 2;

    expect(wheelZValues.every((localZ) => localZ < 0)).toBe(true);
    expect(outerWheelX).toBeLessThanOrEqual(MICROMOUSE_BLUEPRINT.chassis.width / 2);
    expect(innerWheelX).toBeGreaterThan(MICROMOUSE_BLUEPRINT.chassis.colliderWidth / 2);
    expect(frontWheelZ - rearWheelZ).toBeGreaterThan(MICROMOUSE_BLUEPRINT.wheel.radius * 2);
    expect(MICROMOUSE_BLUEPRINT.chassis.centerOfMassOffset.z).toBeGreaterThanOrEqual(rearWheelZ);
    expect(MICROMOUSE_BLUEPRINT.chassis.centerOfMassOffset.z).toBeLessThanOrEqual(frontWheelZ);
  });

  it("places the wheel axle above the low PCB deck", () => {
    const pcbTop = MICROMOUSE_BLUEPRINT.pcb.centerY + MICROMOUSE_BLUEPRINT.pcb.height / 2;

    expect(MICROMOUSE_BLUEPRINT.wheel.axleY).toBeGreaterThan(pcbTop);
    expect(MICROMOUSE_BLUEPRINT.wheel.axleY - MICROMOUSE_BLUEPRINT.wheel.radius).toBeCloseTo(0);
  });

  it("keeps sensors forward while drivetrain electronics sit behind the board center", () => {
    const boardFront =
      MICROMOUSE_BLUEPRINT.pcb.frontArcCenterZ + MICROMOUSE_BLUEPRINT.pcb.frontRadius;
    const boardLength = boardFront - MICROMOUSE_BLUEPRINT.pcb.rearZ;
    const frontSensorZ = Math.max(...MICROMOUSE_BLUEPRINT.sensors.map((sensor) => sensor.localZ));

    expect(boardLength).toBeLessThan(0.6);
    expect(MICROMOUSE_BLUEPRINT.sensors.every((sensor) => sensor.localZ > 0)).toBe(true);
    expect(frontSensorZ).toBeLessThan(boardFront);
    expect(MICROMOUSE_BLUEPRINT.electronics.motorLocalZ).toBeLessThan(0);
    expect(MICROMOUSE_BLUEPRINT.electronics.batteryLocalZ).toBeLessThan(0);
    expect(MICROMOUSE_BLUEPRINT.electronics.connectorLocalZ).toBeLessThan(0);
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
