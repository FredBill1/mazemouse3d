import { describe, expect, it } from "vitest";
import { RandomMotorDriver } from "../rendering/randomMotorDriver";

describe("random motor driver", () => {
  it("produces deterministic commands for a seed", () => {
    expect(sampleCommands(514)).toEqual(sampleCommands(514));
    expect(sampleCommands(514)).not.toEqual(sampleCommands(515));
  });

  it("keeps wheel speeds within configured bounds", () => {
    const driver = new RandomMotorDriver(42, {
      minDurationSeconds: 0.1,
      maxDurationSeconds: 0.1,
      maxWheelRadPerSec: 8,
    });

    for (let index = 0; index < 40; index += 1) {
      const command = driver.next(0.2);

      expect(Math.abs(command.leftRadPerSec)).toBeLessThanOrEqual(8);
      expect(Math.abs(command.rightRadPerSec)).toBeLessThanOrEqual(8);
    }
  });
});

function sampleCommands(seed: number): string[] {
  const driver = new RandomMotorDriver(seed, {
    minDurationSeconds: 0.1,
    maxDurationSeconds: 0.1,
    maxWheelRadPerSec: 12,
  });
  const commands: string[] = [];

  for (let index = 0; index < 8; index += 1) {
    const command = driver.next(0.2);
    commands.push(`${command.leftRadPerSec.toFixed(4)}:${command.rightRadPerSec.toFixed(4)}`);
  }

  return commands;
}
