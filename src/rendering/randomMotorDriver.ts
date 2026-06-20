import {
  STOPPED_COMMAND,
  clampMotorCommand,
  commandMagnitude,
  createSeededRandom,
  type MotorCommand,
} from "./motorDriver";

export type { MotorCommand };
export { commandMagnitude };

export interface RandomMotorDriverOptions {
  readonly minDurationSeconds: number;
  readonly maxDurationSeconds: number;
  readonly maxWheelRadPerSec: number;
}

const DEFAULT_OPTIONS: RandomMotorDriverOptions = {
  minDurationSeconds: 0.4,
  maxDurationSeconds: 0.9,
  maxWheelRadPerSec: 18,
};

export class RandomMotorDriver {
  readonly #random: () => number;
  readonly #options: RandomMotorDriverOptions;
  #remainingSeconds = 0;
  #command: MotorCommand = STOPPED_COMMAND;

  constructor(seed: number, options: Partial<RandomMotorDriverOptions> = {}) {
    this.#random = createSeededRandom(seed);
    this.#options = {
      ...DEFAULT_OPTIONS,
      ...options,
    };
  }

  next(deltaSeconds: number): MotorCommand {
    this.#remainingSeconds -= Math.max(0, deltaSeconds);

    if (this.#remainingSeconds <= 0) {
      this.#command = this.#createRandomCommand();
      this.#remainingSeconds = lerp(
        this.#options.minDurationSeconds,
        this.#options.maxDurationSeconds,
        this.#random(),
      );
    }

    return this.#command;
  }

  get command(): MotorCommand {
    return this.#command;
  }

  #createRandomCommand(): MotorCommand {
    const choice = this.#random();

    if (choice < 0.58) {
      return this.#forwardCommand();
    }

    if (choice < 0.78) {
      return this.#arcCommand();
    }

    if (choice < 0.93) {
      return this.#turnCommand();
    }

    return this.#reverseCommand();
  }

  #forwardCommand(): MotorCommand {
    const base = lerp(0.48, 0.9, this.#random()) * this.#options.maxWheelRadPerSec;
    const drift = lerp(-0.12, 0.12, this.#random()) * this.#options.maxWheelRadPerSec;

    return clampMotorCommand(
      {
        leftRadPerSec: base - drift,
        rightRadPerSec: base + drift,
      },
      this.#options.maxWheelRadPerSec,
    );
  }

  #arcCommand(): MotorCommand {
    const fast = lerp(0.5, 0.9, this.#random()) * this.#options.maxWheelRadPerSec;
    const slow = fast * lerp(0.18, 0.58, this.#random());

    return this.#random() < 0.5
      ? { leftRadPerSec: slow, rightRadPerSec: fast }
      : { leftRadPerSec: fast, rightRadPerSec: slow };
  }

  #turnCommand(): MotorCommand {
    const speed = lerp(0.34, 0.72, this.#random()) * this.#options.maxWheelRadPerSec;

    return this.#random() < 0.5
      ? { leftRadPerSec: -speed, rightRadPerSec: speed }
      : { leftRadPerSec: speed, rightRadPerSec: -speed };
  }

  #reverseCommand(): MotorCommand {
    const speed = lerp(0.28, 0.48, this.#random()) * this.#options.maxWheelRadPerSec;
    const turn = lerp(-0.22, 0.22, this.#random()) * this.#options.maxWheelRadPerSec;

    return clampMotorCommand(
      {
        leftRadPerSec: -speed - turn,
        rightRadPerSec: -speed + turn,
      },
      this.#options.maxWheelRadPerSec,
    );
  }
}

function lerp(min: number, max: number, amount: number): number {
  return min + (max - min) * amount;
}
