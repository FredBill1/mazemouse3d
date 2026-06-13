export interface MotorCommand {
  readonly leftRadPerSec: number;
  readonly rightRadPerSec: number;
}

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

const STOPPED: MotorCommand = {
  leftRadPerSec: 0,
  rightRadPerSec: 0,
};

export class RandomMotorDriver {
  readonly #random: () => number;
  readonly #options: RandomMotorDriverOptions;
  #remainingSeconds = 0;
  #command: MotorCommand = STOPPED;

  constructor(seed: number, options: Partial<RandomMotorDriverOptions> = {}) {
    this.#random = seededRandom(seed);
    this.#options = {
      ...DEFAULT_OPTIONS,
      ...options,
    };
  }

  next(deltaSeconds: number, recoveryMode = false): MotorCommand {
    this.#remainingSeconds -= Math.max(0, deltaSeconds);

    if (recoveryMode) {
      this.#command = this.#createRecoveryCommand();
      this.#remainingSeconds = 0.35;
      return this.#command;
    }

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

    return clampCommand(
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

    return clampCommand(
      {
        leftRadPerSec: -speed - turn,
        rightRadPerSec: -speed + turn,
      },
      this.#options.maxWheelRadPerSec,
    );
  }

  #createRecoveryCommand(): MotorCommand {
    const speed = 0.55 * this.#options.maxWheelRadPerSec;

    return this.#random() < 0.5
      ? { leftRadPerSec: -speed, rightRadPerSec: speed * 0.55 }
      : { leftRadPerSec: speed * 0.55, rightRadPerSec: -speed };
  }
}

export function commandMagnitude(command: MotorCommand): number {
  return Math.max(Math.abs(command.leftRadPerSec), Math.abs(command.rightRadPerSec));
}

function seededRandom(seed: number): () => number {
  let state = (seed ^ 0x9e3779b9) >>> 0;

  return () => {
    state = (state + 0x6d2b79f5) >>> 0;
    let value = state;
    value = Math.imul(value ^ (value >>> 15), value | 1);
    value ^= value + Math.imul(value ^ (value >>> 7), value | 61);
    return ((value ^ (value >>> 14)) >>> 0) / 4294967296;
  };
}

function lerp(min: number, max: number, amount: number): number {
  return min + (max - min) * amount;
}

function clampCommand(command: MotorCommand, maxMagnitude: number): MotorCommand {
  return {
    leftRadPerSec: clamp(command.leftRadPerSec, -maxMagnitude, maxMagnitude),
    rightRadPerSec: clamp(command.rightRadPerSec, -maxMagnitude, maxMagnitude),
  };
}

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}
