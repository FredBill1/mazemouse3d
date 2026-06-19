import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { WALL_BITS, type MazeSnapshot } from "../domain/maze";
import {
  DwaMotorDriver,
  DEFAULT_DWA_OPTIONS,
  type DwaWorkerFactory,
} from "../rendering/dwaMotorDriver";
import type { MotorCommand } from "../rendering/motorDriver";
import type { DwaWorkerRequest, DwaWorkerResponse } from "../domain/dwa";

class FakeDwaWorker {
  static instances: FakeDwaWorker[] = [];

  readonly postMessage = vi.fn();
  readonly terminate = vi.fn();
  readonly #messageListeners = new Set<(event: MessageEvent<DwaWorkerResponse>) => void>();
  readonly #errorListeners = new Set<(event: ErrorEvent) => void>();

  constructor() {
    FakeDwaWorker.instances.push(this);
  }

  addEventListener(
    type: "message" | "error",
    listener: ((event: MessageEvent<DwaWorkerResponse>) => void) | ((event: ErrorEvent) => void),
  ): void {
    if (type === "message") {
      this.#messageListeners.add(listener as (event: MessageEvent<DwaWorkerResponse>) => void);
    } else {
      this.#errorListeners.add(listener as (event: ErrorEvent) => void);
    }
  }

  removeEventListener(
    type: "message" | "error",
    listener: ((event: MessageEvent<DwaWorkerResponse>) => void) | ((event: ErrorEvent) => void),
  ): void {
    if (type === "message") {
      this.#messageListeners.delete(listener as (event: MessageEvent<DwaWorkerResponse>) => void);
    } else {
      this.#errorListeners.delete(listener as (event: ErrorEvent) => void);
    }
  }

  emit(response: DwaWorkerResponse): void {
    const event = { data: response } as MessageEvent<DwaWorkerResponse>;

    for (const listener of this.#messageListeners) {
      listener(event);
    }
  }

  emitError(): void {
    const event = { message: "worker failed" } as ErrorEvent;

    for (const listener of this.#errorListeners) {
      listener(event);
    }
  }
}

describe("DWA motor driver", () => {
  let now = 0;

  beforeEach(() => {
    FakeDwaWorker.instances = [];
    now = 0;
    vi.spyOn(performance, "now").mockImplementation(() => now);
  });

  afterEach(() => {
    vi.restoreAllMocks();
  });

  it("initializes the worker and applies matching command responses", () => {
    const driver = new DwaMotorDriver(openMaze(), { workerFactory: fakeWorkerFactory });
    const worker = workerAt(0);
    const init = postedRequest(worker, 0);

    expect(init.type).toBe("init");
    worker.emit({ type: "ready", requestId: init.requestId });

    now = 20;
    driver.next(1 / 60, telemetryAt(0.5, 0.5, 0));
    const request = postedRequest(worker, 1);
    expect(request.type).toBe("telemetry");

    worker.emit({
      type: "command",
      requestId: request.requestId,
      command: commandResponse({ leftRadPerSec: 12, rightRadPerSec: 10 }),
    });

    expect(driver.next(0, telemetryAt(0.5, 0.5, 0))).toEqual({
      leftRadPerSec: 12,
      rightRadPerSec: 10,
    });
    expect(driver.debug.dwaHz).toBe(1);
    expect(driver.debug.targetDwaHz).toBe(DEFAULT_DWA_OPTIONS.targetControlHz);

    driver.dispose();
  });

  it("ignores stale or out-of-order command responses", () => {
    const driver = readyDriver();
    const worker = workerAt(0);

    now = 20;
    driver.next(1 / 60, telemetryAt(0.5, 0.5, 0));
    const request = postedRequest(worker, 1);
    worker.emit({
      type: "command",
      requestId: request.requestId + 1,
      command: commandResponse({ leftRadPerSec: 30, rightRadPerSec: 30 }),
    });

    expect(driver.command).toEqual({ leftRadPerSec: 0, rightRadPerSec: 0 });
    driver.dispose();
  });

  it("restarts an unresponsive worker and decays the last command", () => {
    const driver = readyDriver();
    const worker = workerAt(0);

    now = 20;
    driver.next(1 / 60, telemetryAt(0.5, 0.5, 0));
    let request = postedRequest(worker, 1);
    worker.emit({
      type: "command",
      requestId: request.requestId,
      command: commandResponse({ leftRadPerSec: 90, rightRadPerSec: 90 }),
    });

    now = 40;
    driver.next(1 / 60, telemetryAt(0.55, 0.5, Math.PI / 2));
    request = postedRequest(worker, 2);
    expect(request.type).toBe("telemetry");

    now = 200;
    const fallback = driver.next(1 / 60, telemetryAt(0.56, 0.5, Math.PI / 2));

    expect(worker.terminate).toHaveBeenCalledOnce();
    expect(FakeDwaWorker.instances).toHaveLength(2);
    expect(driver.debug.restartCount).toBe(1);
    expect(driver.debug.workerStatus).toBe("restarting");
    expect(fallback.leftRadPerSec).toBeGreaterThan(0);
    expect(fallback.leftRadPerSec).toBeLessThan(90);

    driver.dispose();
  });

  it("stops safely after repeated worker failures", () => {
    const driver = readyDriver();

    for (let index = 0; index < 3; index += 1) {
      workerAt(index).emitError();
    }

    const command = driver.next(1 / 60, telemetryAt(0.5, 0.5, 0));

    expect(command).toEqual({ leftRadPerSec: 0, rightRadPerSec: 0 });
    expect(driver.debug.restartCount).toBe(3);

    driver.dispose();
  });

  it("terminates the worker on dispose", () => {
    const driver = new DwaMotorDriver(openMaze(), { workerFactory: fakeWorkerFactory });
    const worker = workerAt(0);

    driver.dispose();

    expect(worker.terminate).toHaveBeenCalledOnce();
  });

  it("uses a default linear speed above the requested threshold", () => {
    expect(DEFAULT_DWA_OPTIONS.maxLinearSpeed).toBeGreaterThanOrEqual(5);
  });
});

function readyDriver(): DwaMotorDriver {
  const driver = new DwaMotorDriver(openMaze(), { workerFactory: fakeWorkerFactory });
  const worker = workerAt(0);
  const init = postedRequest(worker, 0);
  worker.emit({ type: "ready", requestId: init.requestId });

  return driver;
}

const fakeWorkerFactory: DwaWorkerFactory = () => new FakeDwaWorker();

function workerAt(index: number): FakeDwaWorker {
  const worker = FakeDwaWorker.instances[index];

  if (!worker) {
    throw new Error(`expected fake worker ${index}`);
  }

  return worker;
}

function postedRequest(worker: FakeDwaWorker, index: number): DwaWorkerRequest {
  const request = worker.postMessage.mock.calls[index]?.[0];

  if (!request) {
    throw new Error(`expected worker request ${index}`);
  }

  return request as DwaWorkerRequest;
}

function commandResponse(command: MotorCommand) {
  return {
    ...command,
    linearSpeed: 1,
    angularSpeed: 0,
    debug: {
      targetCell: 1,
      pathProgress: 0.2,
      pathLength: 1,
      targetX: 1.5,
      targetZ: 0.5,
      clearance: 0.2,
      score: 1,
      sampledCandidates: 10,
      validCandidates: 4,
      replanCount: 1,
    },
  };
}

function telemetryAt(x: number, z: number, yaw: number) {
  return {
    origin: { x, y: 0.09, z },
    yaw,
    linearVelocity: { x: 0, y: 0, z: 0 },
    angularVelocity: { x: 0, y: 0, z: 0 },
  };
}

function openMaze(): MazeSnapshot {
  return mazeSnapshot([15 & ~WALL_BITS.EAST, 15 & ~WALL_BITS.WEST, 15, 15]);
}

function mazeSnapshot(walls: number[]): MazeSnapshot {
  return {
    size: 2,
    seed: 7,
    iterations: 0,
    initialTemp: 1,
    finalTemp: 1,
    start: 0,
    goals: [0, 1, 2, 3],
    walls: Uint8Array.from(walls),
    solution: [0, 1],
    config: {
      size: 2,
      seed: 7,
      iterations: 0,
      initialTemp: 1,
      finalTemp: 1,
    },
    scoreHistory: [],
    metrics: {
      score: 0,
      shortestPathSteps: 0,
      turnsOnShortestPath: 0,
      longestStraightOnShortestPath: 0,
      longestStraightAnywhere: 0,
      diagonalRunCount: 0,
      longestDiagonalRun: 0,
      deadEnds: 0,
      junctions: 0,
      extraLoops: 0,
      avgDegree: 0,
      pathJunctions: 0,
      sideExitsFromShortestPath: 0,
      bridgeCount: 0,
      pathBridgeCount: 0,
      nonBridgePathEdges: 0,
      pathBridgeRatio: 0,
      full2x2OpenBlocks: 0,
      almost2x2OpenBlocks: 0,
      dense3x3PenaltyUnits: 0,
      degree4Cells: 0,
      adjacentJunctionPairs: 0,
    },
  };
}
