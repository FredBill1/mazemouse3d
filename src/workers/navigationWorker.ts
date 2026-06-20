import initWasm, { NavigationController } from "../generated/maze-nav/maze_nav.js";
import type {
  NavigationCommandResponse,
  NavigationInitRequest,
  NavigationTickRequest,
  NavigationWorkerRequest,
  NavigationWorkerResponse,
} from "../domain/navigation";

let wasmReady: Promise<void> | null = null;
let controller: NavigationController | null = null;
let activeInit: Promise<void> | null = null;

const ctx = self as unknown as Worker;

ctx.addEventListener("message", (event: MessageEvent<NavigationWorkerRequest>) => {
  void handleRequest(event.data);
});

async function handleRequest(request: NavigationWorkerRequest): Promise<void> {
  try {
    if (request.type === "init") {
      activeInit = initializeController(request);
      await activeInit;
      return;
    }

    await activeInit;
    handleTick(request);
  } catch (error) {
    postResponse({
      type: "error",
      requestId: request.type === "init" ? request.requestId : undefined,
      sequence: request.type === "tick" ? request.sequence : undefined,
      message: error instanceof Error ? error.message : String(error),
    });
  }
}

async function initializeController(request: NavigationInitRequest): Promise<void> {
  wasmReady ??= initWasm().then(() => undefined);
  await wasmReady;
  controller?.free();
  controller = new NavigationController({
    size: request.size,
    walls: Array.from(request.walls),
    goals: Array.from(request.goals),
    seed: request.seed,
    config: request.config,
  });
  postResponse({
    type: "ready",
    requestId: request.requestId,
  });
}

function handleTick(request: NavigationTickRequest): void {
  if (!controller) {
    throw new Error("navigation controller is not initialized");
  }

  const startedAt = performance.now();
  const result = controller.tick({
    sequence: request.sequence,
    deltaSeconds: request.deltaSeconds,
    pose: request.pose,
    velocity: request.velocity,
  }) as Omit<NavigationCommandResponse, "type">;
  const workerComputeMs = performance.now() - startedAt;

  postResponse({
    type: "command",
    ...result,
    debug: {
      ...result.debug,
      workerComputeMs,
    },
  });
}

function postResponse(response: NavigationWorkerResponse): void {
  ctx.postMessage(response);
}
