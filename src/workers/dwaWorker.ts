import initWasm, { DwaController } from "../generated/maze-nav/maze_nav.js";
import type { DwaWorkerRequest, DwaWorkerResponse } from "../domain/dwa";

let wasmReady: Promise<void> | null = null;
let controller: DwaController | null = null;
let latestTelemetry: (DwaWorkerRequest & { readonly type: "telemetry" }) | null = null;
let commandTimer: ReturnType<typeof setInterval> | null = null;
let commandInProgress = false;

const ctx = self as unknown as Worker;

ctx.addEventListener("message", (event: MessageEvent<DwaWorkerRequest>) => {
  void handleRequest(event.data);
});

async function handleRequest(request: DwaWorkerRequest): Promise<void> {
  try {
    wasmReady ??= initWasm().then(() => undefined);
    await wasmReady;

    if (request.type === "init") {
      controller?.free();
      controller = new DwaController(request.config);
      latestTelemetry = null;
      restartCommandTimer(request.config.options.controlPeriod ?? 1 / 60);
      post({
        type: "ready",
        requestId: request.requestId,
      });
      return;
    }

    if (!controller) {
      throw new Error("DWA controller has not been initialized");
    }

    latestTelemetry = request;
  } catch (error) {
    post({
      type: "error",
      requestId: request.requestId,
      message: error instanceof Error ? error.message : String(error),
    });
  }
}

function restartCommandTimer(controlPeriodSeconds: number): void {
  if (commandTimer !== null) {
    clearInterval(commandTimer);
  }

  const intervalMs = Math.max(4, controlPeriodSeconds * 1000);
  commandTimer = setInterval(() => computeLatestCommand(), intervalMs);
}

function computeLatestCommand(): void {
  if (!controller || !latestTelemetry || commandInProgress) {
    return;
  }

  commandInProgress = true;

  try {
    post({
      type: "command",
      requestId: latestTelemetry.requestId,
      command: controller.next_command(latestTelemetry.telemetry),
    });
  } catch (error) {
    post({
      type: "error",
      requestId: latestTelemetry.requestId,
      message: error instanceof Error ? error.message : String(error),
    });
  } finally {
    commandInProgress = false;
  }
}

function post(response: DwaWorkerResponse): void {
  ctx.postMessage(response);
}
