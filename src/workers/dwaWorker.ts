import initWasm, { DwaController } from "../generated/maze-nav/maze_nav.js";
import type { DwaWorkerRequest, DwaWorkerResponse } from "../domain/dwa";

let wasmReady: Promise<void> | null = null;
let controller: DwaController | null = null;

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
      post({
        type: "ready",
        requestId: request.requestId,
      });
      return;
    }

    if (!controller) {
      throw new Error("DWA controller has not been initialized");
    }

    post({
      type: "command",
      requestId: request.requestId,
      command: controller.next_command(request.telemetry),
    });
  } catch (error) {
    post({
      type: "error",
      requestId: request.requestId,
      message: error instanceof Error ? error.message : String(error),
    });
  }
}

function post(response: DwaWorkerResponse): void {
  ctx.postMessage(response);
}
