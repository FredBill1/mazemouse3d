import initWasm, { generate_maze as generateMaze } from "../generated/maze-gen/maze_gen.js";
import type { MazeWorkerRequest, MazeWorkerResponse, RawMazeGenOutput } from "../domain/maze";
import { normalizeMazeOutput } from "../domain/mazeAdapter";

let wasmReady: Promise<void> | null = null;

const ctx = self as unknown as Worker;

ctx.addEventListener("message", (event: MessageEvent<MazeWorkerRequest>) => {
  void handleRequest(event.data);
});

async function handleRequest(request: MazeWorkerRequest): Promise<void> {
  if (request.type !== "generateMaze") {
    return;
  }

  try {
    wasmReady ??= initWasm().then(() => undefined);
    await wasmReady;

    const raw = generateMaze(request.config ?? {}) as RawMazeGenOutput;
    const snapshot = normalizeMazeOutput(raw);
    const response: MazeWorkerResponse = {
      type: "mazeGenerated",
      requestId: request.requestId,
      snapshot,
    };

    ctx.postMessage(response, [snapshot.walls.buffer]);
  } catch (error) {
    const response: MazeWorkerResponse = {
      type: "error",
      requestId: request.requestId,
      message: error instanceof Error ? error.message : String(error),
    };

    ctx.postMessage(response);
  }
}
