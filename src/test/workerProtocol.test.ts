import { describe, expect, it } from "vitest";
import type { MazeWorkerRequest, MazeWorkerResponse } from "../domain/maze";

describe("maze worker protocol", () => {
  it("uses request ids for generate requests", () => {
    const request: MazeWorkerRequest = {
      type: "generateMaze",
      requestId: 1,
      config: { size: 16, seed: 42 },
    };

    expect(request.type).toBe("generateMaze");
    expect(request.requestId).toBe(1);
  });

  it("can represent worker errors", () => {
    const response: MazeWorkerResponse = {
      type: "error",
      requestId: 1,
      message: "invalid maze config",
    };

    expect(response).toMatchObject({
      type: "error",
      requestId: 1,
    });
  });
});
