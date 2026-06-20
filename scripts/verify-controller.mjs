import { chromium } from "@playwright/test";
import { createServer } from "vite";

const RUN_SECONDS = 60;
const MIN_AVERAGE_SPEED = 1.2;
const TIMEOUT_MS = 95_000;

const server = await createServer({
  server: {
    host: "127.0.0.1",
    port: 5173,
    strictPort: false,
  },
});

await server.listen();

const urls = server.resolvedUrls?.local ?? [];
const url = urls.find((candidate) => candidate.startsWith("http://127.0.0.1")) ?? urls[0];

if (!url) {
  await server.close();
  throw new Error("Vite did not expose a local URL");
}

const browser = await chromium.launch();
const page = await browser.newPage({
  viewport: {
    width: 1440,
    height: 900,
  },
});

try {
  await page.addInitScript(() => {
    globalThis.__MAZEMOUSE3D_CONTROLLER_SAMPLES__ = [];
    globalThis.setInterval(() => {
      const snapshot = globalThis.__MAZEMOUSE3D_DEBUG_SNAPSHOT__;

      if (!snapshot?.hasMicromouse) {
        return;
      }

      globalThis.__MAZEMOUSE3D_CONTROLLER_SAMPLES__.push({
        elapsedSeconds: snapshot.elapsedSeconds,
        averageSpeed: snapshot.averageSpeed,
        wallCollisions: snapshot.wallCollisions,
        controller: snapshot.controller,
      });

      if (globalThis.__MAZEMOUSE3D_CONTROLLER_SAMPLES__.length > 90) {
        globalThis.__MAZEMOUSE3D_CONTROLLER_SAMPLES__.shift();
      }
    }, 1000);
  });
  await page.goto(`${url}?debugPlanner=1`, { waitUntil: "networkidle" });
  await page.waitForFunction(
    (runSeconds) => {
      const snapshot = globalThis.__MAZEMOUSE3D_DEBUG_SNAPSHOT__;

      return Boolean(snapshot?.hasMicromouse && snapshot.elapsedSeconds >= runSeconds);
    },
    RUN_SECONDS,
    { timeout: TIMEOUT_MS },
  );

  const { snapshot, samples, navigationLog } = await page.evaluate(() => ({
    snapshot: globalThis.__MAZEMOUSE3D_DEBUG_SNAPSHOT__,
    samples: globalThis.__MAZEMOUSE3D_CONTROLLER_SAMPLES__ ?? [],
    navigationLog: globalThis.__MAZEMOUSE3D_NAVIGATION_DEBUG__ ?? [],
  }));

  if (!snapshot) {
    throw new Error("Debug snapshot was not published");
  }

  const failures = [];

  if (snapshot.averageSpeed < MIN_AVERAGE_SPEED) {
    failures.push(
      `average speed ${snapshot.averageSpeed.toFixed(3)} < ${MIN_AVERAGE_SPEED.toFixed(1)}`,
    );
  }

  if (snapshot.wallCollisions !== 0) {
    failures.push(`wall collisions ${snapshot.wallCollisions} !== 0`);
  }

  if (snapshot.controller.dwbHz <= 0) {
    failures.push(`DWB frequency ${snapshot.controller.dwbHz.toFixed(2)} Hz is not running`);
  }

  if (snapshot.controller.smootherHz <= snapshot.controller.dwbHz) {
    failures.push(
      `smoother frequency ${snapshot.controller.smootherHz.toFixed(2)} Hz is not above DWB ${snapshot.controller.dwbHz.toFixed(2)} Hz`,
    );
  }

  const controllerFailures = samples.filter(
    (sample) =>
      sample.elapsedSeconds > 2 &&
      (sample.controller.status.startsWith("planner-error") ||
        sample.controller.currentPoseCollides ||
        sample.controller.validTrajectories === 0),
  );

  if (controllerFailures.length > 0) {
    failures.push(
      `${controllerFailures.length} sampled controller diagnostics reported no valid DWB plan`,
    );
  }

  if (failures.length > 0) {
    const rejectionTotals = sumRejects(
      samples.map((sample) => sample.controller.rejectedTrajectories),
    );
    const recentSamples = samples.slice(-8).map((sample) => ({
      t: Number(sample.elapsedSeconds.toFixed(2)),
      speed: Number(sample.averageSpeed.toFixed(3)),
      wallCollisions: sample.wallCollisions,
      status: sample.controller.status,
      valid: sample.controller.validTrajectories,
      sampled: sample.controller.sampledTrajectories,
      targetV: Number(sample.controller.targetLinearSpeed.toFixed(3)),
      targetW: Number(sample.controller.targetAngularSpeed.toFixed(3)),
      clearance: Number(sample.controller.currentClearance.toFixed(3)),
      pathError: Number(sample.controller.pathTrackingError.toFixed(3)),
      collides: sample.controller.currentPoseCollides,
      rejects: sample.controller.rejectedTrajectories,
      best: sample.controller.best
        ? {
            v: Number(sample.controller.best.linearSpeed.toFixed(3)),
            w: Number(sample.controller.best.angularSpeed.toFixed(3)),
            score: Number(sample.controller.best.score.total.toFixed(3)),
            progress: Number(sample.controller.best.score.progress.toFixed(3)),
            clearance: Number(sample.controller.best.score.minClearance.toFixed(3)),
          }
        : null,
    }));
    const recentLog = navigationLog.slice(-8);

    throw new Error(
      `${failures.join("; ")}\nreject totals: ${JSON.stringify(rejectionTotals)}\nrecent samples: ${JSON.stringify(recentSamples, null, 2)}\nrecent navigation log: ${JSON.stringify(recentLog, null, 2)}`,
    );
  }

  console.log(
    `Controller verification passed: ${snapshot.averageSpeed.toFixed(3)} u/s, ${snapshot.wallCollisions} wall hits, DWB ${snapshot.controller.dwbHz.toFixed(1)} Hz, smoother ${snapshot.controller.smootherHz.toFixed(1)} Hz`,
  );
} finally {
  await browser.close();
  await server.close();
}

function sumRejects(rejections) {
  return rejections.reduce(
    (totals, counts) => ({
      currentPoseCollision: totals.currentPoseCollision + counts.currentPoseCollision,
      rolloutCollision: totals.rolloutCollision + counts.rolloutCollision,
      brakingCollision: totals.brakingCollision + counts.brakingCollision,
      wheelSpeed: totals.wheelSpeed + counts.wheelSpeed,
      trackability: totals.trackability + counts.trackability,
      lowClearance: totals.lowClearance + counts.lowClearance,
      noProgress: totals.noProgress + counts.noProgress,
      noPathProjection: totals.noPathProjection + counts.noPathProjection,
      nonFiniteScore: totals.nonFiniteScore + counts.nonFiniteScore,
    }),
    {
      currentPoseCollision: 0,
      rolloutCollision: 0,
      brakingCollision: 0,
      wheelSpeed: 0,
      trackability: 0,
      lowClearance: 0,
      noProgress: 0,
      noPathProjection: 0,
      nonFiniteScore: 0,
    },
  );
}
