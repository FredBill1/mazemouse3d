import { expect, test } from "@playwright/test";

interface SimulationDebugSnapshot {
  readonly elapsedSeconds: number;
  readonly fps: number;
  readonly dwaHz: number;
  readonly workerStatus: string;
  readonly averageSpeed: number;
  readonly wallCollisionCount: number;
}

declare global {
  interface Window {
    __MAZEMOUSE3D_DEBUG__?: SimulationDebugSnapshot;
  }
}

test("default DWA run stays fast and collision-free for 30 seconds", async ({ page }) => {
  await page.goto("/");

  const canvas = page.locator("canvas.scene-canvas").first();
  await expect(canvas).toBeVisible();
  await page.waitForFunction(() => (window.__MAZEMOUSE3D_DEBUG__?.elapsedSeconds ?? 0) > 0);
  await page.waitForFunction(() => (window.__MAZEMOUSE3D_DEBUG__?.elapsedSeconds ?? 0) >= 30, {
    timeout: 45_000,
  });

  const debug = await page.evaluate(() => window.__MAZEMOUSE3D_DEBUG__);

  expect(debug).toBeTruthy();
  expect(debug?.fps ?? 0).toBeGreaterThan(10);
  expect(debug?.dwaHz ?? 0).toBeGreaterThan(30);
  expect(debug?.workerStatus).toBe("ready");
  expect(debug?.averageSpeed ?? 0).toBeGreaterThanOrEqual(5);
  expect(debug?.wallCollisionCount ?? 1).toBe(0);

  const screenshot = await canvas.screenshot();
  expect(screenshot.byteLength).toBeGreaterThan(1024);
});
