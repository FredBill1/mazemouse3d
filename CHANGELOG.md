# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added

- Project skeleton for the micromouse 3D simulator frontend.
- Dockview workbench shell with 3D scene, top view, and metrics panels.
- Babylon.js scene scaffolding with Havok Physics V2 initialization.
- Rust WASM maze generation bridge loaded through a Web Worker.
- npm scripts for dev, preview, build, format, lint, typecheck, and test.
- Vitest and Rust tests for the first vertical slice.
- Shared Babylon scene rendering through multiple registered view canvases.
- Geometric micromouse model with physics-driven wheel constraints and random motor commands.
- Hash-routed micromouse model preview page with top, side, front, and 45 degree views.
- Rust WASM half-grid A\* navigation crate and planned micromouse motor driver foundation.
- Precomputed default maze data and manual generation script for faster initial page startup.
- Red flickering distance sensor beams with wall hit points in the Babylon.js simulation.
- Rust WASM DWA local controller running through a dedicated Web Worker.
- Simulation debug panel with elapsed time, frame rate, DWA frequency, pose, velocity, distance, average speed, worker health, and wall collision metrics.
- Playwright-based 30 second simulation verification script for speed and collision acceptance.

### Changed

- Converted `crates/maze-gen` into a workspace WASM library crate.
- Removed plotters-based PNG drawing from the active maze generation crate.
- Reworked the micromouse into a compact rear-drive layout with a shorter PCB deck, tighter wheel track, raised axles, adjusted collision mass, transverse motors, wheel clearance fixes, and denser electronic detail.
- Aligned the micromouse PCB visual footprint and chassis collision mesh, including wheel cutouts and rear battery/wire clearance fixes.
- Replaced the default random micromouse motion with repeated random target planning and path following.
- Made maze generation score history opt-in to reduce default generation memory and transfer size.
- Replaced pure pursuit path following with DWA control using the known maze map for high-speed final-run navigation.
- Raised the micromouse control and physics limits to support faster DWA runs while keeping worker watchdog recovery.
- Removed the temporary guided-pose simulation bypass so DWA now drives the physics rig only through differential wheel commands.
- Restored half-grid DWA path planning for 45 degree diagonal runs and changed debug DWA Hz to report actual worker responses against the target rate.
- Retuned the high-speed micromouse mass, center of mass, inertia, damping, and wheel motor force defaults for real physics control.
- Moved DWA command production into a continuous worker-side loop and kept the main thread publishing latest telemetry without SharedArrayBuffer.
- Added chassis pitch/yaw/roll telemetry from the real quaternion and debug reset count reporting.
- Added a small physics force/torque assist derived from differential wheel commands to reduce four-wheel scrub without writing pose or velocity.
