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
- Rust WASM half-grid A\* navigation crate and planned micromouse motor driver with pure pursuit.
- Precomputed default maze data and manual generation script for faster initial page startup.
- Red flickering distance sensor beams with wall hit points in the Babylon.js simulation.
- Dockview debug panel with live elapsed time, FPS, pose, velocity, travel distance, average speed, and wall-hit count.
- Rust WASM DWB-style navigation controller with velocity smoothing in a dedicated Web Worker.
- Manual `npm run verify:controller` browser verification for 60-second speed and wall-collision checks.

### Changed

- Converted `crates/maze-gen` into a workspace WASM library crate.
- Removed plotters-based PNG drawing from the active maze generation crate.
- Reworked the micromouse into a compact rear-drive layout with a shorter PCB deck, tighter wheel track, raised axles, adjusted collision mass, transverse motors, wheel clearance fixes, and denser electronic detail.
- Aligned the micromouse PCB visual footprint and chassis collision mesh, including wheel cutouts and rear battery/wire clearance fixes.
- Replaced the default random micromouse motion with repeated random target planning and path following.
- Made maze generation score history opt-in to reduce default generation memory and transfer size.
- Replaced pure-pursuit path tracking with velocity-aware DWB trajectory sampling, smoothed high-rate wheel commands, green path-point visualization, and controller frequency metrics.
- Tuned the DWB controller, wall thickness, and wheel motor force for the 60-second browser verification target.

### Fixed

- Count micromouse wheel contacts in the Debug panel wall-hit total.
- Prevented stale no-valid local plans from stopping the DWB worker by retrying with collision-safe relaxed clearance scoring.
- Bounded DWB path shortcut collision checks to avoid long worker stalls and stale high-speed commands during replanning.
