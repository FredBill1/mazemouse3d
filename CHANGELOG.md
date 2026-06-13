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

### Changed

- Converted `crates/maze-gen` into a workspace WASM library crate.
- Removed plotters-based PNG drawing from the active maze generation crate.
- Reworked the micromouse into a compact rear-drive layout with a shorter PCB deck, tighter wheel track, raised axles, adjusted collision mass, transverse motors, wheel clearance fixes, and denser electronic detail.
- Aligned the micromouse PCB visual footprint and chassis collision mesh, including wheel cutouts and rear battery/wire clearance fixes.
- Replaced the default random micromouse motion with repeated random target planning and path following.
