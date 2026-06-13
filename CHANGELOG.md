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

### Changed

- Converted `crates/maze-gen` into a workspace WASM library crate.
- Removed plotters-based PNG drawing from the active maze generation crate.
