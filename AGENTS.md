# mazemouse3d

本项目是一个纯前端应用，仿真micromouse比赛，展示迷宫并让机器鼠在迷宫里自主导航

## 技术栈

- npm + Vite + TypeScript
- dockview-core: 面板布局
- Babylon.js + Physics V2(havok): 多视角场景渲染和机器鼠物理引擎
- Rust WASM: 迷宫生成和机器鼠控制算法
- 基于Web Worker的多线程，为了兼容性不使用SharedArrayBuffer

## Git 提交前

- 执行`./package.json`中的`npm run format`和`npm run check`
- 更新`./CHANGELOG.md`
