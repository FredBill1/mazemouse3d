import { Buffer } from "node:buffer";
import { mkdir, readFile, writeFile } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";
import prettier from "prettier";
import ts from "typescript";

const ROOT_DIR = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const MAZE_SOURCE_PATH = path.join(ROOT_DIR, "src", "domain", "maze.ts");
const WASM_JS_PATH = path.join(ROOT_DIR, "src", "generated", "maze-gen", "maze_gen.js");
const WASM_PATH = path.join(ROOT_DIR, "src", "generated", "maze-gen", "maze_gen_bg.wasm");
const OUTPUT_PATH = path.join(ROOT_DIR, "src", "domain", "defaultMazeData.ts");

const DEFAULT_CONFIG_KEYS = ["size", "seed", "iterations", "initialTemp", "finalTemp"];

const defaultConfig = await readDefaultMazeConfig();
const { default: initWasm, generate_maze: generateMaze } = await import(
  pathToFileURL(WASM_JS_PATH).href
);
const wasmBytes = await readFile(WASM_PATH);

await initWasm({ module_or_path: wasmBytes });

const rawMaze = generateMaze({
  ...defaultConfig,
  includeScoreHistory: false,
});

validateMaze(rawMaze, defaultConfig);

const walls = Array.from(rawMaze.walls);
const output = await prettier.format(
  renderDefaultMazeModule(rawMaze, Buffer.from(walls).toString("base64")),
  {
    parser: "typescript",
  },
);

await mkdir(path.dirname(OUTPUT_PATH), { recursive: true });
await writeFile(OUTPUT_PATH, output, "utf8");

async function readDefaultMazeConfig() {
  const sourceText = await readFile(MAZE_SOURCE_PATH, "utf8");
  const source = ts.createSourceFile(MAZE_SOURCE_PATH, sourceText, ts.ScriptTarget.Latest, true);
  let config = null;

  visit(source);

  if (!config) {
    throw new Error("DEFAULT_MAZE_CONFIG was not found");
  }

  return config;

  function visit(node) {
    if (ts.isVariableStatement(node)) {
      for (const declaration of node.declarationList.declarations) {
        if (
          ts.isIdentifier(declaration.name) &&
          declaration.name.text === "DEFAULT_MAZE_CONFIG" &&
          declaration.initializer &&
          ts.isObjectLiteralExpression(declaration.initializer)
        ) {
          config = readNumberObject(declaration.initializer);
          return;
        }
      }
    }

    ts.forEachChild(node, visit);
  }
}

function readNumberObject(objectLiteral) {
  const values = {};

  for (const property of objectLiteral.properties) {
    if (!ts.isPropertyAssignment(property) || !ts.isIdentifier(property.name)) {
      continue;
    }

    values[property.name.text] = readNumberExpression(property.initializer);
  }

  for (const key of DEFAULT_CONFIG_KEYS) {
    if (typeof values[key] !== "number" || !Number.isFinite(values[key])) {
      throw new Error(`DEFAULT_MAZE_CONFIG.${key} must be a finite number`);
    }
  }

  return values;
}

function readNumberExpression(expression) {
  if (ts.isNumericLiteral(expression)) {
    return Number(expression.text);
  }

  if (
    ts.isPrefixUnaryExpression(expression) &&
    expression.operator === ts.SyntaxKind.MinusToken &&
    ts.isNumericLiteral(expression.operand)
  ) {
    return -Number(expression.operand.text);
  }

  throw new Error(`unsupported numeric expression: ${expression.getText()}`);
}

function validateMaze(rawMaze, defaultConfig) {
  for (const key of DEFAULT_CONFIG_KEYS) {
    if (rawMaze[key] !== defaultConfig[key]) {
      throw new Error(`generated maze ${key} ${rawMaze[key]} does not match ${defaultConfig[key]}`);
    }
  }

  if (!Array.isArray(rawMaze.goals) || rawMaze.goals.length !== 4) {
    throw new Error("generated maze must contain four goals");
  }

  if (!rawMaze.walls || rawMaze.walls.length !== rawMaze.size * rawMaze.size) {
    throw new Error("generated maze walls length does not match size * size");
  }

  if (!Array.isArray(rawMaze.solution) || rawMaze.solution.length === 0) {
    throw new Error("generated maze must contain a solution");
  }

  if (!Array.isArray(rawMaze.scoreHistory) || rawMaze.scoreHistory.length !== 0) {
    throw new Error("default maze data must not include scoreHistory");
  }
}

function renderDefaultMazeModule(rawMaze, wallsBase64) {
  return `import {
  DEFAULT_MAZE_CONFIG,
  type MazeMetrics,
  type MazeSnapshot,
  type ResolvedMazeConfig,
} from "./maze";

const GENERATED_CONFIG = ${stringify({
    size: rawMaze.size,
    seed: rawMaze.seed,
    iterations: rawMaze.iterations,
    initialTemp: rawMaze.initialTemp,
    finalTemp: rawMaze.finalTemp,
  })} as const satisfies ResolvedMazeConfig;

const START = ${rawMaze.start} as const;
const GOALS = ${stringify(rawMaze.goals)} as const;
const WALLS_BASE64 = "${wallsBase64}" as const;
const SOLUTION = ${stringify(rawMaze.solution)} as const;
const METRICS = ${stringify(rawMaze.metrics)} as const satisfies MazeMetrics;

export function isPrecomputedDefaultMazeConfig(config: ResolvedMazeConfig): boolean {
  return sameConfig(config, GENERATED_CONFIG) && sameConfig(DEFAULT_MAZE_CONFIG, GENERATED_CONFIG);
}

export function createDefaultMazeSnapshot(): MazeSnapshot {
  return {
    ...GENERATED_CONFIG,
    config: GENERATED_CONFIG,
    start: START,
    goals: [GOALS[0], GOALS[1], GOALS[2], GOALS[3]],
    walls: decodeBase64Bytes(WALLS_BASE64),
    solution: [...SOLUTION],
    metrics: { ...METRICS },
    scoreHistory: [],
  };
}

function sameConfig(left: ResolvedMazeConfig, right: ResolvedMazeConfig): boolean {
  return (
    left.size === right.size &&
    left.seed === right.seed &&
    left.iterations === right.iterations &&
    left.initialTemp === right.initialTemp &&
    left.finalTemp === right.finalTemp
  );
}

function decodeBase64Bytes(value: string): Uint8Array {
  const binary = atob(value);
  const bytes = new Uint8Array(binary.length);

  for (let index = 0; index < binary.length; index += 1) {
    bytes[index] = binary.charCodeAt(index);
  }

  return bytes;
}
`;
}

function stringify(value) {
  return JSON.stringify(value, null, 2);
}
