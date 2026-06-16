import { Ray } from "@babylonjs/core/Culling/ray.js";
import { StandardMaterial } from "@babylonjs/core/Materials/standardMaterial.js";
import { Color3 } from "@babylonjs/core/Maths/math.color.js";
import { Matrix, Quaternion, Vector3 } from "@babylonjs/core/Maths/math.vector.js";
import type { IObserver } from "@babylonjs/core/Misc/observable.js";
import type { AbstractMesh } from "@babylonjs/core/Meshes/abstractMesh.js";
import { Mesh } from "@babylonjs/core/Meshes/mesh.js";
import { VertexData } from "@babylonjs/core/Meshes/mesh.vertexData.js";
import { MeshBuilder } from "@babylonjs/core/Meshes/meshBuilder.js";
import { TransformNode } from "@babylonjs/core/Meshes/transformNode.js";
import { Scene } from "@babylonjs/core/scene.js";
import { PhysicsAggregate } from "@babylonjs/core/Physics/v2/physicsAggregate.js";
import { PhysicsBody } from "@babylonjs/core/Physics/v2/physicsBody.js";
import { Physics6DoFConstraint } from "@babylonjs/core/Physics/v2/physicsConstraint.js";
import {
  PhysicsConstraintAxis,
  PhysicsConstraintAxisLimitMode,
  PhysicsConstraintMotorType,
  PhysicsMotionType,
  PhysicsShapeType,
} from "@babylonjs/core/Physics/v2/IPhysicsEnginePlugin.js";
import { PhysicsMaterialCombineMode } from "@babylonjs/core/Physics/v2/physicsMaterial.js";
import { PhysicsShapeCylinder } from "@babylonjs/core/Physics/v2/physicsShape.js";
import type { MazeSnapshot } from "../domain/maze";
import { cellCenter } from "./mazeGeometry";
import {
  MICROMOUSE_BLUEPRINT,
  initialMouseYaw,
  type SensorBlueprint,
  type WheelBlueprint,
} from "./micromouseModel";
import { commandMagnitude, type MotorCommand, type RobotGroundTruthPose } from "./motorDriver";

interface WheelAssembly {
  readonly layout: WheelBlueprint;
  readonly mesh: Mesh;
  readonly body: PhysicsBody;
  readonly shape: PhysicsShapeCylinder;
  readonly constraint: Physics6DoFConstraint;
}

interface SensorBeamVisual {
  readonly beam: Mesh;
  readonly hit: Mesh;
  readonly material: StandardMaterial;
  readonly hitMaterial: StandardMaterial;
  readonly localOrigin: Vector3;
  readonly localForwardPoint: Vector3;
  readonly flickerPhase: number;
  readonly flickerFrequency: number;
  readonly jitterFrequency: number;
}

export interface MicromouseMaterials {
  readonly board: StandardMaterial;
  readonly boardEdge: StandardMaterial;
  readonly boardTrace: StandardMaterial;
  readonly silkscreen: StandardMaterial;
  readonly chip: StandardMaterial;
  readonly metal: StandardMaterial;
  readonly tire: StandardMaterial;
  readonly hub: StandardMaterial;
  readonly wheelSide: StandardMaterial;
  readonly battery: StandardMaterial;
  readonly batteryLabel: StandardMaterial;
  readonly sensor: StandardMaterial;
  readonly emitter: StandardMaterial;
  readonly gear: StandardMaterial;
  readonly brass: StandardMaterial;
  readonly plasticWhite: StandardMaterial;
  readonly plasticBlack: StandardMaterial;
  readonly wireRed: StandardMaterial;
  readonly wireBlack: StandardMaterial;
  readonly ribbon: StandardMaterial;
  readonly translucent: StandardMaterial;
}

export interface MicromouseDisplayModelOptions {
  readonly parent?: TransformNode | Mesh;
  readonly includeWheels?: boolean;
  readonly namePrefix?: string;
}

const ZERO_COMMAND: MotorCommand = {
  leftRadPerSec: 0,
  rightRadPerSec: 0,
};

const WHEEL_ALIGNMENT = Quaternion.RotationAxis(Vector3.Forward(), -Math.PI / 2);
const LONGITUDINAL_ALIGNMENT = Quaternion.RotationAxis(Vector3.Right(), Math.PI / 2);
const BOARD_ARC_SEGMENTS = 18;
const SENSOR_BEAM_MAX_DISTANCE = 1.4;
const SENSOR_BEAM_RADIUS = 0.006;
const SENSOR_BEAM_LENS_OFFSET = 0.032;
const SENSOR_BEAM_HIT_OFFSET = 0.006;
const SENSOR_BEAM_HIT_DIAMETER = 0.038;
const SENSOR_BEAM_MISS_ALPHA_SCALE = 0.34;

export class MicromouseRig {
  readonly #scene: Scene;
  readonly #materials: MicromouseMaterials;
  readonly #chassis: Mesh;
  readonly #chassisAggregate: PhysicsAggregate;
  readonly #wheels: WheelAssembly[] = [];
  readonly #sensorBeams: SensorBeamVisual[] = [];
  #sensorBeamObserver: IObserver | null = null;
  #sensorBeamSeconds = 0;
  #slowSeconds = 0;
  #disposed = false;

  constructor(scene: Scene, materials: MicromouseMaterials, maze: MazeSnapshot) {
    this.#scene = scene;
    this.#materials = materials;

    const pose = initialPose(maze);
    this.#chassis = this.#createChassis(pose.position, pose.rotation);
    this.#chassisAggregate = new PhysicsAggregate(
      this.#chassis,
      PhysicsShapeType.MESH,
      {
        mass: MICROMOUSE_BLUEPRINT.chassis.mass,
        friction: 0.62,
        restitution: 0.02,
      },
      this.#scene,
    );
    this.#chassisAggregate.shape.material = {
      friction: 0.62,
      staticFriction: 0.72,
      restitution: 0.02,
      frictionCombine: PhysicsMaterialCombineMode.ARITHMETIC_MEAN,
    };
    const currentInertia = this.#chassisAggregate.body.getMassProperties().inertia;
    if (currentInertia === undefined) {
      throw Error("Expected getMassProperties().inertia to be defined.");
    }
    this.#chassisAggregate.body.setMassProperties({
      mass: MICROMOUSE_BLUEPRINT.chassis.mass,
      centerOfMass: vectorFromBlueprint(MICROMOUSE_BLUEPRINT.chassis.centerOfMassOffset),
      inertia: new Vector3(currentInertia.x * 1000, currentInertia.y, currentInertia.z), // prevent wheelie
    });
    this.#chassisAggregate.body.setLinearDamping(0.14);
    this.#chassisAggregate.body.setAngularDamping(0.22);

    createMicromouseDisplayModel(this.#scene, this.#materials, {
      parent: this.#chassis,
      includeWheels: false,
      namePrefix: "micromouse",
    });
    this.#addSensorBeams();
    this.#addWheels(pose.position, pose.rotation);
    this.setMotorCommand(ZERO_COMMAND);
  }

  setMotorCommand(command: MotorCommand): void {
    if (this.#disposed) {
      return;
    }

    for (const wheel of this.#wheels) {
      const target = wheel.layout.side === "left" ? command.leftRadPerSec : command.rightRadPerSec;
      wheel.constraint.setAxisMotorTarget(PhysicsConstraintAxis.ANGULAR_X, target);
    }
  }

  getGroundTruthPose(): RobotGroundTruthPose {
    return micromouseGroundTruthPoseFromChassis(
      this.#chassis.position,
      this.#chassis.rotationQuaternion ?? Quaternion.Identity(),
    );
  }

  shouldUseRecoveryCommand(command: MotorCommand, deltaSeconds: number): boolean {
    if (commandMagnitude(command) < 5) {
      this.#slowSeconds = 0;
      return false;
    }

    const speed = this.#chassisAggregate.body.getLinearVelocity().length();

    if (speed < 0.035) {
      this.#slowSeconds += deltaSeconds;
    } else {
      this.#slowSeconds = 0;
    }

    return this.#slowSeconds > 1.1;
  }

  shouldReset(maze: MazeSnapshot): boolean {
    const position = this.#chassis.position;

    if (
      position.y < -0.25 ||
      position.y > 1.2 ||
      position.x < -0.75 ||
      position.x > maze.size + 0.75 ||
      position.z < -0.75 ||
      position.z > maze.size + 0.75
    ) {
      return true;
    }

    const rotation = this.#chassis.rotationQuaternion;

    if (!rotation) {
      return false;
    }

    const rotationMatrix = Matrix.FromQuaternionToRef(rotation, Matrix.Identity());
    const up = Vector3.TransformNormal(Vector3.Up(), rotationMatrix);

    return up.y < 0.28;
  }

  dispose(): void {
    if (this.#disposed) {
      return;
    }

    this.#disposed = true;

    for (const wheel of this.#wheels) {
      wheel.constraint.dispose();
    }

    for (const wheel of this.#wheels) {
      wheel.body.dispose();
      wheel.shape.dispose();
      wheel.mesh.dispose(false, true);
    }

    this.#wheels.length = 0;
    this.#sensorBeamObserver?.remove();
    this.#sensorBeamObserver = null;

    for (const beam of this.#sensorBeams) {
      beam.beam.dispose(false, true);
      beam.hit.dispose(false, true);
    }

    this.#sensorBeams.length = 0;
    this.#chassisAggregate.dispose();
    this.#chassis.dispose(false, true);
  }

  #createChassis(position: Vector3, rotation: Quaternion): Mesh {
    const mesh = createPcbColliderMesh(this.#scene, "micromouse-chassis-collider");
    mesh.position.copyFrom(position);
    mesh.rotationQuaternion = rotation.clone();
    mesh.isVisible = false;
    mesh.visibility = 0;

    return mesh;
  }

  #addWheels(origin: Vector3, rotation: Quaternion): void {
    for (const layout of MICROMOUSE_BLUEPRINT.wheels) {
      const mesh = this.#createWheelMesh(layout, origin, rotation);
      const body = new PhysicsBody(mesh, PhysicsMotionType.DYNAMIC, false, this.#scene);
      const shape = new PhysicsShapeCylinder(
        new Vector3(-MICROMOUSE_BLUEPRINT.wheel.width / 2, 0, 0),
        new Vector3(MICROMOUSE_BLUEPRINT.wheel.width / 2, 0, 0),
        MICROMOUSE_BLUEPRINT.wheel.radius,
        this.#scene,
      );
      shape.material = {
        friction: 1.65,
        staticFriction: 2.0,
        restitution: 0.01,
        frictionCombine: PhysicsMaterialCombineMode.MAXIMUM,
      };
      body.shape = shape;
      body.setMassProperties({ mass: MICROMOUSE_BLUEPRINT.wheel.mass });
      body.setLinearDamping(0.04);
      body.setAngularDamping(0.02);

      const constraint = this.#createWheelConstraint(layout);
      this.#chassisAggregate.body.addConstraint(body, constraint);
      configureWheelConstraint(constraint);

      this.#wheels.push({
        layout,
        mesh,
        body,
        shape,
        constraint,
      });
    }
  }

  #addSensorBeams(): void {
    for (const layout of MICROMOUSE_BLUEPRINT.sensors) {
      const materialValue = createSensorBeamMaterial(
        this.#scene,
        `micromouse-ir-beam-${layout.id}-material`,
        0.42,
      );
      const hitMaterial = createSensorBeamMaterial(
        this.#scene,
        `micromouse-ir-hit-${layout.id}-material`,
        0.72,
      );
      const beam = MeshBuilder.CreateTube(
        `micromouse-ir-beam-${layout.id}`,
        {
          path: [Vector3.Zero(), Vector3.Forward().scale(SENSOR_BEAM_MAX_DISTANCE)],
          radius: SENSOR_BEAM_RADIUS,
          tessellation: 8,
          updatable: true,
        },
        this.#scene,
      );
      beam.material = materialValue;
      beam.isPickable = false;
      beam.alwaysSelectAsActiveMesh = true;

      const hit = MeshBuilder.CreateSphere(
        `micromouse-ir-hit-${layout.id}`,
        { diameter: SENSOR_BEAM_HIT_DIAMETER, segments: 12 },
        this.#scene,
      );
      hit.material = hitMaterial;
      hit.isPickable = false;
      hit.isVisible = false;
      hit.alwaysSelectAsActiveMesh = true;

      this.#sensorBeams.push({
        beam,
        hit,
        material: materialValue,
        hitMaterial,
        localOrigin: sensorBeamLocalOrigin(layout),
        localForwardPoint: sensorBeamLocalForwardPoint(layout),
        flickerPhase: Math.random() * Math.PI * 2,
        flickerFrequency: 8.5 + Math.random() * 7,
        jitterFrequency: 19 + Math.random() * 17,
      });
    }

    this.#sensorBeamObserver = this.#scene.onBeforeRenderObservable.add(() =>
      this.#updateSensorBeams(),
    );
  }

  #updateSensorBeams(): void {
    if (this.#disposed) {
      return;
    }

    const deltaSeconds = Math.min(this.#scene.getEngine().getDeltaTime(), 80) / 1000;
    this.#sensorBeamSeconds += deltaSeconds;
    const chassisWorld = this.#chassis.computeWorldMatrix(true);

    for (const visual of this.#sensorBeams) {
      const origin = Vector3.TransformCoordinates(visual.localOrigin, chassisWorld);
      const forwardPoint = Vector3.TransformCoordinates(visual.localForwardPoint, chassisWorld);
      const direction = forwardPoint.subtract(origin).normalize();
      const ray = new Ray(origin, direction, SENSOR_BEAM_MAX_DISTANCE);
      const pick = this.#scene.pickWithRay(ray, isSensorBeamTarget);
      const hitPoint = pick?.hit === true ? pick.pickedPoint : null;
      const distance =
        hitPoint && pick
          ? Math.min(pick.distance, SENSOR_BEAM_MAX_DISTANCE)
          : SENSOR_BEAM_MAX_DISTANCE;
      const end = origin.add(direction.scale(distance));
      const flicker = sensorBeamFlicker(this.#sensorBeamSeconds, visual);

      MeshBuilder.CreateTube(
        visual.beam.name,
        {
          path: [origin, end],
          radius: SENSOR_BEAM_RADIUS,
          tessellation: 8,
          instance: visual.beam,
        },
        this.#scene,
      );

      visual.material.alpha = hitPoint ? flicker : flicker * SENSOR_BEAM_MISS_ALPHA_SCALE;

      if (hitPoint) {
        visual.hit.isVisible = true;
        visual.hit.position.copyFrom(hitPoint);
        visual.hit.position.subtractInPlace(direction.scale(SENSOR_BEAM_HIT_OFFSET));
        const hitScale = 0.72 + flicker;
        visual.hit.scaling.set(hitScale, hitScale, hitScale);
        visual.hitMaterial.alpha = Math.min(0.92, flicker + 0.24);
      } else {
        visual.hit.isVisible = false;
      }
    }
  }

  #createWheelMesh(layout: WheelBlueprint, origin: Vector3, rotation: Quaternion): Mesh {
    const mesh = createWheelVisualMesh(
      this.#scene,
      this.#materials,
      layout,
      `micromouse-wheel-${layout.id}`,
    );
    mesh.position.copyFrom(
      localToWorld(new Vector3(layout.localX, wheelAxleLocalY(), layout.localZ), origin, rotation),
    );
    mesh.rotationQuaternion = rotation.clone();

    return mesh;
  }

  #createWheelConstraint(layout: WheelBlueprint): Physics6DoFConstraint {
    const constraint = new Physics6DoFConstraint(
      {
        pivotA: new Vector3(layout.localX, wheelAxleLocalY(), layout.localZ),
        pivotB: Vector3.Zero(),
        axisA: Vector3.Right(),
        axisB: Vector3.Right(),
        perpAxisA: Vector3.Up(),
        perpAxisB: Vector3.Up(),
        collision: false,
      },
      [],
      this.#scene,
    );

    return constraint;
  }
}

export function createMicromouseDisplayModel(
  scene: Scene,
  materials: MicromouseMaterials,
  options: MicromouseDisplayModelOptions = {},
): TransformNode {
  const prefix = options.namePrefix ?? "micromouse-display";
  const root = new TransformNode(`${prefix}-root`, scene);

  if (options.parent) {
    root.parent = options.parent;
  } else {
    root.position.y = MICROMOUSE_BLUEPRINT.chassis.centerY;
  }

  addPcb(scene, materials, root, prefix);
  addRearDriveTrain(scene, materials, root, prefix);
  addBoardElectronics(scene, materials, root, prefix);
  addSensors(scene, materials, root, prefix);
  addCenterOfMassMarker(scene, materials, root, prefix);

  if (options.includeWheels ?? true) {
    for (const layout of MICROMOUSE_BLUEPRINT.wheels) {
      createWheelVisualMesh(scene, materials, layout, `${prefix}-wheel-${layout.id}`, root);
    }
  }

  return root;
}

export function createMicromouseMaterials(scene: Scene): MicromouseMaterials {
  return {
    board: material(scene, "mouse-board-material", new Color3(0.035, 0.42, 0.18), 0.18),
    boardEdge: material(scene, "mouse-board-edge-material", new Color3(0.08, 0.56, 0.26), 0.16),
    boardTrace: material(scene, "mouse-board-trace-material", new Color3(0.86, 0.74, 0.42), 0.35),
    silkscreen: material(scene, "mouse-silkscreen-material", new Color3(0.86, 0.92, 0.88), 0.12),
    chip: material(scene, "mouse-chip-material", new Color3(0.025, 0.028, 0.032), 0.25),
    metal: material(scene, "mouse-metal-material", new Color3(0.75, 0.78, 0.8), 0.75),
    tire: material(scene, "mouse-tire-material", new Color3(0.012, 0.013, 0.014), 0.08),
    hub: material(scene, "mouse-hub-material", new Color3(0.08, 0.065, 0.048), 0.42),
    wheelSide: material(scene, "mouse-wheel-side-material", new Color3(0.82, 0.82, 0.74), 0.28),
    battery: material(scene, "mouse-battery-material", new Color3(0.9, 0.88, 0.78), 0.18),
    batteryLabel: material(
      scene,
      "mouse-battery-label-material",
      new Color3(0.78, 0.84, 0.88),
      0.12,
    ),
    sensor: material(scene, "mouse-sensor-material", new Color3(0.026, 0.031, 0.034), 0.2),
    emitter: material(scene, "mouse-emitter-material", new Color3(0.95, 0.05, 0.035), 0.3),
    gear: material(scene, "mouse-gear-material", new Color3(0.9, 0.9, 0.82), 0.22),
    brass: material(scene, "mouse-brass-material", new Color3(0.88, 0.62, 0.22), 0.45),
    plasticWhite: material(
      scene,
      "mouse-white-plastic-material",
      new Color3(0.86, 0.84, 0.76),
      0.16,
    ),
    plasticBlack: material(
      scene,
      "mouse-black-plastic-material",
      new Color3(0.02, 0.022, 0.025),
      0.18,
    ),
    wireRed: material(scene, "mouse-red-wire-material", new Color3(0.86, 0.04, 0.025), 0.12),
    wireBlack: material(scene, "mouse-black-wire-material", new Color3(0.006, 0.006, 0.007), 0.08),
    ribbon: material(scene, "mouse-ribbon-material", new Color3(0.78, 0.74, 0.66), 0.08),
    translucent: material(
      scene,
      "mouse-translucent-material",
      new Color3(0.66, 0.78, 0.82),
      0.18,
      0.58,
    ),
  };
}

function addPcb(
  scene: Scene,
  materials: MicromouseMaterials,
  parent: TransformNode,
  prefix: string,
): void {
  const board = createPcbMesh(scene, `${prefix}-pcb`);
  board.position.y = relativeY(MICROMOUSE_BLUEPRINT.pcb.centerY);
  board.material = materials.board;
  board.parent = parent;

  const edge = MeshBuilder.CreateBox(
    `${prefix}-rear-pcb-edge`,
    { width: MICROMOUSE_BLUEPRINT.pcb.width, height: 0.02, depth: 0.018 },
    scene,
  );
  edge.position.set(0, relativeY(pcbTopY() + 0.006), MICROMOUSE_BLUEPRINT.pcb.rearZ + 0.01);
  edge.material = materials.boardEdge;
  edge.parent = parent;

  for (const x of [0.055, 0.105]) {
    addBox(
      scene,
      `${prefix}-front-silkscreen`,
      materials.silkscreen,
      parent,
      { width: 0.034, height: 0.002, depth: 0.062 },
      new Vector3(x, relativeY(pcbTopY() + 0.002), 0.09),
    );
  }

  for (const z of [0.14, 0.045, -0.045, -0.13]) {
    addBox(
      scene,
      `${prefix}-center-trace`,
      materials.boardTrace,
      parent,
      { width: 0.006, height: 0.003, depth: 0.095 },
      new Vector3(0, relativeY(pcbTopY() + 0.003), z),
    );
  }

  for (const x of [-0.165, 0.165]) {
    addBox(
      scene,
      `${prefix}-side-trace`,
      materials.boardTrace,
      parent,
      { width: 0.006, height: 0.003, depth: 0.18 },
      new Vector3(x, relativeY(pcbTopY() + 0.003), 0.035),
    );
  }
}

function addRearDriveTrain(
  scene: Scene,
  materials: MicromouseMaterials,
  parent: TransformNode,
  prefix: string,
): void {
  const axleZValues = [...new Set(MICROMOUSE_BLUEPRINT.wheels.map((wheel) => wheel.localZ))];

  for (const axleZ of axleZValues) {
    createCylinderAlongX(
      scene,
      `${prefix}-axle-${axleZ}`,
      materials.metal,
      parent,
      0.018,
      0.39,
      new Vector3(0, wheelAxleLocalY(), axleZ),
      16,
    );

    for (const side of [-1, 1]) {
      addBox(
        scene,
        `${prefix}-axle-upright`,
        materials.metal,
        parent,
        { width: 0.025, height: 0.055, depth: 0.03 },
        new Vector3(side * 0.16, relativeY(0.067), axleZ),
      );
    }
  }

  for (const side of [-1, 1]) {
    addBox(
      scene,
      `${prefix}-side-rail`,
      materials.metal,
      parent,
      { width: 0.022, height: 0.022, depth: 0.23 },
      new Vector3(side * 0.165, relativeY(0.084), -0.162),
    );
  }

  for (const [index, motorZ] of [
    MICROMOUSE_BLUEPRINT.electronics.motorLocalZ + 0.055,
    MICROMOUSE_BLUEPRINT.electronics.motorLocalZ - 0.055,
  ].entries()) {
    createCylinderAlongX(
      scene,
      `${prefix}-coreless-motor-${index}`,
      materials.plasticBlack,
      parent,
      0.078,
      0.23,
      new Vector3(0, relativeY(0.132), motorZ),
      28,
    );

    for (const x of [-0.082, 0.082]) {
      createCylinderAlongX(
        scene,
        `${prefix}-motor-band-${index}`,
        materials.metal,
        parent,
        0.082,
        0.012,
        new Vector3(x, relativeY(0.132), motorZ),
        24,
      );
    }
  }

  for (const motorZ of [
    MICROMOUSE_BLUEPRINT.electronics.motorLocalZ + 0.055,
    MICROMOUSE_BLUEPRINT.electronics.motorLocalZ - 0.055,
  ]) {
    addBox(
      scene,
      `${prefix}-motor-cradle`,
      materials.plasticWhite,
      parent,
      { width: 0.25, height: 0.014, depth: 0.03 },
      new Vector3(0, relativeY(0.086), motorZ),
    );
  }

  for (const layout of MICROMOUSE_BLUEPRINT.wheels) {
    const side = layout.side === "left" ? -1 : 1;
    createCylinderAlongX(
      scene,
      `${prefix}-drive-gear-${layout.id}`,
      materials.gear,
      parent,
      0.128,
      0.018,
      new Vector3(side * 0.168, wheelAxleLocalY(), layout.localZ),
      30,
    );
    createCylinderAlongX(
      scene,
      `${prefix}-pinion-${layout.id}`,
      materials.brass,
      parent,
      0.044,
      0.028,
      new Vector3(side * 0.145, relativeY(0.116), layout.localZ + 0.034),
      18,
    );
  }

  addBox(
    scene,
    `${prefix}-battery-cradle`,
    materials.plasticWhite,
    parent,
    { width: 0.236, height: 0.014, depth: 0.062 },
    new Vector3(0, relativeY(0.09), MICROMOUSE_BLUEPRINT.electronics.batteryLocalZ - 0.078),
  );

  addBox(
    scene,
    `${prefix}-battery`,
    materials.battery,
    parent,
    { width: 0.21, height: 0.056, depth: 0.046 },
    new Vector3(0, relativeY(0.126), MICROMOUSE_BLUEPRINT.electronics.batteryLocalZ - 0.078),
  );
  addBox(
    scene,
    `${prefix}-battery-label`,
    materials.batteryLabel,
    parent,
    { width: 0.148, height: 0.004, depth: 0.03 },
    new Vector3(0, relativeY(0.156), MICROMOUSE_BLUEPRINT.electronics.batteryLocalZ - 0.078),
  );
  addBox(
    scene,
    `${prefix}-battery-foil`,
    materials.brass,
    parent,
    { width: 0.024, height: 0.044, depth: 0.048 },
    new Vector3(0.102, relativeY(0.126), MICROMOUSE_BLUEPRINT.electronics.batteryLocalZ - 0.078),
  );

  addBox(
    scene,
    `${prefix}-motor-connector-left`,
    materials.plasticWhite,
    parent,
    { width: 0.054, height: 0.036, depth: 0.048 },
    new Vector3(-0.05, relativeY(0.106), MICROMOUSE_BLUEPRINT.electronics.connectorLocalZ),
  );
  addBox(
    scene,
    `${prefix}-motor-connector-right`,
    materials.plasticWhite,
    parent,
    { width: 0.054, height: 0.036, depth: 0.048 },
    new Vector3(0.05, relativeY(0.106), MICROMOUSE_BLUEPRINT.electronics.connectorLocalZ),
  );

  for (let index = 0; index < 6; index += 1) {
    const offset = -0.026 + index * 0.0105;
    addTube(
      scene,
      `${prefix}-ribbon-${index}`,
      materials.ribbon,
      parent,
      [
        new Vector3(offset, relativeY(0.07), -0.04),
        new Vector3(offset * 0.65, relativeY(0.108), -0.095),
        new Vector3(0.018 + offset * 0.35, relativeY(0.1), -0.17),
      ],
      0.0032,
    );
  }

  addTube(
    scene,
    `${prefix}-battery-red-wire`,
    materials.wireRed,
    parent,
    [
      new Vector3(0.092, relativeY(0.148), MICROMOUSE_BLUEPRINT.electronics.batteryLocalZ - 0.078),
      new Vector3(0.085, relativeY(0.186), -0.22),
      new Vector3(0.044, relativeY(0.178), -0.11),
      new Vector3(0.022, relativeY(0.145), -0.045),
    ],
    0.005,
  );
  addTube(
    scene,
    `${prefix}-battery-black-wire`,
    materials.wireBlack,
    parent,
    [
      new Vector3(0.112, relativeY(0.13), MICROMOUSE_BLUEPRINT.electronics.batteryLocalZ - 0.078),
      new Vector3(0.12, relativeY(0.184), -0.2),
      new Vector3(0.096, relativeY(0.178), -0.11),
      new Vector3(0.072, relativeY(0.143), -0.045),
    ],
    0.005,
  );
}

function addBoardElectronics(
  scene: Scene,
  materials: MicromouseMaterials,
  parent: TransformNode,
  prefix: string,
): void {
  const chip = addBox(
    scene,
    `${prefix}-main-chip`,
    materials.chip,
    parent,
    { width: 0.102, height: 0.018, depth: 0.102 },
    new Vector3(-0.052, componentY(0.018), 0.07),
  );

  for (let index = 0; index < 9; index += 1) {
    const pinOffset = -0.052 + index * 0.013;
    addBox(
      scene,
      `${prefix}-chip-pin-left`,
      materials.metal,
      chip,
      { width: 0.01, height: 0.004, depth: 0.006 },
      new Vector3(-0.062, 0.004, pinOffset),
    );
    addBox(
      scene,
      `${prefix}-chip-pin-right`,
      materials.metal,
      chip,
      { width: 0.01, height: 0.004, depth: 0.006 },
      new Vector3(0.062, 0.004, pinOffset),
    );
  }

  addBox(
    scene,
    `${prefix}-sensor-window`,
    materials.translucent,
    parent,
    { width: 0.1, height: 0.014, depth: 0.062 },
    new Vector3(0.08, componentY(0.014), 0.02),
  );

  for (let index = 0; index < 5; index += 1) {
    addBox(
      scene,
      `${prefix}-sensor-pad`,
      materials.boardTrace,
      parent,
      { width: 0.01, height: 0.003, depth: 0.04 },
      new Vector3(0.047 + index * 0.016, relativeY(pcbTopY() + 0.005), 0.02),
    );
  }

  const passiveParts = [
    [-0.155, 0.135, 0.03, 0.012],
    [-0.105, 0.16, 0.026, 0.014],
    [-0.005, 0.16, 0.02, 0.012],
    [0.085, 0.135, 0.028, 0.014],
    [0.145, 0.095, 0.034, 0.018],
    [-0.155, 0.03, 0.026, 0.012],
    [-0.12, -0.005, 0.034, 0.014],
    [0.15, -0.005, 0.032, 0.014],
    [-0.165, -0.115, 0.038, 0.018],
    [0.02, -0.12, 0.03, 0.014],
  ] as const;

  for (const [x, z, width, depth] of passiveParts) {
    addBox(
      scene,
      `${prefix}-smd`,
      materials.chip,
      parent,
      { width, height: 0.011, depth },
      new Vector3(x, componentY(0.011), z),
    );
  }

  for (const x of [-0.16, -0.13, 0.16]) {
    createCylinderAlongZ(
      scene,
      `${prefix}-front-cap`,
      materials.plasticBlack,
      parent,
      0.026,
      0.045,
      new Vector3(x, componentY(0.026), 0.06),
      14,
    );
  }
}

function addCenterOfMassMarker(
  scene: Scene,
  materials: MicromouseMaterials,
  parent: TransformNode,
  prefix: string,
): void {
  const marker = MeshBuilder.CreateSphere(
    `${prefix}-center-of-mass-marker`,
    { diameter: 0.026, segments: 12 },
    scene,
  );
  marker.position.set(
    MICROMOUSE_BLUEPRINT.chassis.centerOfMassOffset.x,
    MICROMOUSE_BLUEPRINT.chassis.centerOfMassOffset.y,
    MICROMOUSE_BLUEPRINT.chassis.centerOfMassOffset.z,
  );
  marker.material = materials.emitter;
  marker.parent = parent;
}

function addSensors(
  scene: Scene,
  materials: MicromouseMaterials,
  parent: TransformNode,
  prefix: string,
): void {
  for (const sensorLayout of MICROMOUSE_BLUEPRINT.sensors) {
    const sensor = addBox(
      scene,
      `${prefix}-ir-sensor-${sensorLayout.id}`,
      materials.sensor,
      parent,
      { width: 0.038, height: 0.026, depth: 0.048 },
      new Vector3(sensorLayout.localX, componentY(0.028), sensorLayout.localZ),
      sensorLayout.yaw,
    );

    const lens = MeshBuilder.CreateSphere(
      `${prefix}-ir-lens-${sensorLayout.id}`,
      { diameter: 0.016, segments: 10 },
      scene,
    );
    lens.position.set(0, 0.0, 0.021);
    lens.material = materials.emitter;
    lens.parent = sensor;
  }
}

function sensorBeamLocalOrigin(layout: SensorBlueprint): Vector3 {
  return new Vector3(layout.localX, componentY(0.028), layout.localZ).add(
    sensorBeamLocalDirection(layout.yaw).scale(SENSOR_BEAM_LENS_OFFSET),
  );
}

function sensorBeamLocalForwardPoint(layout: SensorBlueprint): Vector3 {
  return sensorBeamLocalOrigin(layout).add(sensorBeamLocalDirection(layout.yaw));
}

function sensorBeamLocalDirection(yaw: number): Vector3 {
  return new Vector3(Math.sin(yaw), 0, Math.cos(yaw));
}

function sensorBeamFlicker(seconds: number, visual: SensorBeamVisual): number {
  const pulse =
    Math.sin(seconds * visual.flickerFrequency + visual.flickerPhase) * 0.12 +
    Math.sin(seconds * visual.jitterFrequency + visual.flickerPhase * 1.7) * 0.05 +
    (Math.random() - 0.5) * 0.08;

  return clamp(0.42 + pulse, 0.22, 0.68);
}

function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}

function isSensorBeamTarget(mesh: AbstractMesh): boolean {
  const metadata = mesh.metadata as { sensorBeamTarget?: boolean } | null | undefined;

  return mesh.isEnabled() && mesh.isVisible && metadata?.sensorBeamTarget === true;
}

function createSensorBeamMaterial(scene: Scene, name: string, alpha: number): StandardMaterial {
  const result = new StandardMaterial(name, scene);
  result.diffuseColor = new Color3(1, 0.02, 0.015);
  result.emissiveColor = new Color3(1, 0.08, 0.035);
  result.specularColor = new Color3(0.35, 0.05, 0.04);
  result.alpha = alpha;
  result.backFaceCulling = false;
  result.disableLighting = true;

  return result;
}

function createWheelVisualMesh(
  scene: Scene,
  materials: MicromouseMaterials,
  layout: WheelBlueprint,
  name: string,
  parent?: TransformNode,
): Mesh {
  const tire = MeshBuilder.CreateCylinder(
    name,
    {
      diameter: MICROMOUSE_BLUEPRINT.wheel.radius * 2,
      height: MICROMOUSE_BLUEPRINT.wheel.width,
      tessellation: 36,
    },
    scene,
  );
  tire.rotationQuaternion = WHEEL_ALIGNMENT.clone();
  tire.bakeCurrentTransformIntoVertices();
  tire.rotationQuaternion = Quaternion.Identity();
  tire.material = materials.tire;

  if (parent) {
    tire.position.set(layout.localX, wheelAxleLocalY(), layout.localZ);
    tire.parent = parent;
  }

  const outerSign = layout.side === "left" ? -1 : 1;
  const outerX = outerSign * (MICROMOUSE_BLUEPRINT.wheel.width / 2);
  const innerX = (-outerSign * MICROMOUSE_BLUEPRINT.wheel.width) / 2;

  createCylinderAlongX(
    scene,
    `${name}-outer-side`,
    materials.hub,
    tire,
    0.132,
    0.006,
    new Vector3(outerX, 0, 0),
    28,
  );
  createCylinderAlongX(
    scene,
    `${name}-inner-side`,
    materials.wheelSide,
    tire,
    0.142,
    0.006,
    new Vector3(innerX, 0, 0),
    28,
  );
  createCylinderAlongX(
    scene,
    `${name}-hub`,
    materials.metal,
    tire,
    0.044,
    0.012,
    new Vector3(outerX + outerSign * 0.003, 0, 0),
    18,
  );

  for (let index = 0; index < 5; index += 1) {
    const angle = (index / 5) * Math.PI * 2;
    const spoke = MeshBuilder.CreateBox(
      `${name}-spoke`,
      { width: 0.005, height: 0.012, depth: 0.096 },
      scene,
    );
    spoke.position.set(outerX + outerSign * 0.006, 0, 0);
    spoke.rotationQuaternion = Quaternion.RotationAxis(Vector3.Right(), angle);
    spoke.material = materials.brass;
    spoke.parent = tire;
  }

  for (let index = 0; index < 14; index += 1) {
    const angle = (index / 14) * Math.PI * 2;
    const tread = MeshBuilder.CreateBox(
      `${name}-tread`,
      {
        width: MICROMOUSE_BLUEPRINT.wheel.width + 0.006,
        height: 0.006,
        depth: 0.018,
      },
      scene,
    );
    tread.position.set(
      0,
      Math.sin(angle) * (MICROMOUSE_BLUEPRINT.wheel.radius + 0.002),
      Math.cos(angle) * (MICROMOUSE_BLUEPRINT.wheel.radius + 0.002),
    );
    tread.rotationQuaternion = Quaternion.RotationAxis(Vector3.Right(), angle);
    tread.material = materials.plasticBlack;
    tread.parent = tire;
  }

  return tire;
}

function configureWheelConstraint(constraint: Physics6DoFConstraint): void {
  constraint.setAxisMode(PhysicsConstraintAxis.LINEAR_X, PhysicsConstraintAxisLimitMode.LOCKED);
  constraint.setAxisMode(PhysicsConstraintAxis.LINEAR_Y, PhysicsConstraintAxisLimitMode.LOCKED);
  constraint.setAxisMode(PhysicsConstraintAxis.LINEAR_Z, PhysicsConstraintAxisLimitMode.LOCKED);
  constraint.setAxisMode(PhysicsConstraintAxis.ANGULAR_X, PhysicsConstraintAxisLimitMode.FREE);
  constraint.setAxisMode(PhysicsConstraintAxis.ANGULAR_Y, PhysicsConstraintAxisLimitMode.LOCKED);
  constraint.setAxisMode(PhysicsConstraintAxis.ANGULAR_Z, PhysicsConstraintAxisLimitMode.LOCKED);
  constraint.setAxisMotorType(PhysicsConstraintAxis.ANGULAR_X, PhysicsConstraintMotorType.VELOCITY);
  constraint.setAxisMotorMaxForce(
    PhysicsConstraintAxis.ANGULAR_X,
    MICROMOUSE_BLUEPRINT.wheel.motorMaxForce,
  );
  constraint.setAxisFriction(PhysicsConstraintAxis.ANGULAR_X, 0.01);
}

function initialPose(maze: MazeSnapshot): { position: Vector3; rotation: Quaternion } {
  const start = cellCenter(maze.start, maze.size);

  return {
    position: new Vector3(start.x, MICROMOUSE_BLUEPRINT.chassis.centerY, start.z),
    rotation: Quaternion.RotationAxis(Vector3.Up(), initialMouseYaw(maze)),
  };
}

export function micromouseGroundTruthPoseFromChassis(
  position: Vector3,
  rotation: Quaternion,
): RobotGroundTruthPose {
  const origin = localToWorld(micromouseWheelCenterLocal(), position, rotation);
  const rotationMatrix = Matrix.FromQuaternionToRef(rotation, Matrix.Identity());
  const forward = Vector3.TransformNormal(Vector3.Forward(), rotationMatrix);

  return {
    origin: {
      x: origin.x,
      y: origin.y,
      z: origin.z,
    },
    yaw: yawFromRobotForward(forward),
  };
}

export function micromouseWheelCenterLocal(): Vector3 {
  const wheelCount = MICROMOUSE_BLUEPRINT.wheels.length;
  const sum = MICROMOUSE_BLUEPRINT.wheels.reduce(
    (acc, wheel) => ({
      x: acc.x + wheel.localX,
      z: acc.z + wheel.localZ,
    }),
    { x: 0, z: 0 },
  );

  return new Vector3(sum.x / wheelCount, wheelAxleLocalY(), sum.z / wheelCount);
}

export function yawFromRobotForward(forward: Pick<Vector3, "x" | "z">): number {
  return Math.atan2(forward.x, forward.z);
}

function createPcbMesh(scene: Scene, name: string): Mesh {
  const halfHeight = MICROMOUSE_BLUEPRINT.pcb.height / 2;

  return createExtrudedFootprintMesh(scene, name, boardFootprintPoints(), -halfHeight, halfHeight);
}

function createPcbColliderMesh(scene: Scene, name: string): Mesh {
  const pcbBottomY =
    MICROMOUSE_BLUEPRINT.pcb.centerY -
    MICROMOUSE_BLUEPRINT.pcb.height / 2 -
    MICROMOUSE_BLUEPRINT.chassis.centerY;
  const colliderTopY = pcbBottomY + MICROMOUSE_BLUEPRINT.chassis.height;

  return createExtrudedFootprintMesh(scene, name, boardFootprintPoints(), pcbBottomY, colliderTopY);
}

function createExtrudedFootprintMesh(
  scene: Scene,
  name: string,
  footprint: readonly FootprintPoint[],
  bottomY: number,
  topY: number,
): Mesh {
  const points = normalizePolygon(footprint);
  const topTriangles = triangulatePolygon(points);
  const positions: number[] = [];
  const indices: number[] = [];

  for (const point of points) {
    positions.push(point.x, topY, point.z);
  }

  for (const point of points) {
    positions.push(point.x, bottomY, point.z);
  }

  const bottomOffset = points.length;

  for (const [a, b, c] of topTriangles) {
    indices.push(a, c, b);
    indices.push(bottomOffset + a, bottomOffset + b, bottomOffset + c);
  }

  for (let index = 0; index < points.length; index += 1) {
    const next = (index + 1) % points.length;
    indices.push(index, bottomOffset + next, next);
    indices.push(index, bottomOffset + index, bottomOffset + next);
  }

  const normals: number[] = [];
  VertexData.ComputeNormals(positions, indices, normals);

  const data = new VertexData();
  data.positions = positions;
  data.indices = indices;
  data.normals = normals;

  const mesh = new Mesh(name, scene);
  data.applyToMesh(mesh);

  return mesh;
}

interface FootprintPoint {
  readonly x: number;
  readonly z: number;
}

export function boardFootprintPoints(): FootprintPoint[] {
  const { pcb } = MICROMOUSE_BLUEPRINT;
  const halfWidth = pcb.width / 2;
  const sideIntersectionZ = circleSideIntersectionZ(
    halfWidth,
    pcb.frontArcCenterZ,
    pcb.frontRadius,
  );
  const wheelNotch = wheelCutout();
  const points: FootprintPoint[] = [
    { x: wheelNotch.innerX, z: pcb.rearZ },
    { x: wheelNotch.innerX, z: wheelNotch.frontZ },
    { x: halfWidth, z: wheelNotch.frontZ },
    { x: halfWidth, z: sideIntersectionZ },
  ];

  const sideAngle = Math.acos(halfWidth / pcb.frontRadius);

  for (let index = 1; index < BOARD_ARC_SEGMENTS; index += 1) {
    const angle = sideAngle + (index / BOARD_ARC_SEGMENTS) * (Math.PI - sideAngle * 2);
    points.push({
      x: Math.cos(angle) * pcb.frontRadius,
      z: pcb.frontArcCenterZ + Math.sin(angle) * pcb.frontRadius,
    });
  }

  points.push(
    { x: -halfWidth, z: sideIntersectionZ },
    { x: -halfWidth, z: wheelNotch.frontZ },
    { x: -wheelNotch.innerX, z: wheelNotch.frontZ },
    { x: -wheelNotch.innerX, z: pcb.rearZ },
  );

  return points;
}

function circleSideIntersectionZ(halfWidth: number, centerZ: number, radius: number): number {
  return centerZ + Math.sqrt(Math.max(0, radius * radius - halfWidth * halfWidth));
}

function wheelCutout(): { innerX: number; frontZ: number } {
  const margin = 0.012;
  const wheel = MICROMOUSE_BLUEPRINT.wheel;
  const innerX =
    Math.min(...MICROMOUSE_BLUEPRINT.wheels.map((layout) => Math.abs(layout.localX))) -
    wheel.width / 2 -
    margin;
  const frontZ =
    Math.max(...MICROMOUSE_BLUEPRINT.wheels.map((layout) => layout.localZ)) + wheel.radius + margin;

  return {
    innerX,
    frontZ,
  };
}

function normalizePolygon(points: readonly FootprintPoint[]): FootprintPoint[] {
  const normalized: FootprintPoint[] = [];

  for (const point of points) {
    const previous = normalized[normalized.length - 1];

    if (previous && Math.hypot(previous.x - point.x, previous.z - point.z) < 0.000001) {
      continue;
    }

    normalized.push(point);
  }

  if (polygonArea(normalized) < 0) {
    normalized.reverse();
  }

  return normalized;
}

function triangulatePolygon(points: readonly FootprintPoint[]): [number, number, number][] {
  const indices = points.map((_, index) => index);
  const triangles: [number, number, number][] = [];
  let guard = 0;

  while (indices.length > 3 && guard < points.length * points.length) {
    let clipped = false;

    for (let index = 0; index < indices.length; index += 1) {
      const previousIndex = indices[(index + indices.length - 1) % indices.length];
      const currentIndex = indices[index];
      const nextIndex = indices[(index + 1) % indices.length];

      if (!isEar(previousIndex, currentIndex, nextIndex, indices, points)) {
        continue;
      }

      triangles.push([previousIndex, currentIndex, nextIndex]);
      indices.splice(index, 1);
      clipped = true;
      break;
    }

    if (!clipped) {
      break;
    }

    guard += 1;
  }

  if (indices.length === 3) {
    triangles.push([indices[0], indices[1], indices[2]]);
  }

  if (triangles.length === 0) {
    for (let index = 1; index < points.length - 1; index += 1) {
      triangles.push([0, index, index + 1]);
    }
  }

  return triangles;
}

function isEar(
  previousIndex: number,
  currentIndex: number,
  nextIndex: number,
  polygonIndices: readonly number[],
  points: readonly FootprintPoint[],
): boolean {
  const previous = points[previousIndex];
  const current = points[currentIndex];
  const next = points[nextIndex];

  if (cross2D(previous, current, next) <= 0.0000001) {
    return false;
  }

  for (const pointIndex of polygonIndices) {
    if (pointIndex === previousIndex || pointIndex === currentIndex || pointIndex === nextIndex) {
      continue;
    }

    if (pointInTriangle(points[pointIndex], previous, current, next)) {
      return false;
    }
  }

  return true;
}

function pointInTriangle(
  point: FootprintPoint,
  a: FootprintPoint,
  b: FootprintPoint,
  c: FootprintPoint,
): boolean {
  const areaA = cross2D(point, a, b);
  const areaB = cross2D(point, b, c);
  const areaC = cross2D(point, c, a);
  return areaA > 0.0000001 && areaB > 0.0000001 && areaC > 0.0000001;
}

function cross2D(a: FootprintPoint, b: FootprintPoint, c: FootprintPoint): number {
  return (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
}

function polygonArea(points: readonly FootprintPoint[]): number {
  let area = 0;

  for (let index = 0; index < points.length; index += 1) {
    const current = points[index];
    const next = points[(index + 1) % points.length];
    area += current.x * next.z - next.x * current.z;
  }

  return area / 2;
}

function addBox(
  scene: Scene,
  name: string,
  materialValue: StandardMaterial,
  parent: TransformNode | Mesh,
  size: { width: number; height: number; depth: number },
  position: Vector3,
  yaw = 0,
): Mesh {
  const mesh = MeshBuilder.CreateBox(name, size, scene);
  mesh.position.copyFrom(position);
  mesh.rotationQuaternion =
    yaw === 0 ? Quaternion.Identity() : Quaternion.RotationAxis(Vector3.Up(), yaw);
  mesh.material = materialValue;
  mesh.parent = parent;

  return mesh;
}

function createCylinderAlongX(
  scene: Scene,
  name: string,
  materialValue: StandardMaterial,
  parent: TransformNode | Mesh,
  diameter: number,
  height: number,
  position: Vector3,
  tessellation: number,
): Mesh {
  const mesh = MeshBuilder.CreateCylinder(name, { diameter, height, tessellation }, scene);
  mesh.position.copyFrom(position);
  mesh.rotationQuaternion = WHEEL_ALIGNMENT.clone();
  mesh.material = materialValue;
  mesh.parent = parent;

  return mesh;
}

function createCylinderAlongZ(
  scene: Scene,
  name: string,
  materialValue: StandardMaterial,
  parent: TransformNode | Mesh,
  diameter: number,
  height: number,
  position: Vector3,
  tessellation: number,
): Mesh {
  const mesh = MeshBuilder.CreateCylinder(name, { diameter, height, tessellation }, scene);
  mesh.position.copyFrom(position);
  mesh.rotationQuaternion = LONGITUDINAL_ALIGNMENT.clone();
  mesh.material = materialValue;
  mesh.parent = parent;

  return mesh;
}

function addTube(
  scene: Scene,
  name: string,
  materialValue: StandardMaterial,
  parent: TransformNode,
  path: Vector3[],
  radius: number,
): Mesh {
  const mesh = MeshBuilder.CreateTube(name, { path, radius, tessellation: 8 }, scene);
  mesh.material = materialValue;
  mesh.parent = parent;

  return mesh;
}

function pcbTopY(): number {
  return MICROMOUSE_BLUEPRINT.pcb.centerY + MICROMOUSE_BLUEPRINT.pcb.height / 2;
}

function componentY(height: number): number {
  return relativeY(pcbTopY() + height / 2);
}

function relativeY(worldY: number): number {
  return worldY - MICROMOUSE_BLUEPRINT.chassis.centerY;
}

function wheelAxleLocalY(): number {
  return MICROMOUSE_BLUEPRINT.wheel.axleY - MICROMOUSE_BLUEPRINT.chassis.centerY;
}

function localToWorld(local: Vector3, origin: Vector3, rotation: Quaternion): Vector3 {
  return Vector3.TransformCoordinates(local, Matrix.Compose(Vector3.One(), rotation, origin));
}

function vectorFromBlueprint(vector: {
  readonly x: number;
  readonly y: number;
  readonly z: number;
}): Vector3 {
  return new Vector3(vector.x, vector.y, vector.z);
}

function material(
  scene: Scene,
  name: string,
  diffuseColor: Color3,
  specularPower: number,
  alpha = 1,
): StandardMaterial {
  const result = new StandardMaterial(name, scene);
  result.diffuseColor = diffuseColor;
  result.emissiveColor = new Color3(
    diffuseColor.r * 0.1,
    diffuseColor.g * 0.1,
    diffuseColor.b * 0.1,
  );
  result.specularColor = new Color3(specularPower, specularPower, specularPower);
  result.alpha = alpha;
  result.backFaceCulling = false;

  return result;
}
