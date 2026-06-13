import { StandardMaterial } from "@babylonjs/core/Materials/standardMaterial.js";
import { Color3 } from "@babylonjs/core/Maths/math.color.js";
import { Matrix, Quaternion, Vector3 } from "@babylonjs/core/Maths/math.vector.js";
import { Mesh } from "@babylonjs/core/Meshes/mesh.js";
import { MeshBuilder } from "@babylonjs/core/Meshes/meshBuilder.js";
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
import { MICROMOUSE_BLUEPRINT, initialMouseYaw, type WheelBlueprint } from "./micromouseModel";
import { commandMagnitude, type MotorCommand } from "./randomMotorDriver";

interface WheelAssembly {
  readonly layout: WheelBlueprint;
  readonly mesh: Mesh;
  readonly body: PhysicsBody;
  readonly shape: PhysicsShapeCylinder;
  readonly constraint: Physics6DoFConstraint;
}

export interface MicromouseMaterials {
  readonly board: StandardMaterial;
  readonly boardEdge: StandardMaterial;
  readonly chip: StandardMaterial;
  readonly metal: StandardMaterial;
  readonly tire: StandardMaterial;
  readonly hub: StandardMaterial;
  readonly battery: StandardMaterial;
  readonly sensor: StandardMaterial;
  readonly emitter: StandardMaterial;
  readonly gear: StandardMaterial;
}

const ZERO_COMMAND: MotorCommand = {
  leftRadPerSec: 0,
  rightRadPerSec: 0,
};

const WHEEL_ALIGNMENT = Quaternion.RotationAxis(Vector3.Forward(), -Math.PI / 2);

export class MicromouseRig {
  readonly #scene: Scene;
  readonly #materials: MicromouseMaterials;
  readonly #chassis: Mesh;
  readonly #chassisAggregate: PhysicsAggregate;
  readonly #wheels: WheelAssembly[] = [];
  #slowSeconds = 0;
  #disposed = false;

  constructor(scene: Scene, materials: MicromouseMaterials, maze: MazeSnapshot) {
    this.#scene = scene;
    this.#materials = materials;

    const pose = initialPose(maze);
    this.#chassis = this.#createChassis(pose.position, pose.rotation);
    this.#chassisAggregate = new PhysicsAggregate(
      this.#chassis,
      PhysicsShapeType.BOX,
      { mass: 0.18, friction: 0.55, restitution: 0.02 },
      this.#scene,
    );
    this.#chassisAggregate.shape.material = {
      friction: 0.55,
      staticFriction: 0.65,
      restitution: 0.02,
      frictionCombine: PhysicsMaterialCombineMode.ARITHMETIC_MEAN,
    };
    this.#chassisAggregate.body.setMassProperties({
      mass: 0.18,
      centerOfMass: new Vector3(0, -0.035, 0),
    });
    this.#chassisAggregate.body.setLinearDamping(0.12);
    this.#chassisAggregate.body.setAngularDamping(0.18);

    this.#addBoardDetails();
    this.#addMotorDetails();
    this.#addSensors();
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
    this.#chassisAggregate.dispose();
    this.#chassis.dispose(false, true);
  }

  #createChassis(position: Vector3, rotation: Quaternion): Mesh {
    const { chassis } = MICROMOUSE_BLUEPRINT;
    const mesh = MeshBuilder.CreateBox(
      "micromouse-chassis",
      { width: chassis.width, height: chassis.height, depth: chassis.depth },
      this.#scene,
    );
    mesh.position.copyFrom(position);
    mesh.rotationQuaternion = rotation.clone();
    mesh.material = this.#materials.board;

    return mesh;
  }

  #addBoardDetails(): void {
    const upperBoard = MeshBuilder.CreateBox(
      "micromouse-upper-pcb",
      { width: 0.34, height: 0.018, depth: 0.42 },
      this.#scene,
    );
    upperBoard.position.set(0, 0.041, 0.015);
    upperBoard.material = this.#materials.boardEdge;
    upperBoard.parent = this.#chassis;

    const chip = MeshBuilder.CreateBox(
      "micromouse-main-chip",
      { width: 0.11, height: 0.018, depth: 0.11 },
      this.#scene,
    );
    chip.position.set(0, 0.064, 0.09);
    chip.material = this.#materials.chip;
    chip.parent = this.#chassis;

    for (let index = 0; index < 8; index += 1) {
      const pinOffset = -0.0525 + index * 0.015;
      this.#addPin(-0.064, pinOffset);
      this.#addPin(0.064, pinOffset);
    }

    const battery = MeshBuilder.CreateBox(
      "micromouse-battery",
      { width: 0.19, height: 0.058, depth: 0.16 },
      this.#scene,
    );
    battery.position.set(0, 0.082, -0.17);
    battery.material = this.#materials.battery;
    battery.parent = this.#chassis;

    const frontLed = MeshBuilder.CreateSphere(
      "micromouse-front-led",
      { diameter: 0.035, segments: 12 },
      this.#scene,
    );
    frontLed.position.set(0, 0.076, 0.245);
    frontLed.material = this.#materials.emitter;
    frontLed.parent = this.#chassis;
  }

  #addPin(localX: number, localZ: number): void {
    const pin = MeshBuilder.CreateBox(
      "micromouse-chip-pin",
      { width: 0.012, height: 0.006, depth: 0.006 },
      this.#scene,
    );
    pin.position.set(localX, 0.064, 0.09 + localZ);
    pin.material = this.#materials.metal;
    pin.parent = this.#chassis;
  }

  #addMotorDetails(): void {
    for (const side of [-1, 1]) {
      const motor = MeshBuilder.CreateCylinder(
        "micromouse-coreless-motor",
        { diameter: 0.09, height: 0.18, tessellation: 24 },
        this.#scene,
      );
      motor.position.set(side * 0.115, 0.078, -0.13);
      motor.rotationQuaternion = Quaternion.RotationAxis(Vector3.Right(), Math.PI / 2);
      motor.material = this.#materials.metal;
      motor.parent = this.#chassis;

      const gear = MeshBuilder.CreateCylinder(
        "micromouse-drive-gear",
        { diameter: 0.13, height: 0.024, tessellation: 18 },
        this.#scene,
      );
      gear.position.set(side * 0.17, 0.053, -0.235);
      gear.rotationQuaternion = WHEEL_ALIGNMENT.clone();
      gear.material = this.#materials.gear;
      gear.parent = this.#chassis;
    }
  }

  #addSensors(): void {
    for (const sensorLayout of MICROMOUSE_BLUEPRINT.sensors) {
      const sensor = MeshBuilder.CreateBox(
        `micromouse-ir-sensor-${sensorLayout.id}`,
        { width: 0.04, height: 0.026, depth: 0.055 },
        this.#scene,
      );
      sensor.position.set(sensorLayout.localX, 0.064, sensorLayout.localZ);
      sensor.rotationQuaternion = Quaternion.RotationAxis(Vector3.Up(), sensorLayout.yaw);
      sensor.material = this.#materials.sensor;
      sensor.parent = this.#chassis;

      const emitter = MeshBuilder.CreateSphere(
        `micromouse-ir-emitter-${sensorLayout.id}`,
        { diameter: 0.016, segments: 8 },
        this.#scene,
      );
      emitter.position.set(0, 0.002, 0.03);
      emitter.material = this.#materials.emitter;
      emitter.parent = sensor;
    }
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
        friction: 1.55,
        staticFriction: 1.9,
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

  #createWheelMesh(layout: WheelBlueprint, origin: Vector3, rotation: Quaternion): Mesh {
    const mesh = MeshBuilder.CreateCylinder(
      `micromouse-wheel-${layout.id}`,
      {
        diameter: MICROMOUSE_BLUEPRINT.wheel.radius * 2,
        height: MICROMOUSE_BLUEPRINT.wheel.width,
        tessellation: 28,
      },
      this.#scene,
    );
    mesh.rotationQuaternion = WHEEL_ALIGNMENT.clone();
    mesh.bakeCurrentTransformIntoVertices();
    mesh.position.copyFrom(
      localToWorld(
        new Vector3(layout.localX, wheelCenterLocalY(), layout.localZ),
        origin,
        rotation,
      ),
    );
    mesh.rotationQuaternion = rotation.clone();
    mesh.material = this.#materials.tire;

    const hub = MeshBuilder.CreateCylinder(
      `micromouse-wheel-hub-${layout.id}`,
      { diameter: 0.07, height: MICROMOUSE_BLUEPRINT.wheel.width + 0.006, tessellation: 18 },
      this.#scene,
    );
    hub.rotationQuaternion = WHEEL_ALIGNMENT.clone();
    hub.material = this.#materials.hub;
    hub.parent = mesh;

    return mesh;
  }

  #createWheelConstraint(layout: WheelBlueprint): Physics6DoFConstraint {
    const localY = wheelCenterLocalY() - MICROMOUSE_BLUEPRINT.chassis.centerY;
    const constraint = new Physics6DoFConstraint(
      {
        pivotA: new Vector3(layout.localX, localY, layout.localZ),
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

export function createMicromouseMaterials(scene: Scene): MicromouseMaterials {
  return {
    board: material(scene, "mouse-board-material", new Color3(0.05, 0.38, 0.19), 0.2),
    boardEdge: material(scene, "mouse-upper-board-material", new Color3(0.08, 0.5, 0.24), 0.15),
    chip: material(scene, "mouse-chip-material", new Color3(0.025, 0.028, 0.032), 0.25),
    metal: material(scene, "mouse-metal-material", new Color3(0.75, 0.78, 0.8), 0.75),
    tire: material(scene, "mouse-tire-material", new Color3(0.015, 0.016, 0.018), 0.08),
    hub: material(scene, "mouse-hub-material", new Color3(0.78, 0.1, 0.1), 0.35),
    battery: material(scene, "mouse-battery-material", new Color3(0.1, 0.12, 0.16), 0.18),
    sensor: material(scene, "mouse-sensor-material", new Color3(0.035, 0.04, 0.045), 0.2),
    emitter: material(scene, "mouse-emitter-material", new Color3(0.95, 0.05, 0.035), 0.3),
    gear: material(scene, "mouse-gear-material", new Color3(0.86, 0.63, 0.22), 0.45),
  };
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

function wheelCenterLocalY(): number {
  return MICROMOUSE_BLUEPRINT.wheel.radius;
}

function localToWorld(local: Vector3, origin: Vector3, rotation: Quaternion): Vector3 {
  return Vector3.TransformCoordinates(local, Matrix.Compose(Vector3.One(), rotation, origin));
}

function material(
  scene: Scene,
  name: string,
  diffuseColor: Color3,
  specularPower: number,
): StandardMaterial {
  const result = new StandardMaterial(name, scene);
  result.diffuseColor = diffuseColor;
  result.emissiveColor = new Color3(
    diffuseColor.r * 0.1,
    diffuseColor.g * 0.1,
    diffuseColor.b * 0.1,
  );
  result.specularColor = new Color3(specularPower, specularPower, specularPower);

  return result;
}
