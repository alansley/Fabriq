# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Caliko is a Java implementation of the **FABRIK** (Forward And Backward Reaching Inverse Kinematics) algorithm. It is a multi-module Maven project targeting Java 11.

## Build Commands

```bash
# Build all modules
mvn clean package

# Install to local Maven repo
mvn clean install

# Build a specific module
mvn clean package -pl caliko
mvn clean package -pl caliko-visualisation
mvn clean package -pl caliko-demo

# Run tests
mvn test

# Run unit + integration tests
mvn verify

# Generate JavaDoc
mvn javadoc:javadoc
```

No explicit linter or formatter is configured — the project uses standard Java conventions.

## Module Architecture

```
caliko/                   # Core IK library — zero external dependencies
caliko-visualisation/     # Optional OpenGL rendering via LWJGL 3.2.2
caliko-demo/              # Interactive demo app (depends on both above)
caliko-distribution/      # Assembly/packaging configuration
```

### Core Library (`caliko`)

The IK algorithm is implemented through a parallel 2D/3D class hierarchy, unified by generic interfaces:

- `FabrikStructure<T extends FabrikChain, V extends Vectorf>` — container for multiple chains (multi-effector IK)
- `FabrikChain2D` / `FabrikChain3D` — primary solving class; holds bones and iterates FABRIK
- `FabrikBone2D` / `FabrikBone3D` — individual bone segment with start/end vector and length
- `FabrikJoint2D` / `FabrikJoint3D` — joint constraints (rotor/hinge, local/global)

All classes live under `au.edu.federation.caliko` and math utilities under `au.edu.federation.utils`.

**Solving parameters** (configurable per-chain):
- `maxIterationAttempts` — default 20
- `minIterationChange` — default 0.01f
- `solveDistanceThreshold` — default 1.0f

**Basebone constraint types** (enum, shared between 2D/3D):
- `NONE`, `GLOBAL_ROTOR`, `LOCAL_ROTOR`, `GLOBAL_HINGE`, `LOCAL_HINGE`

**Chain connectivity:** Chains connect to other chains via `BoneConnectionPoint` (START or END). The embedded target mode lets a chain track its own independent target.

**Serialization:** `SerializationUtil` handles Java object serialization of chains/structures. Test resources under `caliko/src/test/resources/` contain pre-saved binary files used in deserialization tests.

**Math classes:** Custom `Vec2f`, `Vec3f`, `Mat3f`, `Mat4f` — no third-party math library.

### Visualization Module (`caliko-visualisation`)

Wraps LWJGL 3 (OpenGL, GLFW). Key classes:
- `Entity` — base for all drawable objects
- `FabrikLine2D` / `FabrikLine3D` / `FabrikModel3D` — render bones as lines or OBJ models
- `ShaderProgram` — manages GLSL shaders
- `Camera` — 3D camera with mouse-look

### Demo Application (`caliko-demo`)

Entry point: `au.edu.federation.caliko.demo.Application.main(String[])`

Implements `CalikoDemo` interface, switching between `CalikoDemo2D` and `CalikoDemo3D`. Runtime controls:
- Arrow keys: switch 2D/3D mode, next/prev demo scenario
- Left mouse button: set target (2D) / mouse-look (3D)
- Space: pause/resume, L/M/X/F/P/R: toggle visual options, Esc: quit

## Testing

Tests use JUnit 4 and live in `caliko/src/test/java/au/edu/federation/`:
- `utils/CalikoUnitTests.java` — serialization, angle calculations, chain solving, vector ops
- `caliko/ApplicationPerfTest.java` — performance benchmarks

```bash
# Run all tests
mvn test

# Run a single test class
mvn test -pl caliko -Dtest=CalikoUnitTests

# Run a single test method
mvn test -pl caliko -Dtest=CalikoUnitTests#testMethodName
```
