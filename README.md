# bullet-sharp

A pure C# port of the [Bullet Physics](https://github.com/bulletphysics/bullet3) engine targeting **.NET 8** (`net8.0`), with an API compatible with the original BulletSharp and **no native or P/Invoke dependencies**.

## Packages

| NuGet Package | Project | Precision | Scope |
| --- | --- | --- | --- |
| `BulletSharp.PureCSharp` | `src/BulletSharp` | Single (`float`) | Rigid body dynamics, collision detection, constraints |
| `BulletSharp.PureCSharp.Double` | `src/BulletSharp.Double` | Double (`double`) | Rigid body dynamics, collision detection, constraints |
| `BulletSharp.PureCSharp.SoftBody` | `src/BulletSharp.SoftBody` | Single (`float`) | Classic soft-body simulation (`SoftBody`, `SoftRigidDynamicsWorld`) |
| `BulletSharp.PureCSharp.SoftBody.Double` | `src/BulletSharp.SoftBody.Double` | Double (`double`) | Classic soft-body simulation (`SoftBody`, `SoftRigidDynamicsWorld`) |

The `.Double` projects compile the exact same C# source files with `BT_USE_DOUBLE_PRECISION` defined, switching the internal `btScalar` alias from `System.Single` to `System.Double`. When using soft bodies, ensure the core and `SoftBody` packages use matching precision.

## Features

### Supported

- **Rigid Body Dynamics** (`BulletSharp`): `DiscreteDynamicsWorld`, `RigidBody`, `DefaultMotionState`, `SimulationIslandManager`, `SequentialImpulseConstraintSolver`, and constraints (`Point2PointConstraint`, `HingeConstraint`, `Generic6DofConstraint`, `Generic6DofSpringConstraint`, `ConeTwistConstraint`, `SliderConstraint`, `FixedConstraint`, `GearConstraint`).
- **Collision Detection**:
  - **Broadphase**: `DbvtBroadphase`, `SimpleBroadphase`, `HashedOverlappingPairCache`.
  - **Shapes**: convex primitives (`BoxShape`, `CylinderShape`, etc.), `StaticPlaneShape`, `ConvexHullShape`, `ConvexPointCloudShape`, `UniformScalingShape`, `MultiSphereShape`, `CompoundShape`, `EmptyShape`, triangle meshes (`TriangleMesh`, `TriangleIndexVertexArray`, `BvhTriangleMeshShape`, `ScaledBvhTriangleMeshShape`, `ConvexTriangleMeshShape`, `OptimizedBvh`, `QuantizedBvh`), and `HeightfieldTerrainShape`.
  - **Narrowphase & Queries**: GJK/EPA, `BoxBoxDetector`, `PersistentManifold`, ray tests (`RayTest`), convex sweep tests (`ConvexSweepTest`), contact tests (`ContactTest`), and `IDebugDraw`.
- **Classic Soft-Body Simulation** (`BulletSharp.SoftBody`): ropes, cloth patches, aerodynamics, pressure/volume conservation, tetrahedral meshes, rigid-body anchors, position/velocity/drift solvers, cluster shape matching and joints (`LJoint`, `AJoint`, `CJoint`), `SparseSdf`, `SoftRigidDynamicsWorld`, CPU `DefaultSoftBodySolver`, and `SoftBodyHelpers`.
- **Math & SIMD** (`BulletSharp.Math`): `Vector3`, `Quaternion`, `Matrix` (SharpDX row-vector layout: translation in `M41..M43`, `v * M`), `Transform`, `BtMath`, and `System.Runtime.Intrinsics` (`Sse`) acceleration for bit-exact float operations.

### Intentionally Omitted (vs. C++ Bullet)

- MultiBody / Featherstone (`btMultiBody*`)
- Deformable FEM framework (`btDeformable*`, Krylov/CG/CR solvers, `ReducedDeformableBody`, `btSoftMultiBodyDynamicsWorld`)
- GPU OpenCL / DX11 soft-body solvers (CPU `SoftBodySolver` / `DefaultSoftBodySolver` are included)
- MLCP solvers, GImpact, and `KinematicCharacterController`
- Binary serialization (`btSoftBodyData` / `.bullet` serialization)
- SIMD array helpers `btVector3::maxDot` / `minDot` (scalar reference paths are used instead)

See [`docs/KNOWN_LIMITATIONS.md`](docs/KNOWN_LIMITATIONS.md) for full details.

## Quick Start

Install via NuGet:

```bash
dotnet add package BulletSharp.PureCSharp
# Optional soft-body support:
dotnet add package BulletSharp.PureCSharp.SoftBody
```

Minimal rigid-body simulation example:

```csharp
using BulletSharp;
using BulletSharp.Math;

using var config = new DefaultCollisionConfiguration();
using var dispatcher = new CollisionDispatcher(config);
using var broadphase = new DbvtBroadphase();
using var solver = new SequentialImpulseConstraintSolver();
using var world = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, config);

var gravity = new Vector3(0, -9.81f, 0);
world.SetGravity(ref gravity);

// Static ground box
using var groundShape = new BoxShape(50);
var groundTransform = Matrix.Translation(0, -50, 0);
using var groundInfo = new RigidBodyConstructionInfo(
    0, new DefaultMotionState(groundTransform), groundShape, Vector3.Zero);
using var groundBody = new RigidBody(groundInfo);
world.AddRigidBody(groundBody);

// Dynamic falling box
using var boxShape = new BoxShape(1);
float mass = 1.0f;
boxShape.CalculateLocalInertia(mass, out Vector3 inertia);
var boxTransform = Matrix.Translation(0, 10, 0);
using var boxInfo = new RigidBodyConstructionInfo(
    mass, new DefaultMotionState(boxTransform), boxShape, inertia);
using var boxBody = new RigidBody(boxInfo);
world.AddRigidBody(boxBody);

for (int i = 0; i < 60; i++)
{
    world.StepSimulation(1f / 60f);
}

Matrix transform = boxBody.WorldTransform;
Console.WriteLine($"Box Y after 1s: {transform.M42:F3}");
```

## Building, Testing, and Benchmarks

### Build

```bash
dotnet build BulletSharp.slnx
dotnet build BulletSharp.slnx -c Release
```

Optional MSBuild flags:

- `-p:Precision=double` — builds test/benchmark projects against the `.Double` libraries with `BT_USE_DOUBLE_PRECISION`.
- `-p:SimdApi=true` — float builds only; defines `BT_USE_SSE_IN_API` (default `false`) to enable SSE `Vector3.Normalize` and quaternion paths that may differ in the last ULP, matching Bullet's Windows build option.

### Smoke Tests & Differential Traces

```bash
# Run smoke tests (single precision)
dotnet run --project tests/BulletSharp.SmokeTests

# Run smoke tests (double precision)
dotnet run --project tests/BulletSharp.SmokeTests -p:Precision=double

# Run differential fuzz trace for a given seed
dotnet run --project tests/BulletSharp.SmokeTests -- fuzz <seed>

# Run soft-body differential trace
dotnet run --project tests/BulletSharp.SmokeTests -- soft <scene> [steps]
```

### Benchmarks

```bash
dotnet run -c Release --project benchmarks/BulletSharp.Benchmarks
dotnet run -c Release --project benchmarks/BulletSharp.SoftBody.Benchmarks

# Double precision
dotnet run -c Release --project benchmarks/BulletSharp.Benchmarks -p:Precision=double
dotnet run -c Release --project benchmarks/BulletSharp.SoftBody.Benchmarks -p:Precision=double
```

## Numerical Parity & Validation

The port is validated against a native g++ scalar build of `bullet3` via per-step differential traces in `tests/native-ref/` (`FuzzRef.cpp`, `TraceRef.cpp`, `S7Ref.cpp`, `SoftRef.cpp`, and comparison scripts `cmp.ps1` / `firstdiff.ps1`). Both the constraint/solver path and box-box collision path match the native scalar reference bit-exactly under default float build settings.

## Documentation

- [`docs/KNOWN_LIMITATIONS.md`](docs/KNOWN_LIMITATIONS.md) — supported vs. omitted modules, numerical parity notes, and porting details.
- [`docs/PORTING_GUIDE.md`](docs/PORTING_GUIDE.md) — precision (`btScalar`), math layout, SIMD rules, and repository conventions.

## License

Licensed under the [zlib License](LICENSE).

This project is an independent C# port of the [Bullet Physics Library](https://github.com/bulletphysics/bullet3) (Copyright © 2003–2024 Erwin Coumans and contributors) and is not affiliated with or endorsed by the original authors.
