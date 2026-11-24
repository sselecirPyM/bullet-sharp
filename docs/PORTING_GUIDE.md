# Bullet → C# Porting Guidelines

This document is the authoritative reference for all developers working on this repository.

This repository is a pure C# rewrite of Bullet Physics (C++), with a public API
compatible with the original BulletSharp project. The C++ reference source is
located in a local checkout of the bullet3 repository (master branch, e.g. `../bullet3`).

## Precision (btScalar)

- `src/BulletSharp/GlobalUsings.cs` (and `src/BulletSharp.SoftBody/GlobalUsings.cs`) defines
  the global alias `btScalar`:
  - Single-precision projects (`BulletSharp`, `BulletSharp.SoftBody` → NuGet `BulletSharp.PureCSharp`,
    `BulletSharp.PureCSharp.SoftBody`): `btScalar` is `System.Single`.
  - Double-precision projects (`BulletSharp.Double`, `BulletSharp.SoftBody.Double` → NuGet
    `BulletSharp.PureCSharp.Double`, `BulletSharp.PureCSharp.SoftBody.Double`): compile the same
    source files with `BT_USE_DOUBLE_PRECISION` defined, changing `btScalar` to `System.Double`.
  - Tests and benchmarks accept `-p:Precision=double` to define `BT_USE_DOUBLE_PRECISION` and
    switch their `ProjectReference`s to the `.Double` projects.
- **Do not** use `MathF`/`System.Math` directly on `btScalar` values. Always use
  `BulletSharp.Math.BtMath` instead
  (Sqrt/Sin/Cos/Tan/Asin/Acos/Atan/Atan2/Abs/Floor/Ceil/Pow/Min/Max/Clamp/Fsel/PlaneSpace1).
- Use the constants `BtMath.BtEpsilon / SimdEpsilon / BtLargeFloat / SimdInfinity / Pi / SimdHalfPi / ConvexDistanceMargin`.
- When a floating-point literal requires conversion, write `(btScalar)0.5`, or write
  the value directly (int literals convert implicitly).
- C++ `#if BT_USE_DOUBLE_PRECISION` branches are expressed as C#
  `#if BT_USE_DOUBLE_PRECISION` directives.

## Math Types

`BulletSharp.Math` namespace:
- `Vector3` (public, `btScalar` components): port of btVector3.
  Dot/Cross/Triple/Normalize/Normalized/Length/LengthSquared/Rotate/Absolute/
  MinAxis/MaxAxis/Lerp/SetMin/SetMax and all operators are implemented.
- `Quaternion` (public): port of btQuaternion.
- `Matrix` (public): **SharpDX layout**, with translation in M41..M43 and
  row-vector left multiplication `v*M`.
  `Matrix a*b` means apply `a` first, then `b`. Consistent with the original
  `BulletSharp.Math.Matrix`.
- `Matrix3x3` (**internal**): port of btMatrix3x3, row-major (Row0/1/2 are the
  basis vectors).
- `Transform` (**internal**): port of btTransform (Basis + Origin).
  `t1*t2` = apply `t2` first. `Transform.FromMatrix/ToMatrix` convert to and
  from the public `Matrix`.

## SIMD (port of Bullet's BT_USE_SSE paths)

The SSE code paths of `btVector3.h` / `btQuaternion.h` are ported in
`Math/BtSimd.cs` on top of `System.Runtime.Intrinsics` (`Sse`), following the
same structure as the C++ (see the mapping table in the file header). Rules:

- **Layout**: `Vector3` carries an unused fourth field `W` (btVector3's
  `m_floats[3]`), always kept **0**. In float builds the struct is exactly 16
  bytes and is reinterpreted as `Vector128<float>` via `Unsafe.As` — the C#
  analogue of Bullet's `union { btSimdFloat4 mVec128; btScalar m_floats[4]; }`
  under `ATTRIBUTE_ALIGNED16`. Do **not** write code that lets `W` become
  nonzero; equality/hashing intentionally ignore it.
- **Availability**: like Bullet (`BT_USE_SSE` requires single precision and
  x86/x64), SIMD paths exist only in float builds (`#if !BT_USE_DOUBLE_PRECISION`)
  and check `Sse.IsSupported` at runtime; the scalar fallback is the `#else`
  branch of the C++.
- **Bit-exact by default**: only operations that produce bitwise-identical
  results to the scalar code (same IEEE rounding order) run through SIMD
  without a switch: `+ - * /`, `Dot`, `Cross`, `Triple`, `Lerp`, `Absolute`,
  `SetMin/SetMax`, `SetZero`, quaternion add/sub/negate/scale/inverse. This is
  what keeps the differential fuzz tests (`tests/native-ref/`) bit-identical
  to the native scalar reference.
- **`BT_USE_SSE_IN_API`** (build switch `-p:SimdApi=true`, float builds only,
  default **false** — the same default Bullet uses on Windows): enables the
  paths that change results in the last ulp, exactly like Bullet's own opt-in:
  `Vector3.Normalize` (rsqrt estimate + one Newton-Raphson step), the SSE
  quaternion product/dot/normalize. With this switch the chaotic smoke
  scenarios (S7/S8) can land on a different trajectory — that is expected,
  native Bullet behaves the same when built with `BT_USE_SSE_IN_API`.
- When porting new math code from the C++ `#if defined(BT_USE_SSE...)`
  sections, keep the same rule: bit-exact variants go in unconditionally
  (float-only, `Sse.IsSupported`-guarded), everything else behind
  `BT_USE_SSE_IN_API`.

## Naming Conventions

- internal fields: same name as the C++ member but camelCase
  (e.g. `m_linearVelocity` → `linearVelocity`).
- public API: identical to the original BulletSharp (PascalCase
  properties/methods, e.g. `LinearVelocity`).
- Internal engine methods keep their bt naming
  (e.g. `LocalGetSupportingVertexWithoutMargin`).
- C++ pointer/reference parameters → C# class references or `in`/`ref`/`out`.
- `btAlignedObjectArray<T>` → `List<T>`; object pools use `List<T>` plus a
  counter for reuse.

## Completed Foundation Layer (do not modify; free to use)

- `Enums.cs`: BroadphaseNativeType, CollisionFlags, ActivationState, CollisionFilterGroups,
  CollisionObjectTypes, DispatchFunc, DispatcherFlags, ConstraintParams, TypedConstraintType,
  SolverModes, ContactPointFlags.
- `Collision/AabbUtil.cs`: TransformAabb×2, TestAabbAgainstAabb2, TestTriangleAgainstAabb2,
  Outcode, RayAabb2, RayAabb, Dot3.
- `Collision/NarrowPhase/ManifoldPoint.cs`: ManifoldPoint (class, camelCase internal fields, CopyFrom).
- `Collision/NarrowPhase/PersistentManifold.cs`: PersistentManifold (4-point cache, complete port).
- `Collision/NarrowPhase/ConvexCast.cs`: CastResult, ConvexCast abstract class.
- `Collision/CollisionDispatch/`: CollisionObject, CollisionObjectWrapper, ManifoldResult,
  DispatcherInfo, CollisionAlgorithm, Dispatcher (including DefaultNearCallback/DispatchAllCollisionPairs),
  CollisionDispatcher (including the double-dispatch func table), CollisionConfiguration,
  RaycastCallbacks (RayResultCallback/Closest/AllHits/ConvexResultCallback/Closest/ContactResultCallback/
  LocalRayResult/LocalConvexResult/LocalShapeInfo/TriangleRaycastFlags), CollisionWorld
  (AddCollisionObject/UpdateAabbs/PerformDiscreteCollisionDetection/RayTest/ConvexSweepTest/ContactTest, etc.).
- `Collision/Broadphase/`: BroadphaseProxy (uid, IsConvex and other static predicates), BroadphasePair,
  BroadphaseInterface (abstract), OverlappingPairCache (abstract), HashedOverlappingPairCache,
  OverlappingPairCallback, BroadphaseAabbCallback, BroadphaseRayCallback.
- `Collision/CollisionShapes/CollisionShape.cs`: CollisionShape/ConvexShape/ConvexInternalShape/
  ConcaveShape/PolyhedralConvexShape base classes + TriangleCallback/InternalTriangleIndexCallback.
- `Dynamics/`: MotionState/DefaultMotionState, DynamicsWorld (abstract), TypedConstraint (abstract,
  including ConstraintInfo1/ConstraintInfo2/JointFeedback), ConstraintSolver (abstract), ContactSolverInfo,
  JacobianEntry (internal).
- `IDebugDraw.cs`: IDebugDraw + DebugDrawModes.

## Key Semantic Conventions

1. **CollisionAlgorithm and manifolds**: in an algorithm's CreateFunc, if `ci.Manifold == null`,
   call `ci.Dispatcher.GetNewManifold(body0, body1)` to create and own the manifold
   (`ownManifold = true`); at the start of `ProcessCollision`, call
   `resultOut.SetPersistentManifold(manifold)`; at the end,
   `if (ownManifold) resultOut.RefreshContactPoints()`.
   Override `Cleanup(Dispatcher)`: if ownManifold and manifold != null →
   `dispatcher.ReleaseManifold(manifold)`.
2. **CollisionObjectWrapper**: the internal ctor takes a `Transform`; the wrapper chain is used
   for compound/concave child shapes.
3. **ManifoldResult** construction: for discrete dispatch, use
   `new ManifoldResult(obj0Wrap, obj1Wrap)`.
4. Solver/constraint: `GetInfo1(ConstraintInfo1, ContactSolverInfo)` declares the row count;
   `GetInfo2` writes J rows into the arrays of `ConstraintInfo2` (the index of component s of
   row r is `r*rowskip + s`).
5. `RigidBody` is implemented by Agent D; `TypedConstraint.GetFixedBody()` already provides the
   fixed body.
6. Dispose: all implementations are either empty or perform resource cleanup (no native
   memory); the interface is retained for compatibility.

## Module Ownership (directory boundaries must not be crossed)

| Module | Directory | Contents |
|---|---|---|
| A Shapes | `src/BulletSharp/Collision/CollisionShapes/` | All concrete shapes + meshes/BVH |
| B Narrow phase | `src/BulletSharp/Collision/NarrowPhase/`, `Collision/CollisionDispatch/Algorithms/` | GJK/EPA, collision algorithms |
| C Broadphase | `src/BulletSharp/Collision/Broadphase/` (new files) | Dbvt, DbvtBroadphase, SimpleBroadphase |
| D Dynamics | `src/BulletSharp/Dynamics/` (new files) | RigidBody, world, solvers, constraints |

Rule: **Do not modify files belonging to other modules or to the foundation layer.** If you
find the foundation layer is missing something or contains an error, append a note to
`docs/INTEGRATION_NOTES.md` after completing your own module (module name + issue +
suggestion); the integrator will handle all such issues centrally.

## Building

Modules are currently being developed in parallel, so the project as a whole does not yet
compile (modules reference types that are not yet finished).
Requirements: the files within your own module must be self-consistent and strictly follow
the contracts above; compile errors across modules will be fixed during the integration phase.
