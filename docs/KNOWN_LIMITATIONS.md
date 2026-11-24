# Known Limitations & Porting Notes

This document records the functional differences, simplifications, and design
decisions of this port relative to C++ Bullet. Classic soft-body simulation
(`BulletSoftBody` / `btSoftBody` full pipeline: ropes, cloth, aero,
pressure/volume, tetrahedra, anchors, clusters, `SparseSdf`,
`SoftRigidDynamicsWorld`, and `SoftBodyHelpers`) is ported in
`src/BulletSharp.SoftBody/`. Intentionally omitted: MultiBody/Featherstone, the
MultiBody-dependent Deformable FEM framework (`btDeformable*`, Krylov/CG/CR
solvers, `ReducedDeformableBody`, and `btSoftMultiBodyDynamicsWorld`), GPU
OpenCL/DX11 soft-body solvers (CPU `SoftBodySolver` + `DefaultSoftBodySolver`
are ported), MLCP solvers, GImpact, KinematicCharacterController, binary
serialization (including `btSoftBodyData`), and the SIMD array helpers
`btVector3::maxDot/minDot` (their C++ call sites use the scalar reference
implementations). The regular per-vector SSE paths are ported — see "SIMD" in
`docs/PORTING_GUIDE.md`.

The following are per-module notes from the porting process (organized by module,
including records of resolved issues):

> Issues already resolved during integration:
> - Module A-9 (accessibility of `ConvexShape.Project`): resolved by making
>   `Transform`/`Matrix3x3`/`JacobianEntry` public.
> - Module D2-2 (±SimdInfinity presets): the solver's `ConvertJoint` has been
>   changed to allocate the `constraintError/cfm/lowerLimit/upperLimit` arrays
>   with a stride of `rowskip`, matching C++ semantics.

---

## Numerical Parity with C++ Bullet (differential validation)

The port is validated bit-exactly against a native g++ build of bullet3
(scalar, no SSE) using per-step differential traces. Infrastructure:

- `tests/native-ref/`: native mirror programs (`FuzzRef.cpp`, `TraceRef.cpp`,
  `S7Ref.cpp`, `SoftRef.cpp`) plus instrumented drop-in replacements of the C++ sources
  (`SISolverDump.cpp`, `BoxBoxDump.cpp`, `DbvtBpDump.cpp` — built with the
  original files excluded), and the numeric compare scripts `cmp.ps1` /
  `firstdiff.ps1` (normalize the differing exponent-digit count of `%.8e` vs
  C# `E8`).
- In-library gated instrumentation (all default **off**, zero output): 
  `SequentialImpulseConstraintSolver.SolverDump` (solver rows/impulses,
  counterpart of `SISolverDump.cpp`), `BoxBoxDetector.Dump` (dBoxBox SAT
  internals, counterpart of `BoxBoxDump.cpp`), `DbvtBroadphase.PairDump`
  (broadphase pair lifecycle, counterpart of `DbvtBpDump.cpp`). `tests/
  BulletSharp.SmokeTests/Fuzz.cs` enables them from environment variables
  (`FUZZ_DUMP_STEP`, `DBOX_DUMP`, `DBP_DUMP`).

Validation status after the fixes below: the constraint+solver path
(`trace` mode, 30/200/1000 steps) and the box-box collision path
(`fuzz` mode, seeds 1/2/3/7/42, both `trace` and collision-only `pre`)
match the native build bit-exactly (max abs diff 0 with tolerance 1e-300).

Numeric-parity fixes applied during validation:

1. `Matrix3x3.Scaled` scaled columns instead of rows.
2. `BtMath.Atan2` is now the exact btAtan2 port (see Module D2 note 3).
3. `Quaternion.GetAngle` returns `2*Acos(W)` instead of `Acos(W)`.
4. `Matrix3x3.GetRotation` uses the exact C++ form.
5. `quickSort` is a faithful port of ODE-style Hoare-partition quicksort
   with median pivot (`src/BulletSharp/BtQuickSort.cs`), and all sort call
   sites were switched to it; `sortOverlappingPairs` also had the wrong
   direction (must be descending by uid, like `btBroadphasePairSortPredicate`).
6. `TypedConstraint.breakingImpulseThreshold` default is `SimdInfinity`
   (was 1e18, which clamped joint rows through `BreakingImpulseThreshold`).
7. `CollisionShape.SetSafeMargin` now dispatches through the virtual
   `Margin` setter (previously it wrote the field directly, bypassing the
   margin-compensation overrides in BoxShape/CylinderShape — this changed
   every convex shape's effective half extents, e.g. `BoxShape(0.3).HalfExtentsWithMargin`
   was 0.29 instead of 0.30).
8. `BoxBoxDetector` edge-axis wins now clear `normalR` (C++ TST macro sets
   `normalR = 0`): previously a face-test winner from codes 1–6 could leak
   into a later edge win (codes 7–15) and produce a wrong contact normal.

Note on the S7 smoke test: a fully-locked 6dof pair teleported on one side
conserves center of mass and spacing but its connecting axis slowly rotates;
this is native Bullet behavior (verified with `S7Ref.cpp`, bit-identical
positions/quaternions), so that test asserts the conserved invariants.

---

## Module B2 (Narrow Phase, Remainder)

Owned files (`src/BulletSharp/Collision/CollisionDispatch/Algorithms/`):
`HashedSimplePairCache.cs`, `ConvexConcaveCollisionAlgorithm.cs`,
`CompoundCollisionAlgorithm.cs`, `CompoundCompoundCollisionAlgorithm.cs`.

### Dependency Assumptions on Module A (CompoundShape)

Referenced according to the given contract; CompoundShape must provide (the same
usage already exists in CollisionWorld.cs, assumed to be consistent):

- `int NumChildren`
- `CollisionShape GetChildShape(int i)`
- `Transform GetChildTransformInternal(int i)` (internal; return by value is fine)
- `int UpdateRevision` (corresponds to `btCompoundShape::getUpdateRevision()`;
  incremented by `addChildShape`/`removeChildShape`).
  **If Module A does not provide this member, it must be added**; otherwise the
  revision tracking in Compound(Compound)CollisionAlgorithm will not compile.

### Simplifications / Deviations from C++ master

1. **The btDbvt dynamic AABB tree path is not ported**: the tree pruning via
   `getDynamicAabbTree()` in btCompoundCollisionAlgorithm / btCompoundCompoundCollisionAlgorithm
   (including the `MycollideTT` tree-versus-tree traversal) depends on btDbvt and
   is outside the CompoundShape contract. It is replaced by "iterate over all child
   shapes" and an "O(n0*n1) leaf-pair double loop", respectively — semantically
   equivalent to the no-tree fallback path in C++ (AABB culling still happens inside
   the callback). If Module C/A later provides a Dbvt tree for CompoundShape, the
   tree traversal acceleration can be restored.
2. **HashedSimplePairCache is keyed by (int indexA, int indexB)** (faithful to
   btHashedSimplePairCache/btSimplePair, where userPointer stores the child
   CollisionAlgorithm), implemented with `List<SimplePair> + Dictionary<long,int>`
   (same approach as the HashedOverlappingPairCache port). The proxy-based methods
   mentioned in the task description (CleanProxyFromPairs/NeedsBroadphaseCollision,
   etc.) belong to btHashedOverlappingPairCache; btHashedSimplePairCache never had
   these methods, so they are not implemented.
3. **CompoundCollisionAlgorithm caches child algorithms in a HashedSimplePairCache**
   (key (childIndex, 0)), implemented per the field conventions in the task
   description; C++ master uses a `btAlignedObjectArray<btCollisionAlgorithm*>`
   array (preallocated, direct indexing). The two are functionally equivalent; if
   full parity with master is desired during integration, it can be changed to an
   array.
4. **`resultOut->m_closestPointDistanceThreshold` is treated as 0** (the
   foundation-layer ManifoldResult has no such field, consistent with the first
   batch of Module B): all `BT_CLOSEST_POINT_ALGORITHMS` branches never trigger,
   and execution always takes the contact-point algorithm + cache-reuse branch.
5. **The SDF branch of ConvexConcaveCollisionAlgorithm** (btSdfCollisionShape::queryPoint)
   and the `BT_ENABLE_CONVEX_CONCAVE_EARLY_OUT` early-out optimization are not
   ported; this is equivalent to the default C++ build configuration, which takes
   the regular concave ProcessAllTriangles branch.
6. **The wrapper's preTransform**: C++ master's btCompoundLeafCallback composes
   `m_compoundColObjWrap->m_preTransform`; the C# CollisionObjectWrapper has no
   such member, so this is skipped (the wrappers ported in the first batch never
   had this concept).
7. In C++, the btCompoundCompoundLeafCallback constructor's initializer list has
   the compound0/compound1 parameters swapped (cancelled out by the call-site
   order); the C# version assigns them with the correct semantics directly.
8. `ManifoldResult.setBody0Wrap/setShapeIdentifiersA/B` have no corresponding
   methods in the foundation layer; the internal fields (`body0Wrap`/`partId0`/
   `index0`, etc.) are written directly, which is legal within the same assembly.
9. The global callbacks `gCompoundChildShapePairCallback` /
   `gCompoundCompoundChildShapePairCallback` are ported as
   `CompoundCollisionAlgorithm.CompoundChildShapePairCallback` /
   `CompoundCompoundCollisionAlgorithm.CompoundCompoundChildShapePairCallback`
   (the `ShapePairCallback` delegate, defined in CompoundCollisionAlgorithm.cs).

## Module A (Shapes, Second Batch)

New files (`src/BulletSharp/Collision/CollisionShapes/`):
`StaticPlaneShape.cs`, `ConvexHullShape.cs`, `UniformScalingShape.cs`, `MultiSphereShape.cs`,
`CompoundShape.cs` (including `CompoundShapeChild`), `EmptyShape.cs`, `ConvexPointCloudShape.cs`,
`StridingMeshInterface.cs` (including the `PhyScalarType` enum), `IndexedMesh.cs`, `TriangleMesh.cs`,
`TriangleIndexVertexArray.cs`, `TriangleMeshShape.cs`, `QuantizedBvh.cs` (including `NodeOverlapCallback`,
`TraversalMode`), `OptimizedBvh.cs`, `BvhTriangleMeshShape.cs`, `ScaledBvhTriangleMeshShape.cs`,
`ConvexTriangleMeshShape.cs`, `HeightfieldTerrainShape.cs`.

### Issues / Assumptions / Simplifications

1. **No dedicated AABB-caching base classes**: C++'s `btConvexInternalAabbCachingShape` /
   `btPolyhedralConvexAabbCachingShape` were not ported as intermediate base classes
   (the foundation-layer CollisionShape.cs cannot be modified); their recalcLocalAabb /
   cache fields are inlined directly into the four classes ConvexHullShape /
   ConvexPointCloudShape / ConvexTriangleMeshShape / MultiSphereShape (line-by-line
   corresponding logic, all including margin expansion).
2. **`StridingMeshInterface.GetTriangle(int subPart, int triangleIndex, Vector3[])`**
   is a new internal abstract helper method (replacing C++'s
   getLockedReadOnlyVertexIndexBase locking semantics), used by QuantizedBvh.UpdateBvhNodes
   and by BvhTriangleMeshShape's MyNodeOverlapCallback to fetch triangles.
   **Note**: StridingMeshInterface cannot be derived from outside the assembly
   (internal abstract members cannot be implemented externally); if a user-defined
   mesh interface is needed, the integrator can change it to protected internal /
   public abstract.
3. **IndexedMesh stride semantics**: C# managed arrays have no byte stride, so
   `VertexStride` means "number of components per vertex" (default 3), corresponding
   to C++ `m_vertexStride = 3*sizeof(btScalar)`; indices are fixed as `int[]`
   (PHY_INTEGER), and vertices are fixed as `btScalar[]`. TriangleMesh corresponds
   exactly to the C++ storage layout with use32bitIndices=true +
   use4componentVertices=false.
4. **CompoundShape**: `CompoundShapeChild` is a class (consistent with the original
   BulletSharp), with an internal `Transform transform` field + a public
   `Matrix ChildTransform` accessor; the dynamic aabb tree is not implemented (the
   ctor's enableDynamicAabbTree parameter is retained but ignored), and
   `CreateAabbTreeFromChildren`/`GetDynamicAabbTree` are not provided;
   `CalculatePrincipalAxisTransform` is not ported (it depends on
   Matrix3x3.Diagonalize). The NumChildren required by Module B2 is provided as
   `NumChildShapes`; `GetChildTransformInternal(int)` (internal) and
   `UpdateRevision` are both provided.
5. **QuantizedBvh**: faithful port of buildTree/calcSplittingAxis/sortAndCalcSplittingIndex/
   quantization (setQuantizationValues/quantize/unQuantize)/the three traversal modes
   (stackless, cache-friendly, recursive) and ray traversal (RAYAABB2 path);
   serialization is not included. `Refit`/`RefitPartial`/`UpdateBvhNodes` have also
   been ported (using the GetTriangle helper above). Nodes are structs with 6 ushort
   fields (value semantics, avoiding unsafe fixed-size arrays).
   `OptimizedBvh.Build(...)` is a `new static` static factory (hiding the inherited
   instance `QuantizedBvh.Build`); internally, BvhTriangleMeshShape calls the
   instance Build via a `(QuantizedBvh)` cast.
6. **BvhTriangleMeshShape.ProcessAllTriangles / PerformRaycast**: ported according
   to btBvhTriangleMeshShape.cpp in C++ master — the callback receives triangles in
   **mesh-local space** (already scaled by the mesh Scaling), with no world
   transform (the MyProcessedTriangleCallback/world transform from the task
   description no longer exists in C++ master; the transform is the caller-side
   algorithm's responsibility). `PerformRaycast` follows Module B's
   `TriangleRaycastCallback` signature. `PerformConvexcast` has also been ported.
7. **HeightfieldTerrainShape**: the `PhyScalarType` enum values keep the PHY_*
   naming as required; `heightfieldData` supports float[]/double[]/int[]/short[]/byte[];
   PHY_INTEGER(int[]) is unimplemented in C++'s getRawHeightFieldValue (falls into
   default btAssert), but this port supports it in the same way as short
   (value × heightScale); PHY_FIXEDPOINT88 is not supported (throws
   NotSupportedException).
   **performRaycast (gridRaycast) and buildAccelerator/clearAccelerator (the vbounds
   acceleration grid) are not ported (left as TODO)**.
8. **ConvexHullShape.OptimizeConvexHull / the InitializePolyhedralFeatures of the
   various shapes**: depend on btConvexHullComputer and are not implemented
   (OptimizeConvexHull is an empty method; InitializePolyhedralFeatures throws
   NotSupportedException, consistent with the first batch's BoxShape).
9. **ConvexHullShape.Project is temporarily internal**: the foundation layer's
   `ConvexShape.Project` is itself public but takes an internal `Transform`
   parameter (CS0051, a pre-existing foundation-layer issue); this class's Project
   has the same origin. It can be restored to public once the integrator resolves
   this uniformly (the recommendation is to make Transform public or make these
   methods internal).

## Module D2 (Constraints)

New files (`src/BulletSharp/Dynamics/ConstraintSolver/`):
`ConstraintMath.cs` (shared helpers: NormalizeAngle/AdjustAngleToLimits/GetMotorFactor/FuzzyZero/
WriteSkewSymmetric/CreateAngularJacobianEntry), `HingeConstraint.cs` (including the internal
`AngularLimit` class), `Generic6DofConstraint.cs` (including `RotationalLimitMotor`/
`TranslationalLimitMotor`), `Generic6DofSpringConstraint.cs`, `ConeTwistConstraint.cs`,
`SliderConstraint.cs`, `FixedConstraint.cs`, `GearConstraint.cs`.

### Issues / Assumptions / Simplifications

1. **FixedConstraint does not inherit Generic6DofSpring2Constraint**: C++'s
   btFixedConstraint inherits btGeneric6DofSpring2Constraint (outside the scope of
   this batch, and the foundation layer cannot be modified), so this port inherits
   TypedConstraint directly and implements the 6-row lock itself: 3 linear rows
   (p2p-style, per-axis) + 3 angular rows (basis-vector cross-product error,
   lock semantics equivalent to btFixedConstraint). If Generic6DofSpring2Constraint
   is ported later, the inheritance can be restored for full alignment with C++.
2. **±SimdInfinity row limits are written explicitly**: C++ relies on the solver
   prefilling lowerLimit/upperLimit with ±SIMD_INFINITY before entering getInfo2
   (the temporary array initialization in
   btSequentialImpulseConstraintSolver::convertJoint). This port explicitly writes
   ±SimdInfinity as a fallback for every row whose limits are unused/unset in each
   constraint's GetInfo2, because the C# SequentialImpulseConstraintSolver.
   ConvertJoint's temporary ConstraintInfo2 arrays are zero-initialized and the
   preset values are not copied back after solving — **a potential issue in Module
   D's first batch**. If the integrator adds C++'s preset initialization to
   ConvertJoint, this batch's behavior is unchanged (the explicit writes overwrite
   with equal values).
3. **BtMath.Atan2 is an exact port** (resolved): btAtan2 (the exact lookup-table +
   polynomial version) is ported in BtMath.Atan2; the former fast approximation
   (btAtan2Fast, maximum error ~1e-3 rad) is kept as `BtMath.Atan2Fast` and is used
   only by ConeTwistConstraint's `CorrectAngle` path, matching the C++ call sites
   (`btAtan2Fast` in btConeTwistConstraint.cpp).
4. **HingeConstraint uses the `_BT_USE_CENTER_LIMIT_` path** (AngularLimit manages
   lower/upper/halfRange, corresponding to the default C++ build configuration);
   `HINGE_USE_FRAME_OFFSET = true` (GetInfo2 takes the UsingFrameOffset path by
   default, consistent with C++ master).
5. **Not ported**: HingeAccumulatedAngleConstraint (btHingeAccumulatedAngleConstraint,
   not listed in the task description); all SolveConstraintObsolete legacy solver
   paths are not ported — only the BuildJacobian legacy path code is retained (for
   direct-integration scenarios that do not use a solver, consistent with
   Point2PointConstraint).
6. **Generic6DofConstraint adds a protected constructor taking TypedConstraintType**:
   because the base class TypedConstraint's constraintType is a readonly field,
   this allows Generic6DofSpringConstraint to pass in `TypedConstraintType.D6Spring`.
7. **Single-rigid-body constructors**: the single-rigid-body ctors of GearConstraint /
   SliderConstraint / Generic6DofConstraint are implemented as
   `base(type, GetFixedBody(), rbB)` (C++ passes getFixedBody()); GetFixedBody() is
   a static method provided by TypedConstraint.
8. **Public property names keep the original BulletSharp spelling as required**:
   e.g. `MotorTargetVelosity` (HingeConstraint, corresponding to the velocity
   parameter of C++ enableAngularMotor) — this is not a typo.

## Module S (SoftBody / BulletSoftBody)

New project (`src/BulletSharp.SoftBody/`, `BulletSharp.SoftBody` namespace):
`BulletSharp.SoftBody.csproj`, `GlobalUsings.cs`, `Poly34.cs`, `HullLibrary.cs`,
`SparseSdf.cs`, `SoftBodyEnums.cs`, `SoftBodyDataTypes.cs`, `SoftBodyWorldInfo.cs`,
`SoftBody.cs`, `SoftBody.Simulation.cs`, `SoftBody.Solvers.cs`,
`SoftBody.Collision.cs`, `SoftBody.Clusters.cs`, `SoftBodyInternals.cs`,
`SoftBodyCollisionShape.cs`, `SoftBodyCollisionPolicies.cs`, `SoftBodySolver.cs`,
`DefaultSoftBodySolver.cs`, `SoftBodyRigidBodyCollisionConfiguration.cs`,
`SoftRigidDynamicsWorld.cs`, `SoftBodyHelpers.cs`, and `Collision/`
(`SoftSoftCollisionAlgorithm.cs`, `SoftRigidCollisionAlgorithm.cs`,
`SoftBodyConcaveCollisionAlgorithm.cs`).

Foundation additions in `src/BulletSharp/`: `Matrix3x3.Diagonalize` (Jacobi
symmetric 3×3 eigensolver), `PolarDecomposition` ($M = UH$ for cluster shape
matching), and virtual `CollisionWorld.RayTest` / `RayTestSingle` +
`DebugDrawWorld` hooks (preserving bit-exact behavior on existing rigid-body
code paths).

### Scope: Ported vs. Intentionally Omitted

- **Ported (classic `btSoftBody` pipeline, ~12,900 C++ LOC)**: ropes, cloth
  patches, closed pressurized/volume-conserving meshes, tetrahedral meshes,
  aerodynamic lift/drag, rigid-body anchors, position/velocity/drift solvers,
  cluster shape matching with GJK/EPA cluster collision and `LJoint`/`AJoint`/
  `CJoint` joints, `btSparseSdf<3>` hashed voxel SDF cache, soft-soft /
  soft-convex / soft-concave narrowphase algorithms, `SoftRigidDynamicsWorld`,
  `DefaultSoftBodySolver`, and `SoftBodyHelpers` (`CreateRope`, `CreatePatch`,
  `CreatePatchUV`, `CalculateUV`, `CreateFromTriMesh`, `CreateFromConvexHull`,
  `CreateEllipsoid`, `CreateFromTetGenData`, `CreateFromVtkFile`,
  `ReoptimizeLinkOrder`, barycentric weight helpers, `GenerateBoundaryFaces`,
  and `IDebugDraw` visualization).
- **Omitted — Deformable FEM framework (~6,500+ C++ LOC)**: `btDeformable*`,
  `DeformableBodyInplaceSolverIslandCallback`, Krylov solvers (`btKrylovSolver`,
  `btConjugateGradient`, `btConjugateResidual`, `btCGProjection`,
  `btPreconditioner`), `btImplicitQRSVD`, and the `Deformable*Contact` /
  `CollideSDF_RD` / `CollideSDF_RDF` / `CollideVF_DD` branches in `btSoftBody`
  and `btSoftBodyInternals`. These depend on the unported Featherstone
  `MultiBody` module (`btMultiBodyLinkCollider`, `btMultiBodyDynamicsWorld`).
- **Omitted — `BulletReducedDeformableBody/*` and `btSoftMultiBodyDynamicsWorld`**:
  likewise built on Featherstone `MultiBody`.
- **Omitted — Binary serialization (`btSoftBodyData.h` / `serialize`)**:
  omitted repository-wide.
- **Omitted — GPU OpenCL / DX11 soft-body solvers**: only the `SoftBodySolver`
  abstraction, CPU vertex buffer descriptors, and `DefaultSoftBodySolver` are
  included.

### Architecture & Porting Design Notes

1. **Separate assembly with matched `Precision`**: `BulletSharp.SoftBody` builds
   into its own `BulletSharp.SoftBody.dll` via `ProjectReference` to
   `BulletSharp.csproj`, which grants `InternalsVisibleTo("BulletSharp.SoftBody")`
   (exposing internal hooks such as `CollisionObject.internalType` and
   `DefaultCollisionConfiguration.GetCollisionAlgorithmCreateFunc`). Both
   projects define the `btScalar` global alias from the same MSBuild `Precision`
   property (`float` by default, `-p:Precision=double` for `double`) and must be
   built with matching precision. All scalar math goes through `BtMath.*`.
2. **Reference semantics for topology vs. value semantics for transient data**:
   Persistent topology and solver elements (`Element`, `Material`, `Feature`,
   `Node`, `Link`, `Face`, `Tetra`, `Anchor`, `Note`, `Cluster`, `Joint`) are
   C# `class`es stored in `List<T>`, faithfully modeling C++ raw cross-pointers
   (`Link.m_n[2]`, `Face.m_n[3]`, `Tetra.m_n[4]`, `Anchor.m_node`, and
   `DbvtNode.Data` references in `ndbvt`/`fdbvt`/`cdbvt`) without list-resize
   invalidation or boxing. Small transient structures (`sCti`, `sMedium`,
   `sRayCast`, `Impulse`, `SymMatrix<T>`) are `struct`s, and per-step contact
   lists (`m_rcontacts`, `m_scontacts`) retain backing capacity / pool entries
   so steady-state simulation steps avoid GC allocations.
3. **Per-instance deterministic PRNG seed**: C++ `btSoftBody.h` uses a
   function-local `static unsigned long seed = 243703` (32-bit unsigned LCG on
   Windows MSVC). The C# port stores `uint randSeed = 243703u` per `SoftBody`
   instance (`unchecked { randSeed = 1664525u * randSeed + 1013904223u; }`),
   preserving bit-exact parity with Windows native single-body reference runs
   while preventing static state pollution across multiple worlds/bodies.
4. **Deterministic sorting**: All ordering operations use `BtQuickSort.QuickSort`
   (never `List<T>.Sort`), matching `btAlignedObjectArray::quickSort`.
5. **Validation**: Verified via deterministic smoke tests in
   `tests/BulletSharp.SmokeTests/` (`float` and `double`), native C++
   differential traces in `tests/native-ref/SoftRef.cpp` (rope-anchor,
   cloth–rigid SDF collision, and cluster GJK/EPA collision compared via
   `cmp.ps1` / `firstdiff.ps1`), and steady-state allocation benchmarks in
   `benchmarks/BulletSharp.SoftBody.Benchmarks/` (`[MemoryDiagnoser]`).
