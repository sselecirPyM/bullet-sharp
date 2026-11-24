# Benchmarks: BulletSharp (pure C#) vs Bullet C++ (bullet3)

Single-threaded performance comparison. Both sides run **identical** scenarios: same body
counts, initial positions, step counts, ray counts, and random seeds.

## Scenarios

| Scenario | Description | Main workload |
|---|---|---|
| `boxpile` | 1000 dynamic boxes (10×10×10) falling onto the ground and stacking, 600 steps @ 60Hz | Narrow-phase collision detection + sequential impulse solver |
| `raycast` | 1000 static boxes + ground, 50000 closest-ray queries (deterministic LCG distribution) | Dbvt broadphase + ray traversal |

The world configuration is identical on both sides: `DbvtBroadphase` +
`DefaultCollisionConfiguration` + `CollisionDispatcher` +
`SequentialImpulseConstraintSolver`, gravity (0, -9.81, 0), `stepSimulation(1/60, 1, 1/60)`.
Bullet is single-threaded by default (no task scheduler enabled).

## Build & Run

### C++ (official default bullet3 build)

```bat
cd benchmarks\native
build.bat        # CMake-builds BulletDynamics/BulletCollision/LinearMath static libraries (Release), then compiles benchmark.exe
benchmark.exe
```

Notes:
- Uses the official bullet3 CMake default compile options (`/O2`, SSE enabled,
  `BT_USE_SSE_IN_API` left at its default off).
- Explicitly sets `USE_MSVC_FAST_FLOATINGPOINT=OFF` (`/fp:precise`) to align with .NET's
  IEEE floating-point semantics and avoid an unfair advantage from `/fp:fast`.
- Does not use the amalgamated sources (`btBullet*All.cpp`) — `btVector3.cpp` defines
  `BT_USE_SSE_IN_API` inside the file, and amalgamated compilation would make the macro
  state inconsistent and fail to compile.

### C# (BulletSharp)

```bash
dotnet build -c Release benchmarks/BulletSharp.Benchmarks
dotnet benchmarks/BulletSharp.Benchmarks/bin/Release/net8.0/BulletSharp.Benchmarks.dll
```

Double-precision comparison: add `-DUSE_DOUBLE_PRECISION=ON` when reconfiguring CMake on
the C++ side; add `-p:Precision=double` on the C# side.

## Results (float, Release, measured 2026-11-24, best of 3 runs each)

Measured on a 13th Gen Intel Core i7-13650HX (Windows x64), `.NET 8.0.34` (X64 RyuJIT
`x86-64-v3`) vs MSVC `/O2 /fp:precise`. Both sides run an identical JIT/OS warmup before the
timed region (C++ is precompiled at `/O2`; .NET tiered compilation needs ~100–150 ms of hot
execution before the hot path is promoted to optimized code, so the C# benchmark warms up
60 steps / 3 ray passes untimed and the C++ benchmark runs the same 60-step segment to keep
the measured physics identical).

| Scenario | C++ (bullet3) | C# (BulletSharp) | C#/C++ | C# Alloc |
|---|---|---|---|---|
| boxpile | 493.3 ms (0.822 ms/step) | 798.9 ms (1.332 ms/step) | ~1.6× | 3.9 KB/step |
| raycast | 47.0 ms (0.940 µs/ray) | 39.6 ms (0.792 µs/ray) | ~0.84× | **0 B/ray** |

The `raycast` hit counts match on both sides (`hits=50000`, with steady-state C# ~16% faster
than native C++ at zero managed allocation); behavioral correctness is cross-validated.

### 2026-10-10 optimization round: JIT tiering + array bounds checks

Investigation of "why is the C# side much slower than native" found two separate effects:

1. **Tiered-compilation transient (dominant on raycast).** Under default .NET 8 tiering,
   hot methods that are too large to inline and whose internal loops run only a few
   iterations per call (e.g. `SubSimplexConvexCast.calcTimeOfImpact`,
   `CollisionWorld.rayTestSingle`) stay at unoptimized *Instrumented Tier0* until the
   tier-up machinery promotes them, which empirically takes ~100–150 ms of hot execution
   (`DOTNET_TieredCompilation=0` runs measured ~2× faster on raycast). The original
   50 000-ray benchmark (~150 ms total) measured almost exclusively this transient.
   Fixes:
   - `[MethodImpl(AggressiveOptimization)]` on the convex-cast hot path
     (`SubSimplexConvexCast`/`GjkConvexCast.CalcTimeOfImpact`,
     `CollisionWorld.RayTestSingleInternal`), which also protects short-burst raycasts
     right after startup;
   - proper JIT warmup in the benchmark (see above). Steady-state tiered execution
     (with dynamic PGO) is slightly *faster* than non-tiered.

2. **Array bounds checks the JIT cannot eliminate** (dominant on boxpile):
   - `BtQuickSort.QuickSortInternal` was generic over `IList<T>` — every element access
     was an interface dispatch with an un-eliminable bounds check and every swap went
     through the shared-generic array-store helper (`CastHelpers.StelemRef` /
     `CORINFO_HELP_ARRADDR_ST`); it cost ~11% of the whole boxpile run (island, manifold
     and pair sorting run `quickSort` every step). Rewritten over `Span<T>`
     (`CollectionsMarshal.AsSpan`) keeping the exact btAlignedObjectArray quickSort
     algorithm (identical permutation of equal keys — verified bit-identical against the
     native fuzz references).
   - `AabbUtil.RayAabb2` indexed `ReadOnlySpan<int> signs` (constant indices) and
     `ReadOnlySpan<Vector3> bounds` (variable `bounds[sign[i]]`), leaving ~10 un-eliminated
     bounds checks per broadphase node visit. Rewritten with scalar sign/bounds parameters
     (branch-free selects, inlinable).
   - `Dbvt.rayTestInternal` used `List<DbvtNode>` as the traversal stack (`List` indexer
     checks + `AddWithResize` + array-store helper on push). Replaced with a plain
     `DbvtNode[]` and manual doubling growth, matching the C++ `btAlignedObjectArray`.

Net effect on this machine (same methodology, before → after): boxpile 2.6× → ~1.7×,
raycast 3.2× → ~1.3×. Correctness: all smoke tests unchanged (the one pre-existing S7
failure is identical before/after), and the differential fuzz test matches native
`FuzzRef.exe` bit-for-bit across seeds.

## C# Allocation Optimizations (2026-10-03)

One round of eliminations targeting the "every step / every ray heap-allocates what would
be a C++ stack object" problem in the C# port. The benchmark output includes allocation
statistics (`GC.GetAllocatedBytesForCurrentThread`):

| Scenario | Before | After |
|---|---|---|
| boxpile | 2611 KB/step | 7.5 KB/step (further reduced to 3.9 KB/step as of 2026-11-24; includes pair add/remove during settling, ~83 B/step once fully settled) |
| raycast | 2103 B/ray | **0 B/ray** |

Main techniques (all preserving C++ semantics, assuming single-threaded world usage):

- **stackalloc / Span**: all ODE temp matrices in `BoxBoxDetector` (r1/r2/quad/ret/point/dep
  etc.; ping-pong buffers via `stackalloc` pointers to work around the CS8352 escape
  restriction), the bounds in `Dbvt.RayTestInternal`, the quantized query array and sign
  in `QuantizedBvh`, and the guessVectors in `GjkEpaPenetrationDepthSolver`.
- **Per-instance cached reuse** (equivalents of C++ stack objects): `Dispatcher` caches
  `CollisionObjectWrapper×2` + `ManifoldResult` for near-callbacks (`ManifoldPoint` gets a
  full-reset `Set` and is used as scratch); `ConvexConvexAlgorithm` caches the GJK
  inputs/solver/detector (reused per collision pair); `BoxBoxCollisionAlgorithm` caches
  the `BoxBoxDetector`; `CollisionWorld` caches the raycast `CastResult`/`SphereShape`/
  `VoronoiSimplexSolver`/convex sweepers/`LocalRayResult`; `DbvtBroadphase` caches the
  `DbvtTreeCollider` and `BroadphaseRayTester`.
- **Object pooling**: `HashedOverlappingPairCache` pools `BroadphasePair`;
  `CollisionDispatcher` pools `PersistentManifold` and reuses `ConstructionInfo` (no more
  repeated allocation under pair churn); the solver's `ConvertJoint` pools
  `ConstraintInfo2` and its 8 temp arrays (cleared to zero-init semantics before reuse).
- **Misc**: `UnionFind` arrays only grow, never shrink (same semantics as C++);
  `ClosestPtPointTetrahedron` scratch uses local structs; `ClosestRayResultCallback.Reset`
  supports reusing the callback per ray (benchmark side).
- Pooling was verified with positional checksums: simulation trajectories are bit-identical
  before and after; all 12 smoke tests (including comparison against the C++ golden trace)
  pass.

Known remaining allocations: during settling, each **new** collision pair still allocates
an algorithm object (C++ likewise heap-allocates an algorithm per new pair; the GC reclaims
them on the C# side).

## SoftBody Benchmarks & Steady-State Allocation Check (`BulletSharp.SoftBody.Benchmarks`)

`benchmarks/BulletSharp.SoftBody.Benchmarks` uses **BenchmarkDotNet** with `[MemoryDiagnoser]`
to measure single-step soft-body simulation latency and verify that steady-state stepping performs
zero unexpected managed heap allocations.

### Scenarios (`ClothRigidBodyBenchmarks`)

| Benchmark | Description | Main workload | Steady-state `Allocated` |
|---|---|---|---|
| `ClothOnlyStep` | 9×9 pinned-corner cloth patch (81 nodes, 128 faces, bending constraints, `PIterations = 8`) settled under gravity | `PredictMotion` + `ndbvt` incremental update + PBD link/bending solver + normal update | **0 B/step** |
| `ClothRigidCollisionStep` | Same 9×9 cloth patch catching and supporting a dynamic `SphereShape(0.8)` (`mass = 0.3`) in equilibrium | Above + `SoftRigidCollisionAlgorithm` + `CollideSdfRs` + `SparseSdf.Evaluate` + `PSolve_RContacts` + `SparseSdf.GarbageCollect` | **0 B/step** |

Both worlds warm up for 1200 steps (`20 s` @ 60 Hz) in `[GlobalSetup]` so the cloth and sphere
reach steady-state equilibrium, active `SparseSdf` cells are populated, and transient falling-phase
cells have already expired past `SparseSdf.GarbageCollect(256)`.

### Build & Run

```bash
# Quick BenchmarkDotNet run with [MemoryDiagnoser] (in-process)
dotnet run -c Release --project benchmarks/BulletSharp.SoftBody.Benchmarks -- --filter "*" --job short --inProcess

# Full BenchmarkDotNet run
dotnet run -c Release --project benchmarks/BulletSharp.SoftBody.Benchmarks -- --filter "*"

# Deterministic zero-allocation gate (float & double precision)
dotnet run -c Release --project benchmarks/BulletSharp.SoftBody.Benchmarks -- --verify
dotnet run -c Release --project benchmarks/BulletSharp.SoftBody.Benchmarks -p:Precision=double -- --verify
```

### Results (float, Release, measured 2026-11-24)

Measured on a 13th Gen Intel Core i7-13650HX (`.NET 8.0.34`, X64 RyuJIT `x86-64-v3`,
`ShortRun` / `InProcessEmitToolchain`):

| Benchmark | Mean | StdDev | Ratio | Steady-state `Allocated` |
|---|---:|---:|---:|---:|
| `ClothOnlyStep` | 46.53 µs | 0.565 µs | 1.00 | **0 B/step** |
| `ClothRigidCollisionStep` | 50.33 µs | 0.485 µs | 1.08 | **0 B/step** |

Verified with `--verify` over 600 post-warmup steps: `0 B` total managed heap allocation on
both benchmarks (`centerY=5.772, sphereY=6.769, rcontacts=2` on `ClothRigidCollisionStep`).

### SoftBody Allocation Notes

- `SoftBody` narrowphase handlers (`SDF_RS`, `CL_RS`, `CL_SS`, `VF_SS`) use `Dbvt.CollideTVNoStackAlloc`
  (backed by a per-`SoftBody` reusable `List<DbvtNode>`) and `Dbvt.CollideTTpersistentStack` instead of
  allocating temporary traversal stacks per query, preserving exact traversal order and bit-identical
  simulation trajectories.
- `RContact` and `SContact` are value types stored in capacity-retained `List<T>` buffers cleared each
  step; `SparseSdf` cells and `DefaultSoftBodySolver` body lists are reused in steady state.
