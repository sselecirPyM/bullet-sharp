# Native Bullet Reference Tests (ConstraintMoveRef)

`ConstraintMoveRef.cpp` directly compiles the core sources of a local bullet3 checkout
(BulletDynamics / BulletCollision / LinearMath) and runs scenarios that exactly mirror
S1..S6 in `tests/BulletSharp.SmokeTests/Program.cs`, in order to compare the pure C#
port against native Bullet on "constraint behavior after moving a body".

## Scenarios

| Scenario | Description | Expected (measured on native) |
|------|------|------------------|
| S1 | Locked 6dof, teleport both bodies together | Stay at the new positions (10,5,0)/(12,5,0) |
| S2 | Locked 6dof, teleport only B | Both converge toward the midpoint, keeping offset 2 → (5,5,0)/(7,5,0) |
| S3 | Single-body 6dof (world-anchored), teleport B | Pulled back to the creation position (0,3,0) (frameInA is fixed at constraint creation — by design) |
| S4 | p2p, teleport both bodies together | Stay at the new positions |
| S5 | Same as S1, teleport via proceedToTransform | Stay at the new positions |
| S6 | Teleport after falling asleep, no activate | Body transform is updated, but the motionstate stays at the old position; catches up after activate |
| S7 | Locked 6dof, teleport only A | B is pulled toward the midpoint, center of mass conserved (equal masses ≈6); large error induces rotation (chaotic regime) |
| S8 | p2p, teleport only A | B is pulled along, the two pivots eventually coincide (native A≈6.47/B≈5.53 crossing rotation, chaotic regime) |
| S9 | p2p, violated from the start (spacing 5, expected 2) | Converges exactly to A=(1.5,5,0), B=(3.5,5,0) |

Note: with a locked 6dof, the constraint frames must be taken at the world pivots at
constraint creation time (pivotInA=(1,0,0), pivotInB=(-1,0,0)); otherwise the lock
itself introduces error.

Additionally: `KinematicSleepRef.cpp` (S13) verifies that moving a kinematic body via its
MotionState does **not** work when DISABLE_DEACTIVATION is not set (addRigidBody sets
mass=0 bodies with the static flag to ISLAND_SLEEPING, and saveKinematicState skips
sleeping bodies) — the constraint behaves as if anchored to a fixed point in space; the
port's behavior is byte-identical.
`KinematicPreDARef.cpp` (S14) verifies Coocoo3D's order — `setActivationState(DISABLE_DEACTIVATION)`
**before** `addRigidBody(body, group, mask)`: natively, `setActivationState(ISLAND_SLEEPING)`
has no effect due to the guard (it does not override DISABLE_DEACTIVATION), the kinematic
body stays awake, MotionState driving works, and the constraint follows (measured
A=(10,5,0), B=(12,5,0)). The port previously wrote the state unconditionally here,
overriding DISABLE_DEACTIVATION and causing "constraint attached to a fixed position in
space" — fixed in CollisionObject.SetActivationState.
Same build command as below (just substitute the cpp/exe file names).

## Build & Run (Windows / PowerShell)

```powershell
$bullet3 = if ($env:BULLET3) { $env:BULLET3 } else { "..\bullet3" } # or set $env:BULLET3 to your bullet3 path
$files = Get-ChildItem -Recurse -Filter *.cpp `
    "$bullet3\src\BulletDynamics", `
    "$bullet3\src\BulletCollision", `
    "$bullet3\src\LinearMath" | ForEach-Object { $_.FullName }
g++ -std=c++17 -O2 -I "$bullet3\src" `
    tests\native-ref\ConstraintMoveRef.cpp $files `
    -o tests\native-ref\ConstraintMoveRef.exe
tests\native-ref\ConstraintMoveRef.exe
```

---

# Native Bullet SoftBody Differential Reference (SoftRef)

`SoftRef.cpp` compiles classic `BulletSoftBody` scalar sources (excluding `*Deformable*` and
`btSoftMultiBodyDynamicsWorld.cpp`) alongside `BulletDynamics`, `BulletCollision`, and `LinearMath`,
and mirrors `tests/BulletSharp.SmokeTests/SoftDump.cs` (`dotnet run -- soft <scene> [steps]`) frame-for-frame.

## SoftBody Scenarios

| Scene | Default Steps | Coverage |
|------|------|------|
| `s1` | 180 | `CreateRope` fixed head + `AppendAnchor` suspended rigid box |
| `s2a` | 180 | `CreatePatch` (9x9, fixed corners, bending=2) catching falling rigid sphere via `SparseSdf` |
| `s2b` | 180 | Free `CreatePatch` (6x6) settling onto concave `BvhTriangleMeshShape` (extruded hull + SDF) |
| `s3` | 200 | `CreateEllipsoid` (`res=64`, `kPR=800`) positive-pressure inflation and ground bounce |
| `s4` | 180 | 8-node/5-tetra cube (`SetVolumeMass`, `SetPose` polar-decomposition matching, boundary faces) |
| `s5a` | 180 | Clustered ellipsoids coupled via world & soft-soft `LJoint` + `AJoint` |
| `s5b` | 180 | Multi-cluster (`GenerateClusters(4)`) `CL_RS \| CL_SS` GJK/EPA soft-soft stacking |

## Build & Differential Compare (PowerShell)

```powershell
$bullet3 = if ($env:BULLET3) { $env:BULLET3 } else { "..\bullet3" } # or set $env:BULLET3 to your bullet3 path
$coreFiles = Get-ChildItem -Recurse -Filter *.cpp `
    "$bullet3\src\BulletDynamics", `
    "$bullet3\src\BulletCollision", `
    "$bullet3\src\LinearMath" | ForEach-Object { $_.FullName }
$softFiles = Get-ChildItem -Recurse -Filter *.cpp "$bullet3\src\BulletSoftBody" |
    Where-Object { $_.Name -notlike '*Deformable*' -and $_.Name -ne 'btSoftMultiBodyDynamicsWorld.cpp' } |
    ForEach-Object { $_.FullName }

# Single precision (float)
g++ -std=c++17 -O2 -ffp-contract=off -I "$bullet3\src" `
    tests\native-ref\SoftRef.cpp $coreFiles $softFiles `
    -o tests\native-ref\SoftRef.exe

# Double precision (double): add -DBT_USE_DOUBLE_PRECISION and pair with -p:Precision=double
foreach ($sc in 's1','s2a','s2b','s3','s4','s5a','s5b') {
    tests\native-ref\SoftRef.exe $sc > "$env:TEMP\soft_native_$sc.txt"
    dotnet run --project tests\BulletSharp.SmokeTests -c Release --no-build -- soft $sc > "$env:TEMP\soft_cs_$sc.txt"
    powershell -NoProfile -File tests\native-ref\cmp.ps1 "$env:TEMP\soft_native_$sc.txt" "$env:TEMP\soft_cs_$sc.txt" 5e-2
}
```
