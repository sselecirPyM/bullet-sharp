using BulletSharp;
using BulletSharp.Math;
using BulletSharp.SoftBody;

// BulletSharp pure C# rewrite — smoke tests
// Run: dotnet run (float) / dotnet run -p:Precision=double (double)

int passed = 0, failed = 0;

// Differential fuzz test entry: dotnet run -- fuzz <seed>
if (args.Length >= 2 && args[0] == "fuzz")
    return SmokeFuzz.Run(int.Parse(args[1]), args.Length >= 3 && args[2] == "dump");

// Differential soft-body trace entry: dotnet run -- soft <scene> [steps]
if (args.Length >= 2 && args[0] == "soft")
    return SoftDump.Run(args[1], args.Length >= 3 ? int.Parse(args[2]) : 0);

void Check(string name, bool condition, string detail = "")
{
    if (condition)
    {
        passed++;
        Console.WriteLine($"[PASS] {name}");
    }
    else
    {
        failed++;
        Console.WriteLine($"[FAIL] {name} {detail}");
    }
}

DiscreteDynamicsWorld CreateWorld(out CollisionDispatcher dispatcher, out DbvtBroadphase broadphase)
{
    broadphase = new DbvtBroadphase();
    var config = new DefaultCollisionConfiguration();
    dispatcher = new CollisionDispatcher(config);
    var solver = new SequentialImpulseConstraintSolver();
    var world = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, config);
    var g = new Vector3(0, (btScalar)(-9.81), 0);
    world.SetGravity(ref g);
    return world;
}

RigidBody AddBody(DiscreteDynamicsWorld world, CollisionShape shape, btScalar mass, Matrix startTransform,
    btScalar friction = (btScalar)0.5, btScalar restitution = 0)
{
    Vector3 inertia = Vector3.Zero;
    if (mass != 0)
        shape.CalculateLocalInertia(mass, out inertia);
    var info = new RigidBodyConstructionInfo(mass, new DefaultMotionState(startTransform), shape, inertia)
    {
        Friction = friction,
        Restitution = restitution,
    };
    var body = new RigidBody(info);
    world.AddRigidBody(body);
    return body;
}

// ---------- Test 1: sphere falls onto the ground and settles ----------
{
    var world = CreateWorld(out _, out _);
    AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, Matrix.Translation(new Vector3(0, -1, 0)));
    var sphere = AddBody(world, new SphereShape((btScalar)0.5), 1, Matrix.Translation(new Vector3(0, 5, 0)));

    for (int i = 0; i < 600; i++)
        world.StepSimulation((btScalar)(1.0 / 60.0), 10);

    sphere.MotionState.GetWorldTransform(out Matrix m);
    Check("Sphere rests on the ground", BtMath.Abs(m.M42 - (btScalar)0.5) < (btScalar)0.05, $"y={m.M42}");
    Check("Sphere velocity converges", sphere.LinearVelocity.Length() < (btScalar)0.1, $"|v|={sphere.LinearVelocity.Length()}");
    world.Dispose();
}

// ---------- Test 2: two stacked boxes do not collapse ----------
{
    var world = CreateWorld(out _, out _);
    AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, Matrix.Translation(new Vector3(0, -1, 0)));
    var box1 = AddBody(world, new BoxShape(new Vector3((btScalar)0.5, (btScalar)0.5, (btScalar)0.5)), 1,
        Matrix.Translation(new Vector3(0, (btScalar)0.5, 0)));
    var box2 = AddBody(world, new BoxShape(new Vector3((btScalar)0.5, (btScalar)0.5, (btScalar)0.5)), 1,
        Matrix.Translation(new Vector3((btScalar)0.1, (btScalar)1.6, 0)));

    for (int i = 0; i < 600; i++)
        world.StepSimulation((btScalar)(1.0 / 60.0), 10);

    box1.MotionState.GetWorldTransform(out Matrix m1);
    box2.MotionState.GetWorldTransform(out Matrix m2);
    Check("Bottom box position stable", BtMath.Abs(m1.M42 - (btScalar)0.5) < (btScalar)0.1, $"y={m1.M42}");
    Check("Top box position stable", m2.M42 > (btScalar)1.3 && m2.M42 < (btScalar)1.7, $"y={m2.M42}");
    world.Dispose();
}

// ---------- Test 3: capsule stays upright ----------
{
    var world = CreateWorld(out _, out _);
    AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, Matrix.Translation(new Vector3(0, -1, 0)));
    var capsule = AddBody(world, new CapsuleShape((btScalar)0.4, (btScalar)1.0), 1,
        Matrix.Translation(new Vector3(0, (btScalar)0.95, 0)));

    for (int i = 0; i < 300; i++)
        world.StepSimulation((btScalar)(1.0 / 60.0), 10);

    capsule.MotionState.GetWorldTransform(out Matrix m);
    Check("Capsule settles (reasonable height)", m.M42 > (btScalar)0.8 && m.M42 < (btScalar)1.1, $"y={m.M42}");
    world.Dispose();
}

// ---------- Test 4: 6Dof spring constraint keeps bodies connected ----------
{
    var world = CreateWorld(out _, out _);
    var anchor = AddBody(world, new SphereShape((btScalar)0.2), 0, Matrix.Translation(new Vector3(0, 5, 0)));
    var ball = AddBody(world, new SphereShape((btScalar)0.3), 1, Matrix.Translation(new Vector3(0, 3, 0)));

    var joint = new Generic6DofSpringConstraint(anchor, ball, Matrix.Identity, Matrix.Identity, true);
    joint.LinearLowerLimit = new Vector3(-(btScalar)2.5, -(btScalar)2.5, -(btScalar)2.5);
    joint.LinearUpperLimit = new Vector3((btScalar)2.5, (btScalar)2.5, (btScalar)2.5);
    joint.AngularLowerLimit = Vector3.Zero;
    joint.AngularUpperLimit = Vector3.Zero;
    world.AddConstraint(joint);

    for (int i = 0; i < 300; i++)
        world.StepSimulation((btScalar)(1.0 / 60.0), 10);

    anchor.MotionState.GetWorldTransform(out Matrix ma);
    ball.MotionState.GetWorldTransform(out Matrix mb);
    float dx = (float)(ma.M41 - mb.M41), dy = (float)(ma.M42 - mb.M42), dz = (float)(ma.M43 - mb.M43);
    float dist = MathF.Sqrt(dx * dx + dy * dy + dz * dz);
    Check("6Dof constraint maintains distance", dist < 2.6f, $"dist={dist}");
    world.Dispose();
}

// ---------- Test 5: raycast hit ----------
{
    var world = CreateWorld(out _, out _);
    AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, Matrix.Translation(new Vector3(0, -1, 0)));
    var sphere = AddBody(world, new SphereShape((btScalar)0.5), 1, Matrix.Translation(new Vector3(0, 3, 0)));

    world.StepSimulation((btScalar)(1.0 / 60.0));

    var cb = new ClosestRayResultCallback(new Vector3(0, 10, 0), new Vector3(0, -10, 0));
    world.RayTest(new Vector3(0, 10, 0), new Vector3(0, -10, 0), cb);
    Check("Ray hits sphere", cb.HasHit && cb.CollisionObject == sphere, $"hit={cb.HasHit}");
    if (cb.HasHit)
        Check("Hit point height correct", BtMath.Abs(cb.HitPointWorld.Y - (btScalar)3.5) < (btScalar)0.01, $"y={cb.HitPointWorld.Y}");
    world.Dispose();
}

// ---------- Test 6: collision group filtering ----------
{
    var world = CreateWorld(out _, out _);
    AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, Matrix.Translation(new Vector3(0, -1, 0)));
    var a = AddBody(world, new SphereShape((btScalar)0.5), 1, Matrix.Translation(new Vector3(0, 2, 0)));
    var b = AddBody(world, new SphereShape((btScalar)0.5), 1, Matrix.Translation(new Vector3(0, 4, 0)));
    world.RemoveRigidBody(a);
    world.RemoveRigidBody(b);
    // Groups 4 and 8 do not collide (each mask excludes the other).
    // Note: to collide with the static ground, the mask must include StaticFilter(2) (ground group=StaticFilter).
    world.AddRigidBody(a, (CollisionFilterGroups)4, (CollisionFilterGroups)(1 | 2 | 8));  // collides with default, static, and group 8
    world.AddRigidBody(b, (CollisionFilterGroups)8, (CollisionFilterGroups)(1 | 2));      // collides only with default and static → a and b never collide

    for (int i = 0; i < 300; i++)
        world.StepSimulation((btScalar)(1.0 / 60.0), 10);

    a.MotionState.GetWorldTransform(out Matrix ma);
    b.MotionState.GetWorldTransform(out Matrix mb);
    // Both spheres should fall straight to the ground (passing through each other)
    Check("Group filter: a lands on ground", BtMath.Abs(ma.M42 - (btScalar)0.5) < (btScalar)0.1, $"y={ma.M42}");
    Check("Group filter: b lands on ground", BtMath.Abs(mb.M42 - (btScalar)0.5) < (btScalar)0.1, $"y={mb.M42}");
    world.Dispose();
}

// ---------- Test 7: triangle mesh BVH (sphere lands on the mesh) ----------
{
    var world = CreateWorld(out _, out _);

    // Build a triangle-mesh ground shaped like a large box
    var mesh = new TriangleMesh();
    btScalar s = 50, h = 1;
    Vector3 v0 = new(-s, 0, -s), v1 = new(s, 0, -s), v2 = new(s, 0, s), v3 = new(-s, 0, s);
    Vector3 b0 = new(-s, -h, -s), b1 = new(s, -h, -s), b2 = new(s, -h, s), b3 = new(-s, -h, s);
    mesh.AddTriangle(v0, v2, v1);
    mesh.AddTriangle(v0, v3, v2);
    mesh.AddTriangle(b0, b1, b2);
    mesh.AddTriangle(b0, b2, b3);
    mesh.AddTriangle(v0, v1, b1);
    mesh.AddTriangle(v0, b1, b0);
    mesh.AddTriangle(v1, v2, b2);
    mesh.AddTriangle(v1, b2, b1);
    mesh.AddTriangle(v2, v3, b3);
    mesh.AddTriangle(v2, b3, b2);
    mesh.AddTriangle(v3, v0, b0);
    mesh.AddTriangle(v3, b0, b3);

    var meshShape = new BvhTriangleMeshShape(mesh, true);
    AddBody(world, meshShape, 0, Matrix.Identity);

    var sphere = AddBody(world, new SphereShape((btScalar)0.5), 1, Matrix.Translation(new Vector3(0, 4, 0)));
    for (int i = 0; i < 600; i++)
        world.StepSimulation((btScalar)(1.0 / 60.0), 10);

    sphere.MotionState.GetWorldTransform(out Matrix m);
    Check("Sphere rests on triangle mesh", BtMath.Abs(m.M42 - (btScalar)0.5) < (btScalar)0.1, $"y={m.M42}");

    var cb = new ClosestRayResultCallback(new Vector3(0, 10, 0), new Vector3(0, -10, 0));
    world.RayTest(new Vector3(0, 10, 0), new Vector3(0, -10, 0), cb);
    Check("Ray hits triangle mesh or sphere", cb.HasHit);
    world.Dispose();
}

// ================= Constraint move-follow tests =================
// One-to-one mirror of the native Bullet scenes S1..S6 in tests/native-ref/ConstraintMoveRef.cpp,
// used to locate divergences of the kind "constraint does not follow after moving a body;
// the body is pulled back to its creation position" (especially with 6dof).
// Conventions: no gravity; BoxShape(0.5); stepSimulation(1/60, 10); teleport delta d=(10,0,0).

DiscreteDynamicsWorld CreateWorldNoGravity()
{
    var broadphase = new DbvtBroadphase();
    var config = new DefaultCollisionConfiguration();
    var dispatcher = new CollisionDispatcher(config);
    var solver = new SequentialImpulseConstraintSolver();
    var world = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, config);
    var g = Vector3.Zero;
    world.SetGravity(ref g);
    return world;
}

RigidBody AddBox(DiscreteDynamicsWorld world, btScalar mass, Vector3 pos)
{
    var shape = new BoxShape(new Vector3((btScalar)0.5, (btScalar)0.5, (btScalar)0.5));
    Vector3 inertia = Vector3.Zero;
    if (mass != 0)
        shape.CalculateLocalInertia(mass, out inertia);
    var info = new RigidBodyConstructionInfo(mass, new DefaultMotionState(Matrix.Translation(pos)), shape, inertia);
    var body = new RigidBody(info);
    world.AddRigidBody(body);
    return body;
}

void StepWorld(DiscreteDynamicsWorld world, int n)
{
    for (int i = 0; i < n; i++)
        world.StepSimulation((btScalar)(1.0 / 60.0), 10);
}

void PrintPos(string tag, string name, RigidBody body)
{
    Matrix wm = body.WorldTransform;
    body.MotionState.GetWorldTransform(out Matrix msm);
    Console.WriteLine($"{tag} {name} body=({wm.M41,8:F4} {wm.M42,8:F4} {wm.M43,8:F4}) " +
        $"motionstate=({msm.M41,8:F4} {msm.M42,8:F4} {msm.M43,8:F4}) active={(body.IsActive ? 1 : 0)}");
}

// Translate via btCollisionObject::setWorldTransform (interpolation transform untouched)
void Teleport(RigidBody body, Vector3 delta, bool activate)
{
    Matrix t = body.WorldTransform;
    t.M41 += delta.X; t.M42 += delta.Y; t.M43 += delta.Z;
    body.WorldTransform = t;
    if (activate) body.Activate();
}

// Translate via btRigidBody::proceedToTransform (updates the interpolation transform as well)
void TeleportProceed(RigidBody body, Vector3 delta, bool activate)
{
    Matrix t = body.WorldTransform;
    t.M41 += delta.X; t.M42 += delta.Y; t.M43 += delta.Z;
    body.CenterOfMassTransform = t;
    if (activate) body.Activate();
}

Generic6DofConstraint AddLocked6Dof(DiscreteDynamicsWorld world, RigidBody a, RigidBody b,
    Vector3 pivotInA, Vector3 pivotInB)
{
    var joint = new Generic6DofConstraint(a, b, Matrix.Translation(pivotInA), Matrix.Translation(pivotInB), true);
    joint.LinearLowerLimit = Vector3.Zero;
    joint.LinearUpperLimit = Vector3.Zero;
    joint.AngularLowerLimit = Vector3.Zero;
    joint.AngularUpperLimit = Vector3.Zero;
    world.AddConstraint(joint);
    return joint;
}

bool Near(Matrix m, btScalar x, btScalar y, btScalar z, btScalar eps) =>
    BtMath.Abs(m.M41 - x) < eps && BtMath.Abs(m.M42 - y) < eps && BtMath.Abs(m.M43 - z) < eps;

var d10 = new Vector3(10, 0, 0);

// ---------- S1: locked 6dof, teleport both bodies together → frames are body-local, should follow ----------
{
    var world = CreateWorldNoGravity();
    var a = AddBox(world, 1, new Vector3(0, 5, 0));
    var b = AddBox(world, 1, new Vector3(2, 5, 0));
    AddLocked6Dof(world, a, b, new Vector3(1, 0, 0), new Vector3(-1, 0, 0));
    StepWorld(world, 60);
    Teleport(a, d10, true);
    Teleport(b, d10, true);
    StepWorld(world, 120);
    PrintPos("S1", "A", a);
    PrintPos("S1", "B", b);
    Check("S1 6dof dual teleport: A stays at new position", Near(a.WorldTransform, 10, 5, 0, (btScalar)0.1), $"A={a.WorldTransform.M41},{a.WorldTransform.M42},{a.WorldTransform.M43}");
    Check("S1 6dof dual teleport: B stays at new position", Near(b.WorldTransform, 12, 5, 0, (btScalar)0.1), $"B={b.WorldTransform.M41},{b.WorldTransform.M42},{b.WorldTransform.M43}");
    a.MotionState.GetWorldTransform(out Matrix msA);
    Check("S1 motionstate synced to A's new position", Near(msA, 10, 5, 0, (btScalar)0.5), $"msA={msA.M41},{msA.M42},{msA.M43}");
    world.Dispose();
}

// ---------- S2: locked 6dof, teleport only B → constraint is violated, B should be pulled back to its relative pose ----------
{
    var world = CreateWorldNoGravity();
    var a = AddBox(world, 1, new Vector3(0, 5, 0));
    var b = AddBox(world, 1, new Vector3(2, 5, 0));
    AddLocked6Dof(world, a, b, new Vector3(1, 0, 0), new Vector3(-1, 0, 0));
    StepWorld(world, 60);
    Teleport(b, d10, true);
    StepWorld(world, 120);
    PrintPos("S2", "A", a);
    PrintPos("S2", "B", b);
    // Native reference: equal masses converge toward the midpoint keeping relative offset 2 → A≈(5,5,0), B≈(7,5,0)
    Check("S2 move only B: A dragged toward midpoint", Near(a.WorldTransform, 5, 5, 0, (btScalar)0.5), $"A={a.WorldTransform.M41},{a.WorldTransform.M42},{a.WorldTransform.M43}");
    Check("S2 move only B: B pulled back keeping offset 2", Near(b.WorldTransform, 7, 5, 0, (btScalar)0.5), $"B={b.WorldTransform.M41},{b.WorldTransform.M42},{b.WorldTransform.M43}");
    world.Dispose();
}

// ---------- S3: single-body 6dof (world-anchored), teleport B → frameInA is fixed at creation, B should be pulled back to its creation position ----------
{
    var world = CreateWorldNoGravity();
    var b = AddBox(world, 1, new Vector3(0, 3, 0));
    var joint = new Generic6DofConstraint(b, Matrix.Identity, false);
    joint.LinearLowerLimit = Vector3.Zero;
    joint.LinearUpperLimit = Vector3.Zero;
    joint.AngularLowerLimit = Vector3.Zero;
    joint.AngularUpperLimit = Vector3.Zero;
    world.AddConstraint(joint);
    StepWorld(world, 60);
    Teleport(b, d10, true);
    StepWorld(world, 120);
    PrintPos("S3", "B", b);
    Check("S3 world-anchored 6dof: B pulled back to creation position", Near(b.WorldTransform, 0, 3, 0, (btScalar)0.5), $"B={b.WorldTransform.M41},{b.WorldTransform.M42},{b.WorldTransform.M43}");
    world.Dispose();
}

// ---------- S4: p2p constraint, teleport both bodies together → should follow ----------
{
    var world = CreateWorldNoGravity();
    var a = AddBox(world, 1, new Vector3(0, 5, 0));
    var b = AddBox(world, 1, new Vector3(2, 5, 0));
    var joint = new Point2PointConstraint(a, b, new Vector3(1, 0, 0), new Vector3(-1, 0, 0));
    world.AddConstraint(joint);
    StepWorld(world, 60);
    Teleport(a, d10, true);
    Teleport(b, d10, true);
    StepWorld(world, 120);
    PrintPos("S4", "A", a);
    PrintPos("S4", "B", b);
    Check("S4 p2p dual teleport: A stays at new position", Near(a.WorldTransform, 10, 5, 0, (btScalar)0.5), $"A={a.WorldTransform.M41},{a.WorldTransform.M42},{a.WorldTransform.M43}");
    Check("S4 p2p dual teleport: B stays at new position", Near(b.WorldTransform, 12, 5, 0, (btScalar)0.5), $"B={b.WorldTransform.M41},{b.WorldTransform.M42},{b.WorldTransform.M43}");
    world.Dispose();
}

// ---------- S5: same as S1, but teleport via proceedToTransform (CenterOfMassTransform) ----------
{
    var world = CreateWorldNoGravity();
    var a = AddBox(world, 1, new Vector3(0, 5, 0));
    var b = AddBox(world, 1, new Vector3(2, 5, 0));
    AddLocked6Dof(world, a, b, new Vector3(1, 0, 0), new Vector3(-1, 0, 0));
    StepWorld(world, 60);
    TeleportProceed(a, d10, true);
    TeleportProceed(b, d10, true);
    StepWorld(world, 120);
    PrintPos("S5", "A", a);
    PrintPos("S5", "B", b);
    Check("S5 proceedToTransform dual teleport: A stays at new position", Near(a.WorldTransform, 10, 5, 0, (btScalar)0.1), $"A={a.WorldTransform.M41},{a.WorldTransform.M42},{a.WorldTransform.M43}");
    Check("S5 proceedToTransform dual teleport: B stays at new position", Near(b.WorldTransform, 12, 5, 0, (btScalar)0.1), $"B={b.WorldTransform.M41},{b.WorldTransform.M42},{b.WorldTransform.M43}");
    world.Dispose();
}

// ---------- S6: same as S1, but let the bodies sleep first and teleport without activate → observe body/motionstate divergence ----------
{
    var world = CreateWorldNoGravity();
    var a = AddBox(world, 1, new Vector3(0, 5, 0));
    var b = AddBox(world, 1, new Vector3(2, 5, 0));
    AddLocked6Dof(world, a, b, new Vector3(1, 0, 0), new Vector3(-1, 0, 0));
    StepWorld(world, 240); // falls asleep (default deactivation time 2s)
    PrintPos("S6-sleep", "A", a);
    PrintPos("S6-sleep", "B", b);
    Teleport(a, d10, false);
    Teleport(b, d10, false);
    StepWorld(world, 60);
    PrintPos("S6-noact", "A", a);
    PrintPos("S6-noact", "B", b);
    // The physics body itself is already at the new position (setWorldTransform applies even while sleeping)
    Check("S6 sleep teleport: body transform updated", Near(a.WorldTransform, 10, 5, 0, (btScalar)0.1), $"A={a.WorldTransform.M41},{a.WorldTransform.M42},{a.WorldTransform.M43}");
    a.Activate();
    b.Activate();
    StepWorld(world, 60);
    PrintPos("S6-act", "A", a);
    PrintPos("S6-act", "B", b);
    a.MotionState.GetWorldTransform(out Matrix msA);
    b.MotionState.GetWorldTransform(out Matrix msB);
    Check("S6 after activate: A stays at new position", Near(a.WorldTransform, 10, 5, 0, (btScalar)0.1), $"A={a.WorldTransform.M41},{a.WorldTransform.M42},{a.WorldTransform.M43}");
    Check("S6 after activate: motionstate catches up with A", Near(msA, 10, 5, 0, (btScalar)0.5), $"msA={msA.M41},{msA.M42},{msA.M43}");
    Check("S6 after activate: motionstate catches up with B", Near(msB, 12, 5, 0, (btScalar)0.5), $"msB={msB.M41},{msB.M42},{msB.M43}");
    world.Dispose();
}

// ---------- S7: locked 6dof, teleport only A → B should be pulled along by A to the new pose ----------
{
    var world = CreateWorldNoGravity();
    var a = AddBox(world, 1, new Vector3(0, 5, 0));
    var b = AddBox(world, 1, new Vector3(2, 5, 0));
    AddLocked6Dof(world, a, b, new Vector3(1, 0, 0), new Vector3(-1, 0, 0));
    StepWorld(world, 60);
    Teleport(a, d10, true);
    StepWorld(world, 120);
    PrintPos("S7", "A", a);
    PrintPos("S7", "B", b);
    // Teleporting A across B induces 3D rotation under large error correction (chaotic regime;
    // native S7Ref matches identically: A≈(5.61,5.00,-0.94), B≈(6.39,5.00,0.94)).
    // Assert the essential invariants: B is pulled toward the midpoint (6,5,0), center of mass is conserved, and pivots coincide.
    Matrix wa = a.WorldTransform, wb = b.WorldTransform;
    Vector3 posA = new(wa.M41, wa.M42, wa.M43);
    Vector3 posB = new(wb.M41, wb.M42, wb.M43);
    Vector3 mid = (posA + posB) * (btScalar)0.5;
    Vector3 pivotA = Matrix.TransformCoordinate(new Vector3(1, 0, 0), in wa);
    Vector3 pivotB = Matrix.TransformCoordinate(new Vector3(-1, 0, 0), in wb);
    Check("S7 move only A: B pulled toward midpoint",
        wb.M41 > 4 && (posB - new Vector3(6, 5, 0)).Length() < (btScalar)1.5,
        $"B={wb.M41},{wb.M42},{wb.M43}");
    Check("S7 move only A: center of mass conserved",
        (mid - new Vector3(6, 5, 0)).Length() < (btScalar)0.1,
        $"mid={mid.X},{mid.Y},{mid.Z}");
    Check("S7 move only A: pivots coincide",
        (pivotA - pivotB).Length() < (btScalar)0.2,
        $"|pA-pB|={(pivotA - pivotB).Length()}");
    world.Dispose();
}

// ---------- S8: p2p constraint, teleport only A → B should be pulled along ----------
{
    var world = CreateWorldNoGravity();
    var a = AddBox(world, 1, new Vector3(0, 5, 0));
    var b = AddBox(world, 1, new Vector3(2, 5, 0));
    var joint = new Point2PointConstraint(a, b, new Vector3(1, 0, 0), new Vector3(-1, 0, 0));
    world.AddConstraint(joint);
    StepWorld(world, 60);
    Teleport(a, d10, true);
    StepWorld(world, 120);
    PrintPos("S8", "A", a);
    PrintPos("S8", "B", b);
    // Large error correction induces rotation (chaotic regime; native behaves the same: A≈6.47, B≈5.53 crossing).
    // Assert the essential behavior: B is pulled and the two pivots eventually coincide.
    Matrix wa = a.WorldTransform, wb = b.WorldTransform;
    Vector3 pivotA = Matrix.TransformCoordinate(new Vector3(1, 0, 0), in wa);   // A's local pivot (1,0,0)
    Vector3 pivotB = Matrix.TransformCoordinate(new Vector3(-1, 0, 0), in wb);  // B's local pivot (-1,0,0)
    Check("S8 p2p move only A: B pulled along", wb.M41 > 4, $"B={wb.M41},{wb.M42},{wb.M43}");
    Check("S8 p2p move only A: pivots coincide", (pivotA - pivotB).Length() < (btScalar)0.2,
        $"|pA-pB|={(pivotA - pivotB).Length()}");
    world.Dispose();
}

// ---------- S9: p2p, constraint violated from the start (no teleport) → bodies should pull together ----------
{
    var world = CreateWorldNoGravity();
    var a = AddBox(world, 1, new Vector3(0, 5, 0));
    var b = AddBox(world, 1, new Vector3(5, 5, 0));
    var joint = new Point2PointConstraint(a, b, new Vector3(1, 0, 0), new Vector3(-1, 0, 0));
    world.AddConstraint(joint);
    StepWorld(world, 120);
    PrintPos("S9", "A", a);
    PrintPos("S9", "B", b);
    // Center of mass stays at 2.5, spacing converges to 2 → A≈1.5, B≈3.5
    Check("S9 p2p initially violated: pull together", Near(a.WorldTransform, (btScalar)1.5, 5, 0, (btScalar)0.5) && Near(b.WorldTransform, (btScalar)3.5, 5, 0, (btScalar)0.5),
        $"A={a.WorldTransform.M41} B={b.WorldTransform.M41}");
    world.Dispose();
}


// ---------- SIMD math: Vector3/Quaternion core ops (BtSimd port of btVector3/btQuaternion) ----------
{
    var a = new Vector3((btScalar)1.25, (btScalar)(-2.5), (btScalar)3.75);
    var b = new Vector3((btScalar)(-0.5), (btScalar)4, (btScalar)(-1.5));
    btScalar s = (btScalar)2.5;

    // Lane-wise operations (bit-exact SIMD paths)
    Check("simd v3 add", (a + b) == new Vector3(a.X + b.X, a.Y + b.Y, a.Z + b.Z));
    Check("simd v3 sub", (a - b) == new Vector3(a.X - b.X, a.Y - b.Y, a.Z - b.Z));
    Check("simd v3 neg", -a == new Vector3(-a.X, -a.Y, -a.Z));
    Check("simd v3 scale", (a * s) == new Vector3(a.X * s, a.Y * s, a.Z * s));
    Check("simd v3 mul", (a * b) == new Vector3(a.X * b.X, a.Y * b.Y, a.Z * b.Z));
    Check("simd v3 div", (a / s) == new Vector3(a.X / s, a.Y / s, a.Z / s));
    Check("simd v3 abs", a.Absolute() == new Vector3(BtMath.Abs(a.X), BtMath.Abs(a.Y), BtMath.Abs(a.Z)));
    Check("simd v3 min", a.Min(b) == new Vector3(BtMath.Min(a.X, b.X), BtMath.Min(a.Y, b.Y), BtMath.Min(a.Z, b.Z)));
    Check("simd v3 max", a.Max(b) == new Vector3(BtMath.Max(a.X, b.X), BtMath.Max(a.Y, b.Y), BtMath.Max(a.Z, b.Z)));

    // Reductions and shuffles (bit-exact against the scalar reference formulas)
    Check("simd v3 dot", a.Dot(b) == a.X * b.X + a.Y * b.Y + a.Z * b.Z);
    Check("simd v3 length2", a.LengthSquared() == a.X * a.X + a.Y * a.Y + a.Z * a.Z);
    Vector3 c = a.Cross(b);
    Check("simd v3 cross", c == new Vector3(
        a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X));
    Check("simd v3 triple", a.Triple(b, c) ==
        a.X * (b.Y * c.Z - b.Z * c.Y) + a.Y * (b.Z * c.X - b.X * c.Z) + a.Z * (b.X * c.Y - b.Y * c.X));
    Check("simd v3 lerp", a.Lerp(b, (btScalar)0.25) == new Vector3(
        a.X + (b.X - a.X) * (btScalar)0.25, a.Y + (b.Y - a.Y) * (btScalar)0.25, a.Z + (b.Z - a.Z) * (btScalar)0.25));

    // The unused fourth lane (m_floats[3]) must stay 0 through every operation
    Vector3 w = (a + b) - b;
    w.SetMin(b);
    w.SetMax(a);
    w = w * s + new Vector3(1, 2, 3) - w.Cross(b).Absolute();
    Check("simd v3 w lane invariant", w.W == 0, $"W={w.W}");

    // Normalize: exact in the default build; rsqrt+NR approximation under BT_USE_SSE_IN_API
    Vector3 n = new((btScalar)3, (btScalar)4, (btScalar)5);
    Vector3 nn = n.Normalized();
    Check("simd v3 normalize length", BtMath.Abs(nn.Length() - 1) < (btScalar)1e-5, $"len={nn.Length()}");
    Check("simd v3 normalize direction", (nn - n * (1 / n.Length())).Length() < (btScalar)1e-5);

    // Quaternion operations
    var q1 = new Quaternion((btScalar)0.1, (btScalar)(-0.2), (btScalar)0.3, (btScalar)0.9).Normalized();
    var q2 = new Quaternion((btScalar)(-0.3), (btScalar)0.2, (btScalar)(-0.1), (btScalar)0.8).Normalized();
    Check("simd quat dot", BtMath.Abs(q1.Dot(q2) -
        (q1.X * q2.X + q1.Y * q2.Y + q1.Z * q2.Z + q1.W * q2.W)) < (btScalar)1e-6);
    Check("simd quat mult", BtMath.Abs((q1 * q2).Length() - 1) < (btScalar)1e-5);
    Check("simd quat inverse", (q1 * q1.Inverse() - Quaternion.Identity).Length() < (btScalar)1e-6);

    // Quaternion product must compose rotations ((q2 q1) v (q2 q1)* applies q1 first, then q2)
    Vector3 v = new((btScalar)0.3, (btScalar)(-0.7), (btScalar)1.1);
    Vector3 rSeq = Quaternion.Rotate(q2, Quaternion.Rotate(q1, v));
    Vector3 rComb = Quaternion.Rotate(q2 * q1, v);
    Check("simd quat mult composes rotations", (rSeq - rComb).Length() < (btScalar)1e-4,
        $"diff={(rSeq - rComb).Length()}");
}

// ---------- SoftBody Phase 0: Diagonalize, PolarDecomposition, Poly34 ----------
{
    static btScalar MaxAbsDiff(in Matrix3x3 m1, in Matrix3x3 m2)
    {
        btScalar max = 0;
        for (int r = 0; r < 3; r++)
            for (int col = 0; col < 3; col++)
                max = BtMath.Max(max, BtMath.Abs(m1[r, col] - m2[r, col]));
        return max;
    }

    static void SortRoots(Span<btScalar> s)
    {
        for (int i = 0; i < s.Length - 1; i++)
            for (int j = i + 1; j < s.Length; j++)
                if (s[j] < s[i])
                    (s[i], s[j]) = (s[j], s[i]);
    }

    // T0.1: Matrix3x3.Diagonalize
    var symOrig = new Matrix3x3(
        (btScalar)4.0, (btScalar)1.5, (btScalar)(-0.8),
        (btScalar)1.5, (btScalar)2.5, (btScalar)0.6,
        (btScalar)(-0.8), (btScalar)0.6, (btScalar)3.2);
    var symDiag = symOrig;
    symDiag.Diagonalize(out Matrix3x3 jacobiRot, (btScalar)1e-6, 32);
    btScalar offDiag = BtMath.Abs(symDiag[0, 1]) + BtMath.Abs(symDiag[0, 2]) + BtMath.Abs(symDiag[1, 2]);
    Check("T0.1 Diagonalize zeroes off-diagonal", offDiag < (btScalar)1e-4, $"offDiag={offDiag}");
    btScalar rotOrthoErr = MaxAbsDiff(jacobiRot.Transpose() * jacobiRot, in Matrix3x3.Identity);
    Check("T0.1 Diagonalize rot orthogonal", rotOrthoErr < (btScalar)1e-4 && BtMath.Abs(jacobiRot.Determinant() - 1) < (btScalar)1e-4,
        $"err={rotOrthoErr}");
    Matrix3x3 reconstructed = jacobiRot * symDiag * jacobiRot.Transpose();
    btScalar reconErr = MaxAbsDiff(in reconstructed, in symOrig);
    Check("T0.1 Diagonalize reconstructs A = R*D*R^T", reconErr < (btScalar)1e-4, $"err={reconErr}");

    // T0.2: PolarDecomposition
    var defGrad = new Matrix3x3(
        (btScalar)1.8, (btScalar)(-0.4), (btScalar)0.3,
        (btScalar)0.7, (btScalar)1.3, (btScalar)(-0.2),
        (btScalar)(-0.1), (btScalar)0.5, (btScalar)0.9);
    uint pdIters = PolarDecomposition.PolarDecompose(in defGrad, out Matrix3x3 pdU, out Matrix3x3 pdH);
    btScalar uOrthoErr = MaxAbsDiff(pdU.Transpose() * pdU, in Matrix3x3.Identity);
    btScalar uhErr = MaxAbsDiff(pdU * pdH, in defGrad);
    btScalar hSymErr = MaxAbsDiff(in pdH, pdH.Transpose());
    Check("T0.2 PolarDecompose converges", pdIters < PolarDecomposition.DefaultMaxIterations, $"iters={pdIters}");
    Check("T0.2 PolarDecompose ||U^T U - I|| < 1e-4", uOrthoErr < (btScalar)1e-4, $"err={uOrthoErr}");
    Check("T0.2 PolarDecompose ||U H - A|| < 1e-4 & H symmetric SPD",
        uhErr < (btScalar)1e-4 && hSymErr < (btScalar)1e-5 && pdH.Determinant() > 0, $"uhErr={uhErr}");

    // Singular matrix graceful handling
    var singularMat = default(Matrix3x3);
    uint singIters = PolarDecomposition.PolarDecompose(in singularMat, out _, out _);
    Check("T0.2 PolarDecompose handles singular matrix", singIters == PolarDecomposition.DefaultMaxIterations);

    // T0.3: Poly34
    Span<btScalar> roots = stackalloc btScalar[5];

    // x^2 - 3x + 2 = 0 -> {1, 2}
    int n2 = Poly34.SolveP2(roots, -3, 2);
    SortRoots(roots.Slice(0, 2));
    Check("T0.3 SolveP2 real roots", n2 == 2 &&
        BtMath.Abs(roots[0] - 1) < (btScalar)1e-5 && BtMath.Abs(roots[1] - 2) < (btScalar)1e-5);

    // x^2 + 2x + 5 = 0 -> -1 +/- 2i
    int n2c = Poly34.SolveP2(roots, 2, 5);
    Check("T0.3 SolveP2 complex roots", n2c == 0 &&
        BtMath.Abs(roots[0] - (-1)) < (btScalar)1e-5 && BtMath.Abs(roots[1] - 2) < (btScalar)1e-5);

    // (x + 3)(x - 1)(x - 2) = x^3 - 7x + 6 = 0
    int n3 = Poly34.SolveP3(roots, 0, -7, 6);
    SortRoots(roots.Slice(0, 3));
    Check("T0.3 SolveP3 3 real roots", n3 == 3 &&
        BtMath.Abs(roots[0] - (-3)) < (btScalar)1e-4 &&
        BtMath.Abs(roots[1] - 1) < (btScalar)1e-4 &&
        BtMath.Abs(roots[2] - 2) < (btScalar)1e-4);

    // (x - 2)(x^2 + x + 1) = x^3 - x^2 - x - 2 = 0 -> 1 real root (2), complex -0.5 +/- i*sqrt(3)/2
    int n3c = Poly34.SolveP3(roots, -1, -1, -2);
    Check("T0.3 SolveP3 1 real + complex pair", n3c == 1 &&
        BtMath.Abs(roots[0] - 2) < (btScalar)1e-4 &&
        BtMath.Abs(roots[1] - (-(btScalar)0.5)) < (btScalar)1e-4);

    // (x + 3)(x + 1)(x - 2)(x - 4) = x^4 - 2x^3 - 13x^2 + 14x + 24 = 0
    int n4 = Poly34.SolveP4(roots, -2, -13, 14, 24);
    SortRoots(roots.Slice(0, 4));
    Check("T0.3 SolveP4 4 real roots", n4 == 4 &&
        BtMath.Abs(roots[0] - (-3)) < (btScalar)1e-4 &&
        BtMath.Abs(roots[1] - (-1)) < (btScalar)1e-4 &&
        BtMath.Abs(roots[2] - 2) < (btScalar)1e-4 &&
        BtMath.Abs(roots[3] - 4) < (btScalar)1e-4);

    // (x - 1)(x + 2)(x^2 + 1) = x^4 + x^3 - x^2 + x - 2 = 0 -> 2 real roots {-2, 1}
    int n4m = Poly34.SolveP4(roots, 1, -1, 1, -2);
    SortRoots(roots.Slice(0, 2));
    Check("T0.3 SolveP4 2 real + complex pair", n4m == 2 &&
        BtMath.Abs(roots[0] - (-2)) < (btScalar)1e-4 &&
        BtMath.Abs(roots[1] - 1) < (btScalar)1e-4);

    // (x^2 + 1)(x^2 + 4) = x^4 + 5x^2 + 4 = 0 -> 0 real roots
    int n4c = Poly34.SolveP4(roots, 0, 5, 0, 4);
    Check("T0.3 SolveP4 0 real roots", n4c == 0);

    // (x + 2)(x + 1)(x - 0.5)(x - 1)(x - 3) = x^5 - 1.5x^4 - 6.5x^3 + 4.5x^2 + 5.5x - 3 = 0
    int n5 = Poly34.SolveP5(roots, -(btScalar)1.5, -(btScalar)6.5, (btScalar)4.5, (btScalar)5.5, -3);
    SortRoots(roots.Slice(0, 5));
    Check("T0.3 SolveP5 5 real roots", n5 == 5 &&
        BtMath.Abs(roots[0] - (-2)) < (btScalar)1e-3 &&
        BtMath.Abs(roots[1] - (-1)) < (btScalar)1e-3 &&
        BtMath.Abs(roots[2] - (btScalar)0.5) < (btScalar)1e-3 &&
        BtMath.Abs(roots[3] - 1) < (btScalar)1e-3 &&
        BtMath.Abs(roots[4] - 3) < (btScalar)1e-3);

    // T0.4: DynamicsWorldType
    var wtWorld = CreateWorldNoGravity();
    Check("T0.4 DynamicsWorldType Discrete & SoftRigid=4",
        wtWorld.WorldType == DynamicsWorldType.Discrete && (int)DynamicsWorldType.SoftRigid == 4);
    wtWorld.Dispose();
}

// ---------- SoftBody Phase 3: collision algorithms & configuration ----------
{
    var sbConfig = new SoftBodyRigidBodyCollisionConfiguration();
    var sbDispatcher = new CollisionDispatcher(sbConfig);
    var sbWorldInfo = new SoftBodyWorldInfo { Dispatcher = sbDispatcher };
    sbWorldInfo.SparseSdf.Initialize();

    using var recSolver = new Phase3RecordingSoftBodySolver();
    using var defSolver = new DefaultSoftBodySolver();
    Check("T3.3 DefaultSoftBodySolver initialized", defSolver.CheckInitialized() && defSolver.SolverType == SolverTypes.Default);

    var softA = new SoftBody(sbWorldInfo) { SoftBodySolver = recSolver };
    var softB = new SoftBody(sbWorldInfo) { SoftBodySolver = recSolver };

    var sphereShape = new SphereShape((btScalar)0.5);
    var sphereBody = new RigidBody(new RigidBodyConstructionInfo(0, new DefaultMotionState(Matrix.Identity), sphereShape, Vector3.Zero));

    var triMesh = new TriangleMesh();
    triMesh.AddTriangle(new Vector3(-2, 0, -2), new Vector3(2, 0, -2), new Vector3(0, 0, 2));
    var bvhMeshShape = new BvhTriangleMeshShape(triMesh, true);
    var meshBody = new RigidBody(new RigidBodyConstructionInfo(0, new DefaultMotionState(Matrix.Identity), bvhMeshShape, Vector3.Zero));

    var planeShape = new StaticPlaneShape(new Vector3(0, 1, 0), 0);
    var planeBody = new RigidBody(new RigidBodyConstructionInfo(0, new DefaultMotionState(Matrix.Identity), planeShape, Vector3.Zero));

    var wrapSoftA = new CollisionObjectWrapper(null, softA.CollisionShape, softA, Matrix.Identity, -1, -1);
    var wrapSoftB = new CollisionObjectWrapper(null, softB.CollisionShape, softB, Matrix.Identity, -1, -1);
    var wrapSphere = new CollisionObjectWrapper(null, sphereShape, sphereBody, Matrix.Identity, -1, -1);
    var wrapMesh = new CollisionObjectWrapper(null, bvhMeshShape, meshBody, Matrix.Identity, -1, -1);
    var wrapPlane = new CollisionObjectWrapper(null, planeShape, planeBody, Matrix.Identity, -1, -1);

    // Dispatch table verification for all 5 required SoftBody pairings + rigid fallback
    var algoSoftSoft = sbDispatcher.FindAlgorithm(wrapSoftA, wrapSoftB);
    var algoSoftSphere = sbDispatcher.FindAlgorithm(wrapSoftA, wrapSphere);
    var algoSphereSoft = sbDispatcher.FindAlgorithm(wrapSphere, wrapSoftA);
    var algoSoftMesh = sbDispatcher.FindAlgorithm(wrapSoftA, wrapMesh);
    var algoPlaneSoft = sbDispatcher.FindAlgorithm(wrapPlane, wrapSoftA);
    var algoSphereSphere = sbDispatcher.FindAlgorithm(wrapSphere, wrapSphere);

    Check("T3.4 dispatch Soft-Soft", algoSoftSoft is SoftSoftCollisionAlgorithm);
    Check("T3.4 dispatch Soft-Sphere (not swapped)",
        algoSoftSphere is SoftRigidCollisionAlgorithm { IsSwapped: false });
    Check("T3.4 dispatch Sphere-Soft (swapped)",
        algoSphereSoft is SoftRigidCollisionAlgorithm { IsSwapped: true });
    Check("T3.4 dispatch Soft-TriMesh (not swapped)",
        algoSoftMesh is SoftBodyConcaveCollisionAlgorithm { IsSwapped: false });
    Check("T3.4 dispatch StaticPlane-Soft (swapped)",
        algoPlaneSoft is SoftBodyConcaveCollisionAlgorithm { IsSwapped: true });
    Check("T3.4 dispatch Rigid-Rigid fallback preserved", algoSphereSphere is SphereSphereCollisionAlgorithm);

    // TriIndex bit packing
    var triIdx = new TriIndex(5, 123456, sphereShape);
    Check("T3.4 TriIndex PartId/TriangleIndex/Uid",
        triIdx.PartId == 5 && triIdx.TriangleIndex == 123456 && triIdx.Uid == triIdx.PartIdTriangleIndex);

    // ProcessCollision forwarding & CollisionDisabledObjects filtering
    var dispInfo = new DispatcherInfo();
    algoSoftSoft.ProcessCollision(wrapSoftA, wrapSoftB, dispInfo, null);
    Check("T3.4 SoftSoft ProcessCollision invokes solver", recSolver.SoftSoftCalls == 1);

    algoSoftSphere.ProcessCollision(wrapSoftA, wrapSphere, dispInfo, null);
    algoSphereSoft.ProcessCollision(wrapSphere, wrapSoftA, dispInfo, null);
    Check("T3.4 SoftRigid ProcessCollision normal & swapped", recSolver.SoftRigidCalls == 2);

    softA.CollisionDisabledObjects.Add(sphereBody);
    algoSoftSphere.ProcessCollision(wrapSoftA, wrapSphere, dispInfo, null);
    Check("T3.4 SoftRigid respects CollisionDisabledObjects", recSolver.SoftRigidCalls == 2);
    softA.CollisionDisabledObjects.Clear();

    // Concave triangle extrusion cache hit/miss & ClearCache/Dispose cleanup
    recSolver.LastRigidShape = null;
    algoSoftMesh.ProcessCollision(wrapSoftA, wrapMesh, dispInfo, null);
    CollisionShape firstExtrudedHull = recSolver.LastRigidShape;
    Check("T3.4 SoftBodyConcave extrudes ConvexHullShape",
        firstExtrudedHull is ConvexHullShape && recSolver.SoftRigidCalls == 3);

    algoSoftMesh.ProcessCollision(wrapSoftA, wrapMesh, dispInfo, null);
    CollisionShape cachedExtrudedHull = recSolver.LastRigidShape;
    Check("T3.4 SoftBodyConcave reuses cached hull", ReferenceEquals(firstExtrudedHull, cachedExtrudedHull));

    ((SoftBodyConcaveCollisionAlgorithm)algoSoftMesh).ClearCache();
    algoSoftMesh.ProcessCollision(wrapSoftA, wrapMesh, dispInfo, null);
    CollisionShape rebuiltExtrudedHull = recSolver.LastRigidShape;
    Check("T3.4 SoftBodyConcave ClearCache invalidates cached hull",
        rebuiltExtrudedHull is ConvexHullShape && !ReferenceEquals(cachedExtrudedHull, rebuiltExtrudedHull));

    ((IDisposable)algoSoftSoft).Dispose();
    ((IDisposable)algoSoftSphere).Dispose();
    ((IDisposable)algoSphereSoft).Dispose();
    ((IDisposable)algoSoftMesh).Dispose();
    ((IDisposable)algoPlaneSoft).Dispose();
}

// ================= SoftBody Phase 7 (T7.1): deterministic smoke scenes =================

SoftRigidDynamicsWorld CreateSoftWorld()
{
    var broadphase = new DbvtBroadphase();
    var config = new SoftBodyRigidBodyCollisionConfiguration();
    var dispatcher = new CollisionDispatcher(config);
    var solver = new SequentialImpulseConstraintSolver();
    var world = new SoftRigidDynamicsWorld(dispatcher, broadphase, solver, config);
    var g = new Vector3(0, (btScalar)(-9.81), 0);
    world.SetGravity(ref g);
    world.WorldInfo.Gravity = g;
    return world;
}

bool AllNodesFinite(SoftBody body)
{
    for (int i = 0; i < body.Nodes.Count; i++)
    {
        Vector3 p = body.Nodes[i].Position;
        if (!btScalar.IsFinite(p.X) || !btScalar.IsFinite(p.Y) || !btScalar.IsFinite(p.Z))
            return false;
    }
    return true;
}

btScalar AvgNodeY(SoftBody body)
{
    btScalar sum = 0;
    for (int i = 0; i < body.Nodes.Count; i++)
        sum += body.Nodes[i].Position.Y;
    return sum / (btScalar)body.Nodes.Count;
}

// ---------- T7.1 Scene 1: CreateRope fixed head swinging + AppendAnchor hanging rigid box ----------
{
    var world = CreateSoftWorld();
    AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, Matrix.Translation(new Vector3(0, -1, 0)));

    var rope = SoftBodyHelpers.CreateRope(world.WorldInfo, new Vector3(0, 8, 0), new Vector3(3, 8, 0), 8, 1);
    rope.Cfg.PIterations = 8;
    int lastNode = rope.Nodes.Count - 1;

    var box = AddBody(world, new BoxShape(new Vector3((btScalar)0.4, (btScalar)0.4, (btScalar)0.4)), (btScalar)1.0,
        Matrix.Translation(new Vector3(3, (btScalar)7.5, 0)));
    rope.AppendAnchor(lastNode, box, true);
    world.AddSoftBody(rope);

    Check("T7.1 Rope topology & anchor count",
        rope.Nodes.Count == 10 && rope.Links.Count == 9 && rope.Anchors.Count == 1);

    for (int i = 0; i < 180; i++)
        world.StepSimulation((btScalar)(1.0 / 60.0), 10);

    Matrix boxWorld = box.WorldTransform;
    Vector3 anchorOnBox = Matrix.TransformCoordinate(rope.Anchors[0].Local, in boxWorld);
    btScalar anchorGap = (anchorOnBox - rope.Nodes[lastNode].Position).Length();

    Check("T7.1 Rope head node stays fixed",
        (rope.Nodes[0].Position - new Vector3(0, 8, 0)).Length() < (btScalar)1e-5,
        $"p0={rope.Nodes[0].Position}");
    Check("T7.1 Rope swings downward under gravity",
        rope.Nodes[lastNode].Position.Y < (btScalar)7.0 && rope.Nodes[lastNode].Position.X < (btScalar)2.8,
        $"pEnd={rope.Nodes[lastNode].Position}");
    Check("T7.1 Rope anchor suspends rigid box above ground",
        boxWorld.M42 > (btScalar)2.5 && boxWorld.M42 < (btScalar)7.5 && anchorGap < (btScalar)0.5 && AllNodesFinite(rope),
        $"boxY={boxWorld.M42}, gap={anchorGap}");
    world.Dispose();
}

// ---------- T7.1 Scene 2a: CreatePatch 4 corners fixed catching falling rigid sphere ----------
{
    var world = CreateSoftWorld();
    AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, Matrix.Translation(new Vector3(0, -1, 0)));

    var cloth = SoftBodyHelpers.CreatePatch(
        world.WorldInfo,
        new Vector3(-2, 6, -2),
        new Vector3(2, 6, -2),
        new Vector3(-2, 6, 2),
        new Vector3(2, 6, 2),
        9, 9, 15, true);
    Check("T7.1 Patch topology (9x9)",
        cloth.Nodes.Count == 81 && cloth.Faces.Count == 128 && cloth.Links.Count == 208);

    int bendingCount = cloth.GenerateBendingConstraints(2);
    cloth.Cfg.PIterations = 8;
    cloth.SetTotalMass((btScalar)12.0);
    world.AddSoftBody(cloth);

    var sphere = AddBody(world, new SphereShape((btScalar)0.8), (btScalar)0.3,
        Matrix.Translation(new Vector3(0, (btScalar)7.5, 0)));

    for (int i = 0; i < 360; i++)
        world.StepSimulation((btScalar)(1.0 / 60.0), 10);

    btScalar cornerErr =
        BtMath.Abs(cloth.Nodes[0].Position.Y - 6) +
        BtMath.Abs(cloth.Nodes[8].Position.Y - 6) +
        BtMath.Abs(cloth.Nodes[72].Position.Y - 6) +
        BtMath.Abs(cloth.Nodes[80].Position.Y - 6);
    btScalar centerY = cloth.Nodes[40].Position.Y;
    btScalar sphereY = sphere.WorldTransform.M42;

    Check("T7.1 Patch bending constraints & fixed corners",
        bendingCount > 0 && cornerErr < (btScalar)1e-4, $"cornerErr={cornerErr}");
    Check("T7.1 Patch catches falling sphere above ground",
        centerY < (btScalar)5.9 && centerY > (btScalar)2.0 &&
        sphereY > (btScalar)3.0 && sphereY < (btScalar)7.0 &&
        sphere.LinearVelocity.Length() < (btScalar)0.5 && AllNodesFinite(cloth),
        $"centerY={centerY}, sphereY={sphereY}, |v|={sphere.LinearVelocity.Length()}");
    world.Dispose();
}

// ---------- T7.1 Scene 2b: CreatePatch falling onto BvhTriangleMeshShape concave ground ----------
{
    var world = CreateSoftWorld();

    var mesh = new TriangleMesh();
    for (int gz = -4; gz < 4; gz++)
    {
        for (int gx = -4; gx < 4; gx++)
        {
            btScalar x0 = gx * (btScalar)2, x1 = (gx + 1) * (btScalar)2;
            btScalar z0 = gz * (btScalar)2, z1 = (gz + 1) * (btScalar)2;
            mesh.AddTriangle(new Vector3(x0, 0, z0), new Vector3(x0, 0, z1), new Vector3(x1, 0, z1));
            mesh.AddTriangle(new Vector3(x0, 0, z0), new Vector3(x1, 0, z1), new Vector3(x1, 0, z0));
        }
    }
    var meshShape = new BvhTriangleMeshShape(mesh, true);
    AddBody(world, meshShape, 0, Matrix.Identity);

    var freeCloth = SoftBodyHelpers.CreatePatch(
        world.WorldInfo,
        new Vector3(-(btScalar)1.5, (btScalar)2.5, -(btScalar)1.5),
        new Vector3((btScalar)1.5, (btScalar)2.5, -(btScalar)1.5),
        new Vector3(-(btScalar)1.5, (btScalar)2.5, (btScalar)1.5),
        new Vector3((btScalar)1.5, (btScalar)2.5, (btScalar)1.5),
        6, 6, 0, true);
    freeCloth.GenerateBendingConstraints(2);
    freeCloth.Cfg.PIterations = 4;
    freeCloth.SetTotalMass((btScalar)3.0);
    world.AddSoftBody(freeCloth);

    for (int i = 0; i < 180; i++)
        world.StepSimulation((btScalar)(1.0 / 60.0), 10);

    btScalar minNodeY = (btScalar)1e9, maxNodeY = -(btScalar)1e9;
    for (int i = 0; i < freeCloth.Nodes.Count; i++)
    {
        btScalar ny = freeCloth.Nodes[i].Position.Y;
        minNodeY = BtMath.Min(minNodeY, ny);
        maxNodeY = BtMath.Max(maxNodeY, ny);
    }
    Check("T7.1 Free patch settles on BvhTriangleMeshShape",
        minNodeY > -(btScalar)0.2 && maxNodeY < (btScalar)1.0 && AllNodesFinite(freeCloth),
        $"minY={minNodeY}, maxY={maxNodeY}");
    world.Dispose();
}

// ---------- T7.1 Scene 3: CreateEllipsoid positive pressure (kPR > 0) inflation & bounce ----------
{
    var world = CreateSoftWorld();
    AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, Matrix.Translation(new Vector3(0, -1, 0)));

    var ellipsoid = SoftBodyHelpers.CreateEllipsoid(world.WorldInfo, new Vector3(0, 4, 0), new Vector3(1, 1, 1), 64);
    btScalar initialVol = ellipsoid.Volume;
    Check("T7.1 Ellipsoid closed hull topology & initial volume",
        ellipsoid.Nodes.Count > 30 && ellipsoid.Faces.Count > 60 && initialVol > (btScalar)3.0,
        $"nodes={ellipsoid.Nodes.Count}, faces={ellipsoid.Faces.Count}, vol={initialVol}");

    ellipsoid.Materials[0].Lst = (btScalar)0.2;
    ellipsoid.Cfg.DF = (btScalar)1.0;
    ellipsoid.Cfg.DP = (btScalar)0.001;
    ellipsoid.Cfg.PR = (btScalar)800;
    ellipsoid.Cfg.PIterations = 4;
    ellipsoid.SetTotalMass((btScalar)30);
    world.AddSoftBody(ellipsoid);

    for (int i = 0; i < 15; i++)
        world.StepSimulation((btScalar)(1.0 / 60.0), 10);

    btScalar midAirVol = ellipsoid.Volume;
    Check("T7.1 Ellipsoid inflates under positive pressure",
        midAirVol > initialVol * (btScalar)1.05,
        $"initVol={initialVol}, midAirVol={midAirVol}");

    btScalar minComY = (btScalar)100;
    btScalar maxRebound = 0;
    for (int i = 0; i < 185; i++)
    {
        world.StepSimulation((btScalar)(1.0 / 60.0), 10);
        btScalar cy = AvgNodeY(ellipsoid);
        if (cy < (btScalar)2.5)
        {
            minComY = BtMath.Min(minComY, cy);
            maxRebound = BtMath.Max(maxRebound, cy - minComY);
        }
    }

    Check("T7.1 Pressurized ellipsoid bounces off ground & retains volume",
        minComY > (btScalar)0.4 && minComY < (btScalar)2.2 &&
        maxRebound > (btScalar)0.1 &&
        ellipsoid.Volume > initialVol * (btScalar)0.8 && AllNodesFinite(ellipsoid),
        $"minComY={minComY}, rebound={maxRebound}, finalVol={ellipsoid.Volume}");
    world.Dispose();
}

// ---------- T7.1 Scene 4: CreateFromTetGenData tetra soft body free-fall & pose recovery ----------
{
    var world = CreateSoftWorld();
    AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, Matrix.Translation(new Vector3(0, -1, 0)));

    const string tetNodeData =
        "8 3 0 0\n" +
        "0 -0.5 -0.5 -0.5\n" +
        "1  0.5 -0.5 -0.5\n" +
        "2  0.5  0.5 -0.5\n" +
        "3 -0.5  0.5 -0.5\n" +
        "4 -0.5 -0.5  0.5\n" +
        "5  0.5 -0.5  0.5\n" +
        "6  0.5  0.5  0.5\n" +
        "7 -0.5  0.5  0.5\n";
    const string tetEleData =
        "5 4 0\n" +
        "0 0 1 3 4\n" +
        "1 2 3 1 6\n" +
        "2 5 1 4 6\n" +
        "3 7 4 3 6\n" +
        "4 1 3 4 6\n";

    var tetBody = SoftBodyHelpers.CreateFromTetGenData(world.WorldInfo, tetEleData, null, tetNodeData, false, true, true);
    SoftBodyHelpers.GenerateBoundaryFaces(tetBody);
    Check("T7.1 TetGen topology & boundary faces",
        tetBody.Nodes.Count == 8 && tetBody.Tetras.Count == 5 && tetBody.Links.Count == 18 && tetBody.Faces.Count == 12,
        $"N={tetBody.Nodes.Count}, T={tetBody.Tetras.Count}, L={tetBody.Links.Count}, F={tetBody.Faces.Count}");

    tetBody.Materials[0].Lst = (btScalar)0.6;
    tetBody.Materials[0].Vst = (btScalar)0.8;
    tetBody.Cfg.PIterations = 5;
    tetBody.Cfg.MT = (btScalar)0.2;
    tetBody.SetVolumeMass((btScalar)4.0);
    tetBody.SetPose(false, true);

    Quaternion tilt = new Quaternion((btScalar)0.25, (btScalar)0.1, (btScalar)0.2, (btScalar)0.94).Normalized();
    tetBody.Rotate(in tilt);
    tetBody.Translate(0, (btScalar)3.5, 0);
    tetBody.AddVelocity(new Vector3((btScalar)1.0, 0, 0), 0);
    world.AddSoftBody(tetBody);

    for (int i = 0; i < 300; i++)
        world.StepSimulation((btScalar)(1.0 / 60.0), 10);

    btScalar minY = (btScalar)1e9, maxY = -(btScalar)1e9;
    for (int i = 0; i < tetBody.Nodes.Count; i++)
    {
        btScalar ny = tetBody.Nodes[i].Position.Y;
        minY = BtMath.Min(minY, ny);
        maxY = BtMath.Max(maxY, ny);
    }

    ReadOnlySpan<(int A, int B)> cubeEdges =
    [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ];
    btScalar maxEdgeErr = 0;
    for (int i = 0; i < cubeEdges.Length; i++)
    {
        var (ea, eb) = cubeEdges[i];
        btScalar len = (tetBody.Nodes[ea].Position - tetBody.Nodes[eb].Position).Length();
        maxEdgeErr = BtMath.Max(maxEdgeErr, BtMath.Abs(len - 1));
    }

    Check("T7.1 TetGen body settles on ground without inversion",
        minY > -(btScalar)0.1 && maxY < (btScalar)2.0 && (maxY - minY) > (btScalar)0.75 && AllNodesFinite(tetBody),
        $"minY={minY}, maxY={maxY}");
    Check("T7.1 TetGen pose & edge lengths recover",
        maxEdgeErr < (btScalar)0.15 && BtMath.Abs(tetBody.Pose.Rotation.Determinant() - 1) < (btScalar)1e-3,
        $"maxEdgeErr={maxEdgeErr}, det={tetBody.Pose.Rotation.Determinant()}");
    world.Dispose();
}

// ---------- T7.1 Scene 5a: GenerateClusters + LJoint / AJoint coupling ----------
{
    var world = CreateSoftWorld();
    AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, Matrix.Translation(new Vector3(0, -1, 0)));

    var softA = SoftBodyHelpers.CreateEllipsoid(world.WorldInfo, new Vector3(0, 6, 0),
        new Vector3((btScalar)0.6, (btScalar)0.6, (btScalar)0.6), 32);
    var softB = SoftBodyHelpers.CreateEllipsoid(world.WorldInfo, new Vector3(2, 6, 0),
        new Vector3((btScalar)0.6, (btScalar)0.6, (btScalar)0.6), 32);

    foreach (SoftBody sb in new[] { softA, softB })
    {
        sb.Materials[0].Lst = (btScalar)0.8;
        sb.Cfg.DF = (btScalar)1.0;
        sb.Cfg.PIterations = 4;
        sb.Cfg.CIterations = 8;
        sb.Cfg.Collisions = Collisions.CL_RS | Collisions.CL_SS;
        sb.SetTotalMass((btScalar)2.0);
    }

    int clCountA = softA.GenerateClusters(1);
    int clCountB = softB.GenerateClusters(1);
    world.AddSoftBody(softA);
    world.AddSoftBody(softB);

    // Anchor softA cluster to world frame and hinge-couple softB to softA around Z axis
    softA.AppendLinearJoint(new LJoint.Specs { Position = new Vector3(0, 6, 0) }, new Body());
    softA.AppendAngularJoint(new AJoint.Specs { Axis = new Vector3(0, 0, 1) }, new Body());
    softA.AppendLinearJoint(new LJoint.Specs { Position = new Vector3(1, 6, 0) }, softB);
    softA.AppendAngularJoint(new AJoint.Specs { Axis = new Vector3(0, 0, 1) }, softB);

    Check("T7.1 Cluster & LJoint/AJoint setup",
        clCountA == 1 && clCountB == 1 && softA.Joints.Count == 4);

    for (int i = 0; i < 180; i++)
        world.StepSimulation((btScalar)(1.0 / 60.0), 10);

    Vector3 comA = softA.Clusters[0].Com;
    Vector3 comB = softB.Clusters[0].Com;
    btScalar distAB = (comB - comA).Length();

    Check("T7.1 World LJoint holds softA cluster near anchor",
        (comA - new Vector3(0, 6, 0)).Length() < (btScalar)0.5, $"comA={comA}");
    Check("T7.1 LJoint/AJoint couples softB pendulum motion in XY plane",
        comB.Y < (btScalar)5.5 && comB.Y > (btScalar)2.5 &&
        distAB > (btScalar)1.2 && distAB < (btScalar)2.8 &&
        BtMath.Abs(comB.Z) < (btScalar)0.25 && AllNodesFinite(softA) && AllNodesFinite(softB),
        $"comB={comB}, distAB={distAB}");
    world.Dispose();
}

// ---------- T7.1 Scene 5b: Multi-cluster CL_RS & CL_SS soft-soft stacking collision ----------
{
    var world = CreateSoftWorld();
    AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, Matrix.Translation(new Vector3(0, -1, 0)));

    var softBottom = SoftBodyHelpers.CreateEllipsoid(world.WorldInfo, new Vector3(0, (btScalar)0.8, 0),
        new Vector3((btScalar)1.6, (btScalar)0.5, (btScalar)1.6), 48);
    var softTop = SoftBodyHelpers.CreateEllipsoid(world.WorldInfo, new Vector3(0, (btScalar)2.2, 0),
        new Vector3((btScalar)1.0, (btScalar)0.5, (btScalar)1.0), 48);

    foreach (SoftBody sb in new[] { softBottom, softTop })
    {
        sb.Materials[0].Lst = (btScalar)0.8;
        sb.Cfg.DF = (btScalar)1.0;
        sb.Cfg.PIterations = 4;
        sb.Cfg.CIterations = 6;
        sb.Cfg.Collisions = Collisions.CL_RS | Collisions.CL_SS;
        sb.SetTotalMass((btScalar)3.0);
    }

    int kBottom = softBottom.GenerateClusters(4);
    int kTop = softTop.GenerateClusters(4);
    world.AddSoftBody(softBottom);
    world.AddSoftBody(softTop);

    Check("T7.1 Multi-cluster generation count", kBottom >= 2 && kTop >= 2, $"kBottom={kBottom}, kTop={kTop}");

    for (int i = 0; i < 180; i++)
        world.StepSimulation((btScalar)(1.0 / 60.0), 10);

    btScalar bottomY = AvgNodeY(softBottom);
    btScalar topY = AvgNodeY(softTop);
    Check("T7.1 CL_RS supports bottom clustered soft body on rigid ground",
        bottomY > (btScalar)0.2 && bottomY < (btScalar)1.0 && AllNodesFinite(softBottom),
        $"bottomY={bottomY}");
    Check("T7.1 CL_SS stacks top clustered soft body above bottom",
        topY > bottomY + (btScalar)0.5 && topY < (btScalar)2.2 && AllNodesFinite(softTop),
        $"topY={topY}, bottomY={bottomY}");
    world.Dispose();
}

// ---------- T7.1 Scene 6: SoftRigidDynamicsWorld.RayTest & SoftBody.RayTest penetration ----------
{
    var world = CreateSoftWorld();
    Check("T7.1 SoftRigidDynamicsWorld WorldType", world.WorldType == DynamicsWorldType.SoftRigid);

    var ground = AddBody(world, new BoxShape(new Vector3(20, 1, 20)), 0, Matrix.Translation(new Vector3(0, -1, 0)));
    var sphere = AddBody(world, new SphereShape((btScalar)0.6), 0, Matrix.Translation(new Vector3(0, 2, 0)));

    var cloth = SoftBodyHelpers.CreatePatch(
        world.WorldInfo,
        new Vector3(-2, 5, -2),
        new Vector3(2, 5, -2),
        new Vector3(-2, 5, 2),
        new Vector3(2, 5, 2),
        5, 5, 15, true);
    world.AddSoftBody(cloth);
    world.StepSimulation((btScalar)(1.0 / 60.0), 1);

    Vector3 rayTop = new((btScalar)0.2, 10, (btScalar)0.2);
    Vector3 rayBot = new((btScalar)0.2, -5, (btScalar)0.2);

    var cbClothFirst = new ClosestRayResultCallback(rayTop, rayBot);
    world.RayTest(rayTop, rayBot, cbClothFirst);
    Check("T7.1 World RayTest hits closest soft body before rigid sphere",
        cbClothFirst.HasHit && cbClothFirst.CollisionObject == cloth &&
        SoftBody.Upcast(cbClothFirst.CollisionObject) == cloth &&
        BtMath.Abs(cbClothFirst.HitPointWorld.Y - 5) < (btScalar)0.2,
        $"hit={cbClothFirst.HasHit}, y={cbClothFirst.HitPointWorld.Y}");

    Vector3 rayBelowCloth = new((btScalar)0.2, 4, (btScalar)0.2);
    var cbSphere = new ClosestRayResultCallback(rayBelowCloth, rayBot);
    world.RayTest(rayBelowCloth, rayBot, cbSphere);
    Check("T7.1 World RayTest below cloth hits rigid sphere",
        cbSphere.HasHit && cbSphere.CollisionObject == sphere &&
        BtMath.Abs(cbSphere.HitPointWorld.Y - (btScalar)2.53) < (btScalar)0.05,
        $"hit={cbSphere.HasHit}, y={cbSphere.HitPointWorld.Y}");

    Vector3 rayOutsideSoft = new(5, 10, 5);
    Vector3 rayOutsideSoftEnd = new(5, -5, 5);
    var cbGround = new ClosestRayResultCallback(rayOutsideSoft, rayOutsideSoftEnd);
    world.RayTest(rayOutsideSoft, rayOutsideSoftEnd, cbGround);
    Check("T7.1 World RayTest outside soft body hits rigid ground",
        cbGround.HasHit && cbGround.CollisionObject == ground &&
        BtMath.Abs(cbGround.HitPointWorld.Y) < (btScalar)0.05,
        $"hit={cbGround.HasHit}, y={cbGround.HitPointWorld.Y}");

    sRayCast softHit = default;
    bool directHit = cloth.RayTest(in rayTop, in rayBot, ref softHit);
    Check("T7.1 SoftBody.RayTest reports Face hit & exact fraction",
        directHit && softHit.Body == cloth && softHit.Feature == FeatureType.Face &&
        softHit.Index >= 0 && softHit.Index < cloth.Faces.Count &&
        BtMath.Abs(softHit.Fraction - (btScalar)(1.0 / 3.0)) < (btScalar)0.02,
        $"feat={softHit.Feature}, idx={softHit.Index}, frac={softHit.Fraction}");

    world.RemoveCollisionObject(cloth);
    var cbPenetrateAfterRemove = new ClosestRayResultCallback(rayTop, rayBot);
    world.RayTest(rayTop, rayBot, cbPenetrateAfterRemove);
    Check("T7.1 RemoveCollisionObject unregisters soft body & ray penetrates to sphere",
        world.SoftBodyArray.Count == 0 && cbPenetrateAfterRemove.HasHit &&
        cbPenetrateAfterRemove.CollisionObject == sphere);

    world.Dispose();
}

Console.WriteLine();
Console.WriteLine($"Passed {passed}, Failed {failed}");
return failed == 0 ? 0 : 1;

file sealed class Phase3RecordingSoftBodySolver : SoftBodySolver
{
    public int SoftSoftCalls;
    public int SoftRigidCalls;
    public CollisionShape LastRigidShape;

    public override SolverTypes SolverType => SolverTypes.Default;
    public override bool CheckInitialized() => true;
    public override void Optimize(List<SoftBody> softBodies, bool forceUpdate = false) { }
    public override void CopyBackToSoftBodies(bool bMove = true) { }
    public override void PredictMotion(btScalar solverdt) { }
    public override void SolveConstraints(btScalar solverdt) { }
    public override void UpdateSoftBodies() { }

    public override void ProcessCollision(SoftBody softBody, CollisionObjectWrapper collisionObjectWrap)
    {
        SoftRigidCalls++;
        LastRigidShape = collisionObjectWrap.CollisionShape;
    }

    public override void ProcessCollision(SoftBody softBody, SoftBody otherSoftBody)
    {
        SoftSoftCalls++;
    }
}
