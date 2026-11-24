using System.Diagnostics;
using BulletSharp;
using BulletSharp.Math;

// BulletSharp (pure C#) benchmark — identical scenarios to benchmarks/native/benchmark.cpp (single-threaded).
// Run: dotnet run -c Release (float) / dotnet run -c Release -p:Precision=double (double)

static DiscreteDynamicsWorld CreateWorld()
{
    var broadphase = new DbvtBroadphase();
    var config = new DefaultCollisionConfiguration();
    var dispatcher = new CollisionDispatcher(config);
    var solver = new SequentialImpulseConstraintSolver();
    var world = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, config);
    var g = new Vector3(0, (btScalar)(-9.81), 0);
    world.SetGravity(ref g);
    return world;
}

static RigidBody AddBody(DiscreteDynamicsWorld world, CollisionShape shape, btScalar mass,
    Matrix startTransform, btScalar friction = (btScalar)0.5)
{
    Vector3 inertia = Vector3.Zero;
    if (mass != 0)
        shape.CalculateLocalInertia(mass, out inertia);
    var info = new RigidBodyConstructionInfo(mass, new DefaultMotionState(startTransform), shape, inertia)
    {
        Friction = friction,
    };
    var body = new RigidBody(info);
    world.AddRigidBody(body);
    return body;
}

// Deterministic LCG (same seed as the C++ side, so the ray distribution matches)
uint rngState = 12345;
float NextFloat() // [0, 1)
{
    rngState = rngState * 1664525u + 1013904223u;
    return (rngState >> 8) / 16777216.0f;
}

// ---------- Scenario 1: 1000 boxes fall onto the ground and stack, 600 steps ----------
static void BoxPile()
{
    using var world = CreateWorld();

    AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, Matrix.Translation(new Vector3(0, -1, 0)));

    var box = new BoxShape(new Vector3((btScalar)0.5, (btScalar)0.5, (btScalar)0.5));
    for (int iy = 0; iy < 10; iy++)
        for (int ix = 0; ix < 10; ix++)
            for (int iz = 0; iz < 10; iz++)
                AddBody(world, box, 1, Matrix.Translation(new Vector3(
                    (btScalar)((ix - 4.5) * 1.1),
                    (btScalar)(2.0 + iy * 1.1),
                    (btScalar)((iz - 4.5) * 1.1))));

    const int steps = 600;
    // JIT warmup: run a short segment before timing so the hot code paths are promoted to
    // optimized code (~100-150 ms of hot execution under tiered compilation; C++ is /O2).
    for (int i = 0; i < 60; i++)
        world.StepSimulation((btScalar)(1.0 / 60.0), 1, (btScalar)(1.0 / 60.0));

    long alloc0 = GC.GetAllocatedBytesForCurrentThread();
    long t0 = Stopwatch.GetTimestamp();
    for (int i = 0; i < steps; i++)
        world.StepSimulation((btScalar)(1.0 / 60.0), 1, (btScalar)(1.0 / 60.0));
    double elapsed = Stopwatch.GetElapsedTime(t0).TotalMilliseconds;
    long allocated = GC.GetAllocatedBytesForCurrentThread() - alloc0;

    Console.WriteLine($"boxpile : {elapsed,8:F1} ms  ({steps} steps, {elapsed / steps:F3} ms/step, alloc {allocated / 1024.0 / steps:F1} KB/step)");
}

// ---------- Scenario 2: static box pile + 50000 raycasts ----------
void Raycast()
{
    using var world = CreateWorld();

    AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, Matrix.Translation(new Vector3(0, -1, 0)));

    var box = new BoxShape(new Vector3((btScalar)0.5, (btScalar)0.5, (btScalar)0.5));
    for (int iy = 0; iy < 10; iy++)
        for (int ix = 0; ix < 10; ix++)
            for (int iz = 0; iz < 10; iz++)
                AddBody(world, box, 0, Matrix.Translation(new Vector3(
                    (btScalar)((ix - 4.5) * 1.1),
                    (btScalar)(0.5 + iy * 1.1),
                    (btScalar)((iz - 4.5) * 1.1)))); // static

    world.StepSimulation((btScalar)(1.0 / 60.0), 1, (btScalar)(1.0 / 60.0)); // build the broadphase

    rngState = 12345;
    const int rays = 50000;
    int hits = 0;
    // Reuse the callback instance (on the C++ side the callback lives on the stack, zero
    // allocation; the high-performance C# pattern is reuse + Reset)
    var cb = new ClosestRayResultCallback(Vector3.Zero, Vector3.Zero);

    // JIT warmup: run the whole loop untimed (3 passes; ~100-150 ms of hot execution per
    // pass is needed before .NET tiered compilation promotes the ray path to optimized
    // code; the timed pass below then measures steady state like the precompiled /O2 C++).
    for (int pass = 0; pass < 3; pass++)
    {
        rngState = 12345;
        for (int i = 0; i < rays; i++)
        {
            float x = NextFloat() * 30.0f - 15.0f;
            float z = NextFloat() * 30.0f - 15.0f;
            var from = new Vector3((btScalar)x, 50, (btScalar)z);
            var to = new Vector3((btScalar)x, (btScalar)(-5), (btScalar)z);
            cb.Reset(from, to);
            world.RayTest(from, to, cb);
        }
    }

    rngState = 12345;
    long alloc0 = GC.GetAllocatedBytesForCurrentThread();
    long t0 = Stopwatch.GetTimestamp();
    for (int i = 0; i < rays; i++)
    {
        float x = NextFloat() * 30.0f - 15.0f;
        float z = NextFloat() * 30.0f - 15.0f;
        var from = new Vector3((btScalar)x, 50, (btScalar)z);
        var to = new Vector3((btScalar)x, (btScalar)(-5), (btScalar)z);
        cb.Reset(from, to);
        world.RayTest(from, to, cb);
        if (cb.HasHit) hits++;
    }
    double elapsed = Stopwatch.GetElapsedTime(t0).TotalMilliseconds;
    long allocated = GC.GetAllocatedBytesForCurrentThread() - alloc0;

    Console.WriteLine($"raycast : {elapsed,8:F1} ms  ({rays} rays, {elapsed * 1000.0 / rays:F3} us/ray, hits={hits}, alloc {allocated / rays:F0} B/ray)");
}

BoxPile();
Raycast();
