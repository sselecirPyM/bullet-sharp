using BenchmarkDotNet.Attributes;
using BulletSharp;
using BulletSharp.Math;
using BulletSharp.SoftBody;

namespace SoftBodyBenchmarks;

[MemoryDiagnoser]
public class ClothRigidBodyBenchmarks
{
    private const int WarmupSteps = 1200;
    private static readonly btScalar FixedTimeStep = (btScalar)(1.0 / 60.0);

    private SoftRigidDynamicsWorld clothOnlyWorld;
    private SoftRigidDynamicsWorld clothRigidWorld;
    private SoftBody clothWithSphere;
    private RigidBody sphereBody;

    [GlobalSetup]
    public void Setup()
    {
        clothOnlyWorld = CreateClothScene(addFallingSphere: false, out _, out _);
        for (int i = 0; i < WarmupSteps; i++)
        {
            clothOnlyWorld.StepSimulation(FixedTimeStep, 1, FixedTimeStep);
        }

        clothRigidWorld = CreateClothScene(addFallingSphere: true, out clothWithSphere, out sphereBody);
        for (int i = 0; i < WarmupSteps; i++)
        {
            clothRigidWorld.StepSimulation(FixedTimeStep, 1, FixedTimeStep);
        }

        btScalar sphereY = sphereBody.WorldTransform.M42;
        if (sphereY <= (btScalar)3.0 || sphereY >= (btScalar)7.0)
        {
            throw new InvalidOperationException($"Sphere failed to settle in cloth hammock during setup (sphereY={sphereY}).");
        }
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        clothOnlyWorld?.Dispose();
        clothOnlyWorld = null;
        clothRigidWorld?.Dispose();
        clothRigidWorld = null;
    }

    [Benchmark(Baseline = true)]
    public void ClothOnlyStep()
    {
        clothOnlyWorld.StepSimulation(FixedTimeStep, 1, FixedTimeStep);
    }

    [Benchmark]
    public void ClothRigidCollisionStep()
    {
        clothRigidWorld.StepSimulation(FixedTimeStep, 1, FixedTimeStep);
    }

    public static int VerifyZeroSteadyStateAllocations()
    {
        var bench = new ClothRigidBodyBenchmarks();
        try
        {
            bench.Setup();

            // Promote hot paths past tiered JIT compilation before measuring allocations.
            for (int i = 0; i < 600; i++)
            {
                bench.ClothOnlyStep();
                bench.ClothRigidCollisionStep();
            }

            const int measuredSteps = 600;

            long beforeClothOnly = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < measuredSteps; i++)
            {
                bench.ClothOnlyStep();
            }
            long clothOnlyBytes = GC.GetAllocatedBytesForCurrentThread() - beforeClothOnly;

            long beforeClothRigid = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < measuredSteps; i++)
            {
                bench.ClothRigidCollisionStep();
            }
            long clothRigidBytes = GC.GetAllocatedBytesForCurrentThread() - beforeClothRigid;

            btScalar sphereY = bench.sphereBody.WorldTransform.M42;
            btScalar centerY = bench.clothWithSphere.Nodes[40].Position.Y;
            Console.WriteLine($"ClothOnlyStep           : {clothOnlyBytes} B total over {measuredSteps} steps ({(double)clothOnlyBytes / measuredSteps:F2} B/step)");
            Console.WriteLine($"ClothRigidCollisionStep : {clothRigidBytes} B total over {measuredSteps} steps ({(double)clothRigidBytes / measuredSteps:F2} B/step, centerY={centerY:F3}, sphereY={sphereY:F3}, rcontacts={bench.clothWithSphere.RContacts.Count})");

            if (clothOnlyBytes != 0 || clothRigidBytes != 0)
            {
                Console.Error.WriteLine("FAIL: Unexpected steady-state managed heap allocations detected.");
                return 1;
            }

            if (sphereY <= (btScalar)3.0 || sphereY >= (btScalar)7.0)
            {
                Console.Error.WriteLine($"FAIL: Sphere left expected steady-state hammock height (sphereY={sphereY}).");
                return 1;
            }

            Console.WriteLine("PASS: 0 B/step steady-state managed heap allocation verified.");
            return 0;
        }
        finally
        {
            bench.Cleanup();
        }
    }

    private static SoftRigidDynamicsWorld CreateClothScene(bool addFallingSphere, out SoftBody cloth, out RigidBody sphere)
    {
        var broadphase = new DbvtBroadphase();
        var config = new SoftBodyRigidBodyCollisionConfiguration();
        var dispatcher = new CollisionDispatcher(config);
        var solver = new SequentialImpulseConstraintSolver();
        var world = new SoftRigidDynamicsWorld(dispatcher, broadphase, solver, config);
        var g = new Vector3(0, (btScalar)(-9.81), 0);
        world.SetGravity(ref g);
        world.WorldInfo.Gravity = g;

        AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, Matrix.Translation(new Vector3(0, -1, 0)));

        cloth = SoftBodyHelpers.CreatePatch(
            world.WorldInfo,
            new Vector3(-2, 6, -2),
            new Vector3(2, 6, -2),
            new Vector3(-2, 6, 2),
            new Vector3(2, 6, 2),
            9,
            9,
            15,
            true);
        cloth.GenerateBendingConstraints(2);
        cloth.Cfg.PIterations = 8;
        cloth.SetTotalMass((btScalar)12.0);
        world.AddSoftBody(cloth);

        sphere = addFallingSphere
            ? AddBody(world, new SphereShape((btScalar)0.8), (btScalar)0.3, Matrix.Translation(new Vector3(0, (btScalar)7.5, 0)))
            : null;

        return world;
    }

    private static RigidBody AddBody(
        DiscreteDynamicsWorld world,
        CollisionShape shape,
        btScalar mass,
        Matrix startTransform,
        btScalar friction = (btScalar)0.5)
    {
        Vector3 inertia = Vector3.Zero;
        if (mass != 0)
        {
            shape.CalculateLocalInertia(mass, out inertia);
        }

        var info = new RigidBodyConstructionInfo(mass, new DefaultMotionState(startTransform), shape, inertia)
        {
            Friction = friction,
        };
        var body = new RigidBody(info);
        world.AddRigidBody(body);
        return body;
    }
}
