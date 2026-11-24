using System.Globalization;
using BulletSharp;
using BulletSharp.Math;
using BulletSharp.SoftBody;

// Differential per-frame trace runner for SoftBody — mirrors tests/native-ref/SoftRef.cpp.
// Usage: dotnet run -- soft <s1|s2a|s2b|s3|s4|s5a|s5b|all> [steps]
internal static class SoftDump
{
    private static string F(btScalar v) => ((double)v).ToString("e8", CultureInfo.InvariantCulture);

    private static SoftRigidDynamicsWorld CreateSoftWorld()
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

    private static RigidBody AddBody(SoftRigidDynamicsWorld world, CollisionShape shape, btScalar mass,
        Vector3 pos, btScalar friction = (btScalar)0.5, btScalar restitution = 0)
    {
        Vector3 inertia = Vector3.Zero;
        if (mass != 0)
            shape.CalculateLocalInertia(mass, out inertia);
        var info = new RigidBodyConstructionInfo(mass, new DefaultMotionState(Matrix.Translation(pos)), shape, inertia)
        {
            Friction = friction,
            Restitution = restitution,
        };
        var body = new RigidBody(info);
        world.AddRigidBody(body);
        return body;
    }

    private static void DumpTopo(int bi, SoftBody sb)
    {
        Console.WriteLine(
            $"  sb{bi} topo=({sb.Nodes.Count} {sb.Links.Count} {sb.Faces.Count} {sb.Tetras.Count} {sb.Clusters.Count})");
    }

    private static void DumpRigid(int idx, RigidBody rb)
    {
        Matrix wt = rb.WorldTransform;
        Vector3 p = new(wt.M41, wt.M42, wt.M43);
        Quaternion q = Quaternion.FromMatrix(in wt);
        Vector3 lv = rb.LinearVelocity;
        Vector3 av = rb.AngularVelocity;
        Console.WriteLine(
            $"  rb{idx} pos=({F(p.X)} {F(p.Y)} {F(p.Z)}) q=({F(q.X)} {F(q.Y)} {F(q.Z)} {F(q.W)}) lv=({F(lv.X)} {F(lv.Y)} {F(lv.Z)}) av=({F(av.X)} {F(av.Y)} {F(av.Z)})");
    }

    private static void DumpClusters(int bi, SoftBody sb)
    {
        for (int ci = 0; ci < sb.Clusters.Count; ++ci)
        {
            Cluster cl = sb.Clusters[ci];
            Vector3 com = cl.Com;
            Vector3 lv = cl.LinearVelocity;
            Vector3 av = cl.AngularVelocity;
            Console.WriteLine(
                $"  sb{bi} cl{ci} com=({F(com.X)} {F(com.Y)} {F(com.Z)}) lv=({F(lv.X)} {F(lv.Y)} {F(lv.Z)}) av=({F(av.X)} {F(av.Y)} {F(av.Z)})");
        }
    }

    private static void DumpNodes(int bi, SoftBody sb, ReadOnlySpan<int> indices)
    {
        for (int i = 0; i < indices.Length; ++i)
        {
            int ni = indices[i];
            Node n = sb.Nodes[ni];
            Vector3 x = n.Position;
            Vector3 v = n.Velocity;
            Console.WriteLine(
                $"  sb{bi} n{ni} x=({F(x.X)} {F(x.Y)} {F(x.Z)}) v=({F(v.X)} {F(v.Y)} {F(v.Z)})");
        }
    }

    private static void StepWorld(SoftRigidDynamicsWorld world)
    {
        btScalar dt = (btScalar)1 / (btScalar)60;
        world.StepSimulation(dt, 10, dt);
    }

    private static void RunS1(int steps)
    {
        if (steps <= 0) steps = 180;
        using var world = CreateSoftWorld();
        AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, new Vector3(0, -1, 0));

        var rope = SoftBodyHelpers.CreateRope(world.WorldInfo, new Vector3(0, 8, 0), new Vector3(3, 8, 0), 8, 1);
        rope.Cfg.PIterations = 8;
        int lastNode = rope.Nodes.Count - 1;

        var box = AddBody(world, new BoxShape(new Vector3((btScalar)0.4, (btScalar)0.4, (btScalar)0.4)),
            (btScalar)1.0, new Vector3(3, (btScalar)7.5, 0));
        rope.AppendAnchor(lastNode, box, true);
        world.AddSoftBody(rope);

        ReadOnlySpan<int> sampleIdx = [0, 4, lastNode];
        Console.WriteLine($"scene s1 steps={steps}");
        DumpTopo(0, rope);
        Console.WriteLine("step -1");
        DumpRigid(0, box);
        DumpNodes(0, rope, sampleIdx);

        for (int s = 0; s < steps; ++s)
        {
            StepWorld(world);
            Console.WriteLine($"step {s}");
            DumpRigid(0, box);
            DumpNodes(0, rope, sampleIdx);
        }
    }

    private static void RunS2a(int steps)
    {
        if (steps <= 0) steps = 180;
        using var world = CreateSoftWorld();
        AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, new Vector3(0, -1, 0));

        var cloth = SoftBodyHelpers.CreatePatch(
            world.WorldInfo,
            new Vector3(-2, 6, -2),
            new Vector3(2, 6, -2),
            new Vector3(-2, 6, 2),
            new Vector3(2, 6, 2),
            9, 9, 15, true);
        cloth.GenerateBendingConstraints(2);
        cloth.Cfg.PIterations = 8;
        cloth.SetTotalMass((btScalar)12.0);
        world.AddSoftBody(cloth);

        var sphere = AddBody(world, new SphereShape((btScalar)0.8),
            (btScalar)0.3, new Vector3(0, (btScalar)7.5, 0));

        ReadOnlySpan<int> sampleIdx = [0, 20, 40, 60, 80];
        Console.WriteLine($"scene s2a steps={steps}");
        DumpTopo(0, cloth);
        Console.WriteLine("step -1");
        DumpRigid(0, sphere);
        DumpNodes(0, cloth, sampleIdx);

        for (int s = 0; s < steps; ++s)
        {
            StepWorld(world);
            Console.WriteLine($"step {s}");
            DumpRigid(0, sphere);
            DumpNodes(0, cloth, sampleIdx);
        }
    }

    private static void RunS2b(int steps)
    {
        if (steps <= 0) steps = 180;
        using var world = CreateSoftWorld();

        var mesh = new TriangleMesh();
        for (int gz = -4; gz < 4; ++gz)
        {
            for (int gx = -4; gx < 4; ++gx)
            {
                btScalar x0 = gx * (btScalar)2, x1 = (gx + 1) * (btScalar)2;
                btScalar z0 = gz * (btScalar)2, z1 = (gz + 1) * (btScalar)2;
                mesh.AddTriangle(new Vector3(x0, 0, z0), new Vector3(x0, 0, z1), new Vector3(x1, 0, z1));
                mesh.AddTriangle(new Vector3(x0, 0, z0), new Vector3(x1, 0, z1), new Vector3(x1, 0, z0));
            }
        }
        var meshShape = new BvhTriangleMeshShape(mesh, true);
        AddBody(world, meshShape, 0, Vector3.Zero);

        var freeCloth = SoftBodyHelpers.CreatePatch(
            world.WorldInfo,
            new Vector3((btScalar)(-1.5), (btScalar)2.5, (btScalar)(-1.5)),
            new Vector3((btScalar)1.5, (btScalar)2.5, (btScalar)(-1.5)),
            new Vector3((btScalar)(-1.5), (btScalar)2.5, (btScalar)1.5),
            new Vector3((btScalar)1.5, (btScalar)2.5, (btScalar)1.5),
            6, 6, 0, true);
        freeCloth.GenerateBendingConstraints(2);
        freeCloth.Cfg.PIterations = 4;
        freeCloth.SetTotalMass((btScalar)3.0);
        world.AddSoftBody(freeCloth);

        ReadOnlySpan<int> sampleIdx = [0, 7, 14, 21, 35];
        Console.WriteLine($"scene s2b steps={steps}");
        DumpTopo(0, freeCloth);
        Console.WriteLine("step -1");
        DumpNodes(0, freeCloth, sampleIdx);

        for (int s = 0; s < steps; ++s)
        {
            StepWorld(world);
            Console.WriteLine($"step {s}");
            DumpNodes(0, freeCloth, sampleIdx);
        }
    }

    private static void RunS3(int steps)
    {
        if (steps <= 0) steps = 200;
        using var world = CreateSoftWorld();
        AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, new Vector3(0, -1, 0));

        var ellipsoid = SoftBodyHelpers.CreateEllipsoid(world.WorldInfo, new Vector3(0, 4, 0), new Vector3(1, 1, 1), 64);
        ellipsoid.Materials[0].Lst = (btScalar)0.2;
        ellipsoid.Cfg.DF = (btScalar)1.0;
        ellipsoid.Cfg.DP = (btScalar)0.001;
        ellipsoid.Cfg.PR = (btScalar)800;
        ellipsoid.Cfg.PIterations = 4;
        ellipsoid.SetTotalMass((btScalar)30);
        world.AddSoftBody(ellipsoid);

        int n = ellipsoid.Nodes.Count;
        ReadOnlySpan<int> sampleIdx = [0, n / 4, n / 2, (3 * n) / 4, n - 1];
        Console.WriteLine($"scene s3 steps={steps}");
        DumpTopo(0, ellipsoid);
        Console.WriteLine("step -1");
        Console.WriteLine($"  sb0 vol={F(ellipsoid.Volume)}");
        DumpNodes(0, ellipsoid, sampleIdx);

        for (int s = 0; s < steps; ++s)
        {
            StepWorld(world);
            Console.WriteLine($"step {s}");
            Console.WriteLine($"  sb0 vol={F(ellipsoid.Volume)}");
            DumpNodes(0, ellipsoid, sampleIdx);
        }
    }

    private static void RunS4(int steps)
    {
        if (steps <= 0) steps = 180;
        using var world = CreateSoftWorld();
        AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, new Vector3(0, -1, 0));

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

        ReadOnlySpan<int> sampleIdx = [0, 2, 5, 7];
        Console.WriteLine($"scene s4 steps={steps}");
        DumpTopo(0, tetBody);
        Console.WriteLine("step -1");
        Vector3 pcom0 = tetBody.Pose.Com;
        Console.WriteLine($"  sb0 pose_com=({F(pcom0.X)} {F(pcom0.Y)} {F(pcom0.Z)})");
        DumpNodes(0, tetBody, sampleIdx);

        for (int s = 0; s < steps; ++s)
        {
            StepWorld(world);
            Console.WriteLine($"step {s}");
            Vector3 pcom = tetBody.Pose.Com;
            Console.WriteLine($"  sb0 pose_com=({F(pcom.X)} {F(pcom.Y)} {F(pcom.Z)})");
            DumpNodes(0, tetBody, sampleIdx);
        }
    }

    private static void RunS5a(int steps)
    {
        if (steps <= 0) steps = 180;
        using var world = CreateSoftWorld();
        AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, new Vector3(0, -1, 0));

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

        softA.GenerateClusters(1);
        softB.GenerateClusters(1);
        world.AddSoftBody(softA);
        world.AddSoftBody(softB);

        softA.AppendLinearJoint(new LJoint.Specs { Position = new Vector3(0, 6, 0) }, new Body());
        softA.AppendAngularJoint(new AJoint.Specs { Axis = new Vector3(0, 0, 1) }, new Body());
        softA.AppendLinearJoint(new LJoint.Specs { Position = new Vector3(1, 6, 0) }, softB);
        softA.AppendAngularJoint(new AJoint.Specs { Axis = new Vector3(0, 0, 1) }, softB);

        int nA = softA.Nodes.Count;
        int nB = softB.Nodes.Count;
        ReadOnlySpan<int> idxA = [0, nA / 2, nA - 1];
        ReadOnlySpan<int> idxB = [0, nB / 2, nB - 1];

        Console.WriteLine($"scene s5a steps={steps}");
        DumpTopo(0, softA);
        DumpTopo(1, softB);
        Console.WriteLine("step -1");
        DumpClusters(0, softA);
        DumpClusters(1, softB);
        DumpNodes(0, softA, idxA);
        DumpNodes(1, softB, idxB);

        for (int s = 0; s < steps; ++s)
        {
            StepWorld(world);
            Console.WriteLine($"step {s}");
            DumpClusters(0, softA);
            DumpClusters(1, softB);
            DumpNodes(0, softA, idxA);
            DumpNodes(1, softB, idxB);
        }
    }

    private static void RunS5b(int steps)
    {
        if (steps <= 0) steps = 180;
        using var world = CreateSoftWorld();
        AddBody(world, new BoxShape(new Vector3(50, 1, 50)), 0, new Vector3(0, -1, 0));

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

        softBottom.GenerateClusters(4);
        softTop.GenerateClusters(4);
        world.AddSoftBody(softBottom);
        world.AddSoftBody(softTop);

        int nBot = softBottom.Nodes.Count;
        int nTop = softTop.Nodes.Count;
        ReadOnlySpan<int> idxBot = [0, nBot / 2, nBot - 1];
        ReadOnlySpan<int> idxTop = [0, nTop / 2, nTop - 1];

        Console.WriteLine($"scene s5b steps={steps}");
        DumpTopo(0, softBottom);
        DumpTopo(1, softTop);
        Console.WriteLine("step -1");
        DumpClusters(0, softBottom);
        DumpClusters(1, softTop);
        DumpNodes(0, softBottom, idxBot);
        DumpNodes(1, softTop, idxTop);

        for (int s = 0; s < steps; ++s)
        {
            StepWorld(world);
            Console.WriteLine($"step {s}");
            DumpClusters(0, softBottom);
            DumpClusters(1, softTop);
            DumpNodes(0, softBottom, idxBot);
            DumpNodes(1, softTop, idxTop);
        }
    }

    public static int Run(string scene, int steps = 0)
    {
        switch (scene)
        {
            case "s1": RunS1(steps); break;
            case "s2a": RunS2a(steps); break;
            case "s2b": RunS2b(steps); break;
            case "s3": RunS3(steps); break;
            case "s4": RunS4(steps); break;
            case "s5a": RunS5a(steps); break;
            case "s5b": RunS5b(steps); break;
            case "all":
                RunS1(steps);
                RunS2a(steps);
                RunS2b(steps);
                RunS3(steps);
                RunS4(steps);
                RunS5a(steps);
                RunS5b(steps);
                break;
            default:
                Console.Error.WriteLine($"Unknown scene '{scene}'. Expected s1|s2a|s2b|s3|s4|s5a|s5b|all");
                return 1;
        }
        return 0;
    }
}
