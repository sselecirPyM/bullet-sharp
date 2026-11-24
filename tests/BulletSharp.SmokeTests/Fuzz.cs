using BulletSharp;
using BulletSharp.Math;

// Differential fuzz test for the 6dof spring constraint — call-for-call mirror of
// tests/native-ref/FuzzRef.cpp.
// Usage: dotnet run -- fuzz <seed>; prints the final state of body B for comparison
// with the native FuzzRef.exe.
internal static class SmokeFuzz
{
    private static uint _s;

    private static float Next()
    {
        _s = _s * 1664525u + 1013904223u;
        return (_s >> 8) * (1.0f / 16777216.0f);
    }

    public static int Run(int seed, bool dumpCfg = false)
    {
        _s = (uint)seed * 2654435761u + 12345u;

        var broadphase = new DbvtBroadphase();
        var config = new DefaultCollisionConfiguration();
        var dispatcher = new CollisionDispatcher(config);
        var solver = new SequentialImpulseConstraintSolver();
        var world = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, config);
        var gravity = new Vector3(0, (btScalar)(-9.81), 0);
        world.SetGravity(ref gravity);

        // Random pose for A
        Vector3 pA = new(Next() * 2 - 1, Next() * 2 - 1, Next() * 2 - 1);
        Vector3 axisA = new(Next() * 2 - 1, Next() * 2 - 1, Next() * 2 - 1);
        float angleA = (Next() * 2 - 1) * 0.4f;
        if (axisA.LengthSquared() < 1e-6f) axisA = new Vector3(1, 0, 0);
        axisA.Normalize();
        var qA = new Quaternion(axisA, (btScalar)angleA);

        // B's relative offset and orientation
        Vector3 offB = new((Next() * 2 - 1) * 0.3f, (Next() * 2 - 1) * 0.3f, (Next() * 2 - 1) * 0.3f);
        Vector3 axisB = new(Next() * 2 - 1, Next() * 2 - 1, Next() * 2 - 1);
        float angleB = (Next() * 2 - 1) * 0.4f;
        if (axisB.LengthSquared() < 1e-6f) axisB = new Vector3(1, 0, 0);
        axisB.Normalize();
        var qB = new Quaternion(axisB, (btScalar)angleB);
        float massB = 0.5f + Next() * 2.0f;

        // Constraint frames (coincident in world space, random rotation)
        Vector3 axisF = new(Next() * 2 - 1, Next() * 2 - 1, Next() * 2 - 1);
        float angleF = (Next() * 2 - 1) * 1.0f;
        if (axisF.LengthSquared() < 1e-6f) axisF = new Vector3(1, 0, 0);
        axisF.Normalize();
        var qF = new Quaternion(axisF, (btScalar)angleF);
        Vector3 midF = (pA + pA + offB) * (btScalar)0.5 + new Vector3((Next() * 2 - 1) * 0.1f, (Next() * 2 - 1) * 0.1f, (Next() * 2 - 1) * 0.1f);
        // Row-major: R * T ≡ btTransform(R, t)
        Matrix fW = Matrix.RotationQuaternion(qF) * Matrix.Translation(midF);

        // Limit and spring configuration (6 DOF)
        Vector3 linLo, linHi, angLo, angHi;
        linLo = linHi = angLo = angHi = Vector3.Zero;
        bool[] springOn = new bool[6];
        float[] stiff = new float[6], damp = new float[6];
        for (int i = 0; i < 3; i++)
        {
            float r = Next();
            if (r < 0.3f) { linLo[i] = 0; linHi[i] = 0; }
            else if (r < 0.6f) { float l = 0.1f + r; linLo[i] = -l; linHi[i] = l; }
            else { linLo[i] = -100; linHi[i] = 100; }
        }
        for (int i = 0; i < 3; i++)
        {
            float r = Next();
            if (r < 0.3f) { angLo[i] = 0; angHi[i] = 0; }
            else if (r < 0.6f) { float l = 0.1f + r; angLo[i] = -l; angHi[i] = l; }
            else { angLo[i] = -100; angHi[i] = 100; }
        }
        for (int i = 0; i < 6; i++)
        {
            float r = Next();
            springOn[i] = r >= 0.5f;
            stiff[i] = springOn[i] ? 1.0f + r * 99.0f : 0.0f;
            damp[i] = 0.5f + Next() * 0.5f;
        }

        RigidBody AddBox(btScalar mass, Vector3 pos, Quaternion q)
        {
            var shape = new BoxShape(new Vector3((btScalar)0.3, (btScalar)0.3, (btScalar)0.3));
            Vector3 inertia = Vector3.Zero;
            if (mass != 0) shape.CalculateLocalInertia(mass, out inertia);
            Matrix start = Matrix.RotationQuaternion(q) * Matrix.Translation(pos);
            var info = new RigidBodyConstructionInfo(mass, new DefaultMotionState(start), shape, inertia);
            var body = new RigidBody(info);
            body.ActivationState = ActivationState.DisableDeactivation;
            body.SetSleepingThresholds(0, 0);
            world.AddRigidBody(body);
            return body;
        }

        var a = AddBox(0, pA, qA);
        a.CollisionFlags |= CollisionFlags.KinematicObject;
        var b = AddBox((btScalar)massB, pA + offB, qB);

        Matrix worldA = Matrix.RotationQuaternion(qA) * Matrix.Translation(pA);
        Matrix worldB = Matrix.RotationQuaternion(qB) * Matrix.Translation(pA + offB);
        // Row-major mirror: native fa = worldA.inverse() * fW → row: fW * worldA.Inverse()
        Matrix fa = fW * worldA.Inverse();
        Matrix fb = fW * worldB.Inverse();
        var joint = new Generic6DofSpringConstraint(a, b, fa, fb, true);
        joint.LinearLowerLimit = linLo;
        joint.LinearUpperLimit = linHi;
        joint.AngularLowerLimit = angLo;
        joint.AngularUpperLimit = angHi;
        for (int i = 0; i < 6; i++)
        {
            joint.EnableSpring(i, springOn[i]);
            if (springOn[i])
            {
                joint.SetStiffness(i, (btScalar)stiff[i]);
                joint.SetDamping(i, (btScalar)damp[i]);
            }
        }
        world.AddConstraint(joint);

        if (dumpCfg)
        {
            Console.WriteLine($"cfg pA=({pA.X:F5} {pA.Y:F5} {pA.Z:F5}) qA=({qA.X:F5} {qA.Y:F5} {qA.Z:F5} {qA.W:F5})");
            Vector3 pB = pA + offB;
            Console.WriteLine($"cfg pB=({pB.X:F5} {pB.Y:F5} {pB.Z:F5}) qB=({qB.X:F5} {qB.Y:F5} {qB.Z:F5} {qB.W:F5}) mB={massB:F5}");
            Console.WriteLine($"cfg fW org=({midF.X:F5} {midF.Y:F5} {midF.Z:F5}) qF=({qF.X:F5} {qF.Y:F5} {qF.Z:F5} {qF.W:F5})");
            Console.WriteLine($"cfg fa org=({fa.M41:F5} {fa.M42:F5} {fa.M43:F5}) fb org=({fb.M41:F5} {fb.M42:F5} {fb.M43:F5})");
            Console.WriteLine($"cfg linLo=({linLo.X:F3} {linLo.Y:F3} {linLo.Z:F3}) linHi=({linHi.X:F3} {linHi.Y:F3} {linHi.Z:F3})");
            Console.WriteLine($"cfg angLo=({angLo.X:F3} {angLo.Y:F3} {angLo.Z:F3}) angHi=({angHi.X:F3} {angHi.Y:F3} {angHi.Z:F3})");
            Console.WriteLine($"cfg spring={(springOn[0] ? 1 : 0)}{(springOn[1] ? 1 : 0)}{(springOn[2] ? 1 : 0)}{(springOn[3] ? 1 : 0)}{(springOn[4] ? 1 : 0)}{(springOn[5] ? 1 : 0)} " +
                $"stiff=({stiff[0]:F2} {stiff[1]:F2} {stiff[2]:F2} {stiff[3]:F2} {stiff[4]:F2} {stiff[5]:F2}) " +
                $"damp=({damp[0]:F3} {damp[1]:F3} {damp[2]:F3} {damp[3]:F3} {damp[4]:F3} {damp[5]:F3})");
        }

        for (int i = 0; i < 60; i++)
            world.StepSimulation((btScalar)(1.0 / 60.0), 1);

        {
            Matrix sm = b.WorldTransform;
            Quaternion sq = Quaternion.FromMatrix(in sm);
            Console.WriteLine($"settle pos=({sm.M41:F5} {sm.M42:F5} {sm.M43:F5}) quat=({sq.X:F5} {sq.Y:F5} {sq.Z:F5} {sq.W:F5})");
        }

        // Continuously move kinematic A (deterministic smooth path; double-precision trig
        // keeps both sides identical)
        for (int k = 0; k < 180; k++)
        {
            double dx = 0.2 * Math.Sin(k * 0.11), dy = 0.15 * Math.Sin(k * 0.07), dz = 0.1 * Math.Cos(k * 0.09);
            double wob = 0.15 * Math.Sin(k * 0.05);
            var qk = new Quaternion(new Vector3(0, 1, 0), (btScalar)wob);
            Matrix nt = Matrix.RotationQuaternion(qA * qk) *
                Matrix.Translation(pA + new Vector3((btScalar)dx, (btScalar)dy, (btScalar)dz));
            a.MotionState.WorldTransform = nt;
            world.StepSimulation((btScalar)(1.0 / 60.0), 1);
        }

        Matrix bm = b.WorldTransform;
        Quaternion qb = Quaternion.FromMatrix(in bm);
        Console.WriteLine($"seed={seed} pos=({bm.M41:F5} {bm.M42:F5} {bm.M43:F5}) quat=({qb.X:F5} {qb.Y:F5} {qb.Z:F5} {qb.W:F5})");
        world.Dispose();
        return 0;
    }
}
