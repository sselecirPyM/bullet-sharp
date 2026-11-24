using BulletSharp.Math;

namespace BulletSharp.SoftBody;

/// <summary>
/// Ported from BulletSoftBody/btSoftBody.h and btSoftBody.cpp (position/velocity constraint solvers).
/// Note: native btSoftBody defines VSolve_Links as the sole velocity solver, and applies kPR/kVC/kMT forces in PredictMotion.
/// </summary>
public partial class SoftBody
{
    public delegate void PSolverFn(SoftBody psb, btScalar kst, btScalar ti);

    public delegate void VSolverFn(SoftBody psb, btScalar kst);

    private static readonly PSolverFn s_pSolveAnchors = PSolve_Anchors;
    private static readonly PSolverFn s_pSolveLinks = PSolve_Links;
    private static readonly PSolverFn s_pSolveRContacts = PSolve_RContacts;
    private static readonly PSolverFn s_pSolveSContacts = PSolve_SContacts;
    private static readonly VSolverFn s_vSolveLinks = VSolve_Links;

    public void SetSolver(SolverPresets preset)
    {
        cfg.vsequence.Clear();
        cfg.psequence.Clear();
        cfg.dsequence.Clear();
        switch (preset)
        {
            case SolverPresets.Positions:
                cfg.psequence.Add(PSolver.Anchors);
                cfg.psequence.Add(PSolver.RContacts);
                cfg.psequence.Add(PSolver.SContacts);
                cfg.psequence.Add(PSolver.Linear);
                break;
            case SolverPresets.Velocities:
                cfg.vsequence.Add(VSolver.Linear);

                cfg.psequence.Add(PSolver.Anchors);
                cfg.psequence.Add(PSolver.RContacts);
                cfg.psequence.Add(PSolver.SContacts);

                cfg.dsequence.Add(PSolver.Linear);
                break;
        }
    }

    public void SolveConstraints()
    {
        // Apply clusters
        ApplyClusters(false);

        // Prepare links
        int i, ni;

        for (i = 0, ni = links.Count; i < ni; ++i)
        {
            Link l = links[i];
            l.c3 = l.n[1].q - l.n[0].q;
            l.c2 = 1 / (l.c3.LengthSquared() * l.c0);
        }

        // Prepare anchors
        for (i = 0, ni = anchors.Count; i < ni; ++i)
        {
            Anchor a = anchors[i];
            Vector3 ra = a.body.worldTransform.Basis * a.local;
            a.c0 = SoftBodyInternals.ImpulseMatrix(
                sst.sdt,
                a.node.im,
                a.body.InvMass,
                in a.body.invInertiaTensorWorld,
                in ra);
            a.c1 = ra;
            a.c2 = sst.sdt * a.node.im;
            a.body.Activate();
        }

        // Solve velocities
        if (cfg.viterations > 0)
        {
            // Solve
            for (int isolve = 0; isolve < cfg.viterations; ++isolve)
            {
                for (int iseq = 0; iseq < cfg.vsequence.Count; ++iseq)
                {
                    GetSolver(cfg.vsequence[iseq])(this, 1);
                }
            }
            // Update
            for (i = 0, ni = nodes.Count; i < ni; ++i)
            {
                Node n = nodes[i];
                n.x = n.q + n.v * sst.sdt;
            }
        }

        // Solve positions
        if (cfg.piterations > 0)
        {
            for (int isolve = 0; isolve < cfg.piterations; ++isolve)
            {
                btScalar ti = isolve / (btScalar)cfg.piterations;
                for (int iseq = 0; iseq < cfg.psequence.Count; ++iseq)
                {
                    GetSolver(cfg.psequence[iseq])(this, 1, ti);
                }
            }
            btScalar vc = sst.isdt * (1 - cfg.kDP);
            for (i = 0, ni = nodes.Count; i < ni; ++i)
            {
                Node n = nodes[i];
                n.v = (n.x - n.q) * vc;
                n.f = Vector3.Zero;
            }
        }

        // Solve drift
        if (cfg.diterations > 0)
        {
            btScalar vcf = cfg.kVCF * sst.isdt;
            for (i = 0, ni = nodes.Count; i < ni; ++i)
            {
                Node n = nodes[i];
                n.q = n.x;
            }
            for (int idrift = 0; idrift < cfg.diterations; ++idrift)
            {
                for (int iseq = 0; iseq < cfg.dsequence.Count; ++iseq)
                {
                    GetSolver(cfg.dsequence[iseq])(this, 1, 0);
                }
            }
            for (i = 0, ni = nodes.Count; i < ni; ++i)
            {
                Node n = nodes[i];
                n.v += (n.x - n.q) * vcf;
            }
        }

        // Apply clusters
        DampClusters();
        ApplyClusters(true);
    }

    public void StaticSolve(int iterations)
    {
        for (int isolve = 0; isolve < iterations; ++isolve)
        {
            for (int iseq = 0; iseq < cfg.psequence.Count; ++iseq)
            {
                GetSolver(cfg.psequence[iseq])(this, 1, 0);
            }
        }
    }

    public static void SolveCommonConstraints(IReadOnlyList<SoftBody> bodies, int count, int iterations)
    {
        // Placeholder matching btSoftBody::solveCommonConstraints.
    }

    public static void SolveCommonConstraints(IReadOnlyList<SoftBody> bodies, int iterations)
    {
        // Placeholder matching btSoftBody::solveCommonConstraints.
    }

    public static void PSolve_Anchors(SoftBody psb, btScalar kst, btScalar ti)
    {
        btScalar kAHR = psb.cfg.kAHR * kst;
        btScalar dt = psb.sst.sdt;
        for (int i = 0, ni = psb.anchors.Count; i < ni; ++i)
        {
            Anchor a = psb.anchors[i];
            ref readonly BulletSharp.Math.Transform t = ref a.body.worldTransform;
            Node n = a.node;
            Vector3 wa = t * a.local;
            Vector3 va = a.body.GetVelocityInLocalPoint(ref a.c1) * dt;
            Vector3 vb = n.x - n.q;
            Vector3 vr = (va - vb) + (wa - n.x) * kAHR;
            Vector3 impulse = a.c0 * vr * a.influence;
            n.x += impulse * a.c2;
            Vector3 negImpulse = -impulse;
            a.body.ApplyImpulse(ref negImpulse, ref a.c1);
        }
    }

    public static void PSolve_RContacts(SoftBody psb, btScalar kst, btScalar ti)
    {
        btScalar dt = psb.sst.sdt;
        btScalar mrg = psb.collisionShape.Margin;
        for (int i = 0, ni = psb.rcontacts.Count; i < ni; ++i)
        {
            RContact c = psb.rcontacts[i];
            ref readonly sCti cti = ref c.cti;
            if (cti.colObj.HasContactResponse)
            {
                Vector3 va = Vector3.Zero;
                RigidBody rigidCol = null;

                if (cti.colObj.internalType == CollisionObjectTypes.RigidBody)
                {
                    rigidCol = RigidBody.Upcast(cti.colObj);
                    va = rigidCol != null ? rigidCol.GetVelocityInLocalPoint(ref c.c1) * dt : Vector3.Zero;
                }

                Vector3 vb = c.node.x - c.node.q;
                Vector3 vr = vb - va;
                btScalar dn = Vector3.Dot(vr, cti.normal);
                if (dn <= BtMath.SimdEpsilon)
                {
                    btScalar pen = Vector3.Dot(c.node.x, cti.normal) + cti.offset;
                    btScalar dp = pen < mrg ? pen : mrg;
                    Vector3 fv = vr - (cti.normal * dn);
                    // c0 is the impulse matrix, c3 is 1 - the friction coefficient or 0, c4 is the contact hardness coefficient
                    Vector3 impulse = c.c0 * ((vr - (fv * c.c3) + (cti.normal * (dp * c.c4))) * kst);
                    c.node.x -= impulse * c.c2;

                    if (cti.colObj.internalType == CollisionObjectTypes.RigidBody)
                    {
                        if (rigidCol != null)
                        {
                            rigidCol.ApplyImpulse(ref impulse, ref c.c1);
                        }
                    }
                }
            }
        }
    }

    public static void PSolve_SContacts(SoftBody psb, btScalar kst, btScalar ti)
    {
        for (int i = 0, ni = psb.scontacts.Count; i < ni; ++i)
        {
            SContact c = psb.scontacts[i];
            ref readonly Vector3 nr = ref c.normal;
            Node n = c.node;
            Face f = c.face;
            Vector3 p = SoftBodyInternals.BaryEval(
                in f.n[0].x,
                in f.n[1].x,
                in f.n[2].x,
                in c.weights);
            Vector3 q = SoftBodyInternals.BaryEval(
                in f.n[0].q,
                in f.n[1].q,
                in f.n[2].q,
                in c.weights);
            Vector3 vr = (n.x - n.q) - (p - q);
            Vector3 corr = Vector3.Zero;
            btScalar dot = Vector3.Dot(vr, nr);
            if (dot < 0)
            {
                btScalar j = c.margin - (Vector3.Dot(nr, n.x) - Vector3.Dot(nr, p));
                corr += c.normal * j;
            }
            corr -= SoftBodyInternals.ProjectOnPlane(in vr, in nr) * c.friction;
            n.x += corr * c.cfm[0];
            f.n[0].x -= corr * (c.cfm[1] * c.weights.X);
            f.n[1].x -= corr * (c.cfm[1] * c.weights.Y);
            f.n[2].x -= corr * (c.cfm[1] * c.weights.Z);
        }
    }

    public static void PSolve_Links(SoftBody psb, btScalar kst, btScalar ti)
    {
        for (int i = 0, ni = psb.links.Count; i < ni; ++i)
        {
            Link l = psb.links[i];
            if (l.c0 > 0)
            {
                Node a = l.n[0];
                Node b = l.n[1];
                Vector3 del = b.x - a.x;
                btScalar len = del.LengthSquared();
                if (l.c1 + len > BtMath.SimdEpsilon)
                {
                    btScalar k = ((l.c1 - len) / (l.c0 * (l.c1 + len))) * kst;
                    a.x -= del * (k * a.im);
                    b.x += del * (k * b.im);
                }
            }
        }
    }

    public static void VSolve_Links(SoftBody psb, btScalar kst)
    {
        for (int i = 0, ni = psb.links.Count; i < ni; ++i)
        {
            Link l = psb.links[i];
            Node[] n = l.n;
            btScalar j = -Vector3.Dot(l.c3, n[0].v - n[1].v) * l.c2 * kst;
            n[0].v += l.c3 * (j * n[0].im);
            n[1].v -= l.c3 * (j * n[1].im);
        }
    }

    public static PSolverFn GetSolver(PSolver solver) => solver switch
    {
        PSolver.Anchors => s_pSolveAnchors,
        PSolver.Linear => s_pSolveLinks,
        PSolver.RContacts => s_pSolveRContacts,
        PSolver.SContacts => s_pSolveSContacts,
        _ => null,
    };

    public static VSolverFn GetSolver(VSolver solver) => solver switch
    {
        VSolver.Linear => s_vSolveLinks,
        _ => null,
    };
}
