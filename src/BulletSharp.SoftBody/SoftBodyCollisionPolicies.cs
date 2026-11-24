using BulletSharp.Math;

namespace BulletSharp.SoftBody;

/// <summary>
/// Base class for cluster convex collision policies.
/// Ported from BulletSoftBody/btSoftBodyInternals.h (btSoftColliders::ClusterBase).
/// </summary>
public class ClusterBase : Dbvt.ICollide
{
    internal btScalar erp = 1;
    internal btScalar idt;
    internal btScalar margin;
    internal btScalar friction;
    internal btScalar threshold;

    public btScalar Erp
    {
        get => erp;
        set => erp = value;
    }

    public btScalar Idt
    {
        get => idt;
        set => idt = value;
    }

    public btScalar Margin
    {
        get => margin;
        set => margin = value;
    }

    public btScalar Friction
    {
        get => friction;
        set => friction = value;
    }

    public btScalar Threshold
    {
        get => threshold;
        set => threshold = value;
    }

    public bool SolveContact(in GjkEpaSolver2.Results res, Body ba, Body bb, CJoint joint)
    {
        if (res.distance < margin)
        {
            Vector3 norm = res.normal.Normalized();

            Transform xformA = ba.Xform();
            Transform xformB = bb.Xform();
            Vector3 ra = res.witness0 - xformA.Origin;
            Vector3 rb = res.witness1 - xformB.Origin;
            Vector3 va = ba.Velocity(in ra);
            Vector3 vb = bb.Velocity(in rb);
            Vector3 vrel = va - vb;
            btScalar rvac = Vector3.Dot(vrel, norm);
            btScalar depth = res.distance - margin;

            Vector3 iv = norm * rvac;
            Vector3 fv = vrel - iv;
            joint.bodies[0] = ba;
            joint.bodies[1] = bb;
            joint.refs[0] = ra * xformA.Basis;
            joint.refs[1] = rb * xformB.Basis;
            joint.rpos[0] = ra;
            joint.rpos[1] = rb;
            joint.cfm = 1;
            joint.erp = 1;
            joint.life = 0;
            joint.maxlife = 0;
            joint.split = 1;

            joint.drift = depth * norm;

            joint.normal = norm;
            joint.delete = false;
            joint.friction = fv.LengthSquared() < (rvac * friction * rvac * friction) ? 1 : friction;
            joint.massmatrix = SoftBodyInternals.ImpulseMatrix(
                ba.InvMass(),
                ba.InvWorldInertia(),
                in joint.rpos[0],
                bb.InvMass(),
                bb.InvWorldInertia(),
                in joint.rpos[1]);

            return true;
        }
        return false;
    }
}

/// <summary>
/// Cluster tree vs convex rigid/collision shape narrowphase policy.
/// Ported from BulletSoftBody/btSoftBodyInternals.h (btSoftColliders::CollideCL_RS).
/// </summary>
public class CollideClRs : ClusterBase
{
    internal SoftBody psb;
    internal CollisionObjectWrapper colObjWrap;

    public SoftBody Psb
    {
        get => psb;
        set => psb = value;
    }

    public CollisionObjectWrapper ColObjWrap
    {
        get => colObjWrap;
        set => colObjWrap = value;
    }

    public override void Process(DbvtNode leaf)
    {
        var cluster = (Cluster)leaf.data;
        if (colObjWrap.collisionShape is not ConvexShape rshape)
        {
            return;
        }

        // Don't collide an anchored cluster with a static/kinematic object
        if (colObjWrap.collisionObject.IsStaticOrKinematicObject && cluster.containsAnchor)
        {
            return;
        }

        var cshape = new SoftClusterCollisionShape(cluster);
        Transform identity = Transform.Identity;
        GjkEpaSolver2.Results res = default;
        if (GjkEpaSolver2.SignedDistance(
                cshape,
                in identity,
                rshape,
                in colObjWrap.worldTransform,
                new Vector3(1, 0, 0),
                ref res))
        {
            if (res.distance < margin)
            {
                var pj = new CJoint();
                if (SolveContact(in res, new Body(cluster), new Body(colObjWrap.collisionObject), pj))
                {
                    psb.joints.Add(pj);
                    if (colObjWrap.collisionObject.IsStaticOrKinematicObject)
                    {
                        pj.erp *= psb.cfg.kSKHR_CL;
                        pj.split *= psb.cfg.kSK_SPLT_CL;
                    }
                    else
                    {
                        pj.erp *= psb.cfg.kSRHR_CL;
                        pj.split *= psb.cfg.kSR_SPLT_CL;
                    }
                }
            }
        }
    }

    public void ProcessColObj(SoftBody ps, CollisionObjectWrapper colObWrap)
    {
        psb = ps;
        colObjWrap = colObWrap;
        idt = ps.sst.isdt;
        margin = colObjWrap.collisionShape.Margin + psb.collisionShape.Margin;
        friction = BtMath.Min(psb.cfg.kDF, colObjWrap.collisionObject.Friction);

        colObWrap.collisionShape.GetAabb(in colObWrap.worldTransform, out Vector3 mins, out Vector3 maxs);
        DbvtVolume volume = DbvtVolume.FromMM(mins, maxs);
        volume.Expand(new Vector3(1, 1, 1) * margin);
        ps.cdbvt.CollideTVNoStackAlloc(ps.cdbvt.root, volume, ps.collideTvStack, this);
    }
}

/// <summary>
/// Cluster tree vs cluster tree narrowphase policy (heterogeneous and self-collision).
/// Ported from BulletSoftBody/btSoftBodyInternals.h (btSoftColliders::CollideCL_SS).
/// </summary>
public class CollideClSs : ClusterBase
{
    internal readonly SoftBody[] bodies = new SoftBody[2];

    public SoftBody[] Bodies => bodies;

    public override void Process(DbvtNode la, DbvtNode lb)
    {
        var cla = (Cluster)la.data;
        var clb = (Cluster)lb.data;

        bool connected = false;
        if (ReferenceEquals(bodies[0], bodies[1]) && bodies[0].clusterConnectivity.Count > 0)
        {
            connected = bodies[0].clusterConnectivity[cla.clusterIndex + bodies[0].clusters.Count * clb.clusterIndex];
        }

        if (!connected)
        {
            var csa = new SoftClusterCollisionShape(cla);
            var csb = new SoftClusterCollisionShape(clb);
            Transform identity = Transform.Identity;
            GjkEpaSolver2.Results res = default;
            if (GjkEpaSolver2.SignedDistance(
                    csa,
                    in identity,
                    csb,
                    in identity,
                    cla.com - clb.com,
                    ref res))
            {
                if (res.distance < margin)
                {
                    var pj = new CJoint();
                    if (SolveContact(in res, new Body(cla), new Body(clb), pj))
                    {
                        bodies[0].joints.Add(pj);
                        pj.erp *= BtMath.Max(bodies[0].cfg.kSSHR_CL, bodies[1].cfg.kSSHR_CL);
                        pj.split *= (bodies[0].cfg.kSS_SPLT_CL + bodies[1].cfg.kSS_SPLT_CL) / 2;
                    }
                }
            }
        }
    }

    public void ProcessSoftSoft(SoftBody psa, SoftBody psb)
    {
        idt = psa.sst.isdt;
        margin = psa.collisionShape.Margin + psb.collisionShape.Margin;
        friction = BtMath.Min(psa.cfg.kDF, psb.cfg.kDF);
        bodies[0] = psa;
        bodies[1] = psb;
        psa.cdbvt.CollideTTpersistentStack(psa.cdbvt.root, psb.cdbvt.root, this);
    }
}

/// <summary>
/// Soft body node tree vs rigid/collision object SparseSdf narrowphase policy.
/// Ported from BulletSoftBody/btSoftBodyInternals.h (btSoftColliders::CollideSDF_RS).
/// </summary>
public class CollideSdfRs : Dbvt.ICollide
{
    internal SoftBody psb;
    internal CollisionObjectWrapper colObj1Wrap;
    internal RigidBody rigidBody;
    internal btScalar dynmargin;
    internal btScalar stamargin;

    public SoftBody Psb
    {
        get => psb;
        set => psb = value;
    }

    public CollisionObjectWrapper ColObj1Wrap
    {
        get => colObj1Wrap;
        set => colObj1Wrap = value;
    }

    public RigidBody RigidBody
    {
        get => rigidBody;
        set => rigidBody = value;
    }

    public btScalar DynMargin
    {
        get => dynmargin;
        set => dynmargin = value;
    }

    public btScalar StaMargin
    {
        get => stamargin;
        set => stamargin = value;
    }

    public override void Process(DbvtNode leaf)
    {
        var node = (Node)leaf.data;
        DoNode(node);
    }

    public void DoNode(Node n)
    {
        btScalar m = n.im > 0 ? dynmargin : stamargin;
        RContact c = default;

        if (n.battach == 0 && psb.CheckContact(colObj1Wrap, in n.x, m, ref c.cti))
        {
            btScalar ima = n.im;
            btScalar imb = rigidBody != null ? rigidBody.InvMass : 0;
            btScalar ms = ima + imb;
            if (ms > 0)
            {
                Transform wtr = rigidBody != null ? rigidBody.worldTransform : colObj1Wrap.collisionObject.worldTransform;
                Matrix3x3 iwi = rigidBody != null ? rigidBody.invInertiaTensorWorld : default;
                Vector3 ra = n.x - wtr.Origin;
                Vector3 va = rigidBody != null ? rigidBody.GetVelocityInLocalPoint(ref ra) * psb.sst.sdt : Vector3.Zero;
                Vector3 vb = n.x - n.q;
                Vector3 vr = vb - va;
                btScalar dn = Vector3.Dot(vr, c.cti.normal);
                Vector3 fv = vr - c.cti.normal * dn;
                btScalar fc = psb.cfg.kDF * colObj1Wrap.collisionObject.Friction;
                c.node = n;
                c.c0 = SoftBodyInternals.ImpulseMatrix(psb.sst.sdt, ima, imb, in iwi, in ra);
                c.c1 = ra;
                c.c2 = ima * psb.sst.sdt;
                c.c3 = fv.LengthSquared() < (dn * fc * dn * fc) ? 0 : 1 - fc;
                c.c4 = colObj1Wrap.collisionObject.IsStaticOrKinematicObject ? psb.cfg.kKHR : psb.cfg.kCHR;
                psb.rcontacts.Add(c);
                rigidBody?.Activate();
            }
        }
    }
}

/// <summary>
/// Soft body vertex (node tree) vs soft body face (face tree) narrowphase policy.
/// Ported from BulletSoftBody/btSoftBodyInternals.h (btSoftColliders::CollideVF_SS).
/// </summary>
public class CollideVfSs : Dbvt.ICollide
{
    internal readonly SoftBody[] psb = new SoftBody[2];
    internal btScalar mrg;

    public SoftBody[] Psb => psb;

    public btScalar Mrg
    {
        get => mrg;
        set => mrg = value;
    }

    public override void Process(DbvtNode lnode, DbvtNode lface)
    {
        var node = (Node)lnode.data;
        var face = (Face)lface.data;
        for (int i = 0; i < 3; ++i)
        {
            if (ReferenceEquals(face.n[i], node))
            {
                return;
            }
        }

        Vector3 o = node.x;
        Vector3 p = Vector3.Zero;
        btScalar d = BtMath.SimdInfinity;
        SoftBodyInternals.ProjectOrigin(
            face.n[0].x - o,
            face.n[1].x - o,
            face.n[2].x - o,
            ref p,
            ref d);
        btScalar m = mrg + (o - node.q).Length() * 2;
        if (d < (m * m))
        {
            Node n0 = face.n[0];
            Node n1 = face.n[1];
            Node n2 = face.n[2];
            Vector3 w = SoftBodyInternals.BaryCoord(in n0.x, in n1.x, in n2.x, p + o);
            btScalar ma = node.im;
            btScalar mb = SoftBodyInternals.BaryEval(n0.im, n1.im, n2.im, in w);
            if (n0.im <= 0 || n1.im <= 0 || n2.im <= 0)
            {
                mb = 0;
            }
            btScalar ms = ma + mb;
            if (ms > 0)
            {
                SContact c = default;
                c.normal = d > 0 ? p / -BtMath.Sqrt(d) : face.normal;
                c.margin = m;
                c.node = node;
                c.face = face;
                c.weights = w;
                c.friction = BtMath.Max(psb[0].cfg.kDF, psb[1].cfg.kDF);
                c.cfm.V0 = ma / ms * psb[0].cfg.kSHR;
                c.cfm.V1 = mb / ms * psb[1].cfg.kSHR;
                psb[0].scontacts.Add(c);
            }
        }
    }
}
