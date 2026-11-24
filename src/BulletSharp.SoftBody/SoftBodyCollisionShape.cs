using BulletSharp.Math;

namespace BulletSharp.SoftBody;

/// <summary>
/// Ported from BulletSoftBody/btSoftBodyInternals.h (btSoftBodyCollisionShape).
/// </summary>
public class SoftBodyCollisionShape : ConcaveShape
{
    private sealed class FaceTriangleCollider : Dbvt.ICollide
    {
        private readonly TriangleCallback callback;
        private readonly Vector3[] triangle = new Vector3[3];

        public FaceTriangleCollider(TriangleCallback callback)
        {
            this.callback = callback;
        }

        public override void Process(DbvtNode leaf)
        {
            Face face = (Face)leaf.data;
            triangle[0] = face.n[0].x;
            triangle[1] = face.n[1].x;
            triangle[2] = face.n[2].x;
            callback.ProcessTriangle(triangle, 0, face.index);
        }
    }

    internal readonly SoftBody body;

    public SoftBodyCollisionShape(SoftBody backptr)
    {
        shapeType = BroadphaseNativeType.SoftBodyShape;
        body = backptr;
    }

    public SoftBody Body => body;

    public string Name => "SoftBody";

    public override void ProcessAllTriangles(TriangleCallback callback, Vector3 aabbMin, Vector3 aabbMax)
    {
        if (body == null || body.fdbvt.root == null)
            return;

        DbvtVolume volume = DbvtVolume.FromMM(aabbMin, aabbMax);
        var collider = new FaceTriangleCollider(callback);
        body.fdbvt.CollideTV(body.fdbvt.root, volume, collider);
    }

    /// <summary>
    /// Returns the axis-aligned bounding box in the coordinate frame of the given transform t.
    /// </summary>
    internal override void GetAabb(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax)
    {
        // t is usually identity, except when colliding against CompoundShape. See Issue 512.
        Vector3 mins = body.bounds[0];
        Vector3 maxs = body.bounds[1];

        Vector3 c0 = t * new Vector3(mins.X, mins.Y, mins.Z);
        Vector3 c1 = t * new Vector3(maxs.X, mins.Y, mins.Z);
        Vector3 c2 = t * new Vector3(maxs.X, maxs.Y, mins.Z);
        Vector3 c3 = t * new Vector3(mins.X, maxs.Y, mins.Z);
        Vector3 c4 = t * new Vector3(mins.X, mins.Y, maxs.Z);
        Vector3 c5 = t * new Vector3(maxs.X, mins.Y, maxs.Z);
        Vector3 c6 = t * new Vector3(maxs.X, maxs.Y, maxs.Z);
        Vector3 c7 = t * new Vector3(mins.X, maxs.Y, maxs.Z);

        aabbMin = c0;
        aabbMax = c0;
        aabbMin.SetMin(c1); aabbMax.SetMax(c1);
        aabbMin.SetMin(c2); aabbMax.SetMax(c2);
        aabbMin.SetMin(c3); aabbMax.SetMax(c3);
        aabbMin.SetMin(c4); aabbMax.SetMax(c4);
        aabbMin.SetMin(c5); aabbMax.SetMax(c5);
        aabbMin.SetMin(c6); aabbMax.SetMax(c6);
        aabbMin.SetMin(c7); aabbMax.SetMax(c7);
    }

    public override void SetLocalScaling(Vector3 scaling)
    {
        // Not applicable for soft body shapes.
    }

    public override Vector3 LocalScaling
    {
        get => Vector3.One;
        set { }
    }

    public override void CalculateLocalInertia(btScalar mass, out Vector3 inertia)
    {
        // Not supported for soft body shapes (btAssert(0) in C++).
        inertia = Vector3.Zero;
    }

    public override string ToString() => "SoftBody";
}

/// <summary>
/// Ported from BulletSoftBody/btSoftBodyInternals.h (btSoftClusterCollisionShape).
/// </summary>
public class SoftClusterCollisionShape : ConvexInternalShape
{
    internal readonly Cluster cluster;

    public SoftClusterCollisionShape(Cluster cluster)
    {
        shapeType = BroadphaseNativeType.SoftBodyShape;
        this.cluster = cluster;
        collisionMargin = 0;
    }

    public Cluster Cluster => cluster;

    public string Name => "SOFTCLUSTER";

    public override Vector3 LocalGetSupportingVertex(Vector3 vec)
    {
        List<Node> nodes = cluster.nodes;
        if (nodes.Count == 0)
            return Vector3.Zero;

        btScalar d = vec.Dot(nodes[0].x);
        int j = 0;
        for (int i = 1, ni = nodes.Count; i < ni; ++i)
        {
            btScalar k = vec.Dot(nodes[i].x);
            if (k > d)
            {
                d = k;
                j = i;
            }
        }
        return nodes[j].x;
    }

    public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec) =>
        LocalGetSupportingVertex(vec);

    public override void CalculateLocalInertia(btScalar mass, out Vector3 inertia)
    {
        inertia = Vector3.Zero;
    }

    internal override void GetAabb(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax)
    {
        // Empty stub in C++; here we compute the transformed cluster node bounds expanded by Margin.
        List<Node> nodes = cluster?.nodes;
        if (nodes == null || nodes.Count == 0)
        {
            aabbMin = Vector3.Zero;
            aabbMax = Vector3.Zero;
            return;
        }

        Vector3 first = t * nodes[0].x;
        aabbMin = first;
        aabbMax = first;
        for (int i = 1, ni = nodes.Count; i < ni; ++i)
        {
            Vector3 pt = t * nodes[i].x;
            aabbMin.SetMin(pt);
            aabbMax.SetMax(pt);
        }

        Vector3 marginVec = new(Margin);
        aabbMin -= marginVec;
        aabbMax += marginVec;
    }

    public override btScalar Margin
    {
        get => base.Margin;
        set => base.Margin = value;
    }

    public override string ToString() => "SOFTCLUSTER";
}
