using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btTriangleMeshShape: internal interface of concave triangle mesh shapes.
/// Do not use this class directly; use BvhTriangleMeshShape instead!
/// </summary>
public class TriangleMeshShape : ConcaveShape
{
    internal Vector3 localAabbMin;
    internal Vector3 localAabbMax;
    internal StridingMeshInterface meshInterface;

    /// <summary>btTriangleMeshShape::btTriangleMeshShape</summary>
    public TriangleMeshShape(StridingMeshInterface meshInterface)
    {
        shapeType = BroadphaseNativeType.TriangleMeshShape;
        this.meshInterface = meshInterface;
        if (meshInterface.HasPremadeAabb)
        {
            meshInterface.GetPremadeAabb(out localAabbMin, out localAabbMax);
        }
        else
        {
            RecalcLocalAabb();
        }
    }

    /// <summary>btTriangleMeshShape::getAabb → btTransformAabb(localAabbMin, localAabbMax, margin, t)</summary>
    internal override void GetAabb(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax) =>
        AabbUtil.TransformAabb(in localAabbMin, in localAabbMax, Margin, in t, out aabbMin, out aabbMax);

    /// <summary>btTriangleMeshShape::recalcLocalAabb</summary>
    public void RecalcLocalAabb()
    {
        for (int i = 0; i < 3; i++)
        {
            Vector3 vec = Vector3.Zero;
            vec[i] = 1;
            Vector3 tmp = LocalGetSupportingVertex(vec);
            localAabbMax[i] = tmp[i] + collisionMargin;
            vec[i] = -1;
            tmp = LocalGetSupportingVertex(vec);
            localAabbMin[i] = tmp[i] - collisionMargin;
        }
    }

    /// <summary>btTriangleMeshShape::setLocalScaling</summary>
    public override void SetLocalScaling(Vector3 scaling)
    {
        meshInterface.Scaling = scaling;
        RecalcLocalAabb();
    }

    public override Vector3 LocalScaling
    {
        get => meshInterface.Scaling;
        set => SetLocalScaling(value);
    }

    /// <summary>btTriangleMeshShape::processAllTriangles (FilteredCallback: forwards local-space triangles after AABB filtering)</summary>
    public override void ProcessAllTriangles(TriangleCallback callback, Vector3 aabbMin, Vector3 aabbMax)
    {
        FilteredCallback filterCallback = new(callback, aabbMin, aabbMax);
        meshInterface.InternalProcessAllTriangles(filterCallback, aabbMin, aabbMax);
    }

    private sealed class FilteredCallback : InternalTriangleIndexCallback
    {
        private readonly TriangleCallback callback;
        private readonly Vector3 aabbMin;
        private readonly Vector3 aabbMax;

        public FilteredCallback(TriangleCallback callback, Vector3 aabbMin, Vector3 aabbMax)
        {
            this.callback = callback;
            this.aabbMin = aabbMin;
            this.aabbMax = aabbMax;
        }

        public override void InternalProcessTriangleIndex(Vector3[] triangle, int partId, int triangleIndex)
        {
            if (AabbUtil.TestTriangleAgainstAabb2(in triangle[0], in triangle[1], in triangle[2], in aabbMin, in aabbMax))
            {
                // Check the AABB in triangle space before invoking the callback
                callback.ProcessTriangle(triangle, partId, triangleIndex);
            }
        }
    }

    /// <summary>btTriangleMeshShape::calculateLocalInertia (concave shapes cannot move, so inertia is zero)</summary>
    public override void CalculateLocalInertia(btScalar mass, out Vector3 inertia)
    {
        inertia = Vector3.Zero;
    }

    /// <summary>btTriangleMeshShape::localGetSupportingVertex (iterates all triangles to find the support point)</summary>
    public virtual Vector3 LocalGetSupportingVertex(Vector3 vec)
    {
        SupportVertexCallback supportCallback = new(vec, Transform.Identity);

        Vector3 aabbMax = new(BtMath.BtLargeFloat);
        ProcessAllTriangles(supportCallback, -aabbMax, aabbMax);

        return supportCallback.GetSupportVertexLocal();
    }

    /// <summary>btTriangleMeshShape::localGetSupportingVertexWithoutMargin (btAssert(0) in C++)</summary>
    public virtual Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec) => LocalGetSupportingVertex(vec);

    private sealed class SupportVertexCallback : TriangleCallback
    {
        private Vector3 supportVertexLocal = Vector3.Zero;
        private readonly Transform worldTrans;
        private btScalar maxDot = -BtMath.BtLargeFloat;
        private readonly Vector3 supportVecLocal;

        public SupportVertexCallback(Vector3 supportVecWorld, Transform trans)
        {
            worldTrans = trans;
            supportVecLocal = supportVecWorld * worldTrans.Basis;
        }

        public override void ProcessTriangle(Vector3[] triangle, int partId, int triangleIndex)
        {
            for (int i = 0; i < 3; i++)
            {
                btScalar dot = supportVecLocal.Dot(triangle[i]);
                if (dot > maxDot)
                {
                    maxDot = dot;
                    supportVertexLocal = triangle[i];
                }
            }
        }

        public Vector3 GetSupportVertexWorldSpace() => worldTrans * supportVertexLocal;

        public Vector3 GetSupportVertexLocal() => supportVertexLocal;
    }

    /// <summary>btTriangleMeshShape::getMeshInterface</summary>
    public StridingMeshInterface MeshInterface => meshInterface;

    /// <summary>btTriangleMeshShape::getLocalAabbMin</summary>
    public Vector3 LocalAabbMin => localAabbMin;

    /// <summary>btTriangleMeshShape::getLocalAabbMax</summary>
    public Vector3 LocalAabbMax => localAabbMax;
}
