using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btConvexTriangleMeshShape: the convex hull of a triangle mesh (slower than btConvexHullShape).
/// The benefit is that it uses btStridingMeshInterface, which avoids duplicating triangle mesh data.
/// Most users should prefer the faster ConvexHullShape.
/// Includes the local AABB caching logic of btPolyhedralConvexAabbCachingShape.
/// </summary>
public class ConvexTriangleMeshShape : PolyhedralConvexShape
{
    internal StridingMeshInterface stridingMesh;

    // local AABB cache of btPolyhedralConvexAabbCachingShape
    internal Vector3 cachedLocalAabbMin = new(1, 1, 1);
    internal Vector3 cachedLocalAabbMax = new(-1, -1, -1);
    internal bool isLocalAabbValid;

    /// <summary>btConvexTriangleMeshShape::btConvexTriangleMeshShape(meshInterface, calcAabb)</summary>
    public ConvexTriangleMeshShape(StridingMeshInterface meshInterface, bool calcAabb = true)
    {
        shapeType = BroadphaseNativeType.ConvexTriangleMeshShape;
        stridingMesh = meshInterface;
        if (calcAabb)
        {
            RecalcLocalAabb();
        }
    }

    /// <summary>btConvexTriangleMeshShape::getMeshInterface</summary>
    public StridingMeshInterface MeshInterface => stridingMesh;

    /// <summary>LocalSupportVertexCallback from btConvexTriangleMeshShape.cpp</summary>
    private sealed class LocalSupportVertexCallback : InternalTriangleIndexCallback
    {
        private Vector3 supportVertexLocal = Vector3.Zero;
        public btScalar maxDot = -BtMath.BtLargeFloat;
        public readonly Vector3 supportVecLocal;

        public LocalSupportVertexCallback(Vector3 supportVecLocal)
        {
            this.supportVecLocal = supportVecLocal;
        }

        public override void InternalProcessTriangleIndex(Vector3[] triangle, int partId, int triangleIndex)
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

        public Vector3 GetSupportVertexLocal() => supportVertexLocal;
    }

    /// <summary>btConvexTriangleMeshShape::localGetSupportingVertexWithoutMargin</summary>
    public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec0)
    {
        Vector3 supVec = Vector3.Zero;

        Vector3 vec = vec0;
        btScalar lenSqr = vec.LengthSquared();
        if (lenSqr < (btScalar)0.0001)
        {
            vec.SetValue(1, 0, 0);
        }
        else
        {
            btScalar rlen = 1 / BtMath.Sqrt(lenSqr);
            vec *= rlen;
        }

        LocalSupportVertexCallback supportCallback = new(vec);
        Vector3 aabbMax = new(BtMath.BtLargeFloat);
        stridingMesh.InternalProcessAllTriangles(supportCallback, -aabbMax, aabbMax);
        supVec = supportCallback.GetSupportVertexLocal();

        return supVec;
    }

    /// <summary>btConvexTriangleMeshShape::localGetSupportingVertex (with margin)</summary>
    public override Vector3 LocalGetSupportingVertex(Vector3 vec)
    {
        Vector3 supVertex = LocalGetSupportingVertexWithoutMargin(vec);

        if (Margin != 0)
        {
            Vector3 vecnorm = vec;
            if (vecnorm.LengthSquared() < BtMath.SimdEpsilon * BtMath.SimdEpsilon)
            {
                vecnorm.SetValue(-1, -1, -1);
            }
            vecnorm.Normalize();
            supVertex += Margin * vecnorm;
        }
        return supVertex;
    }

    /// <summary>btPolyhedralConvexAabbCachingShape::getAabb</summary>
    internal override void GetAabb(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax) =>
        AabbUtil.TransformAabb(in cachedLocalAabbMin, in cachedLocalAabbMax, Margin, in t, out aabbMin, out aabbMax);

    /// <summary>btPolyhedralConvexAabbCachingShape::recalcLocalAabb (support points in 6 directions + margin expansion)</summary>
    public void RecalcLocalAabb()
    {
        isLocalAabbValid = true;

        Vector3[] directions =
        {
            new(1, 0, 0), new(0, 1, 0), new(0, 0, 1),
            new(-1, 0, 0), new(0, -1, 0), new(0, 0, -1),
        };
        Vector3[] supporting = new Vector3[6];
        for (int i = 0; i < 6; i++)
        {
            supporting[i] = LocalGetSupportingVertexWithoutMargin(directions[i]);
        }

        for (int i = 0; i < 3; ++i)
        {
            cachedLocalAabbMax[i] = supporting[i][i] + collisionMargin;
            cachedLocalAabbMin[i] = supporting[i + 3][i] - collisionMargin;
        }
    }

    /// <summary>btPolyhedralConvexShape::calculateLocalInertia (approximated by the AABB box)</summary>
    public override void CalculateLocalInertia(btScalar mass, out Vector3 inertia)
    {
        btScalar margin = Margin;

        GetAabb(Transform.Identity, out Vector3 aabbMin, out Vector3 aabbMax);
        Vector3 halfExtents = (aabbMax - aabbMin) * (btScalar)0.5;

        btScalar lx = 2 * (halfExtents.X + margin);
        btScalar ly = 2 * (halfExtents.Y + margin);
        btScalar lz = 2 * (halfExtents.Z + margin);
        btScalar x2 = lx * lx;
        btScalar y2 = ly * ly;
        btScalar z2 = lz * lz;
        btScalar scaledmass = mass * (btScalar)0.08333333;

        inertia = scaledmass * new Vector3(y2 + z2, x2 + z2, x2 + y2);
    }

    /// <summary>btConvexTriangleMeshShape::setLocalScaling</summary>
    public override void SetLocalScaling(Vector3 scaling)
    {
        stridingMesh.Scaling = scaling;
        RecalcLocalAabb();
    }

    public override Vector3 LocalScaling
    {
        get => stridingMesh.Scaling;
        set => SetLocalScaling(value);
    }

    protected override void InitializePolyhedralFeatures() =>
        throw new NotSupportedException("InitializePolyhedralFeatures not ported (requires btConvexHullComputer)");
}
