using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btConvexPointCloudShape: the implicit convex hull of a point cloud (no edge/face information).
/// Includes the local AABB caching logic of btPolyhedralConvexAabbCachingShape.
/// </summary>
public class ConvexPointCloudShape : PolyhedralConvexShape
{
    internal Vector3[] unscaledPoints;
    internal int numPoints;

    // local AABB cache of btPolyhedralConvexAabbCachingShape
    internal Vector3 cachedLocalAabbMin = new(1, 1, 1);
    internal Vector3 cachedLocalAabbMax = new(-1, -1, -1);
    internal bool isLocalAabbValid;

    /// <summary>btConvexPointCloudShape::btConvexPointCloudShape()</summary>
    public ConvexPointCloudShape()
    {
        shapeType = BroadphaseNativeType.ConvexPointCloudShape;
        unscaledPoints = null;
        numPoints = 0;
    }

    /// <summary>btConvexPointCloudShape::btConvexPointCloudShape(points, numPoints, localScaling, computeAabb)</summary>
    public ConvexPointCloudShape(Vector3[] points, int numPoints, Vector3 localScaling, bool computeAabb = true)
    {
        shapeType = BroadphaseNativeType.ConvexPointCloudShape;
        this.localScaling = localScaling;
        unscaledPoints = points;
        this.numPoints = numPoints;

        if (computeAabb)
        {
            RecalcLocalAabb();
        }
    }

    /// <summary>btConvexPointCloudShape::setPoints</summary>
    public void SetPoints(Vector3[] points, int numPoints, bool computeAabb = true)
    {
        SetPoints(points, numPoints, computeAabb, Vector3.One);
    }

    /// <summary>btConvexPointCloudShape::setPoints(points, numPoints, computeAabb, localScaling)</summary>
    public void SetPoints(Vector3[] points, int numPoints, bool computeAabb, Vector3 localScaling)
    {
        unscaledPoints = points;
        this.numPoints = numPoints;
        this.localScaling = localScaling;

        if (computeAabb)
        {
            RecalcLocalAabb();
        }
    }

    /// <summary>btConvexPointCloudShape::getUnscaledPoints</summary>
    public Vector3[] UnscaledPoints => unscaledPoints;

    /// <summary>btConvexPointCloudShape::getNumPoints</summary>
    public int NumPoints => numPoints;

    /// <summary>btConvexPointCloudShape::getScaledPoint</summary>
    public Vector3 GetScaledPoint(int index) => unscaledPoints[index] * localScaling;

    /// <summary>btConvexPointCloudShape::localGetSupportingVertexWithoutMargin</summary>
    public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec0)
    {
        Vector3 supVec = Vector3.Zero;
        btScalar maxDot = -BtMath.BtLargeFloat;

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

        if (numPoints > 0)
        {
            int index = 0;
            for (int i = 0; i < numPoints; i++)
            {
                btScalar newDot = vec.Dot(unscaledPoints[i]);
                if (newDot > maxDot)
                {
                    maxDot = newDot;
                    index = i;
                }
            }
            return GetScaledPoint(index);
        }

        return supVec;
    }

    /// <summary>btConvexPointCloudShape::localGetSupportingVertex (with margin)</summary>
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

    /// <summary>btConvexPointCloudShape::setLocalScaling (supports negative scaling)</summary>
    public override void SetLocalScaling(Vector3 scaling)
    {
        localScaling = scaling;
        RecalcLocalAabb();
    }

    protected override void InitializePolyhedralFeatures() =>
        throw new NotSupportedException("InitializePolyhedralFeatures not ported (requires btConvexHullComputer)");
}
