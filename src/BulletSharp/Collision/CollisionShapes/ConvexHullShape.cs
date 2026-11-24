using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btConvexHullShape: the implicit convex hull of a vertex array.
/// Bullet performs general convex collision detection via localGetSupportingVertex based on GJK/EPA.
/// Includes the local AABB caching logic of btPolyhedralConvexAabbCachingShape.
/// </summary>
public class ConvexHullShape : PolyhedralConvexShape
{
    internal readonly List<Vector3> unscaledPoints = new();

    // local AABB cache of btPolyhedralConvexAabbCachingShape
    internal Vector3 cachedLocalAabbMin = new(1, 1, 1);
    internal Vector3 cachedLocalAabbMax = new(-1, -1, -1);
    internal bool isLocalAabbValid;

    /// <summary>btConvexHullShape::btConvexHullShape() (empty hull; add points one by one with AddPoint)</summary>
    public ConvexHullShape()
    {
        shapeType = BroadphaseNativeType.ConvexHullShape;
        RecalcLocalAabb();
    }

    /// <summary>btConvexHullShape::btConvexHullShape(points, numPoints) (an internal copy of the points is made)</summary>
    public ConvexHullShape(Vector3[] points)
    {
        shapeType = BroadphaseNativeType.ConvexHullShape;
        foreach (Vector3 p in points)
        {
            unscaledPoints.Add(p);
        }
        RecalcLocalAabb();
    }

    /// <summary>btConvexHullShape::addPoint</summary>
    public void AddPoint(Vector3 point, bool recalculateLocalAabb = true)
    {
        unscaledPoints.Add(point);
        if (recalculateLocalAabb)
        {
            RecalcLocalAabb();
        }
    }

    /// <summary>btConvexHullShape::getUnscaledPoints (internal storage; do not modify its semantics)</summary>
    public List<Vector3> UnscaledPoints => unscaledPoints;

    /// <summary>btConvexHullShape::getPoints (obsolete, equivalent to UnscaledPoints)</summary>
    public List<Vector3> Points => unscaledPoints;

    /// <summary>btConvexHullShape::getNumPoints</summary>
    public int NumPoints => unscaledPoints.Count;

    /// <summary>btConvexHullShape::getScaledPoint</summary>
    public Vector3 GetScaledPoint(int i) => unscaledPoints[i] * localScaling;

    /// <summary>btConvexHullShape::localGetSupportingVertexWithoutMargin (max dot over points*scaling)</summary>
    public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec)
    {
        Vector3 supVec = Vector3.Zero;
        btScalar maxDot = -BtMath.BtLargeFloat;

        // Uses dot(a, b*c) = dot(a*b, c). Note: mathematically correct, but not necessarily numerically identical.
        if (unscaledPoints.Count > 0)
        {
            Vector3 scaled = vec * localScaling;
            int index = 0;
            for (int i = 0; i < unscaledPoints.Count; i++)
            {
                btScalar newDot = scaled.Dot(unscaledPoints[i]);
                if (newDot > maxDot)
                {
                    maxDot = newDot;
                    index = i;
                }
            }
            return unscaledPoints[index] * localScaling;
        }

        return supVec;
    }

    /// <summary>btConvexHullShape::localGetSupportingVertex (with margin)</summary>
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

    /// <summary>btPolyhedralConvexAabbCachingShape::getAabb (transforms the cached local AABB + margin)</summary>
    internal override void GetAabb(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax)
    {
        // btConvexInternalAabbCachingShape::getNonvirtualAabb
        AabbUtil.TransformAabb(in cachedLocalAabbMin, in cachedLocalAabbMax, Margin, in t, out aabbMin, out aabbMax);
    }

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

    /// <summary>btConvexHullShape::setLocalScaling (supports negative scaling)</summary>
    public override void SetLocalScaling(Vector3 scaling)
    {
        localScaling = scaling;
        RecalcLocalAabb();
    }

    /// <summary>btConvexHullShape::optimizeConvexHull (not ported; requires btConvexHullComputer)</summary>
    public void OptimizeConvexHull()
    {
        // TODO: not implemented yet, requires btConvexHullComputer
    }

    /// <summary>btConvexHullShape::project (projection for debugging)</summary>
    internal new void Project(in Transform trans, Vector3 dir, out btScalar minProj, out btScalar maxProj,
        out Vector3 witnesPtMin, out Vector3 witnesPtMax)
    {
        minProj = BtMath.BtLargeFloat;
        maxProj = -BtMath.BtLargeFloat;
        witnesPtMin = Vector3.Zero;
        witnesPtMax = Vector3.Zero;

        int numVerts = unscaledPoints.Count;
        for (int i = 0; i < numVerts; i++)
        {
            Vector3 vtx = unscaledPoints[i] * localScaling;
            Vector3 pt = trans * vtx;
            btScalar dp = pt.Dot(dir);
            if (dp < minProj)
            {
                minProj = dp;
                witnesPtMin = pt;
            }
            if (dp > maxProj)
            {
                maxProj = dp;
                witnesPtMax = pt;
            }
        }

        if (minProj > maxProj)
        {
            (minProj, maxProj) = (maxProj, minProj);
            (witnesPtMin, witnesPtMax) = (witnesPtMax, witnesPtMin);
        }
    }

    /// <summary>btConvexHullShape::getNumVertices</summary>
    public int NumVertices => unscaledPoints.Count;

    /// <summary>btConvexHullShape::getNumEdges</summary>
    public int NumEdges => unscaledPoints.Count;

    /// <summary>btConvexHullShape::getEdge</summary>
    public void GetEdge(int i, out Vector3 pa, out Vector3 pb)
    {
        int index0 = i % unscaledPoints.Count;
        int index1 = (i + 1) % unscaledPoints.Count;
        pa = GetScaledPoint(index0);
        pb = GetScaledPoint(index1);
    }

    /// <summary>btConvexHullShape::getVertex</summary>
    public void GetVertex(int i, out Vector3 vtx) => vtx = GetScaledPoint(i);

    protected override void InitializePolyhedralFeatures() =>
        throw new NotSupportedException("InitializePolyhedralFeatures not ported (requires btConvexHullComputer)");
}
