using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btStaticPlaneShape: infinite static collision plane.
/// </summary>
public class StaticPlaneShape : ConcaveShape
{
    internal Vector3 planeNormal;
    internal btScalar planeConstant;

    /// <summary>btStaticPlaneShape::btStaticPlaneShape(planeNormal, planeConstant)</summary>
    public StaticPlaneShape(Vector3 planeNormal, btScalar planeConstant)
    {
        shapeType = BroadphaseNativeType.StaticPlaneShape;
        this.planeNormal = planeNormal.Normalized();
        this.planeConstant = planeConstant;
    }

    /// <summary>btStaticPlaneShape::getAabb (infinite)</summary>
    internal override void GetAabb(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax)
    {
        aabbMin = new Vector3(-BtMath.BtLargeFloat);
        aabbMax = new Vector3(BtMath.BtLargeFloat);
    }

    /// <summary>btStaticPlaneShape::processAllTriangles (generates the two triangles of a plane quad over the query AABB)</summary>
    public override void ProcessAllTriangles(TriangleCallback callback, Vector3 aabbMin, Vector3 aabbMax)
    {
        Vector3 halfExtents = (aabbMax - aabbMin) * (btScalar)0.5;
        btScalar radius = halfExtents.Length();
        Vector3 center = (aabbMax + aabbMin) * (btScalar)0.5;

        // Generate the triangles from the AABB and the plane equation (normal/constant)
        // tangentDir0/tangentDir1 can be precalculated
        BtMath.PlaneSpace1(in planeNormal, out Vector3 tangentDir0, out Vector3 tangentDir1);

        Vector3 projectedCenter = center - (planeNormal.Dot(center) - planeConstant) * planeNormal;

        Vector3[] triangle = new Vector3[3];
        triangle[0] = projectedCenter + tangentDir0 * radius + tangentDir1 * radius;
        triangle[1] = projectedCenter + tangentDir0 * radius - tangentDir1 * radius;
        triangle[2] = projectedCenter - tangentDir0 * radius - tangentDir1 * radius;

        callback.ProcessTriangle(triangle, 0, 0);

        triangle[0] = projectedCenter - tangentDir0 * radius - tangentDir1 * radius;
        triangle[1] = projectedCenter - tangentDir0 * radius + tangentDir1 * radius;
        triangle[2] = projectedCenter + tangentDir0 * radius + tangentDir1 * radius;

        callback.ProcessTriangle(triangle, 0, 1);
    }

    /// <summary>btStaticPlaneShape::calculateLocalInertia (concave shapes cannot move, so inertia is zero)</summary>
    public override void CalculateLocalInertia(btScalar mass, out Vector3 inertia)
    {
        inertia = Vector3.Zero;
    }

    /// <summary>btStaticPlaneShape::getPlaneNormal</summary>
    public Vector3 PlaneNormal
    {
        get => planeNormal;
        set => planeNormal = value;
    }

    /// <summary>btStaticPlaneShape::getPlaneConstant</summary>
    public btScalar PlaneConstant
    {
        get => planeConstant;
        set => planeConstant = value;
    }
}
