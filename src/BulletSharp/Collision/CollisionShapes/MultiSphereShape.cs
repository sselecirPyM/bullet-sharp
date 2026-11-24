using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btMultiSphereShape: convex hull of a set of spheres (can be used to build
/// custom capsules or other smooth volumes). Call RecalcLocalAabb after modifying sphere
/// positions/radii. Includes the local AABB caching logic of btConvexInternalAabbCachingShape.
/// </summary>
public class MultiSphereShape : ConvexInternalShape
{
    internal readonly List<Vector3> localPositionArray = new();
    internal readonly List<btScalar> radiArray = new();

    // Local AABB cache from btConvexInternalAabbCachingShape
    internal Vector3 cachedLocalAabbMin = Vector3.Zero;
    internal Vector3 cachedLocalAabbMax = Vector3.Zero;
    internal bool isLocalAabbValid;

    /// <summary>btMultiSphereShape::btMultiSphereShape(positions, radi, numSpheres)</summary>
    public MultiSphereShape(Vector3[] positions, btScalar[] radii)
    {
        shapeType = BroadphaseNativeType.MultiSphereShape;

        int numSpheres = positions.Length;
        for (int i = 0; i < numSpheres; i++)
        {
            localPositionArray.Add(positions[i]);
            radiArray.Add(radii[i]);
        }

        RecalcLocalAabb();
    }

    /// <summary>btMultiSphereShape::localGetSupportingVertexWithoutMargin</summary>
    public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec0)
    {
        Vector3 supVec = Vector3.Zero;

        btScalar maxDot = -BtMath.BtLargeFloat;

        Vector3 vec = vec0;
        btScalar lenSqr = vec.LengthSquared();
        if (lenSqr < BtMath.SimdEpsilon * BtMath.SimdEpsilon)
        {
            vec.SetValue(1, 0, 0);
        }
        else
        {
            btScalar rlen = 1 / BtMath.Sqrt(lenSqr);
            vec *= rlen;
        }

        int numSpheres = localPositionArray.Count;
        for (int k = 0; k < numSpheres; k += 128)
        {
            int innerCount = BtMath.Min(numSpheres - k, 128);
            for (int i = 0; i < innerCount; i++)
            {
                Vector3 temp = localPositionArray[k + i] * localScaling + vec * localScaling * radiArray[k + i] - vec * Margin;
                btScalar newDot = vec.Dot(temp);
                if (newDot > maxDot)
                {
                    maxDot = newDot;
                    supVec = temp;
                }
            }
        }

        return supVec;
    }

    /// <summary>btConvexInternalAabbCachingShape::getAabb (transforms the cached local AABB + margin)</summary>
    internal override void GetAabb(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax) =>
        AabbUtil.TransformAabb(in cachedLocalAabbMin, in cachedLocalAabbMax, Margin, in t, out aabbMin, out aabbMax);

    /// <summary>btConvexInternalAabbCachingShape::recalcLocalAabb (6-direction support points + margin expansion)</summary>
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

    /// <summary>btMultiSphereShape::calculateLocalInertia (approximated by the box bounding the spheres)</summary>
    public override void CalculateLocalInertia(btScalar mass, out Vector3 inertia)
    {
        Vector3 halfExtents = (cachedLocalAabbMax - cachedLocalAabbMin) * (btScalar)0.5;

        btScalar lx = 2 * halfExtents.X;
        btScalar ly = 2 * halfExtents.Y;
        btScalar lz = 2 * halfExtents.Z;

        inertia = new Vector3(
            mass / (btScalar)12.0 * (ly * ly + lz * lz),
            mass / (btScalar)12.0 * (lx * lx + lz * lz),
            mass / (btScalar)12.0 * (lx * lx + ly * ly));
    }

    /// <summary>btConvexInternalAabbCachingShape::setLocalScaling</summary>
    public override void SetLocalScaling(Vector3 scaling)
    {
        base.SetLocalScaling(scaling);
        RecalcLocalAabb();
    }

    /// <summary>btMultiSphereShape::getSphereCount</summary>
    public int SphereCount => localPositionArray.Count;

    /// <summary>btMultiSphereShape::getSpherePosition</summary>
    public Vector3 GetSpherePosition(int index) => localPositionArray[index];

    /// <summary>btMultiSphereShape::getSphereRadius</summary>
    public btScalar GetSphereRadius(int index) => radiArray[index];
}
