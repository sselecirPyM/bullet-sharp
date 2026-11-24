using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btScaledBvhTriangleMeshShape: scaled instance of an existing btBvhTriangleMeshShape.
/// Note that each btBvhTriangleMeshShape can still have its own local scaling.
/// </summary>
public class ScaledBvhTriangleMeshShape : ConcaveShape
{
    internal Vector3 scaledLocalScaling;
    internal BvhTriangleMeshShape bvhTriMeshShape;

    /// <summary>btScaledBvhTriangleMeshShape::btScaledBvhTriangleMeshShape(childShape, localScaling)</summary>
    public ScaledBvhTriangleMeshShape(BvhTriangleMeshShape childShape, Vector3 localScaling)
    {
        shapeType = BroadphaseNativeType.ScaledTriangleMeshShape;
        scaledLocalScaling = localScaling;
        bvhTriMeshShape = childShape;
    }

    /// <summary>btScaledBvhTriangleMeshShape::processAllTriangles</summary>
    public override void ProcessAllTriangles(TriangleCallback callback, Vector3 aabbMin, Vector3 aabbMax)
    {
        ScaledTriangleCallback scaledCallback = new(callback, scaledLocalScaling);

        Vector3 invLocalScaling = new(
            1 / scaledLocalScaling.X,
            1 / scaledLocalScaling.Y,
            1 / scaledLocalScaling.Z);
        Vector3 scaledAabbMin = Vector3.Zero, scaledAabbMax = Vector3.Zero;

        // Support negative scaling
        scaledAabbMin[0] = scaledLocalScaling.X >= 0 ? aabbMin[0] * invLocalScaling[0] : aabbMax[0] * invLocalScaling[0];
        scaledAabbMin[1] = scaledLocalScaling.Y >= 0 ? aabbMin[1] * invLocalScaling[1] : aabbMax[1] * invLocalScaling[1];
        scaledAabbMin[2] = scaledLocalScaling.Z >= 0 ? aabbMin[2] * invLocalScaling[2] : aabbMax[2] * invLocalScaling[2];

        scaledAabbMax[0] = scaledLocalScaling.X <= 0 ? aabbMin[0] * invLocalScaling[0] : aabbMax[0] * invLocalScaling[0];
        scaledAabbMax[1] = scaledLocalScaling.Y <= 0 ? aabbMin[1] * invLocalScaling[1] : aabbMax[1] * invLocalScaling[1];
        scaledAabbMax[2] = scaledLocalScaling.Z <= 0 ? aabbMin[2] * invLocalScaling[2] : aabbMax[2] * invLocalScaling[2];

        bvhTriMeshShape.ProcessAllTriangles(scaledCallback, scaledAabbMin, scaledAabbMax);
    }

    /// <summary>btScaledTriangleCallback from btScaledBvhTriangleMeshShape.cpp</summary>
    private sealed class ScaledTriangleCallback : TriangleCallback
    {
        private readonly TriangleCallback originalCallback;
        private readonly Vector3 localScaling;

        public ScaledTriangleCallback(TriangleCallback originalCallback, Vector3 localScaling)
        {
            this.originalCallback = originalCallback;
            this.localScaling = localScaling;
        }

        public override void ProcessTriangle(Vector3[] triangle, int partId, int triangleIndex)
        {
            Vector3[] newTriangle = new Vector3[3];
            newTriangle[0] = triangle[0] * localScaling;
            newTriangle[1] = triangle[1] * localScaling;
            newTriangle[2] = triangle[2] * localScaling;
            originalCallback.ProcessTriangle(newTriangle, partId, triangleIndex);
        }
    }

    /// <summary>btScaledBvhTriangleMeshShape::getAabb</summary>
    internal override void GetAabb(in Transform trans, out Vector3 aabbMin, out Vector3 aabbMax)
    {
        Vector3 localAabbMin = bvhTriMeshShape.LocalAabbMin;
        Vector3 localAabbMax = bvhTriMeshShape.LocalAabbMax;

        Vector3 tmpLocalAabbMin = localAabbMin * scaledLocalScaling;
        Vector3 tmpLocalAabbMax = localAabbMax * scaledLocalScaling;

        localAabbMin[0] = scaledLocalScaling.X >= 0 ? tmpLocalAabbMin[0] : tmpLocalAabbMax[0];
        localAabbMin[1] = scaledLocalScaling.Y >= 0 ? tmpLocalAabbMin[1] : tmpLocalAabbMax[1];
        localAabbMin[2] = scaledLocalScaling.Z >= 0 ? tmpLocalAabbMin[2] : tmpLocalAabbMax[2];
        localAabbMax[0] = scaledLocalScaling.X <= 0 ? tmpLocalAabbMin[0] : tmpLocalAabbMax[0];
        localAabbMax[1] = scaledLocalScaling.Y <= 0 ? tmpLocalAabbMin[1] : tmpLocalAabbMax[1];
        localAabbMax[2] = scaledLocalScaling.Z <= 0 ? tmpLocalAabbMin[2] : tmpLocalAabbMax[2];

        Vector3 localHalfExtents = (btScalar)0.5 * (localAabbMax - localAabbMin);
        btScalar margin = bvhTriMeshShape.Margin;
        localHalfExtents += new Vector3(margin);
        Vector3 localCenter = (btScalar)0.5 * (localAabbMax + localAabbMin);

        Matrix3x3 absB = trans.Basis.Absolute();

        Vector3 center = trans * localCenter;

        Vector3 extent = AabbUtil.Dot3(absB.Row0, absB.Row1, absB.Row2, localHalfExtents);
        aabbMin = center - extent;
        aabbMax = center + extent;
    }

    /// <summary>btScaledBvhTriangleMeshShape::setLocalScaling</summary>
    public override void SetLocalScaling(Vector3 scaling) => scaledLocalScaling = scaling;

    public override Vector3 LocalScaling
    {
        get => scaledLocalScaling;
        set => SetLocalScaling(value);
    }

    /// <summary>btScaledBvhTriangleMeshShape::calculateLocalInertia (do not use this as a movable object!)</summary>
    public override void CalculateLocalInertia(btScalar mass, out Vector3 inertia)
    {
        inertia = Vector3.Zero;
    }

    /// <summary>btScaledBvhTriangleMeshShape::getChildShape</summary>
    public BvhTriangleMeshShape ChildShape => bvhTriMeshShape;
}
