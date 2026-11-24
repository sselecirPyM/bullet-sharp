using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btDiscreteCollisionDetectorInterface: interface for closest-point / penetration-depth queries between convex shapes.
/// The closest point lies on the second object (B); the normal points from B's surface towards A.
/// distance is the distance between the closest points on B and A;
/// the closest point on A is closestPointInA = closestPointInB + distance * normalOnSurfaceB.
/// </summary>
public abstract class DiscreteCollisionDetectorInterface
{
    /// <summary>btDiscreteCollisionDetectorInterface::Result (contact point output callback)</summary>
    public abstract class Result
    {
        /// <summary>setShapeIdentifiersA/B provide experimental support for per-triangle and custom material combinations</summary>
        public abstract void SetShapeIdentifiersA(int partId0, int index0);

        public abstract void SetShapeIdentifiersB(int partId1, int index1);

        public abstract void AddContactPoint(Vector3 normalOnBInWorld, Vector3 pointInWorld, btScalar depth);
    }

    /// <summary>btDiscreteCollisionDetectorInterface::ClosestPointInput</summary>
    public class ClosestPointInput
    {
        internal Transform transformA;
        internal Transform transformB;
        internal btScalar maximumDistanceSquared;

        public ClosestPointInput()
        {
            maximumDistanceSquared = BtMath.BtLargeFloat;
        }

        public Matrix TransformA
        {
            get => transformA.ToMatrix();
            set => transformA = Transform.FromMatrix(in value);
        }

        public Matrix TransformB
        {
            get => transformB.ToMatrix();
            set => transformB = Transform.FromMatrix(in value);
        }

        public btScalar MaximumDistanceSquared
        {
            get => maximumDistanceSquared;
            set => maximumDistanceSquared = value;
        }
    }

    /// <summary>
    /// Returns the closest points (distance > 0) or penetration depth (negative distance).
    /// The normal always points from B towards A.
    /// </summary>
    public abstract void GetClosestPoints(ClosestPointInput input, Result output, IDebugDraw debugDraw, bool swapResults = false);
}

/// <summary>btPointCollector: collects the single closest contact point</summary>
public class PointCollector : DiscreteCollisionDetectorInterface.Result
{
    internal Vector3 normalOnBInWorld;
    internal Vector3 pointInWorld;
    internal btScalar distance; // negative means penetration
    internal bool hasResult;

    public PointCollector()
    {
        distance = BtMath.BtLargeFloat;
    }

    /// <summary>In-place reuse (equivalent to reconstructing on the stack in C++)</summary>
    internal void Reset()
    {
        normalOnBInWorld = Vector3.Zero;
        pointInWorld = Vector3.Zero;
        distance = BtMath.BtLargeFloat;
        hasResult = false;
    }

    public Vector3 NormalOnBInWorld => normalOnBInWorld;
    public Vector3 PointInWorld => pointInWorld;
    public btScalar Distance => distance;
    public bool HasResult => hasResult;

    public override void SetShapeIdentifiersA(int partId0, int index0)
    {
    }

    public override void SetShapeIdentifiersB(int partId1, int index1)
    {
    }

    public override void AddContactPoint(Vector3 normalOnBInWorld, Vector3 pointInWorld, btScalar depth)
    {
        if (depth < distance)
        {
            hasResult = true;
            this.normalOnBInWorld = normalOnBInWorld;
            this.pointInWorld = pointInWorld;
            // negative means penetration
            distance = depth;
        }
    }
}

/// <summary>btStorageResult (simple Result implementation that stores the closest contact point)</summary>
public class StorageResult : DiscreteCollisionDetectorInterface.Result
{
    internal Vector3 normalOnSurfaceB;
    internal Vector3 closestPointInB;
    internal btScalar distance; // negative means penetration

    public StorageResult()
    {
        distance = BtMath.BtLargeFloat;
    }

    public Vector3 NormalOnSurfaceB => normalOnSurfaceB;
    public Vector3 ClosestPointInB => closestPointInB;
    public btScalar Distance => distance;

    public override void SetShapeIdentifiersA(int partId0, int index0)
    {
    }

    public override void SetShapeIdentifiersB(int partId1, int index1)
    {
    }

    public override void AddContactPoint(Vector3 normalOnBInWorld, Vector3 pointInWorld, btScalar depth)
    {
        if (depth < distance)
        {
            normalOnSurfaceB = normalOnBInWorld;
            closestPointInB = pointInWorld;
            distance = depth;
        }
    }
}
