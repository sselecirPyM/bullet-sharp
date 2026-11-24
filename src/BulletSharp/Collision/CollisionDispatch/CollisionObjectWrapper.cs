using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btCollisionObjectWrapper (carries child shape info during narrowphase/concave/compound dispatch).
/// </summary>
public class CollisionObjectWrapper
{
    internal CollisionObjectWrapper parent;
    internal CollisionShape collisionShape;
    internal CollisionObject collisionObject;
    internal Transform worldTransform;
    internal int partId;
    internal int index;

    internal CollisionObjectWrapper(CollisionObjectWrapper parent, CollisionShape shape, CollisionObject collisionObject,
        in Transform worldTransform, int partId, int index)
    {
        this.parent = parent;
        collisionShape = shape;
        this.collisionObject = collisionObject;
        this.worldTransform = worldTransform;
        this.partId = partId;
        this.index = index;
    }

    /// <summary>In-place reuse (avoids allocating a wrapper per pair per step during narrowphase dispatch;
    /// equivalent to a C++ stack object).</summary>
    internal void Set(CollisionObjectWrapper parent, CollisionShape shape, CollisionObject collisionObject,
        in Transform worldTransform, int partId, int index)
    {
        this.parent = parent;
        collisionShape = shape;
        this.collisionObject = collisionObject;
        this.worldTransform = worldTransform;
        this.partId = partId;
        this.index = index;
    }

    internal CollisionObjectWrapper(CollisionObjectWrapper parent, CollisionShape shape, CollisionObject collisionObject,
        in Transform worldTransform, in Transform preTransform, int partId, int index)
    {
        this.parent = parent;
        collisionShape = shape;
        this.collisionObject = collisionObject;
        this.worldTransform = worldTransform * preTransform;
        this.partId = partId;
        this.index = index;
    }

    public CollisionObjectWrapper(CollisionObjectWrapper parent, CollisionShape shape, CollisionObject collisionObject,
        Matrix worldTransform, int partId, int index)
        : this(parent, shape, collisionObject, Transform.FromMatrix(in worldTransform), partId, index)
    {
    }

    public CollisionShape CollisionShape => collisionShape;
    public CollisionObject CollisionObject => collisionObject;
    public Matrix WorldTransform => worldTransform.ToMatrix();
    public int PartId => partId;
    public int Index => index;

    internal Transform WorldTransformInternal => worldTransform;
}
