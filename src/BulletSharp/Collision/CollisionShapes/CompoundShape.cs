using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btCompoundShapeChild.
/// </summary>
public class CompoundShapeChild
{
    internal Transform transform;
    internal CollisionShape childShape;
    internal int childShapeType;
    internal btScalar childMargin;

    /// <summary>btCompoundShapeChild::m_transform</summary>
    public Matrix ChildTransform
    {
        get => transform.ToMatrix();
        set => transform = Transform.FromMatrix(in value);
    }

    /// <summary>btCompoundShapeChild::m_childShape</summary>
    public CollisionShape ChildShape
    {
        get => childShape;
        set => childShape = value;
    }

    /// <summary>btCompoundShapeChild::m_childShapeType</summary>
    public int ChildShapeType
    {
        get => childShapeType;
        set => childShapeType = value;
    }

    /// <summary>btCompoundShapeChild::m_childMargin</summary>
    public btScalar ChildMargin
    {
        get => childMargin;
        set => childMargin = value;
    }
}

/// <summary>
/// Ported from btCompoundShape: combines multiple child collision shapes (allows a concave compound as a moving body).
/// Note: this port does not implement the dynamic aabb tree (btDbvt) acceleration.
/// </summary>
public class CompoundShape : CollisionShape
{
    internal readonly List<CompoundShapeChild> children = new();
    internal Vector3 localAabbMin = new(BtMath.BtLargeFloat);
    internal Vector3 localAabbMax = new(-BtMath.BtLargeFloat);

    /// <summary>Incremented when child shapes are added/removed/replaced, for external cache invalidation (btCompoundShape::m_updateRevision)</summary>
    internal int updateRevision = 1;

    internal btScalar collisionMargin;

    /// <summary>btCompoundShape::btCompoundShape (the enableDynamicAabbTree parameter is kept but the dynamic tree is not implemented)</summary>
    public CompoundShape(bool enableDynamicAabbTree = true)
    {
        shapeType = BroadphaseNativeType.CompoundShape;
        collisionMargin = 0;
    }

    /// <summary>btCompoundShape::addChildShape</summary>
    public void AddChildShape(Matrix localTransform, CollisionShape shape)
    {
        updateRevision++;
        Transform childTransform = Transform.FromMatrix(in localTransform);

        CompoundShapeChild child = new()
        {
            transform = childTransform,
            childShape = shape,
            childShapeType = (int)shape.ShapeType,
            childMargin = shape.Margin,
        };

        // extend localAabbMin/localAabbMax
        shape.GetAabb(in childTransform, out Vector3 localAabbMin, out Vector3 localAabbMax);
        for (int i = 0; i < 3; i++)
        {
            if (this.localAabbMin[i] > localAabbMin[i])
            {
                this.localAabbMin[i] = localAabbMin[i];
            }
            if (this.localAabbMax[i] < localAabbMax[i])
            {
                this.localAabbMax[i] = localAabbMax[i];
            }
        }

        children.Add(child);
    }

    /// <summary>btCompoundShape::updateChildTransform</summary>
    public void UpdateChildTransform(int childIndex, Matrix newChildTransform, bool shouldRecalculateLocalAabb = true)
    {
        children[childIndex].transform = Transform.FromMatrix(in newChildTransform);

        if (shouldRecalculateLocalAabb)
        {
            RecalculateLocalAabb();
        }
    }

    /// <summary>btCompoundShape::removeChildShapeByIndex (swap with last, then remove)</summary>
    public void RemoveChildShapeByIndex(int childShapeIndex)
    {
        updateRevision++;
        System.Diagnostics.Debug.Assert(childShapeIndex >= 0 && childShapeIndex < children.Count);
        children[childShapeIndex] = children[children.Count - 1];
        children.RemoveAt(children.Count - 1);
    }

    /// <summary>btCompoundShape::removeChildShape (removes all children using this shape)</summary>
    public void RemoveChildShape(CollisionShape shape)
    {
        updateRevision++;
        // note: multiple children may share the same shape!
        for (int i = children.Count - 1; i >= 0; i--)
        {
            if (children[i].childShape == shape)
            {
                RemoveChildShapeByIndex(i);
            }
        }

        RecalculateLocalAabb();
    }

    /// <summary>btCompoundShape::recalculateLocalAabb (brute-force traversal of all child shapes)</summary>
    public void RecalculateLocalAabb()
    {
        localAabbMin = new Vector3(BtMath.BtLargeFloat);
        localAabbMax = new Vector3(-BtMath.BtLargeFloat);

        for (int j = 0; j < children.Count; j++)
        {
            children[j].childShape.GetAabb(in children[j].transform, out Vector3 localAabbMin, out Vector3 localAabbMax);
            for (int i = 0; i < 3; i++)
            {
                if (this.localAabbMin[i] > localAabbMin[i])
                    this.localAabbMin[i] = localAabbMin[i];
                if (this.localAabbMax[i] < localAabbMax[i])
                    this.localAabbMax[i] = localAabbMax[i];
            }
        }
    }

    /// <summary>btCompoundShape::getNumChildShapes</summary>
    public int NumChildShapes => children.Count;

    /// <summary>Alias of NumChildShapes (for uniform calls from internal algorithms)</summary>
    internal int NumChildren => children.Count;

    /// <summary>btCompoundShape::getChildShape</summary>
    public CollisionShape GetChildShape(int index) => children[index].childShape;

    /// <summary>btCompoundShape::getChildTransform (public Matrix form)</summary>
    public Matrix GetChildTransform(int index) => children[index].transform.ToMatrix();

    /// <summary>btCompoundShape::getChildTransform (internal Transform form)</summary>
    internal Transform GetChildTransformInternal(int index) => children[index].transform;

    /// <summary>btCompoundShape::getChildList</summary>
    public List<CompoundShapeChild> ChildList => children;

    /// <summary>btCompoundShape::getUpdateRevision</summary>
    public int UpdateRevision => updateRevision;

    /// <summary>btCompoundShape::getAabb (transform of the union of the child AABBs)</summary>
    internal override void GetAabb(in Transform trans, out Vector3 aabbMin, out Vector3 aabbMax)
    {
        Vector3 localHalfExtents = (btScalar)0.5 * (localAabbMax - localAabbMin);
        Vector3 localCenter = (btScalar)0.5 * (localAabbMax + localAabbMin);

        // avoid an invalid AABB when there are no children
        if (children.Count == 0)
        {
            localHalfExtents.SetValue(0, 0, 0);
            localCenter.SetValue(0, 0, 0);
        }
        localHalfExtents += new Vector3(Margin);

        Matrix3x3 absB = trans.Basis.Absolute();

        Vector3 center = trans * localCenter;

        Vector3 extent = AabbUtil.Dot3(absB.Row0, absB.Row1, absB.Row2, localHalfExtents);
        aabbMin = center - extent;
        aabbMax = center + extent;
    }

    /// <summary>btCompoundShape::calculateLocalInertia (approximated by the AABB box)</summary>
    public override void CalculateLocalInertia(btScalar mass, out Vector3 inertia)
    {
        GetAabb(Transform.Identity, out Vector3 aabbMin, out Vector3 aabbMax);

        Vector3 halfExtents = (aabbMax - aabbMin) * (btScalar)0.5;

        btScalar lx = 2 * halfExtents.X;
        btScalar ly = 2 * halfExtents.Y;
        btScalar lz = 2 * halfExtents.Z;

        inertia = new Vector3(
            mass / (btScalar)12.0 * (ly * ly + lz * lz),
            mass / (btScalar)12.0 * (lx * lx + lz * lz),
            mass / (btScalar)12.0 * (lx * lx + ly * ly));
    }

    /// <summary>btCompoundShape::setLocalScaling (adjusts each child shape by the scaling ratio)</summary>
    public override void SetLocalScaling(Vector3 scaling)
    {
        for (int i = 0; i < children.Count; i++)
        {
            Transform childTrans = children[i].transform;
            Vector3 childScale = children[i].childShape.LocalScaling;
            childScale = childScale * scaling / localScaling;
            children[i].childShape.SetLocalScaling(childScale);
            childTrans.Origin = childTrans.Origin * scaling / localScaling;
            children[i].transform = childTrans;
        }

        localScaling = scaling;
        RecalculateLocalAabb();
    }

    /// <summary>btCompoundShape::getMargin / setMargin</summary>
    public override btScalar Margin
    {
        get => collisionMargin;
        set => collisionMargin = value;
    }
}
