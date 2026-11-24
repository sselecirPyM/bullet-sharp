using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btCollisionShape. Base class of all collision shapes.
/// </summary>
public abstract class CollisionShape : IDisposable
{
    internal BroadphaseNativeType shapeType;
    internal Vector3 localScaling = Vector3.One;

    /// <summary>Shape type (btBroadphaseProxy::CollisionShapeTypes)</summary>
    public BroadphaseNativeType ShapeType => shapeType;

    /// <summary>Computes the AABB (btCollisionShape::getAabb)</summary>
    internal abstract void GetAabb(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax);

    public void GetAabb(Matrix t, out Vector3 aabbMin, out Vector3 aabbMax) =>
        GetAabb(Transform.FromMatrix(in t), out aabbMin, out aabbMax);

    public void GetBoundingSphere(out Vector3 center, out btScalar radius)
    {
        GetAabb(Transform.Identity, out Vector3 aabbMin, out Vector3 aabbMax);
        radius = (aabbMax - aabbMin).Length() / 2;
        center = (aabbMin + aabbMax) / 2;
    }

    /// <summary>btCollisionShape::getAngularMotionDisc</summary>
    public btScalar GetAngularMotionDisc()
    {
        GetBoundingSphere(out Vector3 center, out btScalar radius);
        radius += center.Length();
        return radius;
    }

    /// <summary>btCollisionShape::getContactBreakingThreshold</summary>
    public btScalar GetContactBreakingThreshold(btScalar defaultFactor) => GetAngularMotionDisc() * defaultFactor;

    /// <summary>Slower but generic AABB computation (btCollisionShape::getAabbSlow)</summary>
    internal virtual void GetAabbSlow(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax)
    {
        GetBoundingSphere(out Vector3 center, out btScalar radius);
        Vector3 extent = new(radius);
        aabbMin = (t * center) - extent;
        aabbMax = (t * center) + extent;
    }

    public virtual void SetLocalScaling(Vector3 scaling) => localScaling = scaling;

    public virtual Vector3 LocalScaling
    {
        get => localScaling;
        set => SetLocalScaling(value);
    }

    /// <summary>btCollisionShape::calculateLocalInertia</summary>
    public abstract void CalculateLocalInertia(btScalar mass, out Vector3 inertia);

    public virtual Vector3 AnisotropicRollingFrictionDirection => Vector3.One;

    /// <summary>margin (only supported by convex internal shapes and concave shapes; the base class returns 0)</summary>
    public virtual btScalar Margin
    {
        get => 0;
        set { }
    }

    public bool IsConvex => BroadphaseProxy.IsConvex(shapeType);

    public bool IsConcave => BroadphaseProxy.IsConcave(shapeType);

    public bool IsCompound => BroadphaseProxy.IsCompound(shapeType);

    public bool IsPolyhedral => BroadphaseProxy.IsPolyhedral(shapeType);

    public bool IsSoftBody => BroadphaseProxy.IsSoftBody(shapeType);

    public bool IsInfinite => BroadphaseProxy.IsInfinite(shapeType);

    public virtual void Dispose()
    {
    }
}

/// <summary>
/// Ported from btConvexShape: base class of all convex shapes (support function interface).
/// </summary>
public abstract class ConvexShape : CollisionShape
{
    /// <summary>btConvexShape::localGetSupportingVertex (with margin)</summary>
    public virtual Vector3 LocalGetSupportingVertex(Vector3 vec)
    {
        Vector3 supVertex = LocalGetSupportingVertexWithoutMargin(vec);
        btScalar margin = Margin;
        if (margin != 0)
        {
            Vector3 vecnorm = vec;
            if (vecnorm.LengthSquared() < BtMath.SimdEpsilon * BtMath.SimdEpsilon)
            {
                vecnorm.SetValue(-1, -1, -1);
            }
            vecnorm.Normalize();
            supVertex += margin * vecnorm;
        }
        return supVertex;
    }

    /// <summary>btConvexShape::localGetSupportingVertexWithoutMargin</summary>
    public abstract Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec);

    /// <summary>btConvexShape::project</summary>
    internal void Project(in Transform trans, Vector3 dir, out btScalar minProj, out btScalar maxProj,
        out Vector3 witnesPtMin, out Vector3 witnesPtMax)
    {
        Vector3 localAxis = dir * trans.Basis;
        Vector3 vtx1 = trans * LocalGetSupportingVertex(localAxis);
        Vector3 vtx2 = trans * LocalGetSupportingVertex(-localAxis);

        minProj = vtx1.Dot(dir);
        maxProj = vtx2.Dot(dir);
        witnesPtMax = vtx2;
        witnesPtMin = vtx1;

        if (minProj > maxProj)
        {
            (minProj, maxProj) = (maxProj, minProj);
            (witnesPtMin, witnesPtMax) = (witnesPtMax, witnesPtMin);
        }
    }

    /// <summary>btConvexShape::getNumPreferredPenetrationDirections (default 0)</summary>
    public virtual int NumPreferredPenetrationDirections => 0;

    public virtual void GetPreferredPenetrationDirection(int index, out Vector3 penetrationVector)
    {
        penetrationVector = Vector3.Zero;
    }
}

/// <summary>
/// Ported from btConvexInternalShape: base class of convex shapes with implicit dimensions and a margin.
/// </summary>
public abstract class ConvexInternalShape : ConvexShape
{
    internal Vector3 implicitShapeDimensions;
    internal btScalar collisionMargin = BtMath.ConvexDistanceMargin;

    protected ConvexInternalShape()
    {
        implicitShapeDimensions = Vector3.Zero;
    }

    /// <summary>btConvexInternalShape::getImplicitShapeDimensions</summary>
    public Vector3 ImplicitShapeDimensions
    {
        get => implicitShapeDimensions;
        set => implicitShapeDimensions = value;
    }

    /// <summary>btConvexInternalShape::setSafeMargin(halfExtents, defaultMarginMultiplier)</summary>
    public void SetSafeMargin(Vector3 halfExtents, btScalar defaultMarginMultiplier = (btScalar)0.1) =>
        SetSafeMargin(halfExtents[halfExtents.MinAxis()], defaultMarginMultiplier);

    public void SetSafeMargin(btScalar minDimension, btScalar defaultMarginMultiplier = (btScalar)0.1)
    {
        btScalar safeMargin = defaultMarginMultiplier * minDimension;
        if (safeMargin < Margin)
        {
            // C++ calls the virtual setMargin; btBoxShape/btCylinderShape override it to
            // compensate implicitShapeDimensions so the overall shape size is unchanged.
            Margin = safeMargin;
        }
    }

    public override btScalar Margin
    {
        get => collisionMargin;
        set => collisionMargin = value;
    }

    public override Vector3 LocalScaling
    {
        get => localScaling;
        set => SetLocalScaling(value);
    }

    /// <summary>btConvexInternalShape::setLocalScaling (keeps the implicit dimensions including margin unchanged)</summary>
    public override void SetLocalScaling(Vector3 scaling)
    {
        Vector3 oldMargin = new(Margin);
        Vector3 implicitShapeDimensionsWithMargin = implicitShapeDimensions + oldMargin;
        Vector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / localScaling;
        localScaling = scaling;
        implicitShapeDimensions = unScaledImplicitShapeDimensionsWithMargin * localScaling - oldMargin;
    }
}

/// <summary>
/// Ported from btConcaveShape: base class of concave shapes (triangle traversal interface).
/// </summary>
public abstract class ConcaveShape : CollisionShape
{
    internal btScalar collisionMargin;

    protected ConcaveShape()
    {
        collisionMargin = 0;
    }

    public override btScalar Margin
    {
        get => collisionMargin;
        set => collisionMargin = value;
    }

    /// <summary>btConcaveShape::processAllTriangles</summary>
    public abstract void ProcessAllTriangles(TriangleCallback callback, Vector3 aabbMin, Vector3 aabbMax);
}

/// <summary>
/// Ported from btPolyhedralConvexShape (base class of polyhedral shapes such as convex hulls).
/// </summary>
public abstract class PolyhedralConvexShape : ConvexInternalShape
{
    protected PolyhedralConvexShape()
    {
    }

    /// <summary>btPolyhedralConvexShape::initialize, called by subclasses</summary>
    protected abstract void InitializePolyhedralFeatures();
}

/// <summary>btTriangleCallback (ProcessTriangle receives world-space triangles)</summary>
public abstract class TriangleCallback
{
    public abstract void ProcessTriangle(Vector3[] triangle, int partId, int triangleIndex);

    public virtual void Dispose()
    {
    }
}

/// <summary>btInternalTriangleIndexCallback</summary>
public abstract class InternalTriangleIndexCallback
{
    public abstract void InternalProcessTriangleIndex(Vector3[] triangle, int partId, int triangleIndex);

    public virtual void Dispose()
    {
    }
}
