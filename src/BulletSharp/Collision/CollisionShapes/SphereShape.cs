using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btSphereShape: implicit sphere centered at the local origin with the given radius.
/// </summary>
public class SphereShape : ConvexInternalShape
{
    /// <summary>btSphereShape::btSphereShape(radius)</summary>
    public SphereShape(btScalar radius)
    {
        shapeType = BroadphaseNativeType.SphereShape;
        localScaling = Vector3.One;
        implicitShapeDimensions = Vector3.Zero;
        implicitShapeDimensions.X = radius;
        collisionMargin = radius;
    }

    /// <summary>btSphereShape::getRadius / setUnscaledRadius</summary>
    public btScalar Radius
    {
        get => implicitShapeDimensions.X * localScaling.X;
        set
        {
            // btSphereShape::setUnscaledRadius
            implicitShapeDimensions.X = value;
            collisionMargin = value;
        }
    }

    /// <summary>
    /// btSphereShape::getMargin: to improve GJK behavior, the margin is the entire radius
    /// (non-uniform scaling is no longer supported). Setting it updates both radius and implicit dims.
    /// </summary>
    public override btScalar Margin
    {
        get => Radius;
        set => Radius = value;
    }

    /// <summary>btSphereShape::localGetSupportingVertexWithoutMargin</summary>
    public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec) => Vector3.Zero;

    /// <summary>btSphereShape::localGetSupportingVertex</summary>
    public override Vector3 LocalGetSupportingVertex(Vector3 vec)
    {
        Vector3 supVertex = LocalGetSupportingVertexWithoutMargin(vec);

        Vector3 vecnorm = vec;
        if (vecnorm.LengthSquared() < BtMath.SimdEpsilon * BtMath.SimdEpsilon)
        {
            vecnorm.SetValue(-1, -1, -1);
        }
        vecnorm.Normalize();
        supVertex += Margin * vecnorm;
        return supVertex;
    }

    /// <summary>btSphereShape::getAabb</summary>
    internal override void GetAabb(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax)
    {
        Vector3 center = t.Origin;
        Vector3 extent = new(Margin);
        aabbMin = center - extent;
        aabbMax = center + extent;
    }

    /// <summary>btSphereShape::calculateLocalInertia (0.4*m*r²)</summary>
    public override void CalculateLocalInertia(btScalar mass, out Vector3 inertia)
    {
        btScalar elem = (btScalar)0.4 * mass * Margin * Margin;
        inertia = new Vector3(elem);
    }

    /// <summary>btConvexInternalShape::setLocalScaling (the sphere only records the scaling; the radius getter multiplies by scaling.X)</summary>
    public override void SetLocalScaling(Vector3 scaling) => localScaling = scaling.Absolute();
}
