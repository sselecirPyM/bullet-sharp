using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btCapsuleShape: a capsule aligned with the Y axis by default
/// (the two sphere centers are `height` apart; total height is height + 2*radius).
/// </summary>
public class CapsuleShape : ConvexInternalShape
{
    internal int upAxis;

    /// <summary>For use by the CapsuleShapeX / CapsuleShapeZ subclasses only.</summary>
    protected CapsuleShape()
    {
        shapeType = BroadphaseNativeType.CapsuleShape;
    }

    /// <summary>btCapsuleShape::btCapsuleShape(radius, height) (Y axis)</summary>
    public CapsuleShape(btScalar radius, btScalar height)
    {
        collisionMargin = radius;
        shapeType = BroadphaseNativeType.CapsuleShape;
        upAxis = 1;
        implicitShapeDimensions = new Vector3(radius, (btScalar)0.5 * height, radius);
    }

    /// <summary>btCapsuleShape::getUpAxis</summary>
    public int UpAxis => upAxis;

    /// <summary>btCapsuleShape::getRadius (implicit dims already include scaling)</summary>
    public btScalar Radius => implicitShapeDimensions[(upAxis + 2) % 3];

    /// <summary>btCapsuleShape::getHalfHeight</summary>
    public btScalar HalfHeight => implicitShapeDimensions[upAxis];

    /// <summary>btCapsuleShape::setMargin (the capsule margin is the entire radius; overriding is not allowed)</summary>
    public override btScalar Margin
    {
        get => collisionMargin;
        set { /* the capsule's entire radius == margin, so the set is ignored */ }
    }

    /// <summary>btCapsuleShape::localGetSupportingVertexWithoutMargin (the two sphere center points)</summary>
    public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec0)
    {
        Vector3 supVec = Vector3.Zero;

        btScalar maxDot = -BtMath.BtLargeFloat;

        Vector3 vec = vec0;
        btScalar lenSqr = vec.LengthSquared();
        if (lenSqr < (btScalar)0.0001)
        {
            vec = Vector3.UnitX;
        }
        else
        {
            btScalar rlen = 1 / BtMath.Sqrt(lenSqr);
            vec *= rlen;
        }

        {
            Vector3 pos = Vector3.Zero;
            pos[UpAxis] = HalfHeight;

            btScalar newDot = vec.Dot(pos);
            if (newDot > maxDot)
            {
                maxDot = newDot;
                supVec = pos;
            }
        }
        {
            Vector3 pos = Vector3.Zero;
            pos[UpAxis] = -HalfHeight;

            btScalar newDot = vec.Dot(pos);
            if (newDot > maxDot)
            {
                maxDot = newDot;
                supVec = pos;
            }
        }

        return supVec;
    }

    /// <summary>btCapsuleShape::getAabb</summary>
    internal override void GetAabb(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax)
    {
        Vector3 halfExtents = new(Radius);
        halfExtents[upAxis] = Radius + HalfHeight;
        Matrix3x3 absB = t.Basis.Absolute();
        Vector3 center = t.Origin;
        Vector3 extent = AabbUtil.Dot3(absB.Row0, absB.Row1, absB.Row2, halfExtents);

        aabbMin = center - extent;
        aabbMax = center + extent;
    }

    /// <summary>btCapsuleShape::calculateLocalInertia (approximated by the bounding box)</summary>
    public override void CalculateLocalInertia(btScalar mass, out Vector3 inertia)
    {
        // approximation: use the inertia of the box bounding the two spheres
        btScalar radius = Radius;

        Vector3 halfExtents = new(radius);
        halfExtents[UpAxis] += HalfHeight;

        btScalar lx = 2 * halfExtents[0];
        btScalar ly = 2 * halfExtents[1];
        btScalar lz = 2 * halfExtents[2];
        btScalar x2 = lx * lx;
        btScalar y2 = ly * ly;
        btScalar z2 = lz * lz;
        btScalar scaledmass = mass * (btScalar)0.08333333;

        inertia = new Vector3(
            scaledmass * (y2 + z2),
            scaledmass * (x2 + z2),
            scaledmass * (x2 + y2));
    }

    /// <summary>btCapsuleShape::setLocalScaling (scales the implicit dims directly and keeps collisionMargin == radius)</summary>
    public override void SetLocalScaling(Vector3 scaling)
    {
        Vector3 unScaledImplicitShapeDimensions = implicitShapeDimensions / localScaling;
        localScaling = scaling;
        implicitShapeDimensions = unScaledImplicitShapeDimensions * scaling;
        // update m_collisionMargin, because the entire radius == margin
        int radiusAxis = (upAxis + 2) % 3;
        collisionMargin = implicitShapeDimensions[radiusAxis];
    }

    /// <summary>btCapsuleShape::getAnisotropicRollingFrictionDirection</summary>
    public override Vector3 AnisotropicRollingFrictionDirection
    {
        get
        {
            Vector3 aniDir = Vector3.Zero;
            aniDir[UpAxis] = 1;
            return aniDir;
        }
    }
}

/// <summary>btCapsuleShapeX: a capsule aligned with the X axis.</summary>
public class CapsuleShapeX : CapsuleShape
{
    public CapsuleShapeX(btScalar radius, btScalar height)
    {
        collisionMargin = radius;
        upAxis = 0;
        implicitShapeDimensions = new Vector3((btScalar)0.5 * height, radius, radius);
    }
}

/// <summary>btCapsuleShapeZ: a capsule aligned with the Z axis.</summary>
public class CapsuleShapeZ : CapsuleShape
{
    public CapsuleShapeZ(btScalar radius, btScalar height)
    {
        collisionMargin = radius;
        upAxis = 2;
        implicitShapeDimensions = new Vector3(radius, radius, (btScalar)0.5 * height);
    }
}
