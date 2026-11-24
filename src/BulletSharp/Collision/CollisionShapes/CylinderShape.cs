using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btCylinderShape: a cylinder centered at the origin with its main axis
/// aligned with the Y axis by default.
/// </summary>
public class CylinderShape : ConvexInternalShape
{
    internal int upAxis;

    /// <summary>btCylinderShape::btCylinderShape(halfExtents) (Y axis)</summary>
    public CylinderShape(Vector3 halfExtents)
    {
        upAxis = 1;
        Vector3 margin = new(Margin);
        implicitShapeDimensions = halfExtents * localScaling - margin;

        SetSafeMargin(halfExtents);

        shapeType = BroadphaseNativeType.CylinderShape;
    }

    /// <summary>btCylinderShape::getHalfExtentsWithMargin</summary>
    public Vector3 HalfExtentsWithMargin => implicitShapeDimensions + new Vector3(Margin);

    /// <summary>btCylinderShape::getHalfExtentsWithoutMargin (includes scaling and margin adjustment)</summary>
    public Vector3 HalfExtentsWithoutMargin => implicitShapeDimensions;

    /// <summary>btCylinderShape::getUpAxis</summary>
    public int UpAxis => upAxis;

    /// <summary>btCylinderShape::getRadius</summary>
    public virtual btScalar Radius => HalfExtentsWithMargin.X;

    /// <summary>btCylinderShape::setMargin (adjusts implicitShapeDimensions to keep the overall size unchanged)</summary>
    public override btScalar Margin
    {
        get => collisionMargin;
        set
        {
            Vector3 oldMargin = new(Margin);
            Vector3 implicitShapeDimensionsWithMargin = implicitShapeDimensions + oldMargin;

            collisionMargin = value;
            Vector3 newMargin = new(Margin);
            implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;
        }
    }

    /// <summary>btCylinderShape::getAabb → btTransformAabb(halfExtents, margin)</summary>
    internal override void GetAabb(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax) =>
        AabbUtil.TransformAabb(in implicitShapeDimensions, Margin, in t, out aabbMin, out aabbMax);

    /// <summary>btCylinderShape::calculateLocalInertia (exact cylinder formula, since Bullet 2.77)</summary>
    public override void CalculateLocalInertia(btScalar mass, out Vector3 inertia)
    {
        // Main axis is Y by default; CylinderShapeX uses X, CylinderShapeZ uses Z
        // (radius is taken from another axis, the third component is ignored)
        btScalar radius2;   // square of the radius
        btScalar height2;   // square of the height
        Vector3 halfExtents = HalfExtentsWithMargin; // cylinder dimensions
        btScalar div12 = mass / 12;
        btScalar div4 = mass / 4;
        btScalar div2 = mass / 2;
        int idxRadius, idxHeight;

        switch (upAxis) // indices of the cylinder radius axis and height axis
        {
            case 0: // along X
                idxRadius = 1;
                idxHeight = 0;
                break;
            case 2: // along Z
                idxRadius = 0;
                idxHeight = 2;
                break;
            default: // along Y
                idxRadius = 0;
                idxHeight = 1;
                break;
        }

        radius2 = halfExtents[idxRadius] * halfExtents[idxRadius];
        height2 = 4 * halfExtents[idxHeight] * halfExtents[idxHeight];

        // inertia tensor diagonal terms
        btScalar t1 = div12 * height2 + div4 * radius2;
        btScalar t2 = div2 * radius2;

        switch (upAxis)
        {
            case 0:
                inertia = new Vector3(t2, t1, t1);
                break;
            case 2:
                inertia = new Vector3(t1, t1, t2);
                break;
            default:
                inertia = new Vector3(t1, t2, t1);
                break;
        }
    }

    // CylinderLocalSupportX/Y/Z: generic implementation with the main axis mapped to (XX, YY, ZZ)
    private static Vector3 CylinderLocalSupport(Vector3 halfExtents, Vector3 v, int xx, int yy, int zz, int cylinderUpAxis)
    {
        btScalar radius = halfExtents[xx];
        btScalar halfHeight = halfExtents[cylinderUpAxis];

        Vector3 tmp = Vector3.Zero;

        btScalar s = BtMath.Sqrt(v[xx] * v[xx] + v[zz] * v[zz]);
        if (s != 0)
        {
            btScalar d = radius / s;
            tmp[xx] = v[xx] * d;
            tmp[yy] = v[yy] < 0 ? -halfHeight : halfHeight;
            tmp[zz] = v[zz] * d;
            return tmp;
        }
        else
        {
            tmp[xx] = radius;
            tmp[yy] = v[yy] < 0 ? -halfHeight : halfHeight;
            tmp[zz] = 0;
            return tmp;
        }
    }

    /// <summary>CylinderLocalSupportY</summary>
    internal static Vector3 CylinderLocalSupportY(Vector3 halfExtents, Vector3 v) =>
        CylinderLocalSupport(halfExtents, v, 0, 1, 2, 1);

    /// <summary>CylinderLocalSupportX</summary>
    internal static Vector3 CylinderLocalSupportX(Vector3 halfExtents, Vector3 v) =>
        CylinderLocalSupport(halfExtents, v, 1, 0, 2, 0);

    /// <summary>CylinderLocalSupportZ</summary>
    internal static Vector3 CylinderLocalSupportZ(Vector3 halfExtents, Vector3 v) =>
        CylinderLocalSupport(halfExtents, v, 0, 2, 1, 2);

    /// <summary>btCylinderShape::localGetSupportingVertexWithoutMargin (Y axis)</summary>
    public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec) =>
        CylinderLocalSupportY(implicitShapeDimensions, vec);

    /// <summary>btCylinderShape::localGetSupportingVertex (with margin)</summary>
    public override Vector3 LocalGetSupportingVertex(Vector3 vec)
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

    /// <summary>btCylinderShape::getAnisotropicRollingFrictionDirection</summary>
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

/// <summary>btCylinderShapeX: a cylinder with its main axis aligned with the X axis.</summary>
public class CylinderShapeX : CylinderShape
{
    public CylinderShapeX(Vector3 halfExtents)
        : base(halfExtents)
    {
        upAxis = 0;
    }

    public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec) =>
        CylinderLocalSupportX(implicitShapeDimensions, vec);

    public override btScalar Radius => HalfExtentsWithMargin.Y;
}

/// <summary>btCylinderShapeZ: a cylinder with its main axis aligned with the Z axis.</summary>
public class CylinderShapeZ : CylinderShape
{
    public CylinderShapeZ(Vector3 halfExtents)
        : base(halfExtents)
    {
        upAxis = 2;
    }

    public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec) =>
        CylinderLocalSupportZ(implicitShapeDimensions, vec);

    public override btScalar Radius => HalfExtentsWithMargin.X;
}
