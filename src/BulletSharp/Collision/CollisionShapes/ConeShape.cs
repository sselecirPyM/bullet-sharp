using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btConeShape: a cone centered at the origin, aligned with the Y axis by default.
/// </summary>
public class ConeShape : ConvexInternalShape
{
    internal btScalar sinAngle;   // m_sinAngle
    internal btScalar radius;     // m_radius
    internal btScalar height;     // m_height
    internal readonly int[] coneIndices = new int[3]; // m_coneIndices

    /// <summary>btConeShape::btConeShape(radius, height) (Y axis)</summary>
    public ConeShape(btScalar radius, btScalar height)
    {
        shapeType = BroadphaseNativeType.ConeShape;
        this.radius = radius;
        this.height = height;
        SetConeUpIndex(1);
        sinAngle = this.radius / BtMath.Sqrt(this.radius * this.radius + this.height * this.height);
    }

    /// <summary>btConeShape::getRadius / setRadius</summary>
    public btScalar Radius
    {
        get => radius;
        set => radius = value;
    }

    /// <summary>btConeShape::getHeight / setHeight</summary>
    public btScalar Height
    {
        get => height;
        set => height = value;
    }

    /// <summary>btConeShape::setConeUpIndex</summary>
    public void SetConeUpIndex(int upIndex)
    {
        switch (upIndex)
        {
            case 0:
                coneIndices[0] = 1;
                coneIndices[1] = 0;
                coneIndices[2] = 2;
                break;
            case 1:
                coneIndices[0] = 0;
                coneIndices[1] = 1;
                coneIndices[2] = 2;
                break;
            case 2:
                coneIndices[0] = 0;
                coneIndices[1] = 2;
                coneIndices[2] = 1;
                break;
            default:
                System.Diagnostics.Debug.Assert(false);
                break;
        }

        implicitShapeDimensions[coneIndices[0]] = radius;
        implicitShapeDimensions[coneIndices[1]] = height;
        implicitShapeDimensions[coneIndices[2]] = radius;
    }

    /// <summary>btConeShape::getConeUpIndex</summary>
    public int ConeUpIndex => coneIndices[1];

    /// <summary>btConeShape::coneLocalSupport</summary>
    private Vector3 ConeLocalSupport(Vector3 v)
    {
        btScalar halfHeight = height * (btScalar)0.5;

        if (v[coneIndices[1]] > v.Length() * sinAngle)
        {
            Vector3 tmp = Vector3.Zero;
            tmp[coneIndices[0]] = 0;
            tmp[coneIndices[1]] = halfHeight;
            tmp[coneIndices[2]] = 0;
            return tmp;
        }
        else
        {
            btScalar s = BtMath.Sqrt(v[coneIndices[0]] * v[coneIndices[0]] + v[coneIndices[2]] * v[coneIndices[2]]);
            if (s > BtMath.SimdEpsilon)
            {
                btScalar d = radius / s;
                Vector3 tmp = Vector3.Zero;
                tmp[coneIndices[0]] = v[coneIndices[0]] * d;
                tmp[coneIndices[1]] = -halfHeight;
                tmp[coneIndices[2]] = v[coneIndices[2]] * d;
                return tmp;
            }
            else
            {
                Vector3 tmp = Vector3.Zero;
                tmp[coneIndices[0]] = 0;
                tmp[coneIndices[1]] = -halfHeight;
                tmp[coneIndices[2]] = 0;
                return tmp;
            }
        }
    }

    /// <summary>btConeShape::localGetSupportingVertexWithoutMargin</summary>
    public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec) => ConeLocalSupport(vec);

    /// <summary>btConeShape::localGetSupportingVertex</summary>
    public override Vector3 LocalGetSupportingVertex(Vector3 vec)
    {
        Vector3 supVertex = ConeLocalSupport(vec);
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

    /// <summary>btConeShape does not override getAabb, so btConvexInternalShape::getAabbSlow (brute force via support function) is used</summary>
    internal override void GetAabb(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax)
    {
        // btConvexInternalShape::getAabbSlow
        btScalar margin = Margin;
        aabbMin = Vector3.Zero;
        aabbMax = Vector3.Zero;
        for (int i = 0; i < 3; i++)
        {
            Vector3 vec = Vector3.Zero;
            vec[i] = 1;

            Vector3 sv = LocalGetSupportingVertex(vec * t.Basis);
            Vector3 tmp = t * sv;
            aabbMax[i] = tmp[i] + margin;

            vec[i] = -1;
            tmp = t * LocalGetSupportingVertex(vec * t.Basis);
            aabbMin[i] = tmp[i] - margin;
        }
    }

    /// <summary>btConeShape::calculateLocalInertia (approximated by the AABB box)</summary>
    public override void CalculateLocalInertia(btScalar mass, out Vector3 inertia)
    {
        GetAabb(Transform.Identity, out Vector3 aabbMin, out Vector3 aabbMax);

        Vector3 halfExtents = (aabbMax - aabbMin) * (btScalar)0.5;

        btScalar margin = Margin;

        btScalar lx = 2 * (halfExtents.X + margin);
        btScalar ly = 2 * (halfExtents.Y + margin);
        btScalar lz = 2 * (halfExtents.Z + margin);
        btScalar x2 = lx * lx;
        btScalar y2 = ly * ly;
        btScalar z2 = lz * lz;
        btScalar scaledmass = mass * (btScalar)0.08333333;

        inertia = scaledmass * new Vector3(y2 + z2, x2 + z2, x2 + y2);
    }

    /// <summary>btConeShape::setLocalScaling (scales radius/height and recomputes sinAngle)</summary>
    public override void SetLocalScaling(Vector3 scaling)
    {
        int axis = coneIndices[1];
        int r1 = coneIndices[0];
        int r2 = coneIndices[2];
        height *= scaling[axis] / localScaling[axis];
        radius *= (scaling[r1] / localScaling[r1] + scaling[r2] / localScaling[r2]) / 2;
        sinAngle = radius / BtMath.Sqrt(radius * radius + height * height);
        // btConvexInternalShape::setLocalScaling
        localScaling = scaling.Absolute();
    }

    /// <summary>btConeShape::getAnisotropicRollingFrictionDirection</summary>
    public override Vector3 AnisotropicRollingFrictionDirection => Vector3.UnitY;
}

/// <summary>btConeShapeX: a cone aligned with the X axis.</summary>
public class ConeShapeX : ConeShape
{
    public ConeShapeX(btScalar radius, btScalar height)
        : base(radius, height)
    {
        SetConeUpIndex(0);
    }

    public override Vector3 AnisotropicRollingFrictionDirection => Vector3.UnitX;
}

/// <summary>btConeShapeZ: a cone aligned with the Z axis.</summary>
public class ConeShapeZ : ConeShape
{
    public ConeShapeZ(btScalar radius, btScalar height)
        : base(radius, height)
    {
        SetConeUpIndex(2);
    }

    public override Vector3 AnisotropicRollingFrictionDirection => Vector3.UnitZ;
}
