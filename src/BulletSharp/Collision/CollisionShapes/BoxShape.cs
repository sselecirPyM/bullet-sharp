using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btBoxShape: an axis-aligned box centered at the origin
/// (half extents are given by the constructor).
/// </summary>
public class BoxShape : PolyhedralConvexShape
{
    /// <summary>btBoxShape::btBoxShape(boxHalfExtents)</summary>
    public BoxShape(Vector3 boxHalfExtents)
    {
        shapeType = BroadphaseNativeType.BoxShape;

        Vector3 margin = new(Margin);
        implicitShapeDimensions = boxHalfExtents * localScaling - margin;

        SetSafeMargin(boxHalfExtents);
    }

    /// <summary>Same half extent on all three axes.</summary>
    public BoxShape(btScalar boxHalfExtent)
        : this(new Vector3(boxHalfExtent))
    {
    }

    /// <summary>Half extents for the three axes.</summary>
    public BoxShape(btScalar boxHalfExtentX, btScalar boxHalfExtentY, btScalar boxHalfExtentZ)
        : this(new Vector3(boxHalfExtentX, boxHalfExtentY, boxHalfExtentZ))
    {
    }

    /// <summary>btBoxShape::getHalfExtentsWithMargin</summary>
    public Vector3 HalfExtentsWithMargin => implicitShapeDimensions + new Vector3(Margin);

    /// <summary>btBoxShape::getHalfExtentsWithoutMargin (includes scaling, excludes margin)</summary>
    public Vector3 HalfExtentsWithoutMargin => implicitShapeDimensions;

    /// <summary>btBoxShape::setMargin (adjusts implicitShapeDimensions to keep the overall size unchanged)</summary>
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

    /// <summary>btBoxShape::localGetSupportingVertex (with margin; picks the vertex by sign)</summary>
    public override Vector3 LocalGetSupportingVertex(Vector3 vec)
    {
        Vector3 halfExtents = HalfExtentsWithMargin;
        return new Vector3(
            BtMath.Fsel(vec.X, halfExtents.X, -halfExtents.X),
            BtMath.Fsel(vec.Y, halfExtents.Y, -halfExtents.Y),
            BtMath.Fsel(vec.Z, halfExtents.Z, -halfExtents.Z));
    }

    /// <summary>btBoxShape::localGetSupportingVertexWithoutMargin</summary>
    public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec)
    {
        Vector3 halfExtents = implicitShapeDimensions;
        return new Vector3(
            BtMath.Fsel(vec.X, halfExtents.X, -halfExtents.X),
            BtMath.Fsel(vec.Y, halfExtents.Y, -halfExtents.Y),
            BtMath.Fsel(vec.Z, halfExtents.Z, -halfExtents.Z));
    }

    /// <summary>btBoxShape::getAabb → btTransformAabb(halfExtents, margin, t)</summary>
    internal override void GetAabb(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax) =>
        AabbUtil.TransformAabb(in implicitShapeDimensions, Margin, in t, out aabbMin, out aabbMax);

    /// <summary>btBoxShape::calculateLocalInertia (box formula with lx = 2*(hx+margin))</summary>
    public override void CalculateLocalInertia(btScalar mass, out Vector3 inertia)
    {
        Vector3 halfExtents = HalfExtentsWithMargin;

        btScalar lx = 2 * halfExtents.X;
        btScalar ly = 2 * halfExtents.Y;
        btScalar lz = 2 * halfExtents.Z;

        inertia = new Vector3(
            mass / 12 * (ly * ly + lz * lz),
            mass / 12 * (lx * lx + lz * lz),
            mass / 12 * (lx * lx + ly * ly));
    }

    /// <summary>btBoxShape::getNumPreferredPenetrationDirections</summary>
    public override int NumPreferredPenetrationDirections => 6;

    /// <summary>btBoxShape::getPreferredPenetrationDirection</summary>
    public override void GetPreferredPenetrationDirection(int index, out Vector3 penetrationVector)
    {
        penetrationVector = index switch
        {
            0 => Vector3.UnitX,
            1 => -Vector3.UnitX,
            2 => Vector3.UnitY,
            3 => -Vector3.UnitY,
            4 => Vector3.UnitZ,
            5 => -Vector3.UnitZ,
            _ => Vector3.Zero,
        };
    }

    protected override void InitializePolyhedralFeatures() =>
        throw new NotSupportedException("InitializePolyhedralFeatures not ported (requires btConvexHullComputer)");
}
