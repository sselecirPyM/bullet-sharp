using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btMinkowskiSum (NarrowPhaseCollision): implicit Minkowski sum of two convex shapes.
/// For advanced users only.
/// </summary>
public class MinkowskiSum : ConvexShape
{
    internal Transform transA = Transform.Identity;
    internal Transform transB = Transform.Identity;
    internal ConvexShape shapeA;
    internal ConvexShape shapeB;

    public MinkowskiSum(ConvexShape shapeA, ConvexShape shapeB)
    {
        shapeType = BroadphaseNativeType.MinkowskiSumShape;
        this.shapeA = shapeA;
        this.shapeB = shapeB;
    }

    /// <summary>btMinkowskiSum::localGetSupportingVertexWithoutMargin</summary>
    public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec)
    {
        Vector3 supVertexA = transA * shapeA.LocalGetSupportingVertexWithoutMargin(vec * transA.Basis);
        Vector3 supVertexB = transB * shapeB.LocalGetSupportingVertexWithoutMargin(-vec * transB.Basis);
        return supVertexA - supVertexB;
    }

    /// <summary>btMinkowskiSumShape::calculateLocalInertia (inertia of the Minkowski sum AABB)</summary>
    public override void CalculateLocalInertia(btScalar mass, out Vector3 inertia)
    {
        Transform identity = Transform.Identity;
        GetAabb(in identity, out Vector3 aabbMin, out Vector3 aabbMax);

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

    public Matrix TransformA
    {
        get => transA.ToMatrix();
        set => transA = Transform.FromMatrix(in value);
    }

    public Matrix TransformB
    {
        get => transB.ToMatrix();
        set => transB = Transform.FromMatrix(in value);
    }

    public override btScalar Margin
    {
        get => shapeA.Margin + shapeB.Margin;
        set { }
    }

    public ConvexShape ShapeA => shapeA;
    public ConvexShape ShapeB => shapeB;

    /// <summary>btCollisionShape::getAabbSlow (generic but slow AABB)</summary>
    internal override void GetAabb(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax) =>
        GetAabbSlow(in t, out aabbMin, out aabbMax);
}
