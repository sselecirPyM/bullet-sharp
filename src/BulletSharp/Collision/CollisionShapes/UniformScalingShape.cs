using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btUniformScalingShape: reuses an instance of an existing convex shape with a
/// uniform scaling factor (saves memory).
/// </summary>
public class UniformScalingShape : ConvexShape
{
    internal ConvexShape childConvexShape;
    internal btScalar uniformScalingFactor;

    /// <summary>btUniformScalingShape::btUniformScalingShape(convexChildShape, uniformScalingFactor)</summary>
    public UniformScalingShape(ConvexShape convexChildShape, btScalar uniformScalingFactor)
    {
        shapeType = BroadphaseNativeType.UniformScalingShape;
        childConvexShape = convexChildShape;
        this.uniformScalingFactor = uniformScalingFactor;
    }

    /// <summary>btUniformScalingShape::getUniformScalingFactor</summary>
    public btScalar UniformScalingFactor => uniformScalingFactor;

    /// <summary>btUniformScalingShape::getChildShape</summary>
    public ConvexShape ChildShape => childConvexShape;

    /// <summary>btUniformScalingShape::localGetSupportingVertexWithoutMargin</summary>
    public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vec)
    {
        Vector3 tmpVertex = childConvexShape.LocalGetSupportingVertexWithoutMargin(vec);
        return tmpVertex * uniformScalingFactor;
    }

    /// <summary>btUniformScalingShape::localGetSupportingVertex</summary>
    public override Vector3 LocalGetSupportingVertex(Vector3 vec)
    {
        Vector3 tmpVertex = childConvexShape.LocalGetSupportingVertex(vec);
        return tmpVertex * uniformScalingFactor;
    }

    /// <summary>btUniformScalingShape::calculateLocalInertia (linear scaling; not strictly physically correct)</summary>
    public override void CalculateLocalInertia(btScalar mass, out Vector3 inertia)
    {
        // This linear scaling is not realistic, but we don't deal with large mass ratios...
        childConvexShape.CalculateLocalInertia(mass, out Vector3 tmpInertia);
        inertia = tmpInertia * uniformScalingFactor;
    }

    /// <summary>btUniformScalingShape::getAabb → getAabbSlow</summary>
    internal override void GetAabb(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax) =>
        GetAabbSlow(in t, out aabbMin, out aabbMax);

    /// <summary>btUniformScalingShape::getAabbSlow (brute-force 6-direction support points)</summary>
    internal override void GetAabbSlow(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax)
    {
        Vector3[] directions =
        {
            new(1, 0, 0), new(0, 1, 0), new(0, 0, 1),
            new(-1, 0, 0), new(0, -1, 0), new(0, 0, -1),
        };
        Vector3[] supporting = new Vector3[6];

        for (int i = 0; i < 6; i++)
        {
            directions[i] = directions[i] * t.Basis;
            // batchedUnitVectorGetSupportingVertexWithoutMargin (equivalent per-element implementation)
            supporting[i] = childConvexShape.LocalGetSupportingVertexWithoutMargin(directions[i]) * uniformScalingFactor;
        }

        Vector3 aabbMin1 = Vector3.Zero, aabbMax1 = Vector3.Zero;
        for (int i = 0; i < 3; ++i)
        {
            aabbMax1[i] = (t * supporting[i])[i];
            aabbMin1[i] = (t * supporting[i + 3])[i];
        }
        Vector3 marginVec = new(Margin);
        aabbMin = aabbMin1 - marginVec;
        aabbMax = aabbMax1 + marginVec;
    }

    /// <summary>btUniformScalingShape::setLocalScaling (forwarded to the child shape)</summary>
    public override void SetLocalScaling(Vector3 scaling) => childConvexShape.SetLocalScaling(scaling);

    public override Vector3 LocalScaling
    {
        get => childConvexShape.LocalScaling;
        set => SetLocalScaling(value);
    }

    /// <summary>btUniformScalingShape::getMargin / setMargin (margin is also scaled by the scaling factor)</summary>
    public override btScalar Margin
    {
        get => childConvexShape.Margin * uniformScalingFactor;
        set => childConvexShape.Margin = value;
    }

    /// <summary>btUniformScalingShape::getNumPreferredPenetrationDirections</summary>
    public override int NumPreferredPenetrationDirections => childConvexShape.NumPreferredPenetrationDirections;

    /// <summary>btUniformScalingShape::getPreferredPenetrationDirection</summary>
    public override void GetPreferredPenetrationDirection(int index, out Vector3 penetrationVector) =>
        childConvexShape.GetPreferredPenetrationDirection(index, out penetrationVector);
}
