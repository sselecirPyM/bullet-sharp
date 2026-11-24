using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btJacobianEntry (a constraint Jacobian row).
/// </summary>
public struct JacobianEntry
{
    public Vector3 LinearJointAxis;
    public Vector3 AJ;
    public Vector3 BJ;
    public btScalar MinImpulse;
    public btScalar MaxImpulse;
    public btScalar ADiag;

    /// <summary>World-space point-to-point constraint</summary>
    public static JacobianEntry PointConstraint(in Matrix3x3 world2A, in Matrix3x3 world2B,
        in Vector3 relPos1, in Vector3 relPos2,
        in Vector3 jointAxis, in Vector3 inertiaInvA, btScalar massInvA,
        in Vector3 inertiaInvB, btScalar massInvB)
    {
        var e = new JacobianEntry
        {
            LinearJointAxis = jointAxis,
        };
        e.AJ = world2A * relPos1.Cross(jointAxis);
        e.BJ = world2B * (-relPos2).Cross(jointAxis);
        e.MinImpulse = 0;
        e.MaxImpulse = BtMath.BtLargeFloat;
        e.ADiag = massInvA + e.AJ.Dot(inertiaInvA * e.AJ) +
                  massInvB + e.BJ.Dot(inertiaInvB * e.BJ);
        return e;
    }

    /// <summary>btJacobianEntry::getRelativeVelocity</summary>
    public readonly btScalar GetRelativeVelocity(in Vector3 linvelA, in Vector3 angvelA, in Vector3 linvelB, in Vector3 angvelB)
    {
        Vector3 linrel = linvelA - linvelB;
        Vector3 angvela = angvelA * AJ;
        Vector3 angvelb = angvelB * BJ;
        linrel += angvela + angvelb;
        btScalar relVel2 = linrel.Dot(LinearJointAxis);
        return relVel2 + BtMath.SimdEpsilon;
    }

    public readonly btScalar Diagonal => ADiag;

    /// <summary>btJacobianEntry::computeImpulseDenominator</summary>
    public static btScalar ComputeImpulseDenominator(in Vector3 pos, in Vector3 inertiaInv, in Vector3 normal)
    {
        Vector3 c0 = pos.Cross(normal);
        Vector3 vec = (inertiaInv * c0).Cross(pos);
        return normal.Dot(vec);
    }
}
