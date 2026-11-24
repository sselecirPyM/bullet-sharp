using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btGearConstraint: a gear constraint that couples the angular velocities
/// of two rigid bodies according to the given local axes and ratio
/// (see Bullet/Demos/ConstraintDemo for a usage example).
/// </summary>
public class GearConstraint : TypedConstraint
{
    internal Vector3 axisInA;
    internal Vector3 axisInB;
    internal btScalar ratio;

    /// <summary>btGearConstraint::btGearConstraint(rbA, rbB, axisInA, axisInB, ratio)</summary>
    public GearConstraint(RigidBody rbA, RigidBody rbB, Vector3 axisInA, Vector3 axisInB, btScalar ratio = 1)
        : base(TypedConstraintType.Gear, rbA, rbB)
    {
        this.axisInA = axisInA;
        this.axisInB = axisInB;
        this.ratio = ratio;
    }

    internal override void GetInfo1(ConstraintInfo1 info, ContactSolverInfo infoGlobal)
    {
        // btGearConstraint::getInfo1
        info.numConstraintRows = 1;
        info.nub = 1;
    }

    internal override void GetInfo2(ConstraintInfo2 info, ContactSolverInfo infoGlobal,
        Transform[] body0Trans, Transform[] body1Trans, int body0Index, int body1Index)
    {
        // btGearConstraint::getInfo2
        Vector3 globalAxisA = rigidBodyA.worldTransform.Basis * axisInA;
        Vector3 globalAxisB = rigidBodyB.worldTransform.Basis * axisInB;

        info.j1AngularAxis[0] = globalAxisA[0];
        info.j1AngularAxis[1] = globalAxisA[1];
        info.j1AngularAxis[2] = globalAxisA[2];

        info.j2AngularAxis[0] = ratio * globalAxisB[0];
        info.j2AngularAxis[1] = ratio * globalAxisB[1];
        info.j2AngularAxis[2] = ratio * globalAxisB[2];

        // C++ relies on the solver to preset lowerLimit/upperLimit to ±SIMD_INFINITY; written explicitly here
        info.lowerLimit[0] = -BtMath.SimdInfinity;
        info.upperLimit[0] = BtMath.SimdInfinity;
    }

    /// <summary>btGearConstraint::getAxisA/setAxisA</summary>
    public Vector3 AxisA
    {
        get => axisInA;
        set => axisInA = value;
    }

    /// <summary>btGearConstraint::getAxisB/setAxisB</summary>
    public Vector3 AxisB
    {
        get => axisInB;
        set => axisInB = value;
    }

    /// <summary>btGearConstraint::getRatio/setRatio</summary>
    public btScalar Ratio
    {
        get => ratio;
        set => ratio = value;
    }
}
