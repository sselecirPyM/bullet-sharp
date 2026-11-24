using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btGeneric6DofSpringConstraint: a generic 6-DOF constraint that allows
/// spring motors on any translational/rotational degree of freedom.
/// Degree-of-freedom indices in enableSpring()/setStiffness():
/// 0-2: translation X/Y/Z; 3-5: rotation X/Y/Z (Euler angles; see the btGeneric6DofConstraint
/// notes for the valid ranges).
/// </summary>
public class Generic6DofSpringConstraint : Generic6DofConstraint
{
    internal bool[] springEnabled = new bool[6];
    internal btScalar[] equilibriumPoint = new btScalar[6];
    internal btScalar[] springStiffness = new btScalar[6];
    internal btScalar[] springDamping = new btScalar[6]; // between 0 and 1 (1 == no damping)

    /// <summary>btGeneric6DofSpringConstraint::btGeneric6DofSpringConstraint(rbA, rbB, frameInA, frameInB, useLinearReferenceFrameA)</summary>
    public Generic6DofSpringConstraint(RigidBody rbA, RigidBody rbB, Matrix frameInA, Matrix frameInB, bool useLinearReferenceFrameA)
        : base(TypedConstraintType.D6Spring, rbA, rbB, in frameInA, in frameInB, useLinearReferenceFrameA)
    {
        Init();
    }

    /// <summary>btGeneric6DofSpringConstraint::btGeneric6DofSpringConstraint(rbB, frameInB, useLinearReferenceFrameB)</summary>
    public Generic6DofSpringConstraint(RigidBody rbB, Matrix frameInB, bool useLinearReferenceFrameB)
        : base(TypedConstraintType.D6Spring, rbB, in frameInB, useLinearReferenceFrameB)
    {
        Init();
    }

    /// <summary>btGeneric6DofSpringConstraint::init</summary>
    protected void Init()
    {
        for (int i = 0; i < 6; i++)
        {
            springEnabled[i] = false;
            equilibriumPoint[i] = 0;
            springStiffness[i] = 0;
            springDamping[i] = 1;
        }
    }

    /// <summary>btGeneric6DofSpringConstraint::enableSpring</summary>
    public void EnableSpring(int index, bool onOff)
    {
        // btAssert((index >= 0) && (index < 6))
        springEnabled[index] = onOff;
        if (index < 3)
        {
            linearLimits.enableMotor[index] = onOff;
        }
        else
        {
            angularLimits[index - 3].enableMotor = onOff;
        }
    }

    /// <summary>btGeneric6DofSpringConstraint::setStiffness</summary>
    public void SetStiffness(int index, btScalar stiffness)
    {
        springStiffness[index] = stiffness;
    }

    /// <summary>btGeneric6DofSpringConstraint::setDamping</summary>
    public void SetDamping(int index, btScalar damping)
    {
        springDamping[index] = damping;
    }

    /// <summary>btGeneric6DofSpringConstraint::setEquilibriumPoint(): uses the current constraint position/orientation as the equilibrium point for all degrees of freedom</summary>
    public void SetEquilibriumPoint()
    {
        CalculateTransforms();
        int i;

        for (i = 0; i < 3; i++)
        {
            equilibriumPoint[i] = calculatedLinearDiff[i];
        }
        for (i = 0; i < 3; i++)
        {
            equilibriumPoint[i + 3] = calculatedAxisAngleDiff[i];
        }
    }

    /// <summary>btGeneric6DofSpringConstraint::setEquilibriumPoint(index): uses the current position as the equilibrium point for the given degree of freedom</summary>
    public void SetEquilibriumPoint(int index)
    {
        CalculateTransforms();
        if (index < 3)
        {
            equilibriumPoint[index] = calculatedLinearDiff[index];
        }
        else
        {
            equilibriumPoint[index] = calculatedAxisAngleDiff[index - 3];
        }
    }

    /// <summary>btGeneric6DofSpringConstraint::setEquilibriumPoint(index, val)</summary>
    public void SetEquilibriumPoint(int index, btScalar val)
    {
        equilibriumPoint[index] = val;
    }

    /// <summary>btGeneric6DofSpringConstraint::isSpringEnabled</summary>
    public bool IsSpringEnabled(int index) => springEnabled[index];

    /// <summary>btGeneric6DofSpringConstraint::getStiffness</summary>
    public btScalar GetStiffness(int index) => springStiffness[index];

    /// <summary>btGeneric6DofSpringConstraint::getDamping</summary>
    public btScalar GetDamping(int index) => springDamping[index];

    /// <summary>btGeneric6DofSpringConstraint::getEquilibriumPoint</summary>
    public btScalar GetEquilibriumPoint(int index) => equilibriumPoint[index];

    /// <summary>btGeneric6DofSpringConstraint::internalUpdateSprings (assumes CalculateTransforms has already been called)</summary>
    protected void InternalUpdateSprings(ConstraintInfo2 info)
    {
        int i;
        for (i = 0; i < 3; i++)
        {
            if (springEnabled[i])
            {
                // Current constraint position
                btScalar currPos = calculatedLinearDiff[i];
                // Difference
                btScalar delta = currPos - equilibriumPoint[i];
                // Hooke's law: spring force = delta * stiffness
                btScalar force = delta * springStiffness[i];
                btScalar velFactor = info.fps * springDamping[i] / info.numIterations;
                linearLimits.targetVelocity[i] = velFactor * force;
                linearLimits.maxMotorForce[i] = BtMath.Abs(force);
            }
        }
        for (i = 0; i < 3; i++)
        {
            if (springEnabled[i + 3])
            {
                // Current constraint angle
                btScalar currPos = calculatedAxisAngleDiff[i];
                // Difference
                btScalar delta = currPos - equilibriumPoint[i + 3];
                // Hooke's law: spring force = -delta * stiffness
                btScalar force = -delta * springStiffness[i + 3];
                btScalar velFactor = info.fps * springDamping[i + 3] / info.numIterations;
                angularLimits[i].targetVelocity = velFactor * force;
                angularLimits[i].maxMotorForce = BtMath.Abs(force);
            }
        }
    }

    internal override void GetInfo2(ConstraintInfo2 info, ContactSolverInfo infoGlobal,
        Transform[] body0Trans, Transform[] body1Trans, int body0Index, int body1Index)
    {
        // btGeneric6DofSpringConstraint::getInfo2:
        // called by the solver during constraint setup; sets the current motor parameters
        // first, then completes the remaining constraint setup
        InternalUpdateSprings(info);
        base.GetInfo2(info, infoGlobal, body0Trans, body1Trans, body0Index, body1Index);
    }

    /// <summary>btGeneric6DofSpringConstraint::setAxis (same implementation as the base class)</summary>
    public override void SetAxis(Vector3 axis1, Vector3 axis2)
    {
        Vector3 zAxis = axis1.Normalized();
        Vector3 yAxis = axis2.Normalized();
        Vector3 xAxis = yAxis.Cross(zAxis); // requires a right-handed coordinate system

        var frameInW = Transform.Identity;
        frameInW.Basis = new Matrix3x3(
            xAxis[0], yAxis[0], zAxis[0],
            xAxis[1], yAxis[1], zAxis[1],
            xAxis[2], yAxis[2], zAxis[2]);

        // Constraint frame converted to local coordinates
        frameInA = rigidBodyA.worldTransform.Inverse() * frameInW;
        frameInB = rigidBodyB.worldTransform.Inverse() * frameInW;

        CalculateTransforms();
    }
}
