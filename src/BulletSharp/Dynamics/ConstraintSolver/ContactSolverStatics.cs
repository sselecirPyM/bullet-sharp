using BulletSharp.Math;
using System.Runtime.CompilerServices;

namespace BulletSharp;

/// <summary>
/// Scalar reference row-solving functions from btSequentialImpulseConstraintSolver.cpp
/// (the inner loop of Projected Gauss-Seidel / Sequential Impulse).
/// The SIMD (SSE2/SSE4/FMA3) versions are not ported.
/// </summary>
internal static class ContactSolverStatics
{
    /// <summary>gNumSplitImpulseRecoveries</summary>
    public static int NumSplitImpulseRecoveries;

    /// <summary>gResolveSingleConstraintRowGeneric_scalar_reference</summary>
    public static btScalar ResolveSingleConstraintRowGenericScalarReference(SolverBody bodyA, SolverBody bodyB, SolverConstraint c)
    {
        btScalar deltaImpulse = c.rhs - c.appliedImpulse * c.cfm;
        btScalar deltaVel1Dotn = c.contactNormal1.Dot(bodyA.deltaLinearVelocity) + c.relpos1CrossNormal.Dot(bodyA.deltaAngularVelocity);
        btScalar deltaVel2Dotn = c.contactNormal2.Dot(bodyB.deltaLinearVelocity) + c.relpos2CrossNormal.Dot(bodyB.deltaAngularVelocity);

        deltaImpulse -= deltaVel1Dotn * c.jacDiagABInv;
        deltaImpulse -= deltaVel2Dotn * c.jacDiagABInv;

        btScalar sum = c.appliedImpulse + deltaImpulse;
        if (sum < c.lowerLimit)
        {
            deltaImpulse = c.lowerLimit - c.appliedImpulse;
            c.appliedImpulse = c.lowerLimit;
        }
        else if (sum > c.upperLimit)
        {
            deltaImpulse = c.upperLimit - c.appliedImpulse;
            c.appliedImpulse = c.upperLimit;
        }
        else
        {
            c.appliedImpulse = sum;
        }

        bodyA.InternalApplyImpulse(c.contactNormal1 * bodyA.invMass, c.angularComponentA, deltaImpulse);
        bodyB.InternalApplyImpulse(c.contactNormal2 * bodyB.invMass, c.angularComponentB, deltaImpulse);

        return deltaImpulse * c.jacDiagAB;
    }

    /// <summary>gResolveSingleConstraintRowLowerLimit_scalar_reference</summary>
    public static btScalar ResolveSingleConstraintRowLowerLimitScalarReference(SolverBody bodyA, SolverBody bodyB, SolverConstraint c)
    {
        btScalar deltaImpulse = c.rhs - c.appliedImpulse * c.cfm;
        btScalar deltaVel1Dotn = c.contactNormal1.Dot(bodyA.deltaLinearVelocity) + c.relpos1CrossNormal.Dot(bodyA.deltaAngularVelocity);
        btScalar deltaVel2Dotn = c.contactNormal2.Dot(bodyB.deltaLinearVelocity) + c.relpos2CrossNormal.Dot(bodyB.deltaAngularVelocity);

        deltaImpulse -= deltaVel1Dotn * c.jacDiagABInv;
        deltaImpulse -= deltaVel2Dotn * c.jacDiagABInv;
        btScalar sum = c.appliedImpulse + deltaImpulse;
        if (sum < c.lowerLimit)
        {
            deltaImpulse = c.lowerLimit - c.appliedImpulse;
            c.appliedImpulse = c.lowerLimit;
        }
        else
        {
            c.appliedImpulse = sum;
        }
        bodyA.InternalApplyImpulse(c.contactNormal1 * bodyA.invMass, c.angularComponentA, deltaImpulse);
        bodyB.InternalApplyImpulse(c.contactNormal2 * bodyB.invMass, c.angularComponentB, deltaImpulse);

        return deltaImpulse * c.jacDiagAB;
    }

    /// <summary>gResolveSplitPenetrationImpulse_scalar_reference</summary>
    public static btScalar ResolveSplitPenetrationImpulseScalarReference(SolverBody bodyA, SolverBody bodyB, SolverConstraint c)
    {
        btScalar deltaImpulse = 0;

        if (c.rhsPenetration != 0)
        {
            NumSplitImpulseRecoveries++;
            deltaImpulse = c.rhsPenetration - c.appliedPushImpulse * c.cfm;
            btScalar deltaVel1Dotn = c.contactNormal1.Dot(bodyA.pushVelocity) + c.relpos1CrossNormal.Dot(bodyA.turnVelocity);
            btScalar deltaVel2Dotn = c.contactNormal2.Dot(bodyB.pushVelocity) + c.relpos2CrossNormal.Dot(bodyB.turnVelocity);

            deltaImpulse -= deltaVel1Dotn * c.jacDiagABInv;
            deltaImpulse -= deltaVel2Dotn * c.jacDiagABInv;
            btScalar sum = c.appliedPushImpulse + deltaImpulse;
            if (sum < c.lowerLimit)
            {
                deltaImpulse = c.lowerLimit - c.appliedPushImpulse;
                c.appliedPushImpulse = c.lowerLimit;
            }
            else
            {
                c.appliedPushImpulse = sum;
            }
            bodyA.InternalApplyPushImpulse(c.contactNormal1 * bodyA.invMass, c.angularComponentA, deltaImpulse);
            bodyB.InternalApplyPushImpulse(c.contactNormal2 * bodyB.invMass, c.angularComponentB, deltaImpulse);
        }
        return deltaImpulse * c.jacDiagAB;
    }
}
