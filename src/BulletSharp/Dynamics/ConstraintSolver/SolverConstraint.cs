using BulletSharp.Math;

namespace BulletSharp;

/// <summary>btSolverConstraint::btSolverConstraintType</summary>
internal enum SolverConstraintType
{
    Contact1D = 0,
    Friction1D,
}

/// <summary>
/// Ported from btSolverConstraint: a 1D constraint along a normal axis, composable to solve
/// contact and friction constraints. A reference type so solver arrays can reuse instances
/// in place (object pooling).
/// </summary>
internal class SolverConstraint
{
    public Vector3 relpos1CrossNormal;
    public Vector3 contactNormal1;

    public Vector3 relpos2CrossNormal;
    /// <summary>Usually contactNormal2 == -contactNormal1, but not always</summary>
    public Vector3 contactNormal2;

    public Vector3 angularComponentA;
    public Vector3 angularComponentB;

    public btScalar appliedPushImpulse;
    public btScalar appliedImpulse;

    public btScalar friction;
    public btScalar jacDiagABInv;
    /// <summary>Precomputed reciprocal of <see cref="jacDiagABInv"/> (1 / jacDiagABInv):
    /// avoids a scalar division per constraint per solver iteration.</summary>
    public btScalar jacDiagAB;
    public btScalar rhs;
    public btScalar cfm;

    public btScalar lowerLimit;
    public btScalar upperLimit;
    public btScalar rhsPenetration;
    /// <summary>union { m_originalContactPoint, m_numRowsForNonContactConstraint }</summary>
    public object originalContactPoint;
    public int numRowsForNonContactConstraint;

    public int overrideNumSolverIterations;
    public int frictionIndex;
    public int solverBodyIdA;
    public int solverBodyIdB;
}
