using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btFixedConstraint: a fixed constraint (locks all 6 degrees of freedom
/// between two rigid bodies).
/// Note: in C++, btFixedConstraint derives from btGeneric6DofSpring2Constraint (not in the
/// scope of this port), so this class derives directly from TypedConstraint and implements
/// equivalent behavior with 3 linear + 3 angular lock rows (the linear part matches
/// btPoint2PointConstraint; the angular part uses the same J1=axis, J2=-axis,
/// error = k * (Σa_i×b_i)·axis convention as btHingeConstraint). See
/// docs/KNOWN_LIMITATIONS.md, Module D2 note 1.
/// </summary>
public class FixedConstraint : TypedConstraint
{
    internal Transform frameInA;
    internal Transform frameInB;

    /// <summary>btFixedConstraint::btFixedConstraint(rbA, rbB, frameInA, frameInB)</summary>
    public FixedConstraint(RigidBody rbA, RigidBody rbB, Matrix frameInA, Matrix frameInB)
        : base(TypedConstraintType.Fixed, rbA, rbB)
    {
        this.frameInA = Transform.FromMatrix(in frameInA);
        this.frameInB = Transform.FromMatrix(in frameInB);
    }

    internal override void GetInfo1(ConstraintInfo1 info, ContactSolverInfo infoGlobal)
    {
        // Fixed 6 rows (3 linear + 3 angular), all bilateral constraints
        info.numConstraintRows = 6;
        info.nub = 0;
    }

    internal override void GetInfo2(ConstraintInfo2 info, ContactSolverInfo infoGlobal,
        Transform[] body0Trans, Transform[] body1Trans, int body0Index, int body1Index)
    {
        GetInfo2NonVirtual(info, rigidBodyA.worldTransform, rigidBodyB.worldTransform);
    }

    /// <summary>6 lock rows: 0-2 linear (pivots coincide), 3-5 angular (relative orientation error)</summary>
    internal void GetInfo2NonVirtual(ConstraintInfo2 info, in Transform transA, in Transform transB)
    {
        int s = info.rowskip;
        Transform trA = transA * frameInA;
        Transform trB = transB * frameInB;

        // ---- Linear part (same three rows as btPoint2PointConstraint) ----
        info.j1LinearAxis[0] = 1;
        info.j1LinearAxis[s + 1] = 1;
        info.j1LinearAxis[2 * s + 2] = 1;

        info.j2LinearAxis[0] = -1;
        info.j2LinearAxis[s + 1] = -1;
        info.j2LinearAxis[2 * s + 2] = -1;

        Vector3 a1 = trA.Origin - transA.Origin;
        Vector3 a1Neg = -a1;
        ConstraintMath.WriteSkewSymmetric(info.j1AngularAxis, s, in a1Neg);

        Vector3 a2 = trB.Origin - transB.Origin;
        ConstraintMath.WriteSkewSymmetric(info.j2AngularAxis, s, in a2);

        btScalar k = info.fps * info.erp;
        for (int i = 0; i < 3; i++)
        {
            info.constraintError[i * s] = k * (trB.Origin[i] - trA.Origin[i]);
        }

        // ---- Angular part (rows 3-5, lock relative orientation) ----
        Vector3 ax0 = trA.Basis.Column(0);
        Vector3 ax1 = trA.Basis.Column(1);
        Vector3 ax2 = trA.Basis.Column(2);
        Vector3 bx0 = trB.Basis.Column(0);
        Vector3 bx1 = trB.Basis.Column(1);
        Vector3 bx2 = trB.Basis.Column(2);
        // Orientation error vector (zero when aligned), same convention as the hinge angular rows
        Vector3 error = (ax0.Cross(bx0) + ax1.Cross(bx1) + ax2.Cross(bx2)) * (btScalar)0.5;

        WriteAngularRow(info, 3, in ax0, k * error.Dot(ax0));
        WriteAngularRow(info, 4, in ax1, k * error.Dot(ax1));
        WriteAngularRow(info, 5, in ax2, k * error.Dot(ax2));

        // All 6 rows are bilateral constraints (C++ relies on the solver to preset ±SIMD_INFINITY; written explicitly here)
        for (int i = 0; i < 6; i++)
        {
            info.lowerLimit[i * s] = -BtMath.SimdInfinity;
            info.upperLimit[i * s] = BtMath.SimdInfinity;
        }
    }

    private static void WriteAngularRow(ConstraintInfo2 info, int row, in Vector3 axis, btScalar error)
    {
        int srow = row * info.rowskip;
        info.j1AngularAxis[srow + 0] = axis[0];
        info.j1AngularAxis[srow + 1] = axis[1];
        info.j1AngularAxis[srow + 2] = axis[2];
        info.j2AngularAxis[srow + 0] = -axis[0];
        info.j2AngularAxis[srow + 1] = -axis[1];
        info.j2AngularAxis[srow + 2] = -axis[2];
        info.constraintError[srow] = error;
    }

    /// <summary>Frame offset A (local space)</summary>
    public Matrix FrameOffsetA
    {
        get => frameInA.ToMatrix();
        set => frameInA = Transform.FromMatrix(in value);
    }

    /// <summary>Frame offset B (local space)</summary>
    public Matrix FrameOffsetB
    {
        get => frameInB.ToMatrix();
        set => frameInB = Transform.FromMatrix(in value);
    }

    /// <summary>Frame setup corresponding to btFixedConstraint</summary>
    public void SetFrames(Matrix frameA, Matrix frameB)
    {
        frameInA = Transform.FromMatrix(in frameA);
        frameInB = Transform.FromMatrix(in frameB);
    }
}
