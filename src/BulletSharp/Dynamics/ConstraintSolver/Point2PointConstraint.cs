using BulletSharp.Math;

namespace BulletSharp;

/// <summary>btConstraintSetting</summary>
public class ConstraintSetting
{
    internal btScalar tau = (btScalar)0.3;
    internal btScalar damping = 1;
    internal btScalar impulseClamp = 0;

    public btScalar Tau
    {
        get => tau;
        set => tau = value;
    }

    public btScalar Damping
    {
        get => damping;
        set => damping = value;
    }

    public btScalar ImpulseClamp
    {
        get => impulseClamp;
        set => impulseClamp = value;
    }
}

/// <summary>btPoint2PointFlags</summary>
[Flags]
public enum Point2PointFlags
{
    None = 0,
    Erp = 1,
    Cfm = 2,
}

/// <summary>
/// Ported from btPoint2PointConstraint: a point-to-point (ball-socket) constraint
/// between two rigid bodies, each with its own local-space pivot point.
/// </summary>
public class Point2PointConstraint : TypedConstraint
{
    internal readonly JacobianEntry[] jac = new JacobianEntry[3]; // 3 orthogonal linear constraints

    internal Vector3 pivotInA;
    internal Vector3 pivotInB;

    internal int flags;
    internal btScalar erp;
    internal btScalar cfm;

    /// <summary>Backward-compatibility flag during the transition to getInfo/getInfo2</summary>
    public bool useSolveConstraintObsolete;

    internal readonly ConstraintSetting setting = new();

    public Point2PointConstraint(RigidBody rbA, RigidBody rbB, Vector3 pivotInA, Vector3 pivotInB)
        : base(TypedConstraintType.Point2Point, rbA, rbB)
    {
        this.pivotInA = pivotInA;
        this.pivotInB = pivotInB;
        flags = 0;
        useSolveConstraintObsolete = false;
    }

    public Point2PointConstraint(RigidBody rbA, Vector3 pivotInA)
        : base(TypedConstraintType.Point2Point, rbA)
    {
        this.pivotInA = pivotInA;
        pivotInB = rbA.worldTransform * pivotInA;
        flags = 0;
        useSolveConstraintObsolete = false;
    }

    /// <summary>btPoint2PointConstraint::buildJacobian</summary>
    public override void BuildJacobian()
    {
        // Needed by both solving methods
        appliedImpulse = 0;

        Vector3 normal = Vector3.Zero;

        for (int i = 0; i < 3; i++)
        {
            normal[i] = 1;
            jac[i] = JacobianEntry.PointConstraint(
                rigidBodyA.worldTransform.Basis.Transpose(),
                rigidBodyB.worldTransform.Basis.Transpose(),
                rigidBodyA.worldTransform * pivotInA - rigidBodyA.CenterOfMassPosition,
                rigidBodyB.worldTransform * pivotInB - rigidBodyB.CenterOfMassPosition,
                normal,
                rigidBodyA.invInertiaLocal,
                rigidBodyA.InvMass,
                rigidBodyB.invInertiaLocal,
                rigidBodyB.InvMass);
            normal[i] = 0;
        }
    }

    internal override void GetInfo1(ConstraintInfo1 info, ContactSolverInfo infoGlobal)
    {
        GetInfo1NonVirtual(info);
    }

    /// <summary>btPoint2PointConstraint::getInfo1NonVirtual</summary>
    internal void GetInfo1NonVirtual(ConstraintInfo1 info)
    {
        if (useSolveConstraintObsolete)
        {
            info.numConstraintRows = 0;
            info.nub = 0;
        }
        else
        {
            info.numConstraintRows = 3;
            info.nub = 3;
        }
    }

    internal override void GetInfo2(ConstraintInfo2 info, ContactSolverInfo infoGlobal,
        Transform[] body0Trans, Transform[] body1Trans, int body0Index, int body1Index)
    {
        GetInfo2NonVirtual(info, rigidBodyA.worldTransform, rigidBodyB.worldTransform);
    }

    /// <summary>btPoint2PointConstraint::getInfo2NonVirtual</summary>
    internal void GetInfo2NonVirtual(ConstraintInfo2 info, in Transform body0Trans, in Transform body1Trans)
    {
        // Set up the Jacobians
        info.j1LinearAxis[0] = 1;
        info.j1LinearAxis[info.rowskip + 1] = 1;
        info.j1LinearAxis[2 * info.rowskip + 2] = 1;

        Vector3 a1 = body0Trans.Basis * pivotInA;
        {
            // Write the skew-symmetric matrix of -a1 into the three rows of J1angularAxis
            Vector3 a1Neg = -a1;
            WriteSkewSymmetric(info.j1AngularAxis, info.rowskip, in a1Neg);
        }

        info.j2LinearAxis[0] = -1;
        info.j2LinearAxis[info.rowskip + 1] = -1;
        info.j2LinearAxis[2 * info.rowskip + 2] = -1;

        Vector3 a2 = body1Trans.Basis * pivotInB;

        {
            // Write the skew-symmetric matrix of a2 into the three rows of J2angularAxis
            WriteSkewSymmetric(info.j2AngularAxis, info.rowskip, in a2);
        }

        // Set the right-hand side
        btScalar currERP = (flags & (int)Point2PointFlags.Erp) != 0 ? erp : info.erp;
        btScalar k = info.fps * currERP;
        int j;
        for (j = 0; j < 3; j++)
        {
            info.constraintError[j * info.rowskip] = k * (a2[j] + body1Trans.Origin[j] - a1[j] - body0Trans.Origin[j]);
        }
        if ((flags & (int)Point2PointFlags.Cfm) != 0)
        {
            for (j = 0; j < 3; j++)
            {
                info.cfm[j * info.rowskip] = cfm;
            }
        }

        btScalar impulseClamp = setting.impulseClamp;
        for (j = 0; j < 3; j++)
        {
            if (setting.impulseClamp > 0)
            {
                info.lowerLimit[j * info.rowskip] = -impulseClamp;
                info.upperLimit[j * info.rowskip] = impulseClamp;
            }
        }
        info.damping = setting.damping;
    }

    /// <summary>btVector3::getSkewSymmetricMatrix (writes 3 rows with row stride rowskip)</summary>
    private static void WriteSkewSymmetric(btScalar[] dest, int rowskip, in Vector3 v)
    {
        dest[0] = 0; dest[1] = -v.Z; dest[2] = v.Y;
        dest[rowskip] = v.Z; dest[rowskip + 1] = 0; dest[rowskip + 2] = -v.X;
        dest[2 * rowskip] = -v.Y; dest[2 * rowskip + 1] = v.X; dest[2 * rowskip + 2] = 0;
    }

    /// <summary>btPoint2PointConstraint::updateRHS</summary>
    public void UpdateRHS(btScalar timeStep)
    {
    }

    public Vector3 PivotInA
    {
        get => pivotInA;
        set => pivotInA = value;
    }

    public Vector3 PivotInB
    {
        get => pivotInB;
        set => pivotInB = value;
    }

    public ConstraintSetting Setting => setting;

    /// <summary>btPoint2PointConstraint::setParam (overrides the global default parameters; axis is optional)</summary>
    public override void SetParam(int num, btScalar value, int axis = -1)
    {
        if (axis != -1)
        {
            // btAssertConstrParams(0)
        }
        else
        {
            switch ((ConstraintParams)num)
            {
                case ConstraintParams.Erp:
                case ConstraintParams.StopErp:
                    erp = value;
                    flags |= (int)Point2PointFlags.Erp;
                    break;
                case ConstraintParams.Cfm:
                case ConstraintParams.StopCfm:
                    cfm = value;
                    flags |= (int)Point2PointFlags.Cfm;
                    break;
                default:
                    break;
            }
        }
    }

    /// <summary>btPoint2PointConstraint::getParam (returns the local parameter value)</summary>
    public override btScalar GetParam(int num, int axis)
    {
        btScalar retVal = BtMath.SimdInfinity;
        if (axis != -1)
        {
            // btAssertConstrParams(0)
        }
        else
        {
            switch ((ConstraintParams)num)
            {
                case ConstraintParams.Erp:
                case ConstraintParams.StopErp:
                    retVal = erp;
                    break;
                case ConstraintParams.Cfm:
                case ConstraintParams.StopCfm:
                    retVal = cfm;
                    break;
                default:
                    break;
            }
        }
        return retVal;
    }

    public int Flags => flags;
}
