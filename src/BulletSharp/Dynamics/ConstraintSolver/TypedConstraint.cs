using BulletSharp.Math;

namespace BulletSharp;

/// <summary>btConstraintInfo1</summary>
public class ConstraintInfo1
{
    public int numConstraintRows;
    public int nub;

    public int NumConstraintRows
    {
        get => numConstraintRows;
        set => numConstraintRows = value;
    }
}

/// <summary>
/// btConstraintInfo2. The J matrices are stored as row-major arrays with row stride rowskip;
/// component s of row r is located at index = r * rowskip + s.
/// </summary>
public class ConstraintInfo2
{
    /// <summary>1 / timeStep</summary>
    public btScalar fps;
    public btScalar erp;

    public btScalar[] j1LinearAxis;
    public btScalar[] j1AngularAxis;
    public btScalar[] j2LinearAxis;
    public btScalar[] j2AngularAxis;

    public int rowskip;

    public btScalar[] constraintError;
    public btScalar[] cfm;

    public btScalar[] lowerLimit;
    public btScalar[] upperLimit;

    public int numIterations;
    public btScalar damping;

    public btScalar Fps { get => fps; set => fps = value; }
    public btScalar Erp { get => erp; set => erp = value; }
    public btScalar[] J1LinearAxis => j1LinearAxis;
    public btScalar[] J1AngularAxis => j1AngularAxis;
    public btScalar[] J2LinearAxis => j2LinearAxis;
    public btScalar[] J2AngularAxis => j2AngularAxis;
    public int RowSkip { get => rowskip; set => rowskip = value; }
    public btScalar[] ConstraintError => constraintError;
    public btScalar[] Cfm => cfm;
    public btScalar[] LowerLimit => lowerLimit;
    public btScalar[] UpperLimit => upperLimit;
    public int NumIterations { get => numIterations; set => numIterations = value; }
    public btScalar Damping { get => damping; set => damping = value; }
}

/// <summary>btJointFeedback</summary>
public class JointFeedback
{
    internal Vector3 appliedForceBodyA;
    internal Vector3 appliedTorqueBodyA;
    internal Vector3 appliedForceBodyB;
    internal Vector3 appliedTorqueBodyB;

    public Vector3 AppliedForceBodyA => appliedForceBodyA;
    public Vector3 AppliedTorqueBodyA => appliedTorqueBodyA;
    public Vector3 AppliedForceBodyB => appliedForceBodyB;
    public Vector3 AppliedTorqueBodyB => appliedTorqueBodyB;
}

/// <summary>
/// Ported from btTypedConstraint: base class of all constraints.
/// </summary>
public abstract class TypedConstraint : IDisposable
{
    private static int nextConstraintUid;
    private static RigidBody fixedBody;

    internal readonly TypedConstraintType constraintType;
    internal int userConstraintType = -1;
    internal int userConstraintId = -1;
    internal int uid;
    internal btScalar breakingImpulseThreshold = BtMath.SimdInfinity;
    internal bool isEnabled = true;
    internal bool needsFeedback;
    internal btScalar appliedImpulse;
    internal btScalar dbgDrawSize = 1;
    internal int overrideNumSolverIterations = -1;
    internal RigidBody rigidBodyA;
    internal RigidBody rigidBodyB;
    internal JointFeedback jointFeedback;

    protected TypedConstraint(TypedConstraintType type, RigidBody rbA)
    {
        constraintType = type;
        rigidBodyA = rbA;
        rigidBodyB = GetFixedBody();
        uid = ++nextConstraintUid;
    }

    protected TypedConstraint(TypedConstraintType type, RigidBody rbA, RigidBody rbB)
    {
        constraintType = type;
        rigidBodyA = rbA;
        rigidBodyB = rbB;
        uid = ++nextConstraintUid;
    }

    /// <summary>btTypedConstraint::getFixedBody (a static rigid body with zero mass)</summary>
    public static RigidBody GetFixedBody()
    {
        return fixedBody ??= CreateFixedBody();
    }

    private static RigidBody CreateFixedBody()
    {
        // btTypedConstraint::getFixedBody: btRigidBody s_fixed(0, 0, 0); s_fixed.setMassProps(0, (0,0,0))
        var body = new RigidBody(0, null, null);
        body.SetMassProps(0, Vector3.Zero);
        return body;
    }

    internal static RigidBody FixedBodyInternal => GetFixedBody();

    public int OverrideNumSolverIterations
    {
        get => overrideNumSolverIterations;
        set => overrideNumSolverIterations = value;
    }

    /// <summary>ODE-style buildJacobian (not needed by the sequential impulse solver; kept for compatibility)</summary>
    public virtual void BuildJacobian()
    {
    }

    public virtual void SetupSolverConstraint(object ca, int solverBodyA, int solverBodyB, btScalar timeStep)
    {
    }

    internal abstract void GetInfo1(ConstraintInfo1 info, ContactSolverInfo infoGlobal);

    internal abstract void GetInfo2(ConstraintInfo2 info, ContactSolverInfo infoGlobal,
        Transform[] body0Trans, Transform[] body1Trans, int body0Index, int body1Index);

    internal virtual void SolveGroupCacheFriendlySetup()
    {
    }

    public int UserConstraintType
    {
        get => userConstraintType;
        set => userConstraintType = value;
    }

    public int UserConstraintId
    {
        get => userConstraintId;
        set => userConstraintId = value;
    }

    public int Uid => uid;

    public bool NeedsFeedback
    {
        get => needsFeedback;
        set => needsFeedback = value;
    }

    public JointFeedback JointFeedback
    {
        get => jointFeedback;
        set => jointFeedback = value;
    }

    public btScalar AppliedImpulse => appliedImpulse;

    internal void InternalSetAppliedImpulse(btScalar value) => appliedImpulse = value;

    public btScalar BreakingImpulseThreshold
    {
        get => breakingImpulseThreshold;
        set => breakingImpulseThreshold = value;
    }

    public bool IsEnabled
    {
        get => isEnabled;
        set => isEnabled = value;
    }

    public RigidBody RigidBodyA => rigidBodyA;

    public RigidBody RigidBodyB => rigidBodyB;

    public TypedConstraintType ConstraintType => constraintType;

    public btScalar DbgDrawSize
    {
        get => dbgDrawSize;
        set => dbgDrawSize = value;
    }

    /// <summary>btTypedConstraint::getParam</summary>
    public virtual btScalar GetParam(int num, int axis) => 0;

    /// <summary>btTypedConstraint::setParam</summary>
    public virtual void SetParam(int num, btScalar value, int axis = -1)
    {
    }

    public virtual void Dispose()
    {
    }
}
