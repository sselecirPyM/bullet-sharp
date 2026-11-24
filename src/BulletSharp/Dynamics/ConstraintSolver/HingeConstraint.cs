using BulletSharp.Math;

namespace BulletSharp;

/// <summary>btHingeFlags</summary>
[Flags]
public enum HingeFlags
{
    None = 0,
    CfmStop = 1,
    ErpStop = 2,
    CfmNorm = 4,
    ErpNorm = 8,
}

/// <summary>
/// Ported from btAngularLimit (btTypedConstraint.h/.cpp, _BT_USE_CENTER_LIMIT_ version):
/// an angular limit expressed as center ± halfRange.
/// </summary>
internal class AngularLimit
{
    internal btScalar center;
    internal btScalar halfRange = -1; // <0 means the limit is not active
    internal btScalar softness = (btScalar)0.9;
    internal btScalar biasFactor = (btScalar)0.3;
    internal btScalar relaxationFactor = 1;
    internal btScalar correction;
    internal btScalar sign;
    internal bool solveLimit;

    /// <summary>btAngularLimit::set (the limit is disabled when low > high)</summary>
    public void Set(btScalar low, btScalar high, btScalar softness = (btScalar)0.9,
        btScalar biasFactor = (btScalar)0.3, btScalar relaxationFactor = 1)
    {
        halfRange = (high - low) / 2;
        center = ConstraintMath.NormalizeAngle(low + halfRange);
        this.softness = softness;
        this.biasFactor = biasFactor;
        this.relaxationFactor = relaxationFactor;
    }

    /// <summary>btAngularLimit::test: checks whether the angle violates the limit and computes correction/sign</summary>
    public void Test(btScalar angle)
    {
        correction = 0;
        sign = 0;
        solveLimit = false;

        if (halfRange >= 0)
        {
            btScalar deviation = ConstraintMath.NormalizeAngle(angle - center);
            if (deviation < -halfRange)
            {
                solveLimit = true;
                correction = -(deviation + halfRange);
                sign = 1;
            }
            else if (deviation > halfRange)
            {
                solveLimit = true;
                correction = halfRange - deviation;
                sign = -1;
            }
        }
    }

    /// <summary>btAngularLimit::getError</summary>
    public btScalar Error => correction * sign;

    /// <summary>btAngularLimit::fit: clamps the angle to the nearest limit</summary>
    public void Fit(ref btScalar angle)
    {
        if (halfRange > 0)
        {
            btScalar relativeAngle = ConstraintMath.NormalizeAngle(angle - center);
            // btEqual(relativeAngle, m_halfRange)
            if (!(relativeAngle <= halfRange && relativeAngle >= -halfRange))
            {
                angle = relativeAngle > 0 ? High : Low;
            }
        }
    }

    public btScalar Low => ConstraintMath.NormalizeAngle(center - halfRange);

    public btScalar High => ConstraintMath.NormalizeAngle(center + halfRange);

    public btScalar Softness => softness;

    public btScalar BiasFactor => biasFactor;

    public btScalar RelaxationFactor => relaxationFactor;

    public btScalar Correction => correction;

    public btScalar Sign => sign;

    public btScalar HalfRange => halfRange;

    public bool IsLimit => solveLimit;
}

/// <summary>
/// Ported from btHingeConstraint: a hinge constraint between two rigid bodies, each with
/// its own local pivot point and hinge axis (the Z axis of the constraint frame is the hinge axis).
/// </summary>
public class HingeConstraint : TypedConstraint
{
    private static readonly Vector3 VHinge = new(0, 0, 1); // hinge axis in constraint space

    internal JacobianEntry[] jac = new JacobianEntry[3];    // 3 orthogonal linear constraints
    internal JacobianEntry[] jacAng = new JacobianEntry[3]; // 2 orthogonal angular constraints + 1 limit/motor

    internal Transform rbAFrame; // constraint axis (assumes Z is the hinge axis)
    internal Transform rbBFrame;

    internal btScalar motorTargetVelocity;
    internal btScalar maxMotorImpulse;

    internal readonly AngularLimit limit = new();

    internal btScalar kHinge;

    internal btScalar accLimitImpulse;
    internal btScalar hingeAngle;
    internal btScalar referenceSign;

    internal bool angularOnly;
    internal bool enableAngularMotor;
    internal bool useSolveConstraintObsolete;
    internal bool useOffsetForConstraintFrame;
    internal bool useReferenceFrameA;

    internal btScalar accMotorImpulse;

    internal int flags;
    internal btScalar normalCFM;
    internal btScalar normalERP;
    internal btScalar stopCFM;
    internal btScalar stopERP;

    /// <summary>btHingeConstraint::btHingeConstraint(rbA, rbB, pivotInA, pivotInB, axisInA, axisInB, useReferenceFrameA)</summary>
    public HingeConstraint(RigidBody rbA, RigidBody rbB, Vector3 pivotInA, Vector3 pivotInB,
        Vector3 axisInA, Vector3 axisInB, bool useReferenceFrameA = false)
        : base(TypedConstraintType.Hinge, rbA, rbB)
    {
        // HINGE_USE_OBSOLETE_SOLVER = false / HINGE_USE_FRAME_OFFSET = true
        useSolveConstraintObsolete = false;
        useOffsetForConstraintFrame = true;
        this.useReferenceFrameA = useReferenceFrameA;

        rbAFrame.Origin = pivotInA;

        // When no frame is given, treat as zero angle and build the orthonormal basis directly from the body transform axes
        Vector3 rbAxisA1 = rbA.worldTransform.Basis.Column(0);

        Vector3 rbAxisA2;
        btScalar projection = axisInA.Dot(rbAxisA1);
        if (projection >= 1 - BtMath.SimdEpsilon)
        {
            rbAxisA1 = -rbA.worldTransform.Basis.Column(2);
            rbAxisA2 = rbA.worldTransform.Basis.Column(1);
        }
        else if (projection <= -1 + BtMath.SimdEpsilon)
        {
            rbAxisA1 = rbA.worldTransform.Basis.Column(2);
            rbAxisA2 = rbA.worldTransform.Basis.Column(1);
        }
        else
        {
            rbAxisA2 = axisInA.Cross(rbAxisA1);
            rbAxisA1 = rbAxisA2.Cross(axisInA);
        }

        rbAFrame.Basis = new Matrix3x3(
            rbAxisA1.X, rbAxisA2.X, axisInA.X,
            rbAxisA1.Y, rbAxisA2.Y, axisInA.Y,
            rbAxisA1.Z, rbAxisA2.Z, axisInA.Z);

        Quaternion rotationArc = Quaternion.ShortestArc(axisInA, axisInB);
        Vector3 rbAxisB1 = Quaternion.Rotate(rotationArc, rbAxisA1);
        Vector3 rbAxisB2 = axisInB.Cross(rbAxisB1);

        rbBFrame.Origin = pivotInB;
        rbBFrame.Basis = new Matrix3x3(
            rbAxisB1.X, rbAxisB2.X, axisInB.X,
            rbAxisB1.Y, rbAxisB2.Y, axisInB.Y,
            rbAxisB1.Z, rbAxisB2.Z, axisInB.Z);

        referenceSign = useReferenceFrameA ? -1 : 1;
    }

    /// <summary>btHingeConstraint::btHingeConstraint(rbA, pivotInA, axisInA, useReferenceFrameA)</summary>
    public HingeConstraint(RigidBody rbA, Vector3 pivotInA, Vector3 axisInA, bool useReferenceFrameA = false)
        : base(TypedConstraintType.Hinge, rbA)
    {
        useSolveConstraintObsolete = false;
        useOffsetForConstraintFrame = true;
        this.useReferenceFrameA = useReferenceFrameA;

        // When no frame is given, treat as zero angle; the fixed axis is in world space
        BtMath.PlaneSpace1(in axisInA, out Vector3 rbAxisA1, out Vector3 rbAxisA2);

        rbAFrame.Origin = pivotInA;
        rbAFrame.Basis = new Matrix3x3(
            rbAxisA1.X, rbAxisA2.X, axisInA.X,
            rbAxisA1.Y, rbAxisA2.Y, axisInA.Y,
            rbAxisA1.Z, rbAxisA2.Z, axisInA.Z);

        Vector3 axisInB = rbA.worldTransform.Basis * axisInA;

        Quaternion rotationArc = Quaternion.ShortestArc(axisInA, axisInB);
        Vector3 rbAxisB1 = Quaternion.Rotate(rotationArc, rbAxisA1);
        Vector3 rbAxisB2 = axisInB.Cross(rbAxisB1);

        rbBFrame.Origin = rbA.worldTransform * pivotInA;
        rbBFrame.Basis = new Matrix3x3(
            rbAxisB1.X, rbAxisB2.X, axisInB.X,
            rbAxisB1.Y, rbAxisB2.Y, axisInB.Y,
            rbAxisB1.Z, rbAxisB2.Z, axisInB.Z);

        referenceSign = useReferenceFrameA ? -1 : 1;
    }

    /// <summary>btHingeConstraint::btHingeConstraint(rbA, rbB, rbAFrame, rbBFrame, useReferenceFrameA)</summary>
    public HingeConstraint(RigidBody rbA, RigidBody rbB, Matrix rbAFrame, Matrix rbBFrame, bool useReferenceFrameA = false)
        : base(TypedConstraintType.Hinge, rbA, rbB)
    {
        useSolveConstraintObsolete = false;
        useOffsetForConstraintFrame = true;
        this.useReferenceFrameA = useReferenceFrameA;

        this.rbAFrame = Transform.FromMatrix(in rbAFrame);
        this.rbBFrame = Transform.FromMatrix(in rbBFrame);

        referenceSign = useReferenceFrameA ? -1 : 1;
    }

    /// <summary>btHingeConstraint::btHingeConstraint(rbA, rbAFrame, useReferenceFrameA)</summary>
    public HingeConstraint(RigidBody rbA, Matrix rbAFrame, bool useReferenceFrameA = false)
        : base(TypedConstraintType.Hinge, rbA)
    {
        useSolveConstraintObsolete = false;
        useOffsetForConstraintFrame = true;
        this.useReferenceFrameA = useReferenceFrameA;

        this.rbAFrame = Transform.FromMatrix(in rbAFrame);
        rbBFrame = this.rbAFrame;
        // When body B is not provided, B is implicitly in world space
        rbBFrame.Origin = rbA.worldTransform * this.rbAFrame.Origin;

        referenceSign = useReferenceFrameA ? -1 : 1;
    }

    /// <summary>btHingeConstraint::buildJacobian (only for the useSolveConstraintObsolete legacy path)</summary>
    public override void BuildJacobian()
    {
        if (useSolveConstraintObsolete)
        {
            appliedImpulse = 0;
            accMotorImpulse = 0;

            if (!angularOnly)
            {
                Vector3 pivotAInW = rigidBodyA.worldTransform * rbAFrame.Origin;
                Vector3 pivotBInW = rigidBodyB.worldTransform * rbBFrame.Origin;
                Vector3 relPos = pivotBInW - pivotAInW;

                Vector3[] normal = new Vector3[3];
                if (relPos.LengthSquared() > BtMath.SimdEpsilon)
                {
                    normal[0] = relPos.Normalized();
                }
                else
                {
                    normal[0] = new Vector3(1, 0, 0);
                }

                BtMath.PlaneSpace1(in normal[0], out normal[1], out normal[2]);

                for (int i = 0; i < 3; i++)
                {
                    jac[i] = JacobianEntry.PointConstraint(
                        rigidBodyA.worldTransform.Basis.Transpose(),
                        rigidBodyB.worldTransform.Basis.Transpose(),
                        pivotAInW - rigidBodyA.CenterOfMassPosition,
                        pivotBInW - rigidBodyB.CenterOfMassPosition,
                        normal[i],
                        rigidBodyA.invInertiaLocal,
                        rigidBodyA.InvMass,
                        rigidBodyB.invInertiaLocal,
                        rigidBodyB.InvMass);
                }
            }

            // Compute two jointAxes orthogonal to the hinge axis and require both bodies to have equal angular velocities along them
            BtMath.PlaneSpace1(rbAFrame.Basis.Column(2), out Vector3 jointAxis0local, out Vector3 jointAxis1local);

            Vector3 jointAxis0 = rigidBodyA.worldTransform.Basis * jointAxis0local;
            Vector3 jointAxis1 = rigidBodyA.worldTransform.Basis * jointAxis1local;
            Vector3 hingeAxisWorld = rigidBodyA.worldTransform.Basis * rbAFrame.Basis.Column(2);

            jacAng[0] = ConstraintMath.CreateAngularJacobianEntry(jointAxis0,
                rigidBodyA.worldTransform.Basis.Transpose(),
                rigidBodyB.worldTransform.Basis.Transpose(),
                rigidBodyA.invInertiaLocal,
                rigidBodyB.invInertiaLocal);

            jacAng[1] = ConstraintMath.CreateAngularJacobianEntry(jointAxis1,
                rigidBodyA.worldTransform.Basis.Transpose(),
                rigidBodyB.worldTransform.Basis.Transpose(),
                rigidBodyA.invInertiaLocal,
                rigidBodyB.invInertiaLocal);

            jacAng[2] = ConstraintMath.CreateAngularJacobianEntry(hingeAxisWorld,
                rigidBodyA.worldTransform.Basis.Transpose(),
                rigidBodyB.worldTransform.Basis.Transpose(),
                rigidBodyA.invInertiaLocal,
                rigidBodyB.invInertiaLocal);

            // Clear the accumulated values
            accLimitImpulse = 0;

            // Test the angular limit
            TestLimit(in rigidBodyA.worldTransform, in rigidBodyB.worldTransform);

            // Compute K = J*W*J' for the hinge axis
            Vector3 axisA = rigidBodyA.worldTransform.Basis * rbAFrame.Basis.Column(2);
            kHinge = 1 / (rigidBodyA.ComputeAngularImpulseDenominator(axisA) +
                          rigidBodyB.ComputeAngularImpulseDenominator(axisA));
        }
    }

    internal override void GetInfo1(ConstraintInfo1 info, ContactSolverInfo infoGlobal)
    {
        // btHingeConstraint::getInfo1
        if (useSolveConstraintObsolete)
        {
            info.numConstraintRows = 0;
            info.nub = 0;
        }
        else
        {
            info.numConstraintRows = 5; // fixed 3 linear + 2 angular
            info.nub = 1;
            // Always prepare additional rows to avoid recomputation (data not yet available)
            TestLimit(in rigidBodyA.worldTransform, in rigidBodyB.worldTransform);
            if (limit.IsLimit || enableAngularMotor)
            {
                info.numConstraintRows++; // 3rd angular row for the limit
                info.nub--;
            }
        }
    }

    /// <summary>btHingeConstraint::getInfo1NonVirtual</summary>
    internal void GetInfo1NonVirtual(ConstraintInfo1 info)
    {
        if (useSolveConstraintObsolete)
        {
            info.numConstraintRows = 0;
            info.nub = 0;
        }
        else
        {
            // Always add the limit row to avoid recomputation (data not yet available)
            info.numConstraintRows = 6; // fixed 3 linear + 2 angular + 1 limit
            info.nub = 0;
        }
    }

    internal override void GetInfo2(ConstraintInfo2 info, ContactSolverInfo infoGlobal,
        Transform[] body0Trans, Transform[] body1Trans, int body0Index, int body1Index)
    {
        // btHingeConstraint::getInfo2 (testLimit was already executed in GetInfo1)
        if (useOffsetForConstraintFrame)
        {
            GetInfo2InternalUsingFrameOffset(info, rigidBodyA.worldTransform, rigidBodyB.worldTransform,
                rigidBodyA.angularVelocity, rigidBodyB.angularVelocity);
        }
        else
        {
            GetInfo2Internal(info, rigidBodyA.worldTransform, rigidBodyB.worldTransform,
                rigidBodyA.angularVelocity, rigidBodyB.angularVelocity);
        }
    }

    /// <summary>btHingeConstraint::getInfo2NonVirtual</summary>
    internal void GetInfo2NonVirtual(ConstraintInfo2 info, in Transform transA, in Transform transB,
        in Vector3 angVelA, in Vector3 angVelB)
    {
        // The regular (virtual) implementation already executed testLimit in GetInfo1; do it here instead
        TestLimit(in transA, in transB);

        GetInfo2Internal(info, in transA, in transB, in angVelA, in angVelB);
    }

    /// <summary>btHingeConstraint::getInfo2Internal (legacy path without frame offsets)</summary>
    internal void GetInfo2Internal(ConstraintInfo2 info, in Transform transA, in Transform transB,
        in Vector3 angVelA, in Vector3 angVelB)
    {
        int skip = info.rowskip;
        // Constraint frames in world space
        Transform trA = transA * rbAFrame;
        Transform trB = transB * rbBFrame;
        // Pivot points
        Vector3 pivotAInW = trA.Origin;
        Vector3 pivotBInW = trB.Origin;

        // 线性部分（全部固定）
        if (!angularOnly)
        {
            info.j1LinearAxis[0] = 1;
            info.j1LinearAxis[skip + 1] = 1;
            info.j1LinearAxis[2 * skip + 2] = 1;

            info.j2LinearAxis[0] = -1;
            info.j2LinearAxis[skip + 1] = -1;
            info.j2LinearAxis[2 * skip + 2] = -1;
        }

        // C++ 依赖求解器对 lowerLimit/upperLimit 预设 ±SIMD_INFINITY；此处显式写入（行 0-4）
        for (int i = 0; i < 5; i++)
        {
            info.lowerLimit[i * skip] = -BtMath.SimdInfinity;
            info.upperLimit[i * skip] = BtMath.SimdInfinity;
        }

        Vector3 a1 = pivotAInW - transA.Origin;
        {
            // -a1 的反对称矩阵写入 J1angularAxis 的三行
            Vector3 a1Neg = -a1;
            ConstraintMath.WriteSkewSymmetric(info.j1AngularAxis, skip, in a1Neg);
        }
        Vector3 a2 = pivotBInW - transB.Origin;
        {
            ConstraintMath.WriteSkewSymmetric(info.j2AngularAxis, skip, in a2);
        }
        // 线性右端项
        btScalar normalErp = (flags & (int)HingeFlags.ErpNorm) != 0 ? normalERP : info.erp;

        btScalar k = info.fps * normalErp;
        if (!angularOnly)
        {
            for (int i = 0; i < 3; i++)
            {
                info.constraintError[i * skip] = k * (pivotBInW[i] - pivotAInW[i]);
            }
        }
        // 使绕 X、Y 的旋转相等：铰轴应是唯一无约束的旋转轴
        // 取铰轴（Z）与两个正交轴（X、Y）
        Vector3 ax1 = trA.Basis.Column(2);
        Vector3 p = trA.Basis.Column(0);
        Vector3 q = trA.Basis.Column(1);
        // 设置两个铰链角行
        int s3 = 3 * skip;
        int s4 = 4 * skip;

        info.j1AngularAxis[s3 + 0] = p[0];
        info.j1AngularAxis[s3 + 1] = p[1];
        info.j1AngularAxis[s3 + 2] = p[2];
        info.j1AngularAxis[s4 + 0] = q[0];
        info.j1AngularAxis[s4 + 1] = q[1];
        info.j1AngularAxis[s4 + 2] = q[2];

        info.j2AngularAxis[s3 + 0] = -p[0];
        info.j2AngularAxis[s3 + 1] = -p[1];
        info.j2AngularAxis[s3 + 2] = -p[2];
        info.j2AngularAxis[s4 + 0] = -q[0];
        info.j2AngularAxis[s4 + 1] = -q[1];
        info.j2AngularAxis[s4 + 2] = -q[2];
        // 计算约束方程右端项：angular_velocity = (erp*fps) * (ax1 x ax2)，再投影到 p、q
        Vector3 ax2 = trB.Basis.Column(2);
        Vector3 u = ax1.Cross(ax2);
        info.constraintError[s3] = k * u.Dot(p);
        info.constraintError[s4] = k * u.Dot(q);
        // 检查角度限位
        int nrow = 4; // 最后填充的行
        int srow;
        btScalar limitErr = 0;
        int limitFlag = 0;
        if (limit.IsLimit)
        {
            limitErr = limit.Correction * referenceSign;
            limitFlag = limitErr > 0 ? 1 : 2;
        }
        // 若铰链有限位或马达，添加额外行
        bool powered = enableAngularMotor;
        if (limitFlag != 0 || powered)
        {
            nrow++;
            srow = nrow * info.rowskip;
            info.j1AngularAxis[srow + 0] = ax1[0];
            info.j1AngularAxis[srow + 1] = ax1[1];
            info.j1AngularAxis[srow + 2] = ax1[2];

            info.j2AngularAxis[srow + 0] = -ax1[0];
            info.j2AngularAxis[srow + 1] = -ax1[1];
            info.j2AngularAxis[srow + 2] = -ax1[2];

            btScalar lostop = LowerLimit;
            btScalar histop = UpperLimit;
            if (limitFlag != 0 && lostop == histop)
            {
                // 马达无效
                powered = false;
            }
            info.constraintError[srow] = 0;
            btScalar currERP = (flags & (int)HingeFlags.ErpStop) != 0 ? stopERP : normalErp;
            if (powered)
            {
                if ((flags & (int)HingeFlags.CfmNorm) != 0)
                {
                    info.cfm[srow] = normalCFM;
                }
                btScalar motFact = ConstraintMath.GetMotorFactor(hingeAngle, lostop, histop, motorTargetVelocity, info.fps * currERP);
                info.constraintError[srow] += motFact * motorTargetVelocity * referenceSign;
                info.lowerLimit[srow] = -maxMotorImpulse;
                info.upperLimit[srow] = maxMotorImpulse;
            }
            if (limitFlag != 0)
            {
                k = info.fps * currERP;
                info.constraintError[srow] += k * limitErr;
                if ((flags & (int)HingeFlags.CfmStop) != 0)
                {
                    info.cfm[srow] = stopCFM;
                }
                if (lostop == histop)
                {
                    // 低位与高位同时限位
                    info.lowerLimit[srow] = -BtMath.SimdInfinity;
                    info.upperLimit[srow] = BtMath.SimdInfinity;
                }
                else if (limitFlag == 1)
                {
                    // 低位限位
                    info.lowerLimit[srow] = 0;
                    info.upperLimit[srow] = BtMath.SimdInfinity;
                }
                else
                {
                    // 高位限位
                    info.lowerLimit[srow] = -BtMath.SimdInfinity;
                    info.upperLimit[srow] = 0;
                }
                // 反弹
                btScalar bounce = limit.RelaxationFactor;
                if (bounce > 0)
                {
                    btScalar vel = angVelA.Dot(ax1);
                    vel -= angVelB.Dot(ax1);
                    // 仅当速度朝向限位且所得 c[] 超过现值时应用反弹
                    if (limitFlag == 1)
                    {
                        // 低位限位
                        if (vel < 0)
                        {
                            btScalar newc = -bounce * vel;
                            if (newc > info.constraintError[srow])
                            {
                                info.constraintError[srow] = newc;
                            }
                        }
                    }
                    else
                    {
                        // 高位限位——计算全部反过来
                        if (vel > 0)
                        {
                            btScalar newc = -bounce * vel;
                            if (newc < info.constraintError[srow])
                            {
                                info.constraintError[srow] = newc;
                            }
                        }
                    }
                }
                info.constraintError[srow] *= limit.BiasFactor;
            }
        }
    }

    /// <summary>btHingeConstraint::getInfo2InternalUsingFrameOffset（默认路径，HINGE_USE_FRAME_OFFSET=true）</summary>
    internal void GetInfo2InternalUsingFrameOffset(ConstraintInfo2 info, in Transform transA, in Transform transB,
        in Vector3 angVelA, in Vector3 angVelB)
    {
        int s = info.rowskip;
        // 世界空间下的约束帧
        Transform trA = transA * rbAFrame;
        Transform trB = transB * rbBFrame;
        // 世界坐标系下两帧的差
        Vector3 ofs = trB.Origin - trA.Origin;
        // 质量权重因子
        btScalar miA = rigidBodyA.InvMass;
        btScalar miB = rigidBodyB.InvMass;
        bool hasStaticBody = miA < BtMath.SimdEpsilon || miB < BtMath.SimdEpsilon;
        btScalar miS = miA + miB;
        btScalar factA;
        if (miS > 0)
        {
            factA = miB / miS;
        }
        else
        {
            factA = (btScalar)0.5;
        }
        btScalar factB = 1 - factA;
        // 铰轴期望方向 = frameA 与 frameB 的 Z 轴（世界系）按质量加权
        Vector3 ax1A = trA.Basis.Column(2);
        Vector3 ax1B = trB.Basis.Column(2);
        Vector3 ax1 = ax1A * factA + ax1B * factB;
        if (ax1.LengthSquared() < BtMath.SimdEpsilon)
        {
            factA = 0;
            factB = 1;
            ax1 = ax1A * factA + ax1B * factB;
        }
        ax1.Normalize();
        // 填充前 3 行：要求 velA + wA x relA == velB + wB x relB
        Transform bodyATrans = transA;
        Transform bodyBTrans = transB;
        int s0 = 0;
        int s1 = s;
        int s2 = s * 2;
        int nrow = 2; // 最后填充的行
        // 世界系下从 bodyB 到 frameB 的向量
        Vector3 relB = trB.Origin - bodyBTrans.Origin;
        // 在铰轴上的投影
        Vector3 projB = ax1 * relB.Dot(ax1);
        // 从 bodyB 指向铰轴（且与之正交）的向量
        Vector3 orthoB = relB - projB;
        // bodyA 同理
        Vector3 relA = trA.Origin - bodyATrans.Origin;
        Vector3 projA = ax1 * relA.Dot(ax1);
        Vector3 orthoA = relA - projA;
        Vector3 totalDist = projA - projB;
        // 得到偏移向量 relA、relB
        relA = orthoA + totalDist * factA;
        relB = orthoB - totalDist * factB;
        // 选择铰轴的平均正交轴
        Vector3 p = orthoB * factA + orthoA * factB;
        btScalar len2 = p.LengthSquared();
        if (len2 > BtMath.SimdEpsilon)
        {
            p /= BtMath.Sqrt(len2);
        }
        else
        {
            p = trA.Basis.Column(1);
        }
        // 再构造一个正交轴
        Vector3 q = ax1.Cross(p);
        // 填充三行
        Vector3 tmpA = relA.Cross(p);
        Vector3 tmpB = relB.Cross(p);
        for (int i = 0; i < 3; i++) info.j1AngularAxis[s0 + i] = tmpA[i];
        for (int i = 0; i < 3; i++) info.j2AngularAxis[s0 + i] = -tmpB[i];
        tmpA = relA.Cross(q);
        tmpB = relB.Cross(q);
        if (hasStaticBody && limit.IsLimit)
        {
            // 使静态-动态对象间的约束更刚硬：角限位触发时从方程中移除 wA（或 wB）
            tmpB *= factB;
            tmpA *= factA;
        }
        for (int i = 0; i < 3; i++) info.j1AngularAxis[s1 + i] = tmpA[i];
        for (int i = 0; i < 3; i++) info.j2AngularAxis[s1 + i] = -tmpB[i];
        tmpA = relA.Cross(ax1);
        tmpB = relB.Cross(ax1);
        if (hasStaticBody)
        {
            // 使静态-动态对象间的约束更刚硬：从方程中移除 wA（或 wB）
            tmpB *= factB;
            tmpA *= factA;
        }
        for (int i = 0; i < 3; i++) info.j1AngularAxis[s2 + i] = tmpA[i];
        for (int i = 0; i < 3; i++) info.j2AngularAxis[s2 + i] = -tmpB[i];

        btScalar normalErp = (flags & (int)HingeFlags.ErpNorm) != 0 ? normalERP : info.erp;
        btScalar k = info.fps * normalErp;

        if (!angularOnly)
        {
            for (int i = 0; i < 3; i++) info.j1LinearAxis[s0 + i] = p[i];
            for (int i = 0; i < 3; i++) info.j1LinearAxis[s1 + i] = q[i];
            for (int i = 0; i < 3; i++) info.j1LinearAxis[s2 + i] = ax1[i];

            for (int i = 0; i < 3; i++) info.j2LinearAxis[s0 + i] = -p[i];
            for (int i = 0; i < 3; i++) info.j2LinearAxis[s1 + i] = -q[i];
            for (int i = 0; i < 3; i++) info.j2LinearAxis[s2 + i] = -ax1[i];

            // 计算右端项的三个元素
            btScalar rhs = k * p.Dot(ofs);
            info.constraintError[s0] = rhs;
            rhs = k * q.Dot(ofs);
            info.constraintError[s1] = rhs;
            rhs = k * ax1.Dot(ofs);
            info.constraintError[s2] = rhs;
        }
        // C++ 依赖求解器对 lowerLimit/upperLimit 预设 ±SIMD_INFINITY；此处显式写入（行 0-4）
        for (int i = 0; i < 5; i++)
        {
            info.lowerLimit[i * s] = -BtMath.SimdInfinity;
            info.upperLimit[i * s] = BtMath.SimdInfinity;
        }
        // 铰轴应是唯一无约束旋转轴；垂直于铰轴的角速度应相等
        int s3 = 3 * s;
        int s4 = 4 * s;
        info.j1AngularAxis[s3 + 0] = p[0];
        info.j1AngularAxis[s3 + 1] = p[1];
        info.j1AngularAxis[s3 + 2] = p[2];
        info.j1AngularAxis[s4 + 0] = q[0];
        info.j1AngularAxis[s4 + 1] = q[1];
        info.j1AngularAxis[s4 + 2] = q[2];

        info.j2AngularAxis[s3 + 0] = -p[0];
        info.j2AngularAxis[s3 + 1] = -p[1];
        info.j2AngularAxis[s3 + 2] = -p[2];
        info.j2AngularAxis[s4 + 0] = -q[0];
        info.j2AngularAxis[s4 + 1] = -q[1];
        info.j2AngularAxis[s4 + 2] = -q[2];
        // 角行右端项：angular_velocity = (erp*fps) * (ax1A x ax1B)，再投影到 p、q
        k = info.fps * normalErp;

        Vector3 u = ax1A.Cross(ax1B);
        info.constraintError[s3] = k * u.Dot(p);
        info.constraintError[s4] = k * u.Dot(q);
        // 检查角度限位
        nrow = 4; // 最后填充的行
        int srow;
        btScalar limitErr = 0;
        int limitFlag = 0;
        if (limit.IsLimit)
        {
            limitErr = limit.Correction * referenceSign;
            limitFlag = limitErr > 0 ? 1 : 2;
        }
        // 若铰链有限位或马达，添加额外行
        bool powered = enableAngularMotor;
        if (limitFlag != 0 || powered)
        {
            nrow++;
            srow = nrow * info.rowskip;
            info.j1AngularAxis[srow + 0] = ax1[0];
            info.j1AngularAxis[srow + 1] = ax1[1];
            info.j1AngularAxis[srow + 2] = ax1[2];

            info.j2AngularAxis[srow + 0] = -ax1[0];
            info.j2AngularAxis[srow + 1] = -ax1[1];
            info.j2AngularAxis[srow + 2] = -ax1[2];

            btScalar lostop = LowerLimit;
            btScalar histop = UpperLimit;
            if (limitFlag != 0 && lostop == histop)
            {
                // 马达无效
                powered = false;
            }
            info.constraintError[srow] = 0;
            btScalar currERP = (flags & (int)HingeFlags.ErpStop) != 0 ? stopERP : normalErp;
            if (powered)
            {
                if ((flags & (int)HingeFlags.CfmNorm) != 0)
                {
                    info.cfm[srow] = normalCFM;
                }
                btScalar motFact = ConstraintMath.GetMotorFactor(hingeAngle, lostop, histop, motorTargetVelocity, info.fps * currERP);
                info.constraintError[srow] += motFact * motorTargetVelocity * referenceSign;
                info.lowerLimit[srow] = -maxMotorImpulse;
                info.upperLimit[srow] = maxMotorImpulse;
            }
            if (limitFlag != 0)
            {
                k = info.fps * currERP;
                info.constraintError[srow] += k * limitErr;
                if ((flags & (int)HingeFlags.CfmStop) != 0)
                {
                    info.cfm[srow] = stopCFM;
                }
                if (lostop == histop)
                {
                    // 低位与高位同时限位
                    info.lowerLimit[srow] = -BtMath.SimdInfinity;
                    info.upperLimit[srow] = BtMath.SimdInfinity;
                }
                else if (limitFlag == 1)
                {
                    // 低位限位
                    info.lowerLimit[srow] = 0;
                    info.upperLimit[srow] = BtMath.SimdInfinity;
                }
                else
                {
                    // 高位限位
                    info.lowerLimit[srow] = -BtMath.SimdInfinity;
                    info.upperLimit[srow] = 0;
                }
                // 反弹
                btScalar bounce = limit.RelaxationFactor;
                if (bounce > 0)
                {
                    btScalar vel = angVelA.Dot(ax1);
                    vel -= angVelB.Dot(ax1);
                    // 仅当速度朝向限位且所得 c[] 超过现值时应用反弹
                    if (limitFlag == 1)
                    {
                        // 低位限位
                        if (vel < 0)
                        {
                            btScalar newc = -bounce * vel;
                            if (newc > info.constraintError[srow])
                            {
                                info.constraintError[srow] = newc;
                            }
                        }
                    }
                    else
                    {
                        // 高位限位——计算全部反过来
                        if (vel > 0)
                        {
                            btScalar newc = -bounce * vel;
                            if (newc < info.constraintError[srow])
                            {
                                info.constraintError[srow] = newc;
                            }
                        }
                    }
                }
                info.constraintError[srow] *= limit.BiasFactor;
            }
        }
    }

    /// <summary>btHingeConstraint::setFrames</summary>
    public void SetFrames(Matrix frameA, Matrix frameB)
    {
        rbAFrame = Transform.FromMatrix(in frameA);
        rbBFrame = Transform.FromMatrix(in frameB);
        BuildJacobian();
    }

    /// <summary>btHingeConstraint::updateRHS</summary>
    public void UpdateRHS(btScalar timeStep)
    {
    }

    /// <summary>btHingeConstraint::getHingeAngle（范围 [-PI,PI]）</summary>
    public btScalar GetHingeAngle()
    {
        return GetHingeAngle(in rigidBodyA.worldTransform, in rigidBodyB.worldTransform);
    }

    /// <summary>btHingeConstraint::getHingeAngle(transA, transB)</summary>
    internal btScalar GetHingeAngle(in Transform transA, in Transform transB)
    {
        Vector3 refAxis0 = transA.Basis * rbAFrame.Basis.Column(0);
        Vector3 refAxis1 = transA.Basis * rbAFrame.Basis.Column(1);
        Vector3 swingAxis = transB.Basis * rbBFrame.Basis.Column(1);
        btScalar angle = BtMath.Atan2(swingAxis.Dot(refAxis0), swingAxis.Dot(refAxis1));
        return referenceSign * angle;
    }

    /// <summary>btHingeConstraint::testLimit</summary>
    internal void TestLimit(in Transform transA, in Transform transB)
    {
        // 计算限位信息
        hingeAngle = GetHingeAngle(in transA, in transB);
        limit.Test(hingeAngle);
    }

    /// <summary>btHingeConstraint::setMotorTarget(qAinB, dt)：qAinB 为 A 相对 B 的旋转</summary>
    public void SetMotorTarget(Quaternion qAinB, btScalar dt)
    {
        // 把目标从刚体空间转换到约束空间
        Quaternion qConstraint = rbBFrame.GetRotation().Inverse() * qAinB * rbAFrame.GetRotation();
        qConstraint.Normalize();

        // 提取“纯”铰链分量
        Vector3 vNoHinge = Quaternion.Rotate(qConstraint, VHinge);
        vNoHinge.Normalize();
        Quaternion qNoHinge = Quaternion.ShortestArc(VHinge, vNoHinge);
        Quaternion qHinge = qNoHinge.Inverse() * qConstraint;
        qHinge.Normalize();

        // 计算钳制到限位内的角度目标
        btScalar targetAngle = qHinge.GetAngle();
        if (targetAngle > BtMath.Pi)
        {
            // 绕远路，翻转四元数后重算
            qHinge = -qHinge;
            targetAngle = qHinge.GetAngle();
        }
        if (qHinge.Z < 0)
        {
            targetAngle = -targetAngle;
        }

        SetMotorTarget(targetAngle, dt);
    }

    /// <summary>btHingeConstraint::setMotorTarget(targetAngle, dt)（底层换算为角速度，需每帧调用）</summary>
    public void SetMotorTarget(btScalar targetAngle, btScalar dt)
    {
        limit.Fit(ref targetAngle);
        // 计算角速度
        btScalar curAngle = GetHingeAngle(in rigidBodyA.worldTransform, in rigidBodyB.worldTransform);
        btScalar dAngle = targetAngle - curAngle;
        motorTargetVelocity = dAngle / dt;
    }

    /// <summary>btHingeConstraint::setAxis</summary>
    public void SetAxis(ref Vector3 axisInA)
    {
        BtMath.PlaneSpace1(in axisInA, out Vector3 rbAxisA1, out Vector3 rbAxisA2);
        Vector3 pivotInA = rbAFrame.Origin;
        rbAFrame.Basis = new Matrix3x3(
            rbAxisA1.X, rbAxisA2.X, axisInA.X,
            rbAxisA1.Y, rbAxisA2.Y, axisInA.Y,
            rbAxisA1.Z, rbAxisA2.Z, axisInA.Z);

        Vector3 axisInB = rigidBodyA.worldTransform.Basis * axisInA;

        Quaternion rotationArc = Quaternion.ShortestArc(axisInA, axisInB);
        Vector3 rbAxisB1 = Quaternion.Rotate(rotationArc, rbAxisA1);
        Vector3 rbAxisB2 = axisInB.Cross(rbAxisB1);

        rbBFrame.Origin = rigidBodyB.worldTransform.Inverse() * (rigidBodyA.worldTransform * pivotInA);

        rbBFrame.Basis = new Matrix3x3(
            rbAxisB1.X, rbAxisB2.X, axisInB.X,
            rbAxisB1.Y, rbAxisB2.Y, axisInB.Y,
            rbAxisB1.Z, rbAxisB2.Z, axisInB.Z);
        rbBFrame.Basis = rigidBodyB.worldTransform.Basis.Inverse() * rbBFrame.Basis;
    }

    /// <summary>btHingeConstraint::setLimit（_BT_USE_CENTER_LIMIT_ 版本）</summary>
    public void SetLimit(btScalar low, btScalar high, btScalar softness = (btScalar)0.9,
        btScalar biasFactor = (btScalar)0.3, btScalar relaxationFactor = 1)
    {
        limit.Set(low, high, softness, biasFactor, relaxationFactor);
    }

    /// <summary>btHingeConstraint::getLimitSoftness</summary>
    public btScalar LimitSoftness => limit.Softness;

    /// <summary>btHingeConstraint::getLimitBiasFactor</summary>
    public btScalar LimitBiasFactor => limit.BiasFactor;

    /// <summary>btHingeConstraint::getLimitRelaxationFactor</summary>
    public btScalar LimitRelaxationFactor => limit.RelaxationFactor;

    /// <summary>btHingeConstraint::setAngularOnly</summary>
    public bool AngularOnly
    {
        get => angularOnly;
        set => angularOnly = value;
    }

    /// <summary>btHingeConstraint::enableAngularMotor</summary>
    public void EnableAngularMotor(bool enableMotor, btScalar targetVelocity, btScalar maxMotorImpulse)
    {
        enableAngularMotor = enableMotor;
        motorTargetVelocity = targetVelocity;
        this.maxMotorImpulse = maxMotorImpulse;
    }

    /// <summary>btHingeConstraint::enableMotor（额外马达 API）</summary>
    public void EnableMotor(bool enableMotor) => enableAngularMotor = enableMotor;

    public bool EnableAngularMotorState => enableAngularMotor;

    /// <summary>btHingeConstraint::setMaxMotorImpulse</summary>
    public btScalar MaxMotorImpulse
    {
        get => maxMotorImpulse;
        set => maxMotorImpulse = value;
    }

    /// <summary>btHingeConstraint::setMotorTargetVelocity（保留原 BulletSharp 的拼写）</summary>
    public btScalar MotorTargetVelosity
    {
        get => motorTargetVelocity;
        set => motorTargetVelocity = value;
    }

    /// <summary>btHingeConstraint::hasLimit</summary>
    public bool HasLimit() => limit.HalfRange > 0;

    /// <summary>btHingeConstraint::getLowerLimit</summary>
    public btScalar LowerLimit => limit.Low;

    /// <summary>btHingeConstraint::getUpperLimit</summary>
    public btScalar UpperLimit => limit.High;

    /// <summary>当前铰链角（范围 [-PI,PI]）</summary>
    public btScalar HingeAngle => GetHingeAngle();

    public Matrix AFrame => rbAFrame.ToMatrix();

    public Matrix BFrame => rbBFrame.ToMatrix();

    /// <summary>btHingeConstraint::getFrameOffsetA</summary>
    public Matrix FrameOffsetA
    {
        get => rbAFrame.ToMatrix();
        set => rbAFrame = Transform.FromMatrix(in value);
    }

    /// <summary>btHingeConstraint::getFrameOffsetB</summary>
    public Matrix FrameOffsetB
    {
        get => rbBFrame.ToMatrix();
        set => rbBFrame = Transform.FromMatrix(in value);
    }

    /// <summary>btHingeConstraint::getSolveLimit</summary>
    internal bool GetSolveLimit() => limit.IsLimit;

    /// <summary>btHingeConstraint::getLimitSign</summary>
    internal btScalar GetLimitSign() => limit.Sign;

    /// <summary>向 getInfo/getInfo2 过渡期间的向后兼容标记</summary>
    public bool UseSolveConstraintObsolete
    {
        get => useSolveConstraintObsolete;
        set => useSolveConstraintObsolete = value;
    }

    /// <summary>btHingeConstraint::getUseFrameOffset/setUseFrameOffset</summary>
    public bool UseOffsetForConstraintFrame
    {
        get => useOffsetForConstraintFrame;
        set => useOffsetForConstraintFrame = value;
    }

    /// <summary>UseOffsetForConstraintFrame 的别名</summary>
    public bool UseFrameOffset
    {
        get => useOffsetForConstraintFrame;
        set => useOffsetForConstraintFrame = value;
    }

    /// <summary>btHingeConstraint::getUseReferenceFrameA/setUseReferenceFrameA</summary>
    public bool UseReferenceFrameA
    {
        get => useReferenceFrameA;
        set => useReferenceFrameA = value;
    }

    /// <summary>btHingeConstraint::setParam（覆盖全局默认参数，可选指定轴，本约束仅支持 -1 或 5）</summary>
    public override void SetParam(int num, btScalar value, int axis = -1)
    {
        if (axis == -1 || axis == 5)
        {
            switch ((ConstraintParams)num)
            {
                case ConstraintParams.StopErp:
                    stopERP = value;
                    flags |= (int)HingeFlags.ErpStop;
                    break;
                case ConstraintParams.StopCfm:
                    stopCFM = value;
                    flags |= (int)HingeFlags.CfmStop;
                    break;
                case ConstraintParams.Cfm:
                    normalCFM = value;
                    flags |= (int)HingeFlags.CfmNorm;
                    break;
                case ConstraintParams.Erp:
                    normalERP = value;
                    flags |= (int)HingeFlags.ErpNorm;
                    break;
                default:
                    // btAssertConstrParams(0)
                    break;
            }
        }
        else
        {
            // btAssertConstrParams(0)
        }
    }

    /// <summary>btHingeConstraint::getParam（返回局部参数值）</summary>
    public override btScalar GetParam(int num, int axis)
    {
        btScalar retVal = 0;
        if (axis == -1 || axis == 5)
        {
            switch ((ConstraintParams)num)
            {
                case ConstraintParams.StopErp:
                    retVal = stopERP;
                    break;
                case ConstraintParams.StopCfm:
                    retVal = stopCFM;
                    break;
                case ConstraintParams.Cfm:
                    retVal = normalCFM;
                    break;
                case ConstraintParams.Erp:
                    retVal = normalERP;
                    break;
                default:
                    // btAssertConstrParams(0)
                    break;
            }
        }
        else
        {
            // btAssertConstrParams(0)
        }
        return retVal;
    }

    /// <summary>btHingeConstraint::getFlags</summary>
    public int Flags => flags;
}
