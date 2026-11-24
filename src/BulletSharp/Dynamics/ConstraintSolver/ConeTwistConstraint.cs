using BulletSharp.Math;

namespace BulletSharp;

/// <summary>btConeTwistFlags</summary>
[Flags]
public enum ConeTwistFlags
{
    None = 0,
    LinCfm = 1,
    LinErp = 2,
    AngCfm = 4,
}

/// <summary>
/// 移植自 btConeTwistConstraint：锥形-扭转约束（可模拟 ragdoll 关节，如上臂、腿）。
/// 平移固定，3 个旋转自由度分为摆动（锥内）与扭转；摆动可分为 swing1/swing2，
/// 限位可不同，呈椭圆锥形。约束参考系中：扭转沿 X 轴，swing1/swing2 分别沿 Z、Y 轴。
/// </summary>
public class ConeTwistConstraint : TypedConstraint
{
    private const btScalar ConeTwistDefFixThresh = (btScalar)0.05; // CONETWIST_DEF_FIX_THRESH
    private static readonly Vector3 VTWIST = new(1, 0, 0);         // 约束空间中的扭转轴

    internal JacobianEntry[] jac = new JacobianEntry[3]; // 3 个正交线性约束

    internal Transform rbAFrame;
    internal Transform rbBFrame;

    internal btScalar limitSoftness;
    internal btScalar biasFactor;
    internal btScalar relaxationFactor;

    internal btScalar damping;

    internal btScalar swingSpan1;
    internal btScalar swingSpan2;
    internal btScalar twistSpan;

    internal btScalar fixThresh;

    internal Vector3 swingAxis;
    internal Vector3 twistAxis;

    internal btScalar kSwing;
    internal btScalar kTwist;

    internal btScalar twistLimitSign;
    internal btScalar swingCorrection;
    internal btScalar twistCorrection;

    internal btScalar twistAngle;

    internal btScalar accSwingLimitImpulse;
    internal btScalar accTwistLimitImpulse;

    internal bool angularOnly;
    internal bool solveTwistLimit;
    internal bool solveSwingLimit;

    internal bool useSolveConstraintObsolete;

    // 尚未使用
    internal btScalar swingLimitRatio;
    internal btScalar twistLimitRatio;
    internal Vector3 twistAxisA;

    // 马达
    internal bool motorEnabled;
    internal bool normalizedMotorStrength;
    internal Quaternion qTarget;
    internal btScalar maxMotorImpulse;
    internal Vector3 accMotorImpulse;

    // 参数
    internal int flags;
    internal btScalar linCFM;
    internal btScalar linERP;
    internal btScalar angCFM;

    /// <summary>btConeTwistConstraint::btConeTwistConstraint(rbA, rbB, rbAFrame, rbBFrame)</summary>
    public ConeTwistConstraint(RigidBody rbA, RigidBody rbB, Matrix rbAFrame, Matrix rbBFrame)
        : base(TypedConstraintType.ConeTwist, rbA, rbB)
    {
        this.rbAFrame = Transform.FromMatrix(in rbAFrame);
        this.rbBFrame = Transform.FromMatrix(in rbBFrame);
        angularOnly = false;
        useSolveConstraintObsolete = false; // CONETWIST_USE_OBSOLETE_SOLVER
        Init();
    }

    /// <summary>btConeTwistConstraint::btConeTwistConstraint(rbA, rbAFrame)</summary>
    public ConeTwistConstraint(RigidBody rbA, Matrix rbAFrame)
        : base(TypedConstraintType.ConeTwist, rbA)
    {
        this.rbAFrame = Transform.FromMatrix(in rbAFrame);
        rbBFrame = this.rbAFrame;
        rbBFrame.Origin = Vector3.Zero;
        angularOnly = false;
        useSolveConstraintObsolete = false;
        Init();
    }

    /// <summary>btConeTwistConstraint::init</summary>
    protected void Init()
    {
        angularOnly = false;
        solveTwistLimit = false;
        solveSwingLimit = false;
        motorEnabled = false;
        maxMotorImpulse = -1;

        SetLimit(BtMath.BtLargeFloat, BtMath.BtLargeFloat, BtMath.BtLargeFloat);
        damping = (btScalar)0.01;
        fixThresh = ConeTwistDefFixThresh;
        flags = 0;
        linCFM = 0;
        linERP = (btScalar)0.7;
        angCFM = 0;
    }

    internal override void GetInfo1(ConstraintInfo1 info, ContactSolverInfo infoGlobal)
    {
        // btConeTwistConstraint::getInfo1
        if (useSolveConstraintObsolete)
        {
            info.numConstraintRows = 0;
            info.nub = 0;
        }
        else
        {
            info.numConstraintRows = 3;
            info.nub = 3;
            CalcAngleInfo2(in rigidBodyA.worldTransform, in rigidBodyB.worldTransform,
                in rigidBodyA.invInertiaTensorWorld, in rigidBodyB.invInertiaTensorWorld);
            if (solveSwingLimit)
            {
                info.numConstraintRows++;
                info.nub--;
                if (swingSpan1 < fixThresh && swingSpan2 < fixThresh)
                {
                    info.numConstraintRows++;
                    info.nub--;
                }
            }
            if (solveTwistLimit)
            {
                info.numConstraintRows++;
                info.nub--;
            }
        }
    }

    /// <summary>btConeTwistConstraint::getInfo1NonVirtual（SPU 上无法获取对象变换，总是保留 6 行）</summary>
    internal void GetInfo1NonVirtual(ConstraintInfo1 info)
    {
        info.numConstraintRows = 6;
        info.nub = 0;
    }

    internal override void GetInfo2(ConstraintInfo2 info, ContactSolverInfo infoGlobal,
        Transform[] body0Trans, Transform[] body1Trans, int body0Index, int body1Index)
    {
        // btConeTwistConstraint::getInfo2
        GetInfo2NonVirtual(info, rigidBodyA.worldTransform, rigidBodyB.worldTransform,
            rigidBodyA.invInertiaTensorWorld, rigidBodyB.invInertiaTensorWorld);
    }

    /// <summary>btConeTwistConstraint::getInfo2NonVirtual</summary>
    internal void GetInfo2NonVirtual(ConstraintInfo2 info, in Transform transA, in Transform transB,
        in Matrix3x3 invInertiaWorldA, in Matrix3x3 invInertiaWorldB)
    {
        CalcAngleInfo2(in transA, in transB, in invInertiaWorldA, in invInertiaWorldB);

        // 设置雅可比
        info.j1LinearAxis[0] = 1;
        info.j1LinearAxis[info.rowskip + 1] = 1;
        info.j1LinearAxis[2 * info.rowskip + 2] = 1;
        Vector3 a1 = transA.Basis * rbAFrame.Origin;
        {
            Vector3 a1Neg = -a1;
            ConstraintMath.WriteSkewSymmetric(info.j1AngularAxis, info.rowskip, in a1Neg);
        }
        info.j2LinearAxis[0] = -1;
        info.j2LinearAxis[info.rowskip + 1] = -1;
        info.j2LinearAxis[2 * info.rowskip + 2] = -1;
        Vector3 a2 = transB.Basis * rbBFrame.Origin;
        {
            ConstraintMath.WriteSkewSymmetric(info.j2AngularAxis, info.rowskip, in a2);
        }
        // 设置右端项
        btScalar linErp = (flags & (int)ConeTwistFlags.LinErp) != 0 ? linERP : info.erp;
        btScalar k = info.fps * linErp;
        int j;
        for (j = 0; j < 3; j++)
        {
            info.constraintError[j * info.rowskip] = k * (a2[j] + transB.Origin[j] - a1[j] - transA.Origin[j]);
            info.lowerLimit[j * info.rowskip] = -BtMath.SimdInfinity;
            info.upperLimit[j * info.rowskip] = BtMath.SimdInfinity;
            if ((flags & (int)ConeTwistFlags.LinCfm) != 0)
            {
                info.cfm[j * info.rowskip] = linCFM;
            }
        }
        int row = 3;
        int srow = row * info.rowskip;
        Vector3 ax1;
        // 角限位
        if (solveSwingLimit)
        {
            btScalar[] j1 = info.j1AngularAxis;
            btScalar[] j2 = info.j2AngularAxis;
            if (swingSpan1 < fixThresh && swingSpan2 < fixThresh)
            {
                Transform trA = transA * rbAFrame;
                Vector3 p = trA.Basis.Column(1);
                Vector3 q = trA.Basis.Column(2);
                int srow1 = srow + info.rowskip;
                j1[srow + 0] = p[0];
                j1[srow + 1] = p[1];
                j1[srow + 2] = p[2];
                j1[srow1 + 0] = q[0];
                j1[srow1 + 1] = q[1];
                j1[srow1 + 2] = q[2];
                j2[srow + 0] = -p[0];
                j2[srow + 1] = -p[1];
                j2[srow + 2] = -p[2];
                j2[srow1 + 0] = -q[0];
                j2[srow1 + 1] = -q[1];
                j2[srow1 + 2] = -q[2];
                btScalar fact = info.fps * relaxationFactor;
                info.constraintError[srow] = fact * swingAxis.Dot(p);
                info.constraintError[srow1] = fact * swingAxis.Dot(q);
                info.lowerLimit[srow] = -BtMath.SimdInfinity;
                info.upperLimit[srow] = BtMath.SimdInfinity;
                info.lowerLimit[srow1] = -BtMath.SimdInfinity;
                info.upperLimit[srow1] = BtMath.SimdInfinity;
                srow = srow1 + info.rowskip;
            }
            else
            {
                ax1 = swingAxis * relaxationFactor * relaxationFactor;
                j1[srow + 0] = ax1[0];
                j1[srow + 1] = ax1[1];
                j1[srow + 2] = ax1[2];
                j2[srow + 0] = -ax1[0];
                j2[srow + 1] = -ax1[1];
                j2[srow + 2] = -ax1[2];
                btScalar k2 = info.fps * biasFactor;

                info.constraintError[srow] = k2 * swingCorrection;
                if ((flags & (int)ConeTwistFlags.AngCfm) != 0)
                {
                    info.cfm[srow] = angCFM;
                }
                // swingCorrection 总是为正或 0
                info.lowerLimit[srow] = 0;
                info.upperLimit[srow] = motorEnabled && maxMotorImpulse >= 0 ? maxMotorImpulse : BtMath.SimdInfinity;
                srow += info.rowskip;
            }
        }
        if (solveTwistLimit)
        {
            ax1 = twistAxis * relaxationFactor * relaxationFactor;
            btScalar[] j1 = info.j1AngularAxis;
            btScalar[] j2 = info.j2AngularAxis;
            j1[srow + 0] = ax1[0];
            j1[srow + 1] = ax1[1];
            j1[srow + 2] = ax1[2];
            j2[srow + 0] = -ax1[0];
            j2[srow + 1] = -ax1[1];
            j2[srow + 2] = -ax1[2];
            btScalar k2 = info.fps * biasFactor;
            info.constraintError[srow] = k2 * twistCorrection;
            if ((flags & (int)ConeTwistFlags.AngCfm) != 0)
            {
                info.cfm[srow] = angCFM;
            }
            if (twistSpan > 0)
            {
                if (twistCorrection > 0)
                {
                    info.lowerLimit[srow] = 0;
                    info.upperLimit[srow] = BtMath.SimdInfinity;
                }
                else
                {
                    info.lowerLimit[srow] = -BtMath.SimdInfinity;
                    info.upperLimit[srow] = 0;
                }
            }
            else
            {
                info.lowerLimit[srow] = -BtMath.SimdInfinity;
                info.upperLimit[srow] = BtMath.SimdInfinity;
            }
            srow += info.rowskip;
        }
    }

    /// <summary>btConeTwistConstraint::buildJacobian（仅供 useSolveConstraintObsolete 旧路径）</summary>
    public override void BuildJacobian()
    {
        if (useSolveConstraintObsolete)
        {
            appliedImpulse = 0;
            accTwistLimitImpulse = 0;
            accSwingLimitImpulse = 0;
            accMotorImpulse = Vector3.Zero;

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

            CalcAngleInfo2(in rigidBodyA.worldTransform, in rigidBodyB.worldTransform,
                in rigidBodyA.invInertiaTensorWorld, in rigidBodyB.invInertiaTensorWorld);
        }
    }

    /// <summary>btConeTwistConstraint::updateRHS</summary>
    public void UpdateRHS(btScalar timeStep)
    {
    }

    /// <summary>computeAngularImpulseDenominator（btConeTwistConstraint.cpp 静态辅助）</summary>
    private static btScalar ComputeAngularImpulseDenominator(in Vector3 axis, in Matrix3x3 invInertiaWorld)
    {
        Vector3 vec = axis * invInertiaWorld;
        return axis.Dot(vec);
    }

    /// <summary>btConeTwistConstraint::calcAngleInfo（旧版本，仅供旧求解路径）</summary>
    public void CalcAngleInfo()
    {
        swingCorrection = 0;
        twistLimitSign = 0;
        solveTwistLimit = false;
        solveSwingLimit = false;

        Vector3 b1Axis1 = Vector3.Zero, b1Axis2 = Vector3.Zero, b1Axis3 = Vector3.Zero;

        b1Axis1 = rigidBodyA.worldTransform.Basis * rbAFrame.Basis.Column(0);
        Vector3 b2Axis1 = rigidBodyB.worldTransform.Basis * rbBFrame.Basis.Column(0);

        btScalar swing1 = 0, swing2 = 0;

        btScalar swx, swy;
        const btScalar thresh = 10;
        btScalar fact;

        // 把帧变换到世界空间
        if (swingSpan1 >= (btScalar)0.05)
        {
            b1Axis2 = rigidBodyA.worldTransform.Basis * rbAFrame.Basis.Column(1);
            swx = b2Axis1.Dot(b1Axis1);
            swy = b2Axis1.Dot(b1Axis2);
            swing1 = BtMath.Atan2Fast(swy, swx); // C++: btAtan2Fast
            fact = (swy * swy + swx * swx) * thresh * thresh;
            fact = fact / (fact + 1);
            swing1 *= fact;
        }

        if (swingSpan2 >= (btScalar)0.05)
        {
            b1Axis3 = rigidBodyA.worldTransform.Basis * rbAFrame.Basis.Column(2);
            swx = b2Axis1.Dot(b1Axis1);
            swy = b2Axis1.Dot(b1Axis3);
            swing2 = BtMath.Atan2Fast(swy, swx); // C++: btAtan2Fast
            fact = (swy * swy + swx * swx) * thresh * thresh;
            fact = fact / (fact + 1);
            swing2 *= fact;
        }

        btScalar rMaxAngle1Sq = 1 / (swingSpan1 * swingSpan1);
        btScalar rMaxAngle2Sq = 1 / (swingSpan2 * swingSpan2);
        btScalar ellipseAngle = BtMath.Abs(swing1 * swing1) * rMaxAngle1Sq + BtMath.Abs(swing2 * swing2) * rMaxAngle2Sq;

        if (ellipseAngle > 1)
        {
            swingCorrection = ellipseAngle - 1;
            solveSwingLimit = true;
            // 计算所需的轴与因子
            swingAxis = b2Axis1.Cross(b1Axis2 * b2Axis1.Dot(b1Axis2) + b1Axis3 * b2Axis1.Dot(b1Axis3));
            swingAxis.Normalize();
            btScalar swingAxisSign = b2Axis1.Dot(b1Axis1) >= 0 ? 1 : -1;
            swingAxis *= swingAxisSign;
        }

        // 扭转限位
        if (twistSpan >= 0)
        {
            Vector3 b2Axis2 = rigidBodyB.worldTransform.Basis * rbBFrame.Basis.Column(1);
            Quaternion rotationArc = Quaternion.ShortestArc(b2Axis1, b1Axis1);
            Vector3 twistRef = Quaternion.Rotate(rotationArc, b2Axis2);
            btScalar twist = BtMath.Atan2Fast(twistRef.Dot(b1Axis3), twistRef.Dot(b1Axis2)); // C++: btAtan2Fast
            twistAngle = twist;

            btScalar lockedFreeFactor = twistSpan > (btScalar)0.05 ? 1 : 0;
            if (twist <= -twistSpan * lockedFreeFactor)
            {
                twistCorrection = -(twist + twistSpan);
                solveTwistLimit = true;
                twistAxis = (b2Axis1 + b1Axis1) * (btScalar)0.5;
                twistAxis.Normalize();
                twistAxis *= -1;
            }
            else if (twist > twistSpan * lockedFreeFactor)
            {
                twistCorrection = twist - twistSpan;
                solveTwistLimit = true;
                twistAxis = (b2Axis1 + b1Axis1) * (btScalar)0.5;
                twistAxis.Normalize();
            }
        }
    }

    /// <summary>btConeTwistConstraint::calcAngleInfo2（Transform/Matrix3x3 为内部类型，故为 internal）</summary>
    internal void CalcAngleInfo2(in Transform transA, in Transform transB,
        in Matrix3x3 invInertiaWorldA, in Matrix3x3 invInertiaWorldB)
    {
        swingCorrection = 0;
        twistLimitSign = 0;
        solveTwistLimit = false;
        solveSwingLimit = false;
        // 计算 A 相对 B 的旋转（约束空间）
        if (motorEnabled && !useSolveConstraintObsolete)
        {
            // 假定 setMotorTarget() 已被调用且马达目标 m_qTarget 在约束限位内
            // TODO：拆分为纯摆动与纯扭转
            // 计算世界系下的期望变换
            var trPose = new Transform(qTarget, Vector3.Zero);
            Transform trA = transA * rbAFrame;
            Transform trB = transB * rbBFrame;
            Transform trDeltaAB = trB * trPose * trA.Inverse();
            Quaternion qDeltaAB = trDeltaAB.GetRotation();
            var swingAxisLocal = new Vector3(qDeltaAB.X, qDeltaAB.Y, qDeltaAB.Z);
            btScalar swingAxisLen2 = swingAxisLocal.LengthSquared();
            if (ConstraintMath.FuzzyZero(swingAxisLen2))
            {
                return;
            }
            swingAxis = swingAxisLocal;
            swingAxis.Normalize();
            swingCorrection = qDeltaAB.GetAngle();
            if (!ConstraintMath.FuzzyZero(swingCorrection))
            {
                solveSwingLimit = true;
            }
            return;
        }

        {
            // 计算 A 相对 B 的旋转（约束空间）
            Quaternion qA = transA.GetRotation() * rbAFrame.GetRotation();
            Quaternion qB = transB.GetRotation() * rbBFrame.GetRotation();
            Quaternion qAB = qB.Inverse() * qA;
            // 把旋转拆分为锥（摆动）与扭转（全部从 B 的视角进行）
            Vector3 vConeNoTwist = Quaternion.Rotate(qAB, VTWIST);
            vConeNoTwist.Normalize();
            Quaternion qABCone = Quaternion.ShortestArc(VTWIST, vConeNoTwist);
            qABCone.Normalize();
            Quaternion qABTwist = qABCone.Inverse() * qAB;
            qABTwist.Normalize();

            if (swingSpan1 >= fixThresh && swingSpan2 >= fixThresh)
            {
                ComputeConeLimitInfo(in qABCone, out btScalar swingAngle, out Vector3 swingAxisOut, out btScalar swingLimit);

                if (swingAngle > swingLimit * limitSoftness)
                {
                    solveSwingLimit = true;

                    // 计算限位比例：0->1，0 == 软限位起点，1 == 硬限位
                    swingLimitRatio = 1;
                    if (swingAngle < swingLimit && limitSoftness < 1 - BtMath.SimdEpsilon)
                    {
                        swingLimitRatio = (swingAngle - swingLimit * limitSoftness) /
                                          (swingLimit - swingLimit * limitSoftness);
                    }

                    // 摆动修正试图回到软限位
                    swingCorrection = swingAngle - swingLimit * limitSoftness;

                    // 摆动轴调整（基于椭圆法线）
                    AdjustSwingAxisToUseEllipseNormal(ref swingAxisOut);

                    // 计算所需的轴与因子
                    swingAxis = Quaternion.Rotate(qB, -swingAxisOut);

                    twistAxisA = Vector3.Zero;

                    kSwing = 1 /
                        (ComputeAngularImpulseDenominator(in swingAxis, in invInertiaWorldA) +
                         ComputeAngularImpulseDenominator(in swingAxis, in invInertiaWorldB));
                }
            }
            else
            {
                // 未设置限位，或至少一个摆动限位太小——此时等价于铰链或固定关节
                Vector3 ivA = transA.Basis * rbAFrame.Basis.Column(0);
                Vector3 jvA = transA.Basis * rbAFrame.Basis.Column(1);
                Vector3 kvA = transA.Basis * rbAFrame.Basis.Column(2);
                Vector3 ivB = transB.Basis * rbBFrame.Basis.Column(0);
                Vector3 target = Vector3.Zero;
                btScalar x = ivB.Dot(ivA);
                btScalar y = ivB.Dot(jvA);
                btScalar z = ivB.Dot(kvA);
                if (swingSpan1 < fixThresh && swingSpan2 < fixThresh)
                {
                    // 固定关节，需要多加一行约束
                    if (!ConstraintMath.FuzzyZero(y) || !ConstraintMath.FuzzyZero(z))
                    {
                        solveSwingLimit = true;
                        swingAxis = -ivB.Cross(ivA);
                    }
                }
                else
                {
                    if (swingSpan1 < fixThresh)
                    {
                        // 绕 Y 轴的铰链
                        if (!ConstraintMath.FuzzyZero(x) || !ConstraintMath.FuzzyZero(z))
                        {
                            solveSwingLimit = true;
                            if (swingSpan2 >= fixThresh)
                            {
                                y = 0;
                                btScalar span2 = BtMath.Atan2(z, x);
                                if (span2 > swingSpan2)
                                {
                                    x = BtMath.Cos(swingSpan2);
                                    z = BtMath.Sin(swingSpan2);
                                }
                                else if (span2 < -swingSpan2)
                                {
                                    x = BtMath.Cos(swingSpan2);
                                    z = -BtMath.Sin(swingSpan2);
                                }
                            }
                        }
                    }
                    else
                    {
                        // 绕 Z 轴的铰链
                        if (!ConstraintMath.FuzzyZero(x) || !ConstraintMath.FuzzyZero(y))
                        {
                            solveSwingLimit = true;
                            if (swingSpan1 >= fixThresh)
                            {
                                z = 0;
                                btScalar span1 = BtMath.Atan2(y, x);
                                if (span1 > swingSpan1)
                                {
                                    x = BtMath.Cos(swingSpan1);
                                    y = BtMath.Sin(swingSpan1);
                                }
                                else if (span1 < -swingSpan1)
                                {
                                    x = BtMath.Cos(swingSpan1);
                                    y = -BtMath.Sin(swingSpan1);
                                }
                            }
                        }
                    }
                    target[0] = x * ivA[0] + y * jvA[0] + z * kvA[0];
                    target[1] = x * ivA[1] + y * jvA[1] + z * kvA[1];
                    target[2] = x * ivA[2] + y * jvA[2] + z * kvA[2];
                    target.Normalize();
                    swingAxis = -ivB.Cross(target);
                    swingCorrection = swingAxis.Length();

                    if (!ConstraintMath.FuzzyZero(swingCorrection))
                        swingAxis.Normalize();
                }
            }

            if (twistSpan >= 0)
            {
                ComputeTwistLimitInfo(in qABTwist, out twistAngle, out Vector3 vTwistAxis);

                if (twistAngle > twistSpan * limitSoftness)
                {
                    solveTwistLimit = true;

                    twistLimitRatio = 1;
                    if (twistAngle < twistSpan && limitSoftness < 1 - BtMath.SimdEpsilon)
                    {
                        twistLimitRatio = (twistAngle - twistSpan * limitSoftness) /
                                          (twistSpan - twistSpan * limitSoftness);
                    }

                    // 扭转修正试图回到软限位
                    twistCorrection = twistAngle - twistSpan * limitSoftness;

                    twistAxis = Quaternion.Rotate(qB, -vTwistAxis);

                    kTwist = 1 /
                        (ComputeAngularImpulseDenominator(in twistAxis, in invInertiaWorldA) +
                         ComputeAngularImpulseDenominator(in twistAxis, in invInertiaWorldB));
                }

                if (solveSwingLimit)
                    twistAxisA = Quaternion.Rotate(qA, -vTwistAxis);
            }
            else
            {
                twistAngle = 0;
            }
        }
    }

    /// <summary>
    /// btConeTwistConstraint::computeConeLimitInfo：给定约束空间中的锥旋转（前提：已去除扭转），
    /// 计算对应的摆动角与摆动轴，以及该锥“姿态”对应的摆动限位角。
    /// </summary>
    protected void ComputeConeLimitInfo(in Quaternion qCone,
        out btScalar swingAngle, out Vector3 vSwingAxis, out btScalar swingLimit)
    {
        vSwingAxis = Vector3.Zero;
        swingLimit = 0;
        swingAngle = qCone.GetAngle();
        if (swingAngle > BtMath.SimdEpsilon)
        {
            vSwingAxis = new Vector3(qCone.X, qCone.Y, qCone.Z);
            vSwingAxis.Normalize();

            // 计算给定摆动的限位：
            // 给定摆动轴，寻找与限位锥椭圆的交点。
            // 先计算从中心指向椭圆表面的方向（即摆动轴旋转 90 度的 2D 垂直向量，
            // vSwingAxis 是锥旋转（z,y 分量），换元并旋转为 (x,y) 坐标）
            btScalar xEllipse = vSwingAxis.Y;
            btScalar yEllipse = -vSwingAxis.Z;

            // 用向量斜率求与椭圆相交线段的长度：
            //  x^2   y^2
            //  --- + --- = 1，a、b 分别为半轴 2 与 1（即两个限位）
            //  a^2   b^2
            swingLimit = swingSpan1; // 若 xEllipse == 0，为纯 vSwingAxis.Z 旋转，直接用 swingspan1
            if (BtMath.Abs(xEllipse) > BtMath.SimdEpsilon)
            {
                btScalar surfaceSlope2 = yEllipse * yEllipse / (xEllipse * xEllipse);
                btScalar norm = 1 / (swingSpan2 * swingSpan2);
                norm += surfaceSlope2 / (swingSpan1 * swingSpan1);
                btScalar swingLimit2 = (1 + surfaceSlope2) / norm;
                swingLimit = BtMath.Sqrt(swingLimit2);
            }
        }
        else if (swingAngle < 0)
        {
            // 不应发生
        }
    }

    /// <summary>btConeTwistConstraint::GetPointForAngle（椭圆锥上取点，用于调试绘制等）</summary>
    public Vector3 GetPointForAngle(btScalar fAngleInRadians, btScalar fLength)
    {
        // 用锥角（0 -> 2*PI 沿锥面）计算椭圆上的 x/y
        btScalar xEllipse = BtMath.Cos(fAngleInRadians);
        btScalar yEllipse = BtMath.Sin(fAngleInRadians);

        // 用向量斜率求与椭圆相交线段的长度
        btScalar swingLimit = swingSpan1; // 若 xEllipse == 0，直接用轴 b（1）
        if (BtMath.Abs(xEllipse) > BtMath.SimdEpsilon)
        {
            btScalar surfaceSlope2 = yEllipse * yEllipse / (xEllipse * xEllipse);
            btScalar norm = 1 / (swingSpan2 * swingSpan2);
            norm += surfaceSlope2 / (swingSpan1 * swingSpan1);
            btScalar swingLimit2 = (1 + surfaceSlope2) / norm;
            swingLimit = BtMath.Sqrt(swingLimit2);
        }

        // 转换到约束空间中的点：
        // 注意：扭转是 X 轴，swing 1、2 分别沿 Z、Y 轴
        var vSwingAxis = new Vector3(0, xEllipse, -yEllipse);
        var qSwing = new Quaternion(vSwingAxis, swingLimit);
        var vPointInConstraintSpace = new Vector3(fLength, 0, 0);
        return Quaternion.Rotate(qSwing, vPointInConstraintSpace);
    }

    /// <summary>
    /// btConeTwistConstraint::computeTwistLimitInfo：给定约束空间中的扭转旋转（前提：已去除锥分量），
    /// 计算其角度与轴。
    /// </summary>
    protected void ComputeTwistLimitInfo(in Quaternion qTwist,
        out btScalar twistAngleOut, out Vector3 vTwistAxis)
    {
        Quaternion qMinTwist = qTwist;
        twistAngleOut = qTwist.GetAngle();

        if (twistAngleOut > BtMath.Pi)
        {
            // 绕远路，翻转四元数后重算
            qMinTwist = -qTwist;
            twistAngleOut = qMinTwist.GetAngle();
        }

        vTwistAxis = new Vector3(qMinTwist.X, qMinTwist.Y, qMinTwist.Z);
        if (twistAngleOut > BtMath.SimdEpsilon)
            vTwistAxis.Normalize();
    }

    /// <summary>btConeTwistConstraint::adjustSwingAxisToUseEllipseNormal</summary>
    protected void AdjustSwingAxisToUseEllipseNormal(ref Vector3 vSwingAxis)
    {
        // 摆动轴作为“无扭转”的锥旋转计算，但锥限位是椭圆而非圆形（swingspan1 != swingspan2 时）。
        // 超出限位时，回到锥内最近的方向并非指向中心的向量；使用椭圆法线更稳定。

        // 把摆动轴转换为从中心指向椭圆表面的方向（即 2D 旋转 90 度）
        btScalar y = -vSwingAxis.Z;
        btScalar z = vSwingAxis.Y;

        if (BtMath.Abs(z) > BtMath.SimdEpsilon) // 避免除 0；z == 0 时无需更新
        {
            // 计算当前“点”处椭圆表面的梯度/法线
            btScalar grad = y / z;
            grad *= swingSpan2 / swingSpan1;

            // 调整 y/z 表示该点法线（而非指向该点的向量）
            if (y > 0)
                y = BtMath.Abs(grad * z);
            else
                y = -BtMath.Abs(grad * z);

            // 把椭圆方向转换回摆动轴
            vSwingAxis.Z = -y;
            vSwingAxis.Y = z;
            vSwingAxis.Normalize();
        }
    }

    /// <summary>btConeTwistConstraint::setMotorTarget（q 为 bodyA 相对 bodyB 的期望旋转；越限的目标会被钳制）</summary>
    public void SetMotorTarget(Quaternion q)
    {
        Quaternion qConstraint = rbBFrame.GetRotation().Inverse() * q * rbAFrame.GetRotation();
        SetMotorTargetInConstraintSpace(qConstraint);
    }

    /// <summary>btConeTwistConstraint::getMotorTarget</summary>
    public Quaternion MotorTarget => qTarget;

    /// <summary>btConeTwistConstraint::setMotorTargetInConstraintSpace（q 为约束空间中 frameA 相对 frameB 的期望旋转）</summary>
    public void SetMotorTargetInConstraintSpace(Quaternion q)
    {
        qTarget = q;

        // 把马达目标钳制到限位内
        const btScalar softness = 1; //m_limitSoftness;

        // 拆分为扭转与锥
        Vector3 vTwisted = Quaternion.Rotate(qTarget, VTWIST);
        Quaternion qTargetCone = Quaternion.ShortestArc(VTWIST, vTwisted);
        qTargetCone.Normalize();
        Quaternion qTargetTwist = qTargetCone.Inverse() * qTarget;
        qTargetTwist.Normalize();

        // 钳制锥
        if (swingSpan1 >= (btScalar)0.05 && swingSpan2 >= (btScalar)0.05)
        {
            ComputeConeLimitInfo(in qTargetCone, out btScalar swingAngle, out Vector3 swingAxisOut, out btScalar swingLimit);

            if (BtMath.Abs(swingAngle) > BtMath.SimdEpsilon)
            {
                if (swingAngle > swingLimit * softness)
                    swingAngle = swingLimit * softness;
                else if (swingAngle < -swingLimit * softness)
                    swingAngle = -swingLimit * softness;
                qTargetCone = new Quaternion(swingAxisOut, swingAngle);
            }
        }

        // 钳制扭转
        if (twistSpan >= (btScalar)0.05)
        {
            ComputeTwistLimitInfo(in qTargetTwist, out btScalar twistAngleOut, out Vector3 twistAxisOut);

            if (BtMath.Abs(twistAngleOut) > BtMath.SimdEpsilon)
            {
                if (twistAngleOut > twistSpan * softness)
                    twistAngleOut = twistSpan * softness;
                else if (twistAngleOut < -twistSpan * softness)
                    twistAngleOut = -twistSpan * softness;
                qTargetTwist = new Quaternion(twistAxisOut, twistAngleOut);
            }
        }

        qTarget = qTargetCone * qTargetTwist;
    }

    /// <summary>btConeTwistConstraint::setAngularOnly</summary>
    public bool AngularOnly
    {
        get => angularOnly;
        set => angularOnly = value;
    }

    /// <summary>btConeTwistConstraint::setLimit(limitIndex, limitValue)（3=扭转, 4=swing2, 5=swing1）</summary>
    public void SetLimit(int limitIndex, btScalar limitValue)
    {
        switch (limitIndex)
        {
            case 3:
                twistSpan = limitValue;
                break;
            case 4:
                swingSpan2 = limitValue;
                break;
            case 5:
                swingSpan1 = limitValue;
                break;
            default:
                break;
        }
    }

    /// <summary>btConeTwistConstraint::getLimit(limitIndex)（3=扭转, 4=swing2, 5=swing1）</summary>
    public btScalar GetLimit(int limitIndex)
    {
        switch (limitIndex)
        {
            case 3: return twistSpan;
            case 4: return swingSpan2;
            case 5: return swingSpan1;
            default: return 0;
        }
    }

    /// <summary>
    /// btConeTwistConstraint::setLimit(swingSpan1, swingSpan2, twistSpan, softness, biasFactor, relaxationFactor)。
    /// softness：0->1 建议 ~0.8->1，表示限位内自由移动的百分比；
    /// biasFactor：0->1 建议 0.3 左右，抵抗零阶（角度）限位违反的强度；
    /// relaxationFactor：0->1 建议接近 1，值越小约束越少对抗违反角限的速度。
    /// </summary>
    public void SetLimit(btScalar swingSpan1, btScalar swingSpan2, btScalar twistSpan,
        btScalar softness = 1, btScalar biasFactor = (btScalar)0.3, btScalar relaxationFactor = 1)
    {
        this.swingSpan1 = swingSpan1;
        this.swingSpan2 = swingSpan2;
        this.twistSpan = twistSpan;

        limitSoftness = softness;
        this.biasFactor = biasFactor;
        this.relaxationFactor = relaxationFactor;
    }

    public Matrix AFrame => rbAFrame.ToMatrix();

    public Matrix BFrame => rbBFrame.ToMatrix();

    /// <summary>btConeTwistConstraint::getSolveTwistLimit</summary>
    public bool SolveTwistLimit => solveTwistLimit;

    /// <summary>btConeTwistConstraint::getSolveSwingLimit</summary>
    public bool SolveSwingLimit => solveSwingLimit;

    /// <summary>btConeTwistConstraint::getTwistLimitSign</summary>
    public btScalar TwistLimitSign => twistLimitSign;

    /// <summary>btConeTwistConstraint::getSwingSpan1</summary>
    public btScalar SwingSpan1 => swingSpan1;

    /// <summary>btConeTwistConstraint::getSwingSpan2</summary>
    public btScalar SwingSpan2 => swingSpan2;

    /// <summary>btConeTwistConstraint::getTwistSpan</summary>
    public btScalar TwistSpan => twistSpan;

    /// <summary>btConeTwistConstraint::getLimitSoftness</summary>
    public btScalar LimitSoftness => limitSoftness;

    /// <summary>btConeTwistConstraint::getBiasFactor</summary>
    public btScalar BiasFactor => biasFactor;

    /// <summary>btConeTwistConstraint::getRelaxationFactor</summary>
    public btScalar RelaxationFactor => relaxationFactor;

    /// <summary>btConeTwistConstraint::getTwistAngle</summary>
    public btScalar TwistAngle => twistAngle;

    /// <summary>btConeTwistConstraint::isPastSwingLimit</summary>
    public bool IsPastSwingLimit() => solveSwingLimit;

    /// <summary>btConeTwistConstraint::getDamping/setDamping</summary>
    public btScalar Damping
    {
        get => damping;
        set => damping = value;
    }

    /// <summary>btConeTwistConstraint::enableMotor/isMotorEnabled</summary>
    public void EnableMotor(bool b) => motorEnabled = b;

    /// <summary>btConeTwistConstraint::isMotorEnabled</summary>
    public bool IsMotorEnabled() => motorEnabled;

    /// <summary>btConeTwistConstraint::getMaxMotorImpulse</summary>
    public btScalar MaxMotorImpulse => maxMotorImpulse;

    /// <summary>btConeTwistConstraint::isMaxMotorImpulseNormalized</summary>
    public bool IsMaxMotorImpulseNormalized() => normalizedMotorStrength;

    /// <summary>btConeTwistConstraint::setMaxMotorImpulse</summary>
    public void SetMaxMotorImpulse(btScalar maxMotorImpulse)
    {
        this.maxMotorImpulse = maxMotorImpulse;
        normalizedMotorStrength = false;
    }

    /// <summary>btConeTwistConstraint::setMaxMotorImpulseNormalized</summary>
    public void SetMaxMotorImpulseNormalized(btScalar maxMotorImpulse)
    {
        this.maxMotorImpulse = maxMotorImpulse;
        normalizedMotorStrength = true;
    }

    /// <summary>btConeTwistConstraint::getFixThresh/setFixThresh</summary>
    public btScalar FixThresh
    {
        get => fixThresh;
        set => fixThresh = value;
    }

    /// <summary>btConeTwistConstraint::getFrameOffsetA</summary>
    public Matrix FrameOffsetA
    {
        get => rbAFrame.ToMatrix();
        set => rbAFrame = Transform.FromMatrix(in value);
    }

    /// <summary>btConeTwistConstraint::getFrameOffsetB</summary>
    public Matrix FrameOffsetB
    {
        get => rbBFrame.ToMatrix();
        set => rbBFrame = Transform.FromMatrix(in value);
    }

    /// <summary>btConeTwistConstraint::setFrames</summary>
    public virtual void SetFrames(Matrix frameA, Matrix frameB)
    {
        rbAFrame = Transform.FromMatrix(in frameA);
        rbBFrame = Transform.FromMatrix(in frameB);
        BuildJacobian();
    }

    /// <summary>btConeTwistConstraint::setParam（覆盖全局默认参数，axis 0..2 为线性，3..5 为角）</summary>
    public override void SetParam(int num, btScalar value, int axis = -1)
    {
        switch ((ConstraintParams)num)
        {
            case ConstraintParams.Erp:
            case ConstraintParams.StopErp:
                if (axis >= 0 && axis < 3)
                {
                    linERP = value;
                    flags |= (int)ConeTwistFlags.LinErp;
                }
                else
                {
                    biasFactor = value;
                }
                break;
            case ConstraintParams.Cfm:
            case ConstraintParams.StopCfm:
                if (axis >= 0 && axis < 3)
                {
                    linCFM = value;
                    flags |= (int)ConeTwistFlags.LinCfm;
                }
                else
                {
                    angCFM = value;
                    flags |= (int)ConeTwistFlags.AngCfm;
                }
                break;
            default:
                // btAssertConstrParams(0)
                break;
        }
    }

    /// <summary>btConeTwistConstraint::getParam（返回局部参数值）</summary>
    public override btScalar GetParam(int num, int axis)
    {
        btScalar retVal = 0;
        switch ((ConstraintParams)num)
        {
            case ConstraintParams.Erp:
            case ConstraintParams.StopErp:
                if (axis >= 0 && axis < 3)
                {
                    retVal = linERP;
                }
                else if (axis >= 3 && axis < 6)
                {
                    retVal = biasFactor;
                }
                else
                {
                    // btAssertConstrParams(0)
                }
                break;
            case ConstraintParams.Cfm:
            case ConstraintParams.StopCfm:
                if (axis >= 0 && axis < 3)
                {
                    retVal = linCFM;
                }
                else if (axis >= 3 && axis < 6)
                {
                    retVal = angCFM;
                }
                else
                {
                    // btAssertConstrParams(0)
                }
                break;
            default:
                // btAssertConstrParams(0)
                break;
        }
        return retVal;
    }

    /// <summary>btConeTwistConstraint::getFlags</summary>
    public int Flags => flags;
}
