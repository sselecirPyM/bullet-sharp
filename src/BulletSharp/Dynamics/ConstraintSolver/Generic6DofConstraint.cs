using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// 移植自 btRotationalLimitMotor：通用关节的单轴旋转 limit/motor 参数。
/// </summary>
public class RotationalLimitMotor
{
    internal btScalar loLimit = 1;          //!< 关节限位
    internal btScalar hiLimit = -1;         //!< 关节限位
    internal btScalar targetVelocity;       //!< 马达目标速度
    internal btScalar maxMotorForce = 6;    //!< 马达最大力
    internal btScalar maxLimitForce = 300;  //!< 限位最大力
    internal btScalar damping = 1;          //!< 阻尼
    internal btScalar limitSoftness = (btScalar)0.5; //!< 松弛因子
    internal btScalar normalCFM;            //!< 约束力混合因子
    internal btScalar stopERP = (btScalar)0.2;       //!< 到达限位时的误差容差因子
    internal btScalar stopCFM;              //!< 到达限位时的约束力混合因子
    internal btScalar bounce;               //!< 恢复系数
    internal bool enableMotor;

    // 临时变量
    internal btScalar currentLimitError;    //! 当前限位违反量
    internal btScalar currentPosition;      //! 当前角度值
    internal int currentLimit;              //!< 0=自由, 1=低位限位, 2=高位限位
    internal btScalar accumulatedImpulse;

    /// <summary>btRotationalLimitMotor::isLimited</summary>
    public bool IsLimited() => loLimit <= hiLimit;

    /// <summary>btRotationalLimitMotor::needApplyTorques</summary>
    public bool NeedApplyTorques() => currentLimit != 0 || enableMotor;

    /// <summary>btRotationalLimitMotor::testLimitValue（计算 currentLimit 与 currentLimitError）</summary>
    public int TestLimitValue(btScalar testValue)
    {
        if (loLimit > hiLimit)
        {
            currentLimit = 0; // 无违反
            return 0;
        }
        if (testValue < loLimit)
        {
            currentLimit = 1; // 低位限位违反
            currentLimitError = testValue - loLimit;
            if (currentLimitError > BtMath.Pi)
                currentLimitError -= BtMath.Simd2Pi;
            else if (currentLimitError < -BtMath.Pi)
                currentLimitError += BtMath.Simd2Pi;
            return 1;
        }
        else if (testValue > hiLimit)
        {
            currentLimit = 2; // 高位限位违反
            currentLimitError = testValue - hiLimit;
            if (currentLimitError > BtMath.Pi)
                currentLimitError -= BtMath.Simd2Pi;
            else if (currentLimitError < -BtMath.Pi)
                currentLimitError += BtMath.Simd2Pi;
            return 2;
        }

        currentLimit = 0; // 无违反
        return 0;
    }

    /// <summary>btRotationalLimitMotor::solveAngularLimits（旧式求解路径）</summary>
    internal btScalar SolveAngularLimits(btScalar timeStep, ref Vector3 axis, btScalar jacDiagABInv,
        RigidBody body0, RigidBody body1)
    {
        if (!NeedApplyTorques()) return 0;

        btScalar targetVel = targetVelocity;
        btScalar maxMotorFrc = maxMotorForce;

        // 当前误差修正
        if (currentLimit != 0)
        {
            targetVel = -stopERP * currentLimitError / timeStep;
            maxMotorFrc = maxLimitForce;
        }

        maxMotorFrc *= timeStep;

        // 当前速度差
        Vector3 angVelA = body0.angularVelocity;
        Vector3 angVelB = body1.angularVelocity;

        Vector3 velDiff = angVelA - angVelB;

        btScalar relVel = axis.Dot(velDiff);

        // 修正速度
        btScalar motorRelVel = limitSoftness * (targetVel - damping * relVel);

        if (motorRelVel < BtMath.SimdEpsilon && motorRelVel > -BtMath.SimdEpsilon)
        {
            return 0; // 无需施加力
        }

        // 修正冲量
        btScalar unclippedMotorImpulse = (1 + bounce) * motorRelVel * jacDiagABInv;

        // 裁剪修正冲量
        btScalar clippedMotorImpulse;
        if (unclippedMotorImpulse > 0)
        {
            clippedMotorImpulse = unclippedMotorImpulse > maxMotorFrc ? maxMotorFrc : unclippedMotorImpulse;
        }
        else
        {
            clippedMotorImpulse = unclippedMotorImpulse < -maxMotorFrc ? -maxMotorFrc : unclippedMotorImpulse;
        }

        // 与累计冲量合并
        btScalar lo = -BtMath.BtLargeFloat;
        btScalar hi = BtMath.BtLargeFloat;

        btScalar oldAccumImpulse = accumulatedImpulse;
        btScalar sum = oldAccumImpulse + clippedMotorImpulse;
        accumulatedImpulse = sum > hi ? 0 : sum < lo ? 0 : sum;

        clippedMotorImpulse = accumulatedImpulse - oldAccumImpulse;

        Vector3 motorImp = clippedMotorImpulse * axis;

        body0.ApplyTorqueImpulse(ref motorImp);
        Vector3 negMotorImp = -motorImp;
        body1.ApplyTorqueImpulse(ref negMotorImp);

        return clippedMotorImpulse;
    }

    public btScalar LoLimit { get => loLimit; set => loLimit = value; }
    public btScalar HiLimit { get => hiLimit; set => hiLimit = value; }
    public btScalar TargetVelocity { get => targetVelocity; set => targetVelocity = value; }
    public btScalar MaxMotorForce { get => maxMotorForce; set => maxMotorForce = value; }
    public btScalar MaxLimitForce { get => maxLimitForce; set => maxLimitForce = value; }
    public btScalar Damping { get => damping; set => damping = value; }
    public btScalar LimitSoftness { get => limitSoftness; set => limitSoftness = value; }
    public btScalar NormalCFM { get => normalCFM; set => normalCFM = value; }
    public btScalar StopERP { get => stopERP; set => stopERP = value; }
    public btScalar StopCFM { get => stopCFM; set => stopCFM = value; }
    public btScalar Bounce { get => bounce; set => bounce = value; }
    public bool EnableMotor { get => enableMotor; set => enableMotor = value; }
    public btScalar CurrentLimitError { get => currentLimitError; set => currentLimitError = value; }
    public btScalar CurrentPosition { get => currentPosition; set => currentPosition = value; }
    public int CurrentLimit { get => currentLimit; set => currentLimit = value; }
    public btScalar AccumulatedImpulse { get => accumulatedImpulse; set => accumulatedImpulse = value; }
}

/// <summary>
/// 移植自 btTranslationalLimitMotor：通用关节的平移 limit/motor 参数（按轴分量）。
/// </summary>
public class TranslationalLimitMotor
{
    internal Vector3 lowerLimit;         //!< 约束下限
    internal Vector3 upperLimit;         //!< 约束上限
    internal Vector3 accumulatedImpulse;
    //! Linear_Limit_parameters
    internal btScalar limitSoftness = (btScalar)0.7; //!< 线性限位 soft 系数
    internal btScalar damping = 1;                   //!< 线性限位阻尼
    internal btScalar restitution = (btScalar)0.5;   //! 线性限位反弹参数
    internal Vector3 normalCFM;                      //!< 约束力混合因子
    internal Vector3 stopERP = new((btScalar)0.2, (btScalar)0.2, (btScalar)0.2); //!< 到达限位时的误差容差因子
    internal Vector3 stopCFM;                        //!< 到达限位时的约束力混合因子
    internal bool[] enableMotor = new bool[3];
    internal Vector3 targetVelocity;     //!< 马达目标速度
    internal Vector3 maxMotorForce;      //!< 马达最大力
    internal Vector3 currentLimitError;  //! 当前限位违反量
    internal Vector3 currentLinearDiff;  //! 约束帧当前相对偏移
    internal int[] currentLimit = new int[3]; //!< 0=自由, 1=下限, 2=上限

    /// <summary>btTranslationalLimitMotor::isLimited（upper &lt; lower 为自由，upper == lower 为锁定，upper &gt; lower 为限位）</summary>
    public bool IsLimited(int limitIndex) => upperLimit[limitIndex] >= lowerLimit[limitIndex];

    /// <summary>btTranslationalLimitMotor::needApplyForce</summary>
    public bool NeedApplyForceLinear(int limitIndex) => currentLimit[limitIndex] != 0 || enableMotor[limitIndex];

    /// <summary>btTranslationalLimitMotor::testLimitValue</summary>
    public int TestLimitValue(int limitIndex, btScalar testValue)
    {
        btScalar loLimit = lowerLimit[limitIndex];
        btScalar hiLimit = upperLimit[limitIndex];
        if (loLimit > hiLimit)
        {
            currentLimit[limitIndex] = 0; // 无违反
            currentLimitError[limitIndex] = 0;
            return 0;
        }

        if (testValue < loLimit)
        {
            currentLimit[limitIndex] = 2; // 低位限位违反
            currentLimitError[limitIndex] = testValue - loLimit;
            return 2;
        }
        else if (testValue > hiLimit)
        {
            currentLimit[limitIndex] = 1; // 高位限位违反
            currentLimitError[limitIndex] = testValue - hiLimit;
            return 1;
        }

        currentLimit[limitIndex] = 0; // 无违反
        currentLimitError[limitIndex] = 0;
        return 0;
    }

    /// <summary>btTranslationalLimitMotor::solveLinearAxis（旧式求解路径）</summary>
    internal btScalar SolveLinearAxis(
        btScalar timeStep,
        btScalar jacDiagABInv,
        RigidBody body1, in Vector3 pointInA,
        RigidBody body2, in Vector3 pointInB,
        int limitIndex,
        in Vector3 axisNormalOnA,
        in Vector3 anchorPos)
    {
        // 求相对速度
        Vector3 relPos1 = anchorPos - body1.CenterOfMassPosition;
        Vector3 relPos2 = anchorPos - body2.CenterOfMassPosition;

        Vector3 vel1 = body1.GetVelocityInLocalPoint(ref relPos1);
        Vector3 vel2 = body2.GetVelocityInLocalPoint(ref relPos2);
        Vector3 vel = vel1 - vel2;

        btScalar relVel = axisNormalOnA.Dot(vel);

        // 应用位移修正

        // 位置误差（零阶误差）
        Vector3 pointDiff = pointInA - pointInB;
        btScalar depth = -pointDiff.Dot(axisNormalOnA);
        btScalar lo = -BtMath.BtLargeFloat;
        btScalar hi = BtMath.BtLargeFloat;

        btScalar minLimit = lowerLimit[limitIndex];
        btScalar maxLimit = upperLimit[limitIndex];

        // 处理限位
        if (minLimit < maxLimit)
        {
            if (depth > maxLimit)
            {
                depth -= maxLimit;
                lo = 0;
            }
            else
            {
                if (depth < minLimit)
                {
                    depth -= minLimit;
                    hi = 0;
                }
                else
                {
                    return 0;
                }
            }
        }

        btScalar normalImpulse = limitSoftness * (restitution * depth / timeStep - damping * relVel) * jacDiagABInv;

        btScalar oldNormalImpulse = accumulatedImpulse[limitIndex];
        btScalar sum = oldNormalImpulse + normalImpulse;
        accumulatedImpulse[limitIndex] = sum > hi ? 0 : sum < lo ? 0 : sum;
        normalImpulse = accumulatedImpulse[limitIndex] - oldNormalImpulse;

        Vector3 impulseVector = axisNormalOnA * normalImpulse;
        body1.ApplyImpulse(ref impulseVector, ref relPos1);
        Vector3 negImpulseVector = -impulseVector;
        body2.ApplyImpulse(ref negImpulseVector, ref relPos2);

        return normalImpulse;
    }

    public Vector3 LowerLimit { get => lowerLimit; set => lowerLimit = value; }
    public Vector3 UpperLimit { get => upperLimit; set => upperLimit = value; }
    public Vector3 AccumulatedImpulse { get => accumulatedImpulse; set => accumulatedImpulse = value; }
    public btScalar LimitSoftness { get => limitSoftness; set => limitSoftness = value; }
    public btScalar Damping { get => damping; set => damping = value; }
    public btScalar Restitution { get => restitution; set => restitution = value; }
    public Vector3 NormalCFM { get => normalCFM; set => normalCFM = value; }
    public Vector3 StopERP { get => stopERP; set => stopERP = value; }
    public Vector3 StopCFM { get => stopCFM; set => stopCFM = value; }
    public Vector3 TargetVelocity { get => targetVelocity; set => targetVelocity = value; }
    public Vector3 MaxMotorForce { get => maxMotorForce; set => maxMotorForce = value; }
    public Vector3 CurrentLimitError { get => currentLimitError; set => currentLimitError = value; }
    public Vector3 CurrentLinearDiff { get => currentLinearDiff; set => currentLinearDiff = value; }

    /// <summary>enableMotor[index] 访问</summary>
    public bool GetEnableMotor(int index) => enableMotor[index];
    /// <summary>enableMotor[index] 访问</summary>
    public void SetEnableMotor(int index, bool onOff) => enableMotor[index] = onOff;
    /// <summary>currentLimit[index] 访问（0=自由, 1=下限, 2=上限）</summary>
    public int GetCurrentLimit(int index) => currentLimit[index];
}

/// <summary>bt6DofFlags（每轴占 BT_6DOF_FLAGS_AXIS_SHIFT=3 位）</summary>
[Flags]
public enum SixDofFlags
{
    None = 0,
    CfmNorm = 1,
    CfmStop = 2,
    ErpStop = 4,
}

/// <summary>
/// 移植自 btGeneric6DofConstraint：通用 6 自由度约束，
/// 任意 6 个自由度可设为自由/限位/锁定，支持旋转马达。
/// </summary>
public class Generic6DofConstraint : TypedConstraint
{
    /// <summary>BT_6DOF_FLAGS_AXIS_SHIFT（每轴标志位数）</summary>
    protected const int AxisShift = 3;

    // 相对帧
    internal Transform frameInA; //!< 相对刚体 A 的约束空间
    internal Transform frameInB; //!< 相对刚体 B 的约束空间

    // 雅可比
    internal JacobianEntry[] jacLinear = new JacobianEntry[3]; //!< 3 个正交线性约束
    internal JacobianEntry[] jacAng = new JacobianEntry[3];    //!< 3 个正交角约束

    // 线性限位参数
    internal readonly TranslationalLimitMotor linearLimits = new();

    // 角（铰链）限位参数
    internal readonly RotationalLimitMotor[] angularLimits =
        { new RotationalLimitMotor(), new RotationalLimitMotor(), new RotationalLimitMotor() };

    internal Transform calculatedTransformA;
    internal Transform calculatedTransformB;
    internal Vector3 calculatedAxisAngleDiff;
    internal Vector3[] calculatedAxis = new Vector3[3];
    internal Vector3 calculatedLinearDiff;
    internal btScalar factA;
    internal btScalar factB;
    internal bool hasStaticBody;

    internal Vector3 anchorPos; // 求解线性轴时刚体 A、B 枢轴之间的点

    internal bool useLinearReferenceFrameA;
    internal bool useOffsetForConstraintFrame;

    internal int flags;

    internal bool useSolveConstraintObsolete;

    /// <summary>btGeneric6DofConstraint::btGeneric6DofConstraint(rbA, rbB, frameInA, frameInB, useLinearReferenceFrameA)</summary>
    public Generic6DofConstraint(RigidBody rbA, RigidBody rbB, Matrix frameInA, Matrix frameInB, bool useLinearReferenceFrameA)
        : this(TypedConstraintType.D6, rbA, rbB, in frameInA, in frameInB, useLinearReferenceFrameA)
    {
    }

    /// <summary>供 Generic6DofSpringConstraint 指定约束类型的构造路径</summary>
    protected Generic6DofConstraint(TypedConstraintType type, RigidBody rbA, RigidBody rbB,
        in Matrix frameInA, in Matrix frameInB, bool useLinearReferenceFrameA)
        : base(type, rbA, rbB)
    {
        // D6_USE_FRAME_OFFSET = true / D6_USE_OBSOLETE_METHOD = false
        this.frameInA = Transform.FromMatrix(in frameInA);
        this.frameInB = Transform.FromMatrix(in frameInB);
        this.useLinearReferenceFrameA = useLinearReferenceFrameA;
        useOffsetForConstraintFrame = true;
        useSolveConstraintObsolete = false;
        CalculateTransforms();
    }

    /// <summary>btGeneric6DofConstraint::btGeneric6DofConstraint(rbB, frameInB, useLinearReferenceFrameB)</summary>
    public Generic6DofConstraint(RigidBody rbB, Matrix frameInB, bool useLinearReferenceFrameB)
        : this(TypedConstraintType.D6, rbB, in frameInB, useLinearReferenceFrameB)
    {
    }

    /// <summary>供 Generic6DofSpringConstraint 指定约束类型的构造路径</summary>
    protected Generic6DofConstraint(TypedConstraintType type, RigidBody rbB, in Matrix frameInB, bool useLinearReferenceFrameB)
        : base(type, GetFixedBody(), rbB)
    {
        // 未提供刚体 A 时，A 隐式为世界空间
        this.frameInB = Transform.FromMatrix(in frameInB);
        useLinearReferenceFrameA = useLinearReferenceFrameB;
        useOffsetForConstraintFrame = true;
        useSolveConstraintObsolete = false;
        frameInA = rbB.worldTransform * this.frameInB;
        CalculateTransforms();
    }

    /// <summary>btGetMatrixElem（行主序索引：行 = index%3，列 = index/3）</summary>
    private static btScalar GetMatrixElem(in Matrix3x3 mat, int index) => mat[index % 3, index / 3];

    /// <summary>matrixToEulerXYZ（来自 geometrictools 的 MatrixToEulerXYZ）</summary>
    private static bool MatrixToEulerXyz(in Matrix3x3 mat, out Vector3 xyz)
    {
        // rot =  cy*cz          -cy*sz           sy
        //        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
        //       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy
        xyz = default;
        btScalar fi = GetMatrixElem(in mat, 2);
        if (fi < 1)
        {
            if (fi > -1)
            {
                xyz[0] = BtMath.Atan2(-GetMatrixElem(in mat, 5), GetMatrixElem(in mat, 8));
                xyz[1] = BtMath.Asin(GetMatrixElem(in mat, 2));
                xyz[2] = BtMath.Atan2(-GetMatrixElem(in mat, 1), GetMatrixElem(in mat, 0));
                return true;
            }
            else
            {
                // 警告：不唯一。XA - ZA = -atan2(r10,r11)
                xyz[0] = -BtMath.Atan2(GetMatrixElem(in mat, 3), GetMatrixElem(in mat, 4));
                xyz[1] = -BtMath.SimdHalfPi;
                xyz[2] = 0;
                return false;
            }
        }
        else
        {
            // 警告：不唯一。XAngle + ZAngle = atan2(r10,r11)
            xyz[0] = BtMath.Atan2(GetMatrixElem(in mat, 3), GetMatrixElem(in mat, 4));
            xyz[1] = BtMath.SimdHalfPi;
            xyz[2] = 0;
        }
        return false;
    }

    /// <summary>btGeneric6DofConstraint::calculateAngleInfo：计算两刚体间的欧拉角与约束轴</summary>
    internal void CalculateAngleInfo()
    {
        Matrix3x3 relativeFrame = calculatedTransformA.Basis.Inverse() * calculatedTransformB.Basis;
        MatrixToEulerXyz(in relativeFrame, out calculatedAxisAngleDiff);
        // 欧拉角模式下，实际并不沿 axis[0]、axis[2] 约束角速度（但会使用 axis[1]）：
        //    d(angle[0])/dt = 0  ->  沿 ax[1] x ax[2] 约束 w2-w1
        //    d(angle[1])/dt = 0  ->  沿 ax[1]
        //    d(angle[2])/dt = 0  ->  沿 ax[0] x ax[1]
        Vector3 axis0 = calculatedTransformB.Basis.Column(0);
        Vector3 axis2 = calculatedTransformA.Basis.Column(2);

        calculatedAxis[1] = axis2.Cross(axis0);
        calculatedAxis[0] = calculatedAxis[1].Cross(axis2);
        calculatedAxis[2] = axis0.Cross(calculatedAxis[1]);

        calculatedAxis[0].Normalize();
        calculatedAxis[1].Normalize();
        calculatedAxis[2].Normalize();
    }

    /// <summary>btGeneric6DofConstraint::calculateTransforms()</summary>
    public void CalculateTransforms()
    {
        CalculateTransforms(in rigidBodyA.worldTransform, in rigidBodyB.worldTransform);
    }

    /// <summary>btGeneric6DofConstraint::calculateTransforms(transA, transB)</summary>
    internal void CalculateTransforms(in Transform transA, in Transform transB)
    {
        calculatedTransformA = transA * frameInA;
        calculatedTransformB = transB * frameInB;
        CalculateLinearInfo();
        CalculateAngleInfo();
        if (useOffsetForConstraintFrame)
        {
            // 依质量取权重因子
            btScalar miA = rigidBodyA.InvMass;
            btScalar miB = rigidBodyB.InvMass;
            hasStaticBody = miA < BtMath.SimdEpsilon || miB < BtMath.SimdEpsilon;
            btScalar miS = miA + miB;
            if (miS > 0)
            {
                factA = miB / miS;
            }
            else
            {
                factA = (btScalar)0.5;
            }
            factB = 1 - factA;
        }
    }

    /// <summary>公开 Matrix 版本的 calculateTransforms</summary>
    public void CalculateTransforms(Matrix transA, Matrix transB)
    {
        Transform ta = Transform.FromMatrix(in transA);
        Transform tb = Transform.FromMatrix(in transB);
        CalculateTransforms(in ta, in tb);
    }

    /// <summary>btGeneric6DofConstraint::buildLinearJacobian</summary>
    internal void BuildLinearJacobian(out JacobianEntry jacLinearOut, in Vector3 normalWorld,
        in Vector3 pivotAInW, in Vector3 pivotBInW)
    {
        jacLinearOut = JacobianEntry.PointConstraint(
            rigidBodyA.worldTransform.Basis.Transpose(),
            rigidBodyB.worldTransform.Basis.Transpose(),
            pivotAInW - rigidBodyA.CenterOfMassPosition,
            pivotBInW - rigidBodyB.CenterOfMassPosition,
            normalWorld,
            rigidBodyA.invInertiaLocal,
            rigidBodyA.InvMass,
            rigidBodyB.invInertiaLocal,
            rigidBodyB.InvMass);
    }

    /// <summary>btGeneric6DofConstraint::buildAngularJacobian</summary>
    internal void BuildAngularJacobian(out JacobianEntry jacAngularOut, in Vector3 jointAxisW)
    {
        jacAngularOut = ConstraintMath.CreateAngularJacobianEntry(jointAxisW,
            rigidBodyA.worldTransform.Basis.Transpose(),
            rigidBodyB.worldTransform.Basis.Transpose(),
            rigidBodyA.invInertiaLocal,
            rigidBodyB.invInertiaLocal);
    }

    /// <summary>btGeneric6DofConstraint::testAngularLimitMotor（须先调用 CalculateTransforms）</summary>
    public bool TestAngularLimitMotor(int axisIndex)
    {
        btScalar angle = calculatedAxisAngleDiff[axisIndex];
        angle = ConstraintMath.AdjustAngleToLimits(angle, angularLimits[axisIndex].loLimit, angularLimits[axisIndex].hiLimit);
        angularLimits[axisIndex].currentPosition = angle;
        // 测试限位
        angularLimits[axisIndex].TestLimitValue(angle);
        return angularLimits[axisIndex].NeedApplyTorques();
    }

    /// <summary>btGeneric6DofConstraint::buildJacobian（仅供 useSolveConstraintObsolete 旧路径）</summary>
    public override void BuildJacobian()
    {
        if (useSolveConstraintObsolete)
        {
            // 为下一步仿真清零累计冲量
            linearLimits.accumulatedImpulse = Vector3.Zero;
            int i;
            for (i = 0; i < 3; i++)
            {
                angularLimits[i].accumulatedImpulse = 0;
            }
            // 计算变换
            CalculateTransforms(in rigidBodyA.worldTransform, in rigidBodyB.worldTransform);

            CalcAnchorPos();
            Vector3 pivotAInW = anchorPos;
            Vector3 pivotBInW = anchorPos;

            Vector3 normalWorld;
            // 线性部分
            for (i = 0; i < 3; i++)
            {
                if (linearLimits.IsLimited(i))
                {
                    if (useLinearReferenceFrameA)
                        normalWorld = calculatedTransformA.Basis.Column(i);
                    else
                        normalWorld = calculatedTransformB.Basis.Column(i);

                    BuildLinearJacobian(out jacLinear[i], in normalWorld, in pivotAInW, in pivotBInW);
                }
            }

            // 角部分
            for (i = 0; i < 3; i++)
            {
                // 计算误差角
                if (TestAngularLimitMotor(i))
                {
                    normalWorld = GetAxis(i);
                    // 创建角约束原子
                    BuildAngularJacobian(out jacAng[i], in normalWorld);
                }
            }
        }
    }

    internal override void GetInfo1(ConstraintInfo1 info, ContactSolverInfo infoGlobal)
    {
        // btGeneric6DofConstraint::getInfo1
        if (useSolveConstraintObsolete)
        {
            info.numConstraintRows = 0;
            info.nub = 0;
        }
        else
        {
            // 准备约束
            CalculateTransforms(in rigidBodyA.worldTransform, in rigidBodyB.worldTransform);
            info.numConstraintRows = 0;
            info.nub = 6;
            int i;
            // 测试线性限位
            for (i = 0; i < 3; i++)
            {
                if (linearLimits.NeedApplyForceLinear(i))
                {
                    info.numConstraintRows++;
                    info.nub--;
                }
            }
            // 测试角限位
            for (i = 0; i < 3; i++)
            {
                if (TestAngularLimitMotor(i))
                {
                    info.numConstraintRows++;
                    info.nub--;
                }
            }
        }
    }

    /// <summary>btGeneric6DofConstraint::getInfo1NonVirtual</summary>
    internal void GetInfo1NonVirtual(ConstraintInfo1 info)
    {
        if (useSolveConstraintObsolete)
        {
            info.numConstraintRows = 0;
            info.nub = 0;
        }
        else
        {
            // 预分配全部 6 行
            info.numConstraintRows = 6;
            info.nub = 0;
        }
    }

    internal override void GetInfo2(ConstraintInfo2 info, ContactSolverInfo infoGlobal,
        Transform[] body0Trans, Transform[] body1Trans, int body0Index, int body1Index)
    {
        // btGeneric6DofConstraint::getInfo2（依赖 GetInfo1 中完成的 calculateTransforms）
        Transform transA = rigidBodyA.worldTransform;
        Transform transB = rigidBodyB.worldTransform;
        Vector3 linVelA = rigidBodyA.linearVelocity;
        Vector3 linVelB = rigidBodyB.linearVelocity;
        Vector3 angVelA = rigidBodyA.angularVelocity;
        Vector3 angVelB = rigidBodyB.angularVelocity;

        if (useOffsetForConstraintFrame)
        {
            // 为稳定性先求解角限位
            int row = SetAngularLimits(info, 0, in transA, in transB, in linVelA, in linVelB, in angVelA, in angVelB);
            SetLinearLimits(info, row, in transA, in transB, in linVelA, in linVelB, in angVelA, in angVelB);
        }
        else
        {
            // 保留旧版本以兼容
            int row = SetLinearLimits(info, 0, in transA, in transB, in linVelA, in linVelB, in angVelA, in angVelB);
            SetAngularLimits(info, row, in transA, in transB, in linVelA, in linVelB, in angVelA, in angVelB);
        }
    }

    /// <summary>btGeneric6DofConstraint::getInfo2NonVirtual</summary>
    internal void GetInfo2NonVirtual(ConstraintInfo2 info, in Transform transA, in Transform transB,
        in Vector3 linVelA, in Vector3 linVelB, in Vector3 angVelA, in Vector3 angVelB)
    {
        // 准备约束
        CalculateTransforms(in transA, in transB);

        for (int i = 0; i < 3; i++)
        {
            TestAngularLimitMotor(i);
        }

        if (useOffsetForConstraintFrame)
        {
            // 为稳定性先求解角限位
            int row = SetAngularLimits(info, 0, in transA, in transB, in linVelA, in linVelB, in angVelA, in angVelB);
            SetLinearLimits(info, row, in transA, in transB, in linVelA, in linVelB, in angVelA, in angVelB);
        }
        else
        {
            // 保留旧版本以兼容
            int row = SetLinearLimits(info, 0, in transA, in transB, in linVelA, in linVelB, in angVelA, in angVelB);
            SetAngularLimits(info, row, in transA, in transB, in linVelA, in linVelB, in angVelA, in angVelB);
        }
    }

    /// <summary>btGeneric6DofConstraint::setLinearLimits（Transform 为内部类型）</summary>
    internal int SetLinearLimits(ConstraintInfo2 info, int row, in Transform transA, in Transform transB,
        in Vector3 linVelA, in Vector3 linVelB, in Vector3 angVelA, in Vector3 angVelB)
    {
        // 求解线性限位
        var limot = new RotationalLimitMotor();
        for (int i = 0; i < 3; i++)
        {
            if (linearLimits.NeedApplyForceLinear(i))
            {
                // 复用旋转马达代码
                limot.bounce = 0;
                limot.currentLimit = linearLimits.currentLimit[i];
                limot.currentPosition = linearLimits.currentLinearDiff[i];
                limot.currentLimitError = linearLimits.currentLimitError[i];
                limot.damping = linearLimits.damping;
                limot.enableMotor = linearLimits.enableMotor[i];
                limot.hiLimit = linearLimits.upperLimit[i];
                limot.limitSoftness = linearLimits.limitSoftness;
                limot.loLimit = linearLimits.lowerLimit[i];
                limot.maxLimitForce = 0;
                limot.maxMotorForce = linearLimits.maxMotorForce[i];
                limot.targetVelocity = linearLimits.targetVelocity[i];
                Vector3 axis = calculatedTransformA.Basis.Column(i);
                int axisFlags = flags >> (i * AxisShift);
                limot.normalCFM = (axisFlags & (int)SixDofFlags.CfmNorm) != 0 ? linearLimits.normalCFM[i] : info.cfm[0];
                limot.stopCFM = (axisFlags & (int)SixDofFlags.CfmStop) != 0 ? linearLimits.stopCFM[i] : info.cfm[0];
                limot.stopERP = (axisFlags & (int)SixDofFlags.ErpStop) != 0 ? linearLimits.stopERP[i] : info.erp;
                if (useOffsetForConstraintFrame)
                {
                    int indx1 = (i + 1) % 3;
                    int indx2 = (i + 2) % 3;
                    int rotAllowed = 1; // 绕当前轴正交方向的旋转
                    if (angularLimits[indx1].currentLimit != 0 && angularLimits[indx2].currentLimit != 0)
                    {
                        rotAllowed = 0;
                    }
                    row += GetLimitMotorInfo2(limot, in transA, in transB, in linVelA, in linVelB, in angVelA, in angVelB, info, row, in axis, 0, rotAllowed);
                }
                else
                {
                    row += GetLimitMotorInfo2(limot, in transA, in transB, in linVelA, in linVelB, in angVelA, in angVelB, info, row, in axis, 0);
                }
            }
        }
        return row;
    }

    /// <summary>btGeneric6DofConstraint::setAngularLimits（Transform 为内部类型）</summary>
    internal int SetAngularLimits(ConstraintInfo2 info, int rowOffset, in Transform transA, in Transform transB,
        in Vector3 linVelA, in Vector3 linVelB, in Vector3 angVelA, in Vector3 angVelB)
    {
        int row = rowOffset;
        // 求解角限位
        for (int i = 0; i < 3; i++)
        {
            if (angularLimits[i].NeedApplyTorques())
            {
                Vector3 axis = GetAxis(i);
                int axisFlags = flags >> ((i + 3) * AxisShift);
                if ((axisFlags & (int)SixDofFlags.CfmNorm) == 0)
                {
                    angularLimits[i].normalCFM = info.cfm[0];
                }
                if ((axisFlags & (int)SixDofFlags.CfmStop) == 0)
                {
                    angularLimits[i].stopCFM = info.cfm[0];
                }
                if ((axisFlags & (int)SixDofFlags.ErpStop) == 0)
                {
                    angularLimits[i].stopERP = info.erp;
                }
                row += GetLimitMotorInfo2(angularLimits[i],
                    in transA, in transB, in linVelA, in linVelB, in angVelA, in angVelB, info, row, in axis, 1);
            }
        }

        return row;
    }

    /// <summary>btGeneric6DofConstraint::updateRHS</summary>
    public void UpdateRHS(btScalar timeStep)
    {
    }

    /// <summary>btGeneric6DofConstraint::setFrames</summary>
    public virtual void SetFrames(Matrix frameA, Matrix frameB)
    {
        frameInA = Transform.FromMatrix(in frameA);
        frameInB = Transform.FromMatrix(in frameB);
        BuildJacobian();
        CalculateTransforms();
    }

    /// <summary>btGeneric6DofConstraint::getAxis（世界坐标下的旋转轴，须先 BuildJacobian）</summary>
    public Vector3 GetAxis(int axisIndex) => calculatedAxis[axisIndex];

    /// <summary>btGeneric6DofConstraint::getRelativePivotPosition（须先 CalculateTransforms）</summary>
    public btScalar GetRelativePivotPosition(int axisIndex) => calculatedLinearDiff[axisIndex];

    /// <summary>btGeneric6DofConstraint::getAngle（相对欧拉角，须先 CalculateTransforms）</summary>
    public btScalar GetAngle(int axisIndex) => calculatedAxisAngleDiff[axisIndex];

    /// <summary>btGeneric6DofConstraint::calcAnchorPos</summary>
    public virtual void CalcAnchorPos()
    {
        btScalar imA = rigidBodyA.InvMass;
        btScalar imB = rigidBodyB.InvMass;
        btScalar weight;
        if (imB == 0)
        {
            weight = 1;
        }
        else
        {
            weight = imA / (imA + imB);
        }
        Vector3 pA = calculatedTransformA.Origin;
        Vector3 pB = calculatedTransformB.Origin;
        anchorPos = pA * weight + pB * (1 - weight);
    }

    /// <summary>btGeneric6DofConstraint::calculateLinearInfo</summary>
    internal void CalculateLinearInfo()
    {
        calculatedLinearDiff = calculatedTransformB.Origin - calculatedTransformA.Origin;
        calculatedLinearDiff = calculatedTransformA.Basis.Inverse() * calculatedLinearDiff;
        for (int i = 0; i < 3; i++)
        {
            linearLimits.currentLinearDiff[i] = calculatedLinearDiff[i];
            linearLimits.TestLimitValue(i, calculatedLinearDiff[i]);
        }
    }

    /// <summary>btGeneric6DofConstraint::get_limit_motor_info2（线性/角限位与马达行写入；Transform 为内部类型）</summary>
    internal int GetLimitMotorInfo2(
        RotationalLimitMotor limot,
        in Transform transA, in Transform transB,
        in Vector3 linVelA, in Vector3 linVelB, in Vector3 angVelA, in Vector3 angVelB,
        ConstraintInfo2 info, int row, in Vector3 ax1, int rotational, int rotAllowed = 0)
    {
        int srow = row * info.rowskip;
        bool powered = limot.enableMotor;
        int limit = limot.currentLimit;
        if (powered || limit != 0)
        {
            // 若关节有马达或限位，添加额外行
            btScalar[] j1 = rotational != 0 ? info.j1AngularAxis : info.j1LinearAxis;
            btScalar[] j2 = rotational != 0 ? info.j2AngularAxis : info.j2LinearAxis;
            j1[srow + 0] = ax1[0];
            j1[srow + 1] = ax1[1];
            j1[srow + 2] = ax1[2];

            j2[srow + 0] = -ax1[0];
            j2[srow + 1] = -ax1[1];
            j2[srow + 2] = -ax1[2];

            if (rotational == 0)
            {
                if (useOffsetForConstraintFrame)
                {
                    // 世界系下从 bodyB 到 frameB 的向量
                    Vector3 relB = calculatedTransformB.Origin - transB.Origin;
                    // 在约束轴上的投影
                    Vector3 projB = ax1 * relB.Dot(ax1);
                    // 从 bodyB 指向约束轴（且与之正交）的向量
                    Vector3 orthoB = relB - projB;
                    // bodyA 同理
                    Vector3 relA = calculatedTransformA.Origin - transA.Origin;
                    Vector3 projA = ax1 * relA.Dot(ax1);
                    Vector3 orthoA = relA - projA;
                    // 帧 A、B 之间沿约束轴的期望偏移
                    btScalar desiredOffs = limot.currentPosition - limot.currentLimitError;
                    // 从 bodyA 中心投影到 bodyB 中心投影的期望向量
                    Vector3 totalDist = projA + ax1 * desiredOffs - projB;
                    // 得到偏移向量 relA、relB
                    relA = orthoA + totalDist * factA;
                    relB = orthoB - totalDist * factB;
                    Vector3 tmpA = relA.Cross(ax1);
                    Vector3 tmpB = relB.Cross(ax1);
                    if (hasStaticBody && rotAllowed == 0)
                    {
                        tmpA *= factA;
                        tmpB *= factB;
                    }
                    int i;
                    for (i = 0; i < 3; i++) info.j1AngularAxis[srow + i] = tmpA[i];
                    for (i = 0; i < 3; i++) info.j2AngularAxis[srow + i] = -tmpB[i];
                }
                else
                {
                    // Linear Torque Decoupling 向量
                    Vector3 c = calculatedTransformB.Origin - transA.Origin;
                    Vector3 ltd = c.Cross(ax1);
                    info.j1AngularAxis[srow + 0] = ltd[0];
                    info.j1AngularAxis[srow + 1] = ltd[1];
                    info.j1AngularAxis[srow + 2] = ltd[2];

                    c = calculatedTransformB.Origin - transB.Origin;
                    ltd = -c.Cross(ax1);
                    info.j2AngularAxis[srow + 0] = ltd[0];
                    info.j2AngularAxis[srow + 1] = ltd[1];
                    info.j2AngularAxis[srow + 2] = ltd[2];
                }
            }
            // 若低位与高位同时限位，关节马达无效
            if (limit != 0 && limot.loLimit == limot.hiLimit) powered = false;
            info.constraintError[srow] = 0;
            if (powered)
            {
                info.cfm[srow] = limot.normalCFM;
                if (limit == 0)
                {
                    btScalar tagVel = rotational != 0 ? limot.targetVelocity : -limot.targetVelocity;

                    btScalar motFact = ConstraintMath.GetMotorFactor(limot.currentPosition,
                        limot.loLimit,
                        limot.hiLimit,
                        tagVel,
                        info.fps * limot.stopERP);
                    info.constraintError[srow] += motFact * limot.targetVelocity;
                    info.lowerLimit[srow] = -limot.maxMotorForce / info.fps;
                    info.upperLimit[srow] = limot.maxMotorForce / info.fps;
                }
            }
            if (limit != 0)
            {
                btScalar k = info.fps * limot.stopERP;
                if (rotational == 0)
                {
                    info.constraintError[srow] += k * limot.currentLimitError;
                }
                else
                {
                    info.constraintError[srow] += -k * limot.currentLimitError;
                }
                info.cfm[srow] = limot.stopCFM;
                if (limot.loLimit == limot.hiLimit)
                {
                    // 低位与高位同时限位
                    info.lowerLimit[srow] = -BtMath.SimdInfinity;
                    info.upperLimit[srow] = BtMath.SimdInfinity;
                }
                else
                {
                    if (limit == 1)
                    {
                        info.lowerLimit[srow] = 0;
                        info.upperLimit[srow] = BtMath.SimdInfinity;
                    }
                    else
                    {
                        info.lowerLimit[srow] = -BtMath.SimdInfinity;
                        info.upperLimit[srow] = 0;
                    }
                    // 处理反弹
                    if (limot.bounce > 0)
                    {
                        // 计算关节速度
                        btScalar vel;
                        if (rotational != 0)
                        {
                            vel = angVelA.Dot(ax1);
                            vel -= angVelB.Dot(ax1);
                        }
                        else
                        {
                            vel = linVelA.Dot(ax1);
                            vel -= linVelB.Dot(ax1);
                        }
                        // 仅当速度朝向限位且所得 c[] 超过现值时应用反弹
                        if (limit == 1)
                        {
                            if (vel < 0)
                            {
                                btScalar newc = -limot.bounce * vel;
                                if (newc > info.constraintError[srow])
                                    info.constraintError[srow] = newc;
                            }
                        }
                        else
                        {
                            if (vel > 0)
                            {
                                btScalar newc = -limot.bounce * vel;
                                if (newc < info.constraintError[srow])
                                    info.constraintError[srow] = newc;
                            }
                        }
                    }
                }
            }
            return 1;
        }
        else
        {
            return 0;
        }
    }

    /// <summary>btGeneric6DofConstraint::setLinearLowerLimit</summary>
    public Vector3 LinearLowerLimit
    {
        get => linearLimits.lowerLimit;
        set => linearLimits.lowerLimit = value;
    }

    /// <summary>btGeneric6DofConstraint::setLinearUpperLimit</summary>
    public Vector3 LinearUpperLimit
    {
        get => linearLimits.upperLimit;
        set => linearLimits.upperLimit = value;
    }

    /// <summary>btGeneric6DofConstraint::setAngularLowerLimit（写入前归一化角度）</summary>
    public Vector3 AngularLowerLimit
    {
        get => new Vector3(angularLimits[0].loLimit, angularLimits[1].loLimit, angularLimits[2].loLimit);
        set
        {
            for (int i = 0; i < 3; i++)
                angularLimits[i].loLimit = ConstraintMath.NormalizeAngle(value[i]);
        }
    }

    /// <summary>btGeneric6DofConstraint::setAngularUpperLimit（写入前归一化角度）</summary>
    public Vector3 AngularUpperLimit
    {
        get => new Vector3(angularLimits[0].hiLimit, angularLimits[1].hiLimit, angularLimits[2].hiLimit);
        set
        {
            for (int i = 0; i < 3; i++)
                angularLimits[i].hiLimit = ConstraintMath.NormalizeAngle(value[i]);
        }
    }

    /// <summary>btGeneric6DofConstraint::getRotationalLimitMotor</summary>
    public RotationalLimitMotor GetRotLimitMotor(int index) => angularLimits[index];

    /// <summary>btGeneric6DofConstraint::getTranslationalLimitMotor</summary>
    public TranslationalLimitMotor GetTranslationalLimitMotor() => linearLimits;

    /// <summary>btGeneric6DofConstraint::setLimit（前 3 为线性，后 3 为角）</summary>
    public void SetLimit(int axis, btScalar lo, btScalar hi)
    {
        if (axis < 3)
        {
            linearLimits.lowerLimit[axis] = lo;
            linearLimits.upperLimit[axis] = hi;
        }
        else
        {
            lo = ConstraintMath.NormalizeAngle(lo);
            hi = ConstraintMath.NormalizeAngle(hi);
            angularLimits[axis - 3].loLimit = lo;
            angularLimits[axis - 3].hiLimit = hi;
        }
    }

    /// <summary>btGeneric6DofConstraint::isLimited（前 3 为线性，后 3 为角）</summary>
    public bool IsLimited(int limitIndex)
    {
        if (limitIndex < 3)
        {
            return linearLimits.IsLimited(limitIndex);
        }
        return angularLimits[limitIndex - 3].IsLimited();
    }

    /// <summary>btGeneric6DofConstraint::getCalculatedTransformA</summary>
    public Matrix CalculatedTransformA => calculatedTransformA.ToMatrix();

    /// <summary>btGeneric6DofConstraint::getCalculatedTransformB</summary>
    public Matrix CalculatedTransformB => calculatedTransformB.ToMatrix();

    /// <summary>btGeneric6DofConstraint::getFrameOffsetA</summary>
    public Matrix FrameOffsetA
    {
        get => frameInA.ToMatrix();
        set => frameInA = Transform.FromMatrix(in value);
    }

    /// <summary>btGeneric6DofConstraint::getFrameOffsetB</summary>
    public Matrix FrameOffsetB
    {
        get => frameInB.ToMatrix();
        set => frameInB = Transform.FromMatrix(in value);
    }

    /// <summary>btGeneric6DofConstraint::getUseFrameOffset/setUseFrameOffset</summary>
    public bool UseFrameOffset
    {
        get => useOffsetForConstraintFrame;
        set => useOffsetForConstraintFrame = value;
    }

    /// <summary>btGeneric6DofConstraint::getUseLinearReferenceFrameA/setUseLinearReferenceFrameA</summary>
    public bool UseLinearReferenceFrameA
    {
        get => useLinearReferenceFrameA;
        set => useLinearReferenceFrameA = value;
    }

    /// <summary>btGeneric6DofConstraint::m_useSolveConstraintObsolete（向后兼容标记）</summary>
    public bool UseSolveConstraintObsolete
    {
        get => useSolveConstraintObsolete;
        set => useSolveConstraintObsolete = value;
    }

    /// <summary>btGeneric6DofConstraint::setParam（覆盖全局默认参数，axis 0..5）</summary>
    public override void SetParam(int num, btScalar value, int axis = -1)
    {
        if (axis >= 0 && axis < 3)
        {
            switch ((ConstraintParams)num)
            {
                case ConstraintParams.StopErp:
                    linearLimits.stopERP[axis] = value;
                    flags |= (int)SixDofFlags.ErpStop << (axis * AxisShift);
                    break;
                case ConstraintParams.StopCfm:
                    linearLimits.stopCFM[axis] = value;
                    flags |= (int)SixDofFlags.CfmStop << (axis * AxisShift);
                    break;
                case ConstraintParams.Cfm:
                    linearLimits.normalCFM[axis] = value;
                    flags |= (int)SixDofFlags.CfmNorm << (axis * AxisShift);
                    break;
                default:
                    // btAssertConstrParams(0)
                    break;
            }
        }
        else if (axis >= 3 && axis < 6)
        {
            switch ((ConstraintParams)num)
            {
                case ConstraintParams.StopErp:
                    angularLimits[axis - 3].stopERP = value;
                    flags |= (int)SixDofFlags.ErpStop << (axis * AxisShift);
                    break;
                case ConstraintParams.StopCfm:
                    angularLimits[axis - 3].stopCFM = value;
                    flags |= (int)SixDofFlags.CfmStop << (axis * AxisShift);
                    break;
                case ConstraintParams.Cfm:
                    angularLimits[axis - 3].normalCFM = value;
                    flags |= (int)SixDofFlags.CfmNorm << (axis * AxisShift);
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

    /// <summary>btGeneric6DofConstraint::getParam（返回局部参数值）</summary>
    public override btScalar GetParam(int num, int axis)
    {
        btScalar retVal = 0;
        if (axis >= 0 && axis < 3)
        {
            switch ((ConstraintParams)num)
            {
                case ConstraintParams.StopErp:
                    retVal = linearLimits.stopERP[axis];
                    break;
                case ConstraintParams.StopCfm:
                    retVal = linearLimits.stopCFM[axis];
                    break;
                case ConstraintParams.Cfm:
                    retVal = linearLimits.normalCFM[axis];
                    break;
                default:
                    // btAssertConstrParams(0)
                    break;
            }
        }
        else if (axis >= 3 && axis < 6)
        {
            switch ((ConstraintParams)num)
            {
                case ConstraintParams.StopErp:
                    retVal = angularLimits[axis - 3].stopERP;
                    break;
                case ConstraintParams.StopCfm:
                    retVal = angularLimits[axis - 3].stopCFM;
                    break;
                case ConstraintParams.Cfm:
                    retVal = angularLimits[axis - 3].normalCFM;
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

    /// <summary>btGeneric6DofConstraint::setAxis</summary>
    public virtual void SetAxis(Vector3 axis1, Vector3 axis2)
    {
        Vector3 zAxis = axis1.Normalized();
        Vector3 yAxis = axis2.Normalized();
        Vector3 xAxis = yAxis.Cross(zAxis); // 需要右手坐标系

        var frameInW = Transform.Identity;
        frameInW.Basis = new Matrix3x3(
            xAxis[0], yAxis[0], zAxis[0],
            xAxis[1], yAxis[1], zAxis[1],
            xAxis[2], yAxis[2], zAxis[2]);

        // 转换到局部坐标系的约束帧
        frameInA = rigidBodyA.worldTransform.Inverse() * frameInW;
        frameInB = rigidBodyB.worldTransform.Inverse() * frameInW;

        CalculateTransforms();
    }

    /// <summary>btGeneric6DofConstraint::getFlags</summary>
    public int Flags => flags;
}
