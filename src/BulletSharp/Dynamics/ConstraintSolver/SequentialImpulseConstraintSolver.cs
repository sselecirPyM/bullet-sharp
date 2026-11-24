using BulletSharp.Math;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BulletSharp;

/// <summary>
/// 移植自 btSequentialImpulseConstraintSolver：顺序脉冲约束求解器
/// （Projected Gauss-Seidel）。仅移植 scalar reference 行求解路径，
/// SIMD（btSimdScalar/SSE/batch restack）与多线程（BT_THREADSAFE）不在范围内。
/// </summary>
public class SequentialImpulseConstraintSolver : ConstraintSolver
{
    // 基础层 ContactSolverInfo 缺少的字段，使用 btContactSolverInfo 构造默认值
    private const btScalar FrictionErpDefault = (btScalar)0.2;
    private const btScalar RestitutionVelocityThresholdDefault = (btScalar)0.2;

    protected ulong btSeed2;
    protected int cachedSolverMode;

    // ---- 对象池（替代 btAlignedObjectArray 的 resizeNoInitialize/expand）----
    private readonly List<SolverBody> tmpSolverBodyPool = new();
    private int solverBodyCount;

    private readonly List<SolverConstraint> tmpSolverContactConstraintPool = new();
    private int contactConstraintCount;

    private readonly List<SolverConstraint> tmpSolverNonContactConstraintPool = new();
    private int nonContactConstraintCount;

    private readonly List<SolverConstraint> tmpSolverContactFrictionConstraintPool = new();
    private int contactFrictionConstraintCount;

    private readonly List<SolverConstraint> tmpSolverContactRollingFrictionConstraintPool = new();
    private int contactRollingFrictionConstraintCount;

    /// <summary>True when at least one contact constraint of the current solve group has a
    /// non-zero rhsPenetration; allows skipping the split-impulse iteration loop entirely.</summary>
    private bool hasSplitImpulsePenetration;

    private readonly List<ConstraintInfo1> tmpConstraintSizesPool = new();

    private int[] orderNonContactConstraintPool = [];
    private int[] orderTmpConstraintPool = [];
    private int[] orderFrictionConstraintPool = [];

    private int fixedBodyId = -1;
    private int maxOverrideNumSolverIterations;
    private btScalar leastSquaresResidual;

    /// <summary>GetInfo2 的可复用单元素变换数组（配合基础层抽象签名）</summary>
    private readonly Transform[] info2Body0Trans = new Transform[1];
    private readonly Transform[] info2Body1Trans = new Transform[1];

    public SequentialImpulseConstraintSolver()
    {
        btSeed2 = 0;
        cachedSolverMode = 0;
    }

    /// <summary>m_btSeed2（随机化约束顺序的种子）</summary>
    public ulong RandSeed
    {
        get => btSeed2;
        set => btSeed2 = value;
    }

    /// <summary>btSequentialImpulseConstraintSolver::btRand2（LCG 随机数）</summary>
    private ulong BtRand2()
    {
        btSeed2 = (1664525UL * btSeed2 + 1013904223UL) & 0xffffffffUL;
        return btSeed2;
    }

    /// <summary>btSequentialImpulseConstraintSolver::btRandInt2（参见 ODE dRandInt）</summary>
    private int BtRandInt2(int n)
    {
        ulong un = (ulong)n;
        ulong r = BtRand2();

        if (un <= 0x00010000UL)
        {
            r ^= r >> 16;
            if (un <= 0x00000100UL)
            {
                r ^= r >> 8;
                if (un <= 0x00000010UL)
                {
                    r ^= r >> 4;
                    if (un <= 0x00000004UL)
                    {
                        r ^= r >> 2;
                        if (un <= 0x00000002UL)
                        {
                            r ^= r >> 1;
                        }
                    }
                }
            }
        }

        return (int)(r % un);
    }

    // ---------------- 池管理 ----------------

    // ConvertJoint 的复用 scratch（C++ 中为栈上临时缓冲）
    private ConstraintInfo2 jointInfo2;
    private btScalar[] j1LinearPool, j1AngularPool, j2LinearPool, j2AngularPool;
    private btScalar[] constraintErrorPool, cfmPool, lowerLimitPool, upperLimitPool;

    /// <summary>确保 scratch 数组容量并清零使用区（约束代码依赖零初始化语义）</summary>
    private static btScalar[] EnsureScalarArray(ref btScalar[] arr, int size)
    {
        if (arr == null || arr.Length < size)
            arr = new btScalar[size];
        else
            Array.Clear(arr, 0, size);
        return arr;
    }

    private SolverBody ExpandSolverBody()
    {
        if (solverBodyCount == tmpSolverBodyPool.Count)
            tmpSolverBodyPool.Add(new SolverBody());
        return tmpSolverBodyPool[solverBodyCount++];
    }

    private static SolverConstraint ExpandConstraint(List<SolverConstraint> pool, ref int count)
    {
        if (count == pool.Count)
            pool.Add(new SolverConstraint());
        return pool[count++];
    }

    private static void EnsureIntPool(ref int[] pool, int size)
    {
        if (pool.Length < size)
            pool = new int[size];
    }

    /// <summary>btSequentialImpulseConstraintSolver::initSolverBody</summary>
    private static void InitSolverBody(SolverBody solverBody, CollisionObject collisionObject, btScalar timeStep)
    {
        RigidBody rb = collisionObject != null ? RigidBody.Upcast(collisionObject) : null;

        solverBody.deltaLinearVelocity = Vector3.Zero;
        solverBody.deltaAngularVelocity = Vector3.Zero;
        solverBody.pushVelocity = Vector3.Zero;
        solverBody.turnVelocity = Vector3.Zero;

        if (rb != null)
        {
            solverBody.worldTransform = rb.worldTransform;
            solverBody.invMass = new Vector3(rb.InvMass, rb.InvMass, rb.InvMass) * rb.LinearFactor;
            solverBody.originalBody = rb;
            solverBody.angularFactor = rb.angularFactor;
            solverBody.linearFactor = rb.LinearFactor;
            solverBody.linearVelocity = rb.linearVelocity;
            solverBody.angularVelocity = rb.angularVelocity;
            solverBody.externalForceImpulse = rb.totalForce * rb.InvMass * timeStep;
            solverBody.externalTorqueImpulse = rb.totalTorque * rb.invInertiaTensorWorld * timeStep;
        }
        else
        {
            solverBody.worldTransform = Transform.Identity;
            solverBody.invMass = Vector3.Zero;
            solverBody.originalBody = null;
            solverBody.angularFactor = new Vector3(1, 1, 1);
            solverBody.linearFactor = new Vector3(1, 1, 1);
            solverBody.linearVelocity = Vector3.Zero;
            solverBody.angularVelocity = Vector3.Zero;
            solverBody.externalForceImpulse = Vector3.Zero;
            solverBody.externalTorqueImpulse = Vector3.Zero;
        }
    }

    /// <summary>btSequentialImpulseConstraintSolver::restitutionCurve</summary>
    private static btScalar RestitutionCurve(btScalar relVel, btScalar restitution, btScalar velocityThreshold)
    {
        if (BtMath.Abs(relVel) < velocityThreshold)
            return 0;

        btScalar rest = restitution * -relVel;
        return rest;
    }

    /// <summary>btSequentialImpulseConstraintSolver::applyAnisotropicFriction</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ApplyAnisotropicFriction(CollisionObject colObj, ref Vector3 frictionDirection, AnisotropicFrictionFlags frictionMode)
    {
        if (colObj != null && colObj.HasAnisotropicFriction(frictionMode))
        {
            // 变换到局部坐标
            Vector3 locLateral = frictionDirection * colObj.worldTransform.Basis;
            Vector3 frictionScaling = colObj.anisotropicFriction;
            // 应用各向异性摩擦
            locLateral *= frictionScaling;
            // 再变换回世界坐标
            frictionDirection = colObj.worldTransform.Basis * locLateral;
        }
    }

    /// <summary>btSequentialImpulseConstraintSolver::setupFrictionConstraint</summary>
    private void SetupFrictionConstraint(SolverConstraint solverConstraint, in Vector3 normalAxis,
        int solverBodyIdA, int solverBodyIdB, ManifoldPoint cp, in Vector3 relPos1, in Vector3 relPos2,
        CollisionObject colObj0, CollisionObject colObj1, btScalar relaxation, ContactSolverInfo infoGlobal,
        btScalar desiredVelocity = 0, btScalar cfmSlip = 0)
    {
        SolverBody solverBodyA = tmpSolverBodyPool[solverBodyIdA];
        SolverBody solverBodyB = tmpSolverBodyPool[solverBodyIdB];

        RigidBody body0 = solverBodyA.originalBody;
        RigidBody bodyA = solverBodyB.originalBody;

        solverConstraint.solverBodyIdA = solverBodyIdA;
        solverConstraint.solverBodyIdB = solverBodyIdB;

        solverConstraint.friction = cp.combinedFriction;
        solverConstraint.originalContactPoint = null;

        solverConstraint.appliedImpulse = 0;
        solverConstraint.appliedPushImpulse = 0;

        if (body0 != null)
        {
            solverConstraint.contactNormal1 = normalAxis;
            Vector3 ftorqueAxis1 = relPos1.Cross(solverConstraint.contactNormal1);
            solverConstraint.relpos1CrossNormal = ftorqueAxis1;
            solverConstraint.angularComponentA = body0.invInertiaTensorWorld * ftorqueAxis1 * body0.angularFactor;
        }
        else
        {
            solverConstraint.contactNormal1 = Vector3.Zero;
            solverConstraint.relpos1CrossNormal = Vector3.Zero;
            solverConstraint.angularComponentA = Vector3.Zero;
        }

        if (bodyA != null)
        {
            solverConstraint.contactNormal2 = -normalAxis;
            Vector3 ftorqueAxis1 = relPos2.Cross(solverConstraint.contactNormal2);
            solverConstraint.relpos2CrossNormal = ftorqueAxis1;
            solverConstraint.angularComponentB = bodyA.invInertiaTensorWorld * ftorqueAxis1 * bodyA.angularFactor;
        }
        else
        {
            solverConstraint.contactNormal2 = Vector3.Zero;
            solverConstraint.relpos2CrossNormal = Vector3.Zero;
            solverConstraint.angularComponentB = Vector3.Zero;
        }

        {
            Vector3 vec;
            btScalar denom0 = 0;
            btScalar denom1 = 0;
            if (body0 != null)
            {
                vec = solverConstraint.angularComponentA.Cross(relPos1);
                denom0 = body0.InvMass + normalAxis.Dot(vec);
            }
            if (bodyA != null)
            {
                vec = (-solverConstraint.angularComponentB).Cross(relPos2);
                denom1 = bodyA.InvMass + normalAxis.Dot(vec);
            }
            btScalar denom = relaxation / (denom0 + denom1);
            solverConstraint.jacDiagABInv = denom;
            solverConstraint.jacDiagAB = 1 / denom;
        }

        {
            btScalar vel1Dotn = solverConstraint.contactNormal1.Dot(body0 != null ? solverBodyA.linearVelocity + solverBodyA.externalForceImpulse : Vector3.Zero)
                + solverConstraint.relpos1CrossNormal.Dot(body0 != null ? solverBodyA.angularVelocity : Vector3.Zero);
            btScalar vel2Dotn = solverConstraint.contactNormal2.Dot(bodyA != null ? solverBodyB.linearVelocity + solverBodyB.externalForceImpulse : Vector3.Zero)
                + solverConstraint.relpos2CrossNormal.Dot(bodyA != null ? solverBodyB.angularVelocity : Vector3.Zero);

            btScalar relVel = vel1Dotn + vel2Dotn;

            btScalar velocityError = desiredVelocity - relVel;
            btScalar velocityImpulse = velocityError * solverConstraint.jacDiagABInv;

            btScalar penetrationImpulse = 0;

            if ((cp.contactPointFlags & ContactPointFlags.FrictionAnchor) != 0)
            {
                btScalar distance = (cp.positionWorldOnA - cp.positionWorldOnB).Dot(normalAxis);
                btScalar positionalError = -distance * FrictionErpDefault / infoGlobal.timeStep;
                penetrationImpulse = positionalError * solverConstraint.jacDiagABInv;
            }

            solverConstraint.rhs = penetrationImpulse + velocityImpulse;
            solverConstraint.rhsPenetration = 0;
            solverConstraint.cfm = cfmSlip;
            solverConstraint.lowerLimit = -solverConstraint.friction;
            solverConstraint.upperLimit = solverConstraint.friction;
        }
    }

    /// <summary>btSequentialImpulseConstraintSolver::addFrictionConstraint</summary>
    private SolverConstraint AddFrictionConstraint(in Vector3 normalAxis, int solverBodyIdA, int solverBodyIdB,
        int frictionIndex, ManifoldPoint cp, in Vector3 relPos1, in Vector3 relPos2,
        CollisionObject colObj0, CollisionObject colObj1, btScalar relaxation, ContactSolverInfo infoGlobal,
        btScalar desiredVelocity = 0, btScalar cfmSlip = 0)
    {
        SolverConstraint solverConstraint = ExpandConstraint(tmpSolverContactFrictionConstraintPool, ref contactFrictionConstraintCount);
        solverConstraint.frictionIndex = frictionIndex;
        SetupFrictionConstraint(solverConstraint, normalAxis, solverBodyIdA, solverBodyIdB, cp, relPos1, relPos2,
            colObj0, colObj1, relaxation, infoGlobal, desiredVelocity, cfmSlip);
        return solverConstraint;
    }

    /// <summary>btSequentialImpulseConstraintSolver::setupTorsionalFrictionConstraint</summary>
    private void SetupTorsionalFrictionConstraint(SolverConstraint solverConstraint, in Vector3 normalAxis1,
        int solverBodyIdA, int solverBodyIdB, ManifoldPoint cp, btScalar combinedTorsionalFriction,
        in Vector3 relPos1, in Vector3 relPos2, CollisionObject colObj0, CollisionObject colObj1,
        btScalar relaxation, btScalar desiredVelocity = 0, btScalar cfmSlip = 0)
    {
        Vector3 normalAxis = Vector3.Zero;

        solverConstraint.contactNormal1 = normalAxis;
        solverConstraint.contactNormal2 = -normalAxis;
        SolverBody solverBodyA = tmpSolverBodyPool[solverBodyIdA];
        SolverBody solverBodyB = tmpSolverBodyPool[solverBodyIdB];

        RigidBody body0 = solverBodyA.originalBody;
        RigidBody bodyA = solverBodyB.originalBody;

        solverConstraint.solverBodyIdA = solverBodyIdA;
        solverConstraint.solverBodyIdB = solverBodyIdB;

        solverConstraint.friction = combinedTorsionalFriction;
        solverConstraint.originalContactPoint = null;

        solverConstraint.appliedImpulse = 0;
        solverConstraint.appliedPushImpulse = 0;

        {
            Vector3 ftorqueAxis1 = -normalAxis1;
            solverConstraint.relpos1CrossNormal = ftorqueAxis1;
            solverConstraint.angularComponentA = body0 != null
                ? body0.invInertiaTensorWorld * ftorqueAxis1 * body0.angularFactor
                : Vector3.Zero;
        }
        {
            Vector3 ftorqueAxis1 = normalAxis1;
            solverConstraint.relpos2CrossNormal = ftorqueAxis1;
            solverConstraint.angularComponentB = bodyA != null
                ? bodyA.invInertiaTensorWorld * ftorqueAxis1 * bodyA.angularFactor
                : Vector3.Zero;
        }

        {
            Vector3 iMJaA = body0 != null ? body0.invInertiaTensorWorld * solverConstraint.relpos1CrossNormal : Vector3.Zero;
            Vector3 iMJaB = bodyA != null ? bodyA.invInertiaTensorWorld * solverConstraint.relpos2CrossNormal : Vector3.Zero;
            btScalar sum = 0;
            sum += iMJaA.Dot(solverConstraint.relpos1CrossNormal);
            sum += iMJaB.Dot(solverConstraint.relpos2CrossNormal);
            solverConstraint.jacDiagABInv = 1 / sum;
            solverConstraint.jacDiagAB = 1 / solverConstraint.jacDiagABInv;
        }

        {
            btScalar vel1Dotn = solverConstraint.contactNormal1.Dot(body0 != null ? solverBodyA.linearVelocity + solverBodyA.externalForceImpulse : Vector3.Zero)
                + solverConstraint.relpos1CrossNormal.Dot(body0 != null ? solverBodyA.angularVelocity : Vector3.Zero);
            btScalar vel2Dotn = solverConstraint.contactNormal2.Dot(bodyA != null ? solverBodyB.linearVelocity + solverBodyB.externalForceImpulse : Vector3.Zero)
                + solverConstraint.relpos2CrossNormal.Dot(bodyA != null ? solverBodyB.angularVelocity : Vector3.Zero);

            btScalar relVel = vel1Dotn + vel2Dotn;

            btScalar velocityError = desiredVelocity - relVel;
            btScalar velocityImpulse = velocityError * solverConstraint.jacDiagABInv;
            solverConstraint.rhs = velocityImpulse;
            solverConstraint.cfm = cfmSlip;
            solverConstraint.lowerLimit = -solverConstraint.friction;
            solverConstraint.upperLimit = solverConstraint.friction;
        }
    }

    /// <summary>btSequentialImpulseConstraintSolver::addTorsionalFrictionConstraint</summary>
    private SolverConstraint AddTorsionalFrictionConstraint(in Vector3 normalAxis, int solverBodyIdA, int solverBodyIdB,
        int frictionIndex, ManifoldPoint cp, btScalar combinedTorsionalFriction, in Vector3 relPos1, in Vector3 relPos2,
        CollisionObject colObj0, CollisionObject colObj1, btScalar relaxation,
        btScalar desiredVelocity = 0, btScalar cfmSlip = 0)
    {
        SolverConstraint solverConstraint = ExpandConstraint(tmpSolverContactRollingFrictionConstraintPool, ref contactRollingFrictionConstraintCount);
        solverConstraint.frictionIndex = frictionIndex;
        SetupTorsionalFrictionConstraint(solverConstraint, normalAxis, solverBodyIdA, solverBodyIdB, cp,
            combinedTorsionalFriction, relPos1, relPos2, colObj0, colObj1, relaxation, desiredVelocity, cfmSlip);
        return solverConstraint;
    }

    /// <summary>btSequentialImpulseConstraintSolver::getOrInitSolverBody（非 BT_THREADSAFE 版本）</summary>
    internal int GetOrInitSolverBody(CollisionObject body, btScalar timeStep)
    {
        int solverBodyIdA = -1;

        if (body.companionId >= 0)
        {
            // body 已转换
            solverBodyIdA = body.companionId;
        }
        else
        {
            RigidBody rb = RigidBody.Upcast(body);
            // 同时转换活跃对象与运动学对象（需要其速度）
            if (rb != null && (rb.InvMass != 0 || rb.IsKinematicObject))
            {
                solverBodyIdA = solverBodyCount;
                SolverBody solverBody = ExpandSolverBody();
                InitSolverBody(solverBody, body, timeStep);
                body.companionId = solverBodyIdA;
            }
            else
            {
                if (fixedBodyId < 0)
                {
                    fixedBodyId = solverBodyCount;
                    SolverBody fixedBody = ExpandSolverBody();
                    InitSolverBody(fixedBody, null, timeStep);
                }
                return fixedBodyId;
            }
        }

        return solverBodyIdA;
    }

    /// <summary>btSequentialImpulseConstraintSolver::setupContactConstraint</summary>
    private void SetupContactConstraint(SolverConstraint solverConstraint, int solverBodyIdA, int solverBodyIdB,
        ManifoldPoint cp, ContactSolverInfo infoGlobal, out btScalar relaxation,
        in Vector3 relPos1, in Vector3 relPos2)
    {
        SolverBody bodyA = tmpSolverBodyPool[solverBodyIdA];
        SolverBody bodyB = tmpSolverBodyPool[solverBodyIdB];

        RigidBody rb0 = bodyA.originalBody;
        RigidBody rb1 = bodyB.originalBody;

        relaxation = infoGlobal.sor;
        btScalar invTimeStep = 1 / infoGlobal.timeStep;

        // cfm = 1 / ( dt * kp + kd )
        // erp = dt * kp / ( dt * kp + kd )
        btScalar cfm = infoGlobal.globalCfm;
        btScalar erp = infoGlobal.erp2;

        if ((cp.contactPointFlags & ContactPointFlags.HasContactCfm) != 0 ||
            (cp.contactPointFlags & ContactPointFlags.HasContactErp) != 0)
        {
            if ((cp.contactPointFlags & ContactPointFlags.HasContactCfm) != 0)
                cfm = cp.contactCfm;
            if ((cp.contactPointFlags & ContactPointFlags.HasContactErp) != 0)
                erp = cp.contactErp;
        }
        else
        {
            if ((cp.contactPointFlags & ContactPointFlags.ContactStiffnessDamping) != 0)
            {
                btScalar denom = infoGlobal.timeStep * cp.CombinedContactStiffness1 + cp.CombinedContactDamping1;
                if (denom < BtMath.SimdEpsilon)
                {
                    denom = BtMath.SimdEpsilon;
                }
                cfm = 1 / denom;
                erp = infoGlobal.timeStep * cp.CombinedContactStiffness1 / denom;
            }
        }

        cfm *= invTimeStep;

        Vector3 torqueAxis0 = relPos1.Cross(cp.normalWorldOnB);
        solverConstraint.angularComponentA = rb0 != null
            ? rb0.invInertiaTensorWorld * torqueAxis0 * rb0.angularFactor
            : Vector3.Zero;
        Vector3 torqueAxis1 = relPos2.Cross(cp.normalWorldOnB);
        solverConstraint.angularComponentB = rb1 != null
            ? rb1.invInertiaTensorWorld * -torqueAxis1 * rb1.angularFactor
            : Vector3.Zero;

        {
            Vector3 vec;
            btScalar denom0 = 0;
            btScalar denom1 = 0;
            if (rb0 != null)
            {
                vec = solverConstraint.angularComponentA.Cross(relPos1);
                denom0 = rb0.InvMass + cp.normalWorldOnB.Dot(vec);
            }
            if (rb1 != null)
            {
                vec = (-solverConstraint.angularComponentB).Cross(relPos2);
                denom1 = rb1.InvMass + cp.normalWorldOnB.Dot(vec);
            }

            btScalar denom = relaxation / (denom0 + denom1 + cfm);
            solverConstraint.jacDiagABInv = denom;
            solverConstraint.jacDiagAB = 1 / denom;
        }

        if (rb0 != null)
        {
            solverConstraint.contactNormal1 = cp.normalWorldOnB;
            solverConstraint.relpos1CrossNormal = torqueAxis0;
        }
        else
        {
            solverConstraint.contactNormal1 = Vector3.Zero;
            solverConstraint.relpos1CrossNormal = Vector3.Zero;
        }
        if (rb1 != null)
        {
            solverConstraint.contactNormal2 = -cp.normalWorldOnB;
            solverConstraint.relpos2CrossNormal = -torqueAxis1;
        }
        else
        {
            solverConstraint.contactNormal2 = Vector3.Zero;
            solverConstraint.relpos2CrossNormal = Vector3.Zero;
        }

        btScalar restitution = 0;
        btScalar penetration = cp.Distance + infoGlobal.linearSlop;

        {
            solverConstraint.friction = cp.combinedFriction;

            if (cp.combinedRestitution == 0)
            {
                // 零恢复系数：局部相对速度（含叉乘/点积）无需计算
                restitution = 0;
            }
            else
            {
                Vector3 rel1 = relPos1;
                Vector3 rel2 = relPos2;
                Vector3 vel1 = rb0 != null ? rb0.GetVelocityInLocalPoint(ref rel1) : Vector3.Zero;
                Vector3 vel2 = rb1 != null ? rb1.GetVelocityInLocalPoint(ref rel2) : Vector3.Zero;

                Vector3 vel = vel1 - vel2;
                btScalar relVel = cp.normalWorldOnB.Dot(vel);

                restitution = RestitutionCurve(relVel, cp.combinedRestitution, RestitutionVelocityThresholdDefault);
                if (restitution <= 0)
                {
                    restitution = 0;
                }
            }
        }

        // warm starting（或禁用则置零）
        if ((infoGlobal.solverMode & SolverModes.UseWarmStarting) != 0)
        {
            solverConstraint.appliedImpulse = cp.appliedImpulse * infoGlobal.warmstartingFactor;
            if (rb0 != null)
                bodyA.InternalApplyImpulse(solverConstraint.contactNormal1 * bodyA.invMass, solverConstraint.angularComponentA, solverConstraint.appliedImpulse);
            if (rb1 != null)
                bodyB.InternalApplyImpulse(-solverConstraint.contactNormal2 * bodyB.invMass, -solverConstraint.angularComponentB, -solverConstraint.appliedImpulse);
        }
        else
        {
            solverConstraint.appliedImpulse = 0;
        }

        solverConstraint.appliedPushImpulse = 0;

        {
            Vector3 externalForceImpulseA = bodyA.originalBody != null ? bodyA.externalForceImpulse : Vector3.Zero;
            Vector3 externalTorqueImpulseA = bodyA.originalBody != null ? bodyA.externalTorqueImpulse : Vector3.Zero;
            Vector3 externalForceImpulseB = bodyB.originalBody != null ? bodyB.externalForceImpulse : Vector3.Zero;
            Vector3 externalTorqueImpulseB = bodyB.originalBody != null ? bodyB.externalTorqueImpulse : Vector3.Zero;

            btScalar vel1Dotn = solverConstraint.contactNormal1.Dot(bodyA.linearVelocity + externalForceImpulseA)
                + solverConstraint.relpos1CrossNormal.Dot(bodyA.angularVelocity + externalTorqueImpulseA);
            btScalar vel2Dotn = solverConstraint.contactNormal2.Dot(bodyB.linearVelocity + externalForceImpulseB)
                + solverConstraint.relpos2CrossNormal.Dot(bodyB.angularVelocity + externalTorqueImpulseB);
            btScalar relVel = vel1Dotn + vel2Dotn;

            btScalar positionalError = 0;
            btScalar velocityError = restitution - relVel; // * damping

            if (penetration > 0)
            {
                positionalError = 0;
                velocityError -= penetration * invTimeStep;
            }
            else
            {
                positionalError = -penetration * erp * invTimeStep;
            }

            btScalar penetrationImpulse = positionalError * solverConstraint.jacDiagABInv;
            btScalar velocityImpulse = velocityError * solverConstraint.jacDiagABInv;

            if (!infoGlobal.splitImpulse || penetration > infoGlobal.splitImpulsePenetrationThreshold)
            {
                // 位置与速度合并进 rhs
                solverConstraint.rhs = penetrationImpulse + velocityImpulse;
                solverConstraint.rhsPenetration = 0;
            }
            else
            {
                // 位置与速度拆分到 rhs 与 rhsPenetration
                solverConstraint.rhs = velocityImpulse;
                solverConstraint.rhsPenetration = penetrationImpulse;
                if (penetrationImpulse != 0)
                {
                    hasSplitImpulsePenetration = true;
                }
            }
            solverConstraint.cfm = cfm * solverConstraint.jacDiagABInv;
            solverConstraint.lowerLimit = 0;
            solverConstraint.upperLimit = (btScalar)1e10f;
        }
    }

    /// <summary>btSequentialImpulseConstraintSolver::setFrictionConstraintImpulse</summary>
    private void SetFrictionConstraintImpulse(SolverConstraint solverConstraint, int solverBodyIdA, int solverBodyIdB,
        ManifoldPoint cp, ContactSolverInfo infoGlobal)
    {
        {
            SolverConstraint frictionConstraint1 = tmpSolverContactFrictionConstraintPool[solverConstraint.frictionIndex];
            frictionConstraint1.appliedImpulse = 0;
        }

        if ((infoGlobal.solverMode & SolverModes.Use2FrictionDirections) != 0)
        {
            SolverConstraint frictionConstraint2 = tmpSolverContactFrictionConstraintPool[solverConstraint.frictionIndex + 1];
            frictionConstraint2.appliedImpulse = 0;
        }
    }

    /// <summary>btSequentialImpulseConstraintSolver::convertContact</summary>
    private void ConvertContact(PersistentManifold manifold, ContactSolverInfo infoGlobal)
    {
        CollisionObject colObj0 = manifold.body0;
        CollisionObject colObj1 = manifold.body1;

        int solverBodyIdA = GetOrInitSolverBody(colObj0, infoGlobal.timeStep);
        int solverBodyIdB = GetOrInitSolverBody(colObj1, infoGlobal.timeStep);

        SolverBody solverBodyA = tmpSolverBodyPool[solverBodyIdA];
        SolverBody solverBodyB = tmpSolverBodyPool[solverBodyIdB];

        // 避免两个静态对象之间的碰撞响应
        if (solverBodyA == null || (solverBodyA.invMass.FuzzyZero() && (solverBodyB == null || solverBodyB.invMass.FuzzyZero())))
            return;

        int rollingFriction = 1;
        for (int j = 0; j < manifold.NumContacts; j++)
        {
            ManifoldPoint cp = manifold.GetContactPoint(j);

            if (cp.Distance <= manifold.ContactProcessingThreshold)
            {
                Vector3 relPos1;
                Vector3 relPos2;

                int frictionIndex = contactConstraintCount;
                SolverConstraint solverConstraint = ExpandConstraint(tmpSolverContactConstraintPool, ref contactConstraintCount);
                solverConstraint.solverBodyIdA = solverBodyIdA;
                solverConstraint.solverBodyIdB = solverBodyIdB;

                solverConstraint.originalContactPoint = cp;

                Vector3 pos1 = cp.positionWorldOnA;
                Vector3 pos2 = cp.positionWorldOnB;

                relPos1 = pos1 - colObj0.worldTransform.Origin;
                relPos2 = pos2 - colObj1.worldTransform.Origin;

                solverBodyA.GetVelocityInLocalPointNoDelta(in relPos1, out Vector3 vel1);
                solverBodyB.GetVelocityInLocalPointNoDelta(in relPos2, out Vector3 vel2);

                Vector3 vel = vel1 - vel2;
                btScalar relVel = cp.normalWorldOnB.Dot(vel);

                SetupContactConstraint(solverConstraint, solverBodyIdA, solverBodyIdB, cp, infoGlobal,
                    out btScalar relaxation, in relPos1, in relPos2);

                ///// 设置摩擦约束
                solverConstraint.frictionIndex = contactFrictionConstraintCount;

                if (cp.combinedRollingFriction > 0 && rollingFriction > 0)
                {
                    {
                        AddTorsionalFrictionConstraint(cp.normalWorldOnB, solverBodyIdA, solverBodyIdB, frictionIndex,
                            cp, cp.combinedSpinningFriction, relPos1, relPos2, colObj0, colObj1, relaxation);
                        BtMath.PlaneSpace1(in cp.normalWorldOnB, out Vector3 axis0, out Vector3 axis1);
                        axis0.Normalize();
                        axis1.Normalize();

                        ApplyAnisotropicFriction(colObj0, ref axis0, AnisotropicFrictionFlags.AnisotropicRollingFriction);
                        ApplyAnisotropicFriction(colObj1, ref axis0, AnisotropicFrictionFlags.AnisotropicRollingFriction);
                        ApplyAnisotropicFriction(colObj0, ref axis1, AnisotropicFrictionFlags.AnisotropicRollingFriction);
                        ApplyAnisotropicFriction(colObj1, ref axis1, AnisotropicFrictionFlags.AnisotropicRollingFriction);
                        if (axis0.Length() > (btScalar)0.001)
                            AddTorsionalFrictionConstraint(axis0, solverBodyIdA, solverBodyIdB, frictionIndex, cp,
                                cp.combinedRollingFriction, relPos1, relPos2, colObj0, colObj1, relaxation);
                        if (axis1.Length() > (btScalar)0.001)
                            AddTorsionalFrictionConstraint(axis1, solverBodyIdA, solverBodyIdB, frictionIndex, cp,
                                cp.combinedRollingFriction, relPos1, relPos2, colObj0, colObj1, relaxation);
                    }
                }

                // Bullet 提供多种摩擦方向设置选项（见 C++ 注释）：
                // 默认基于相对线速度每帧重算单一摩擦方向；SOLVER_USE_2_FRICTION_DIRECTIONS 启用双摩擦方向；
                // SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION 与相对速度无关；
                // 用户可通过 contact callback 手动指定（BT_CONTACT_FLAG_LATERAL_FRICTION_INITIALIZED）。

                if ((infoGlobal.solverMode & SolverModes.EnableFrictionDirectionCaching) == 0 ||
                    (cp.contactPointFlags & ContactPointFlags.LateralFrictionInitialized) == 0)
                {
                    cp.lateralFrictionDir1 = vel - cp.normalWorldOnB * relVel;
                    btScalar latRelVel = cp.lateralFrictionDir1.LengthSquared();
                    if ((infoGlobal.solverMode & SolverModes.DisableVelocityDependentFrictionDirection) == 0 &&
                        latRelVel > BtMath.SimdEpsilon)
                    {
                        cp.lateralFrictionDir1 *= 1 / BtMath.Sqrt(latRelVel);
                        Vector3 dir1 = cp.lateralFrictionDir1;
                        ApplyAnisotropicFriction(colObj0, ref dir1, AnisotropicFrictionFlags.AnisotropicFriction);
                        ApplyAnisotropicFriction(colObj1, ref dir1, AnisotropicFrictionFlags.AnisotropicFriction);
                        cp.lateralFrictionDir1 = dir1;
                        AddFrictionConstraint(cp.lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex,
                            cp, relPos1, relPos2, colObj0, colObj1, relaxation, infoGlobal);

                        if ((infoGlobal.solverMode & SolverModes.Use2FrictionDirections) != 0)
                        {
                            Vector3 dir2 = cp.lateralFrictionDir1.Cross(cp.normalWorldOnB);
                            dir2.Normalize();
                            ApplyAnisotropicFriction(colObj0, ref dir2, AnisotropicFrictionFlags.AnisotropicFriction);
                            ApplyAnisotropicFriction(colObj1, ref dir2, AnisotropicFrictionFlags.AnisotropicFriction);
                            cp.lateralFrictionDir2 = dir2;
                            AddFrictionConstraint(cp.lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex,
                                cp, relPos1, relPos2, colObj0, colObj1, relaxation, infoGlobal);
                        }
                    }
                    else
                    {
                        BtMath.PlaneSpace1(in cp.normalWorldOnB, out Vector3 dir1, out Vector3 dir2);

                        ApplyAnisotropicFriction(colObj0, ref dir1, AnisotropicFrictionFlags.AnisotropicFriction);
                        ApplyAnisotropicFriction(colObj1, ref dir1, AnisotropicFrictionFlags.AnisotropicFriction);
                        cp.lateralFrictionDir1 = dir1;
                        AddFrictionConstraint(cp.lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex,
                            cp, relPos1, relPos2, colObj0, colObj1, relaxation, infoGlobal);

                        if ((infoGlobal.solverMode & SolverModes.Use2FrictionDirections) != 0)
                        {
                            ApplyAnisotropicFriction(colObj0, ref dir2, AnisotropicFrictionFlags.AnisotropicFriction);
                            ApplyAnisotropicFriction(colObj1, ref dir2, AnisotropicFrictionFlags.AnisotropicFriction);
                            cp.lateralFrictionDir2 = dir2;
                            AddFrictionConstraint(cp.lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex,
                                cp, relPos1, relPos2, colObj0, colObj1, relaxation, infoGlobal);
                        }

                        if ((infoGlobal.solverMode & SolverModes.Use2FrictionDirections) != 0 &&
                            (infoGlobal.solverMode & SolverModes.DisableVelocityDependentFrictionDirection) != 0)
                        {
                            cp.contactPointFlags |= ContactPointFlags.LateralFrictionInitialized;
                        }
                    }
                }
                else
                {
                    AddFrictionConstraint(cp.lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex,
                        cp, relPos1, relPos2, colObj0, colObj1, relaxation, infoGlobal, cp.contactMotion1, cp.frictionCfm);

                    if ((infoGlobal.solverMode & SolverModes.Use2FrictionDirections) != 0)
                        AddFrictionConstraint(cp.lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex,
                            cp, relPos1, relPos2, colObj0, colObj1, relaxation, infoGlobal, cp.contactMotion2, cp.frictionCfm);
                }
                SetFrictionConstraintImpulse(solverConstraint, solverBodyIdA, solverBodyIdB, cp, infoGlobal);
            }
        }
    }

    /// <summary>btSequentialImpulseConstraintSolver::convertContacts</summary>
    private void ConvertContacts(List<PersistentManifold> manifoldPtr, int numManifolds, ContactSolverInfo infoGlobal)
    {
        for (int i = 0; i < numManifolds; i++)
        {
            PersistentManifold manifold = manifoldPtr[i];
            ConvertContact(manifold, infoGlobal);
        }
    }

    /// <summary>btSequentialImpulseConstraintSolver::convertJoint
    /// （C++ 通过 rowskip=sizeof(btSolverConstraint) 的指针别名直接写 SolverConstraint 字段；
    /// C# 改为按 rowskip=3 的临时数组传给 GetInfo2，调用后拷回 SolverConstraint）</summary>
    private void ConvertJoint(int currentRow, TypedConstraint constraint, ConstraintInfo1 info1,
        int solverBodyIdA, int solverBodyIdB, ContactSolverInfo infoGlobal)
    {
        RigidBody rbA = constraint.rigidBodyA;
        RigidBody rbB = constraint.rigidBodyB;

        SolverBody bodyAPtr = tmpSolverBodyPool[solverBodyIdA];
        SolverBody bodyBPtr = tmpSolverBodyPool[solverBodyIdB];

        int overrideNumSolverIterations = constraint.OverrideNumSolverIterations > 0
            ? constraint.OverrideNumSolverIterations
            : infoGlobal.numIterations;
        if (overrideNumSolverIterations > maxOverrideNumSolverIterations)
            maxOverrideNumSolverIterations = overrideNumSolverIterations;

        int numRows = info1.numConstraintRows;
        for (int j = 0; j < numRows; j++)
        {
            SolverConstraint row = tmpSolverNonContactConstraintPool[currentRow + j];
            // C++ memset(&row, 0, sizeof(btSolverConstraint))
            row.relpos1CrossNormal = Vector3.Zero;
            row.contactNormal1 = Vector3.Zero;
            row.relpos2CrossNormal = Vector3.Zero;
            row.contactNormal2 = Vector3.Zero;
            row.angularComponentA = Vector3.Zero;
            row.angularComponentB = Vector3.Zero;
            row.friction = 0;
            row.jacDiagABInv = 0;
            row.jacDiagAB = 1 / row.jacDiagABInv;
            row.rhs = 0;
            row.cfm = 0;
            row.rhsPenetration = 0;
            row.originalContactPoint = null;
            row.numRowsForNonContactConstraint = 0;
            row.frictionIndex = 0;
            row.lowerLimit = -BtMath.SimdInfinity;
            row.upperLimit = BtMath.SimdInfinity;
            row.appliedImpulse = 0;
            row.appliedPushImpulse = 0;
            row.solverBodyIdA = solverBodyIdA;
            row.solverBodyIdB = solverBodyIdB;
            row.overrideNumSolverIterations = overrideNumSolverIterations;
        }

        // 复用 ConstraintInfo2 与临时数组（C++ 中为栈对象/临时缓冲）；
        // 注意约束代码依赖未写行为零（C++ memset），复用数组必须按使用量清零
        jointInfo2 ??= new ConstraintInfo2();
        var info2 = jointInfo2;
        info2.fps = 1 / infoGlobal.timeStep;
        info2.erp = infoGlobal.erp;
        info2.rowskip = 3; // Vector3 为 3 个 btScalar；J 行 r 分量 s 的索引 = r * rowskip + s

        int arrSize = numRows * 3;
        var j1Linear = EnsureScalarArray(ref j1LinearPool, arrSize);
        var j1Angular = EnsureScalarArray(ref j1AngularPool, arrSize);
        var j2Linear = EnsureScalarArray(ref j2LinearPool, arrSize);
        var j2Angular = EnsureScalarArray(ref j2AngularPool, arrSize);
        // C++ 中 info2 的 constraintError/cfm/lowerLimit/upperLimit 指向 btSolverConstraint
        // 结构体字段，与 J 轴数组一样按 rowskip 步进（约束代码以 [row*rowskip] 写入），
        // 因此这 4 个数组同样按 numRows*rowskip 分配，回读时按同一步长取值。
        var constraintError = EnsureScalarArray(ref constraintErrorPool, arrSize);
        var cfm = EnsureScalarArray(ref cfmPool, arrSize);
        var lowerLimit = EnsureScalarArray(ref lowerLimitPool, arrSize);
        var upperLimit = EnsureScalarArray(ref upperLimitPool, arrSize);
        // C++ 中这些指针别名到 memset 后的 btSolverConstraint 字段，
        // 其 lower/upperLimit 被显式置为 ∓SIMD_INFINITY。
        // 约束（如 p2p 未设 impulseClamp 时）不写的行必须保持 ±INF 默认值，
        // 否则沿用 EnsureScalarArray 的零值会把冲量钳制为 0，约束完全失效。
        for (int i = 0; i < arrSize; i++)
        {
            lowerLimit[i] = -BtMath.SimdInfinity;
            upperLimit[i] = BtMath.SimdInfinity;
        }
        info2.j1LinearAxis = j1Linear;
        info2.j1AngularAxis = j1Angular;
        info2.j2LinearAxis = j2Linear;
        info2.j2AngularAxis = j2Angular;
        info2.constraintError = constraintError;
        // C++ 仅将首行 cfm 预设为 globalCfm（其余已被 memset 清零）
        cfm[0] = infoGlobal.globalCfm;
        info2.cfm = cfm;
        info2.lowerLimit = lowerLimit;
        info2.upperLimit = upperLimit;
        info2.damping = infoGlobal.damping;
        info2.numIterations = infoGlobal.numIterations;

        info2Body0Trans[0] = bodyAPtr.worldTransform;
        info2Body1Trans[0] = bodyBPtr.worldTransform;
        constraint.GetInfo2(info2, infoGlobal, info2Body0Trans, info2Body1Trans, 0, 0);

        // 将 GetInfo2 结果拷回 SolverConstraint
        for (int j = 0; j < numRows; j++)
        {
            SolverConstraint solverConstraint = tmpSolverNonContactConstraintPool[currentRow + j];
            int s = j * 3;
            solverConstraint.contactNormal1 = new Vector3(j1Linear[s], j1Linear[s + 1], j1Linear[s + 2]);
            solverConstraint.relpos1CrossNormal = new Vector3(j1Angular[s], j1Angular[s + 1], j1Angular[s + 2]);
            solverConstraint.contactNormal2 = new Vector3(j2Linear[s], j2Linear[s + 1], j2Linear[s + 2]);
            solverConstraint.relpos2CrossNormal = new Vector3(j2Angular[s], j2Angular[s + 1], j2Angular[s + 2]);
            solverConstraint.rhs = constraintError[s];
            solverConstraint.cfm = cfm[s];
            solverConstraint.lowerLimit = lowerLimit[s];
            solverConstraint.upperLimit = upperLimit[s];
        }

        // 完成约束设置的收尾工作
        for (int j = 0; j < numRows; j++)
        {
            SolverConstraint solverConstraint = tmpSolverNonContactConstraintPool[currentRow + j];

            if (solverConstraint.upperLimit >= constraint.BreakingImpulseThreshold)
            {
                solverConstraint.upperLimit = constraint.BreakingImpulseThreshold;
            }

            if (solverConstraint.lowerLimit <= -constraint.BreakingImpulseThreshold)
            {
                solverConstraint.lowerLimit = -constraint.BreakingImpulseThreshold;
            }

            solverConstraint.originalContactPoint = constraint;

            {
                Vector3 ftorqueAxis1 = solverConstraint.relpos1CrossNormal;
                solverConstraint.angularComponentA = rbA.invInertiaTensorWorld * ftorqueAxis1 * rbA.angularFactor;
            }
            {
                Vector3 ftorqueAxis2 = solverConstraint.relpos2CrossNormal;
                solverConstraint.angularComponentB = rbB.invInertiaTensorWorld * ftorqueAxis2 * rbB.angularFactor;
            }

            {
                Vector3 iMJlA = solverConstraint.contactNormal1 * rbA.InvMass;
                Vector3 iMJaA = rbA.invInertiaTensorWorld * solverConstraint.relpos1CrossNormal;
                Vector3 iMJlB = solverConstraint.contactNormal2 * rbB.InvMass; // 法线符号？
                Vector3 iMJaB = rbB.invInertiaTensorWorld * solverConstraint.relpos2CrossNormal;

                btScalar sum = iMJlA.Dot(solverConstraint.contactNormal1);
                sum += iMJaA.Dot(solverConstraint.relpos1CrossNormal);
                sum += iMJlB.Dot(solverConstraint.contactNormal2);
                sum += iMJaB.Dot(solverConstraint.relpos2CrossNormal);
                btScalar fsum = BtMath.Abs(sum);
                btScalar sorRelaxation = 1; // todo: 从 globalInfo 获取？
                solverConstraint.jacDiagABInv = fsum > BtMath.SimdEpsilon ? sorRelaxation / sum : 0;
                solverConstraint.jacDiagAB = 1 / solverConstraint.jacDiagABInv;
            }

            {
                Vector3 externalForceImpulseA = bodyAPtr.originalBody != null ? bodyAPtr.externalForceImpulse : Vector3.Zero;
                Vector3 externalTorqueImpulseA = bodyAPtr.originalBody != null ? bodyAPtr.externalTorqueImpulse : Vector3.Zero;

                Vector3 externalForceImpulseB = bodyBPtr.originalBody != null ? bodyBPtr.externalForceImpulse : Vector3.Zero;
                Vector3 externalTorqueImpulseB = bodyBPtr.originalBody != null ? bodyBPtr.externalTorqueImpulse : Vector3.Zero;

                btScalar vel1Dotn = solverConstraint.contactNormal1.Dot(rbA.linearVelocity + externalForceImpulseA)
                    + solverConstraint.relpos1CrossNormal.Dot(rbA.angularVelocity + externalTorqueImpulseA);

                btScalar vel2Dotn = solverConstraint.contactNormal2.Dot(rbB.linearVelocity + externalForceImpulseB)
                    + solverConstraint.relpos2CrossNormal.Dot(rbB.angularVelocity + externalTorqueImpulseB);

                btScalar relVel = vel1Dotn + vel2Dotn;
                btScalar restitution = 0;
                btScalar positionalError = solverConstraint.rhs; // 已由 getConstraintInfo2 填入
                btScalar velocityError = restitution - relVel * info2.damping;
                btScalar penetrationImpulse = positionalError * solverConstraint.jacDiagABInv;
                btScalar velocityImpulse = velocityError * solverConstraint.jacDiagABInv;
                solverConstraint.rhs = penetrationImpulse + velocityImpulse;
                solverConstraint.appliedImpulse = 0;
            }
        }
    }

    /// <summary>btSequentialImpulseConstraintSolver::convertJoints</summary>
    private void ConvertJoints(List<TypedConstraint> constraints, int numConstraints, ContactSolverInfo infoGlobal)
    {
        for (int j = 0; j < numConstraints; j++)
        {
            TypedConstraint constraint = constraints[j];
            constraint.BuildJacobian();
            constraint.InternalSetAppliedImpulse(0);
        }

        int totalNumRows = 0;

        // 计算约束行总数
        while (tmpConstraintSizesPool.Count < numConstraints)
            tmpConstraintSizesPool.Add(new ConstraintInfo1());

        for (int i = 0; i < numConstraints; i++)
        {
            ConstraintInfo1 info1 = tmpConstraintSizesPool[i];
            JointFeedback fb = constraints[i].JointFeedback;
            if (fb != null)
            {
                fb.appliedForceBodyA = Vector3.Zero;
                fb.appliedTorqueBodyA = Vector3.Zero;
                fb.appliedForceBodyB = Vector3.Zero;
                fb.appliedTorqueBodyB = Vector3.Zero;
            }

            if (constraints[i].IsEnabled)
            {
                constraints[i].GetInfo1(info1, infoGlobal);
            }
            else
            {
                info1.numConstraintRows = 0;
                info1.nub = 0;
            }
            totalNumRows += info1.numConstraintRows;
        }
        while (tmpSolverNonContactConstraintPool.Count < totalNumRows)
            tmpSolverNonContactConstraintPool.Add(new SolverConstraint());
        nonContactConstraintCount = totalNumRows;

        // 设置 btSolverConstraints
        int currentRow = 0;

        for (int i = 0; i < numConstraints; i++)
        {
            ConstraintInfo1 info1 = tmpConstraintSizesPool[i];

            if (info1.numConstraintRows != 0)
            {
                TypedConstraint constraint = constraints[i];
                RigidBody rbA = constraint.rigidBodyA;
                RigidBody rbB = constraint.rigidBodyB;

                int solverBodyIdA = GetOrInitSolverBody(rbA, infoGlobal.timeStep);
                int solverBodyIdB = GetOrInitSolverBody(rbB, infoGlobal.timeStep);

                ConvertJoint(currentRow, constraint, info1, solverBodyIdA, solverBodyIdB, infoGlobal);
            }
            currentRow += info1.numConstraintRows;
        }
    }

    /// <summary>btSequentialImpulseConstraintSolver::convertBodies</summary>
    private void ConvertBodies(List<CollisionObject> bodies, int numBodies, ContactSolverInfo infoGlobal)
    {
        for (int i = 0; i < numBodies; i++)
        {
            bodies[i].companionId = -1;
        }

        solverBodyCount = 0;

        for (int i = 0; i < numBodies; i++)
        {
            int bodyId = GetOrInitSolverBody(bodies[i], infoGlobal.timeStep);

            RigidBody body = RigidBody.Upcast(bodies[i]);
            if (body != null && body.InvMass != 0)
            {
                SolverBody solverBody = tmpSolverBodyPool[bodyId];
                Vector3 gyroForce = Vector3.Zero;
                if ((body.Flags & (int)RigidBodyFlags.EnableGyroscopicForceExplicit) != 0)
                {
                    gyroForce = body.ComputeGyroscopicForceExplicit(infoGlobal.maxGyroscopicForce);
                    solverBody.externalTorqueImpulse -= gyroForce * body.invInertiaTensorWorld * infoGlobal.timeStep;
                }
                if ((body.Flags & (int)RigidBodyFlags.EnableGyroscopicForceImplicitWorld) != 0)
                {
                    gyroForce = body.ComputeGyroscopicImpulseImplicitWorld(infoGlobal.timeStep);
                    solverBody.externalTorqueImpulse += gyroForce;
                }
                if ((body.Flags & (int)RigidBodyFlags.EnableGyroscopicForceImplicitBody) != 0)
                {
                    gyroForce = body.ComputeGyroscopicImpulseImplicitBody(infoGlobal.timeStep);
                    solverBody.externalTorqueImpulse += gyroForce;
                }
            }
        }
    }

    /// <summary>btSequentialImpulseConstraintSolver::solveGroupCacheFriendlySetup</summary>
    public override btScalar SolveGroupCacheFriendlySetup(List<CollisionObject> bodies, int numBodies,
        List<PersistentManifold> manifoldPtr, int numManifolds,
        List<TypedConstraint> constraints, int numConstraints,
        ContactSolverInfo infoGlobal, IDebugDraw debugDrawer)
    {
        fixedBodyId = -1;

        // C++ 在此处按 solverMode 切换 SIMD/标量行求解函数；本移植固定使用标量参考实现
        cachedSolverMode = (int)infoGlobal.solverMode;
        maxOverrideNumSolverIterations = 0;
        hasSplitImpulsePenetration = false;

        // 转换所有 bodies
        ConvertBodies(bodies, numBodies, infoGlobal);

        ConvertJoints(constraints, numConstraints, infoGlobal);

        contactConstraintCount = 0;
        contactFrictionConstraintCount = 0;
        contactRollingFrictionConstraintCount = 0;
        ConvertContacts(manifoldPtr, numManifolds, infoGlobal);

        int numNonContactPool = nonContactConstraintCount;
        int numConstraintPool = contactConstraintCount;
        int numFrictionPool = contactFrictionConstraintCount;

        // @todo: 此类临时内存应使用栈分配器，solver bodies/constraints 同理
        EnsureIntPool(ref orderNonContactConstraintPool, numNonContactPool);
        if ((infoGlobal.solverMode & SolverModes.Use2FrictionDirections) != 0)
            EnsureIntPool(ref orderTmpConstraintPool, numConstraintPool * 2);
        else
            EnsureIntPool(ref orderTmpConstraintPool, numConstraintPool);

        EnsureIntPool(ref orderFrictionConstraintPool, numFrictionPool);
        {
            int i;
            for (i = 0; i < numNonContactPool; i++)
            {
                orderNonContactConstraintPool[i] = i;
            }
            for (i = 0; i < numConstraintPool; i++)
            {
                orderTmpConstraintPool[i] = i;
            }
            for (i = 0; i < numFrictionPool; i++)
            {
                orderFrictionConstraintPool[i] = i;
            }
        }

        return 0;
    }

    /// <summary>btSequentialImpulseConstraintSolver::solveSingleIteration</summary>
    private btScalar SolveSingleIteration(int iteration, List<CollisionObject> bodies, int numBodies,
        List<PersistentManifold> manifoldPtr, int numManifolds,
        List<TypedConstraint> constraints, int numConstraints,
        ContactSolverInfo infoGlobal, IDebugDraw debugDrawer)
    {
        btScalar leastSquaresResidual = 0;

        int numNonContactPool = nonContactConstraintCount;
        int numConstraintPool = contactConstraintCount;
        int numFrictionPool = contactFrictionConstraintCount;

        if ((infoGlobal.solverMode & SolverModes.RandomizeOrder) != 0)
        {
            {
                for (int j = 0; j < numNonContactPool; ++j)
                {
                    int tmp = orderNonContactConstraintPool[j];
                    int swapi = BtRandInt2(j + 1);
                    orderNonContactConstraintPool[j] = orderNonContactConstraintPool[swapi];
                    orderNonContactConstraintPool[swapi] = tmp;
                }

                // contact/friction 约束的求解次数不超过 numIterations
                if (iteration < infoGlobal.numIterations)
                {
                    for (int j = 0; j < numConstraintPool; ++j)
                    {
                        int tmp = orderTmpConstraintPool[j];
                        int swapi = BtRandInt2(j + 1);
                        orderTmpConstraintPool[j] = orderTmpConstraintPool[swapi];
                        orderTmpConstraintPool[swapi] = tmp;
                    }

                    for (int j = 0; j < numFrictionPool; ++j)
                    {
                        int tmp = orderFrictionConstraintPool[j];
                        int swapi = BtRandInt2(j + 1);
                        orderFrictionConstraintPool[j] = orderFrictionConstraintPool[swapi];
                        orderFrictionConstraintPool[swapi] = tmp;
                    }
                }
            }
        }

        // 求解所有 joint 约束
        {
            var nonContactPool = CollectionsMarshal.AsSpan(tmpSolverNonContactConstraintPool);
            var solverBodies = CollectionsMarshal.AsSpan(tmpSolverBodyPool);
            var orderNonContact = orderNonContactConstraintPool.AsSpan();
            for (int j = 0; j < numNonContactPool; j++)
            {
                SolverConstraint constraint = nonContactPool[orderNonContact[j]];
                if (iteration < constraint.overrideNumSolverIterations)
                {
                    btScalar residual = ContactSolverStatics.ResolveSingleConstraintRowGenericScalarReference(
                        solverBodies[constraint.solverBodyIdA], solverBodies[constraint.solverBodyIdB], constraint);
                    leastSquaresResidual = BtMath.Max(leastSquaresResidual, residual * residual);
                }
            }
        }

        if (iteration < infoGlobal.numIterations)
        {
            for (int j = 0; j < numConstraints; j++)
            {
                if (constraints[j].IsEnabled && constraints[j] is ISolveConstraintObsolete obsolete)
                {
                    int bodyAid = GetOrInitSolverBody(constraints[j].rigidBodyA, infoGlobal.timeStep);
                    int bodyBid = GetOrInitSolverBody(constraints[j].rigidBodyB, infoGlobal.timeStep);
                    SolverBody bodyA = tmpSolverBodyPool[bodyAid];
                    SolverBody bodyB = tmpSolverBodyPool[bodyBid];
                    obsolete.SolveConstraintObsolete(bodyA, bodyB, infoGlobal.timeStep);
                }
            }

            // 求解所有 contact 约束
            if ((infoGlobal.solverMode & SolverModes.InterleaveContactAndFriction) != 0)
            {
                int numPoolConstraints = contactConstraintCount;
                int multiplier = (infoGlobal.solverMode & SolverModes.Use2FrictionDirections) != 0 ? 2 : 1;
                var contactPool = CollectionsMarshal.AsSpan(tmpSolverContactConstraintPool);
                var frictionPool = CollectionsMarshal.AsSpan(tmpSolverContactFrictionConstraintPool);
                var solverBodies = CollectionsMarshal.AsSpan(tmpSolverBodyPool);
                var orderTmp = orderTmpConstraintPool.AsSpan();
                var orderFriction = orderFrictionConstraintPool.AsSpan();

                for (int c = 0; c < numPoolConstraints; c++)
                {
                    btScalar totalImpulse;

                    {
                        SolverConstraint solveManifold = contactPool[orderTmp[c]];
                        btScalar residual = ContactSolverStatics.ResolveSingleConstraintRowLowerLimitScalarReference(
                            solverBodies[solveManifold.solverBodyIdA], solverBodies[solveManifold.solverBodyIdB], solveManifold);
                        leastSquaresResidual = BtMath.Max(leastSquaresResidual, residual * residual);

                        totalImpulse = solveManifold.appliedImpulse;
                    }
                    bool applyFriction = true;
                    if (applyFriction)
                    {
                        {
                            SolverConstraint solveManifold = frictionPool[orderFriction[c * multiplier]];

                            if (totalImpulse > 0)
                            {
                                solveManifold.lowerLimit = -(solveManifold.friction * totalImpulse);
                                solveManifold.upperLimit = solveManifold.friction * totalImpulse;

                                btScalar residual = ContactSolverStatics.ResolveSingleConstraintRowGenericScalarReference(
                                    solverBodies[solveManifold.solverBodyIdA], solverBodies[solveManifold.solverBodyIdB], solveManifold);
                                leastSquaresResidual = BtMath.Max(leastSquaresResidual, residual * residual);
                            }
                        }

                        if ((infoGlobal.solverMode & SolverModes.Use2FrictionDirections) != 0)
                        {
                            SolverConstraint solveManifold = frictionPool[orderFriction[c * multiplier + 1]];

                            if (totalImpulse > 0)
                            {
                                solveManifold.lowerLimit = -(solveManifold.friction * totalImpulse);
                                solveManifold.upperLimit = solveManifold.friction * totalImpulse;

                                btScalar residual = ContactSolverStatics.ResolveSingleConstraintRowGenericScalarReference(
                                    solverBodies[solveManifold.solverBodyIdA], solverBodies[solveManifold.solverBodyIdB], solveManifold);
                                leastSquaresResidual = BtMath.Max(leastSquaresResidual, residual * residual);
                            }
                        }
                    }
                }
            }
            else // SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS
            {
                // 先求解全部 contact 约束，再求解摩擦约束（不交错）
                int numPoolConstraints = contactConstraintCount;
                int j;
                var contactPool = CollectionsMarshal.AsSpan(tmpSolverContactConstraintPool);
                var frictionPool = CollectionsMarshal.AsSpan(tmpSolverContactFrictionConstraintPool);
                var solverBodies = CollectionsMarshal.AsSpan(tmpSolverBodyPool);
                var orderTmp = orderTmpConstraintPool.AsSpan();
                var orderFriction = orderFrictionConstraintPool.AsSpan();

                for (j = 0; j < numPoolConstraints; j++)
                {
                    SolverConstraint solveManifold = contactPool[orderTmp[j]];
                    btScalar residual = ContactSolverStatics.ResolveSingleConstraintRowLowerLimitScalarReference(
                        solverBodies[solveManifold.solverBodyIdA], solverBodies[solveManifold.solverBodyIdB], solveManifold);
                    leastSquaresResidual = BtMath.Max(leastSquaresResidual, residual * residual);
                }

                // 求解所有摩擦约束
                int numFrictionPoolConstraints = contactFrictionConstraintCount;
                for (j = 0; j < numFrictionPoolConstraints; j++)
                {
                    SolverConstraint solveManifold = frictionPool[orderFriction[j]];
                    btScalar totalImpulse = contactPool[solveManifold.frictionIndex].appliedImpulse;

                    if (totalImpulse > 0)
                    {
                        solveManifold.lowerLimit = -(solveManifold.friction * totalImpulse);
                        solveManifold.upperLimit = solveManifold.friction * totalImpulse;

                        btScalar residual = ContactSolverStatics.ResolveSingleConstraintRowGenericScalarReference(
                            solverBodies[solveManifold.solverBodyIdA], solverBodies[solveManifold.solverBodyIdB], solveManifold);
                        leastSquaresResidual = BtMath.Max(leastSquaresResidual, residual * residual);
                    }
                }
            }

            int numRollingFrictionPoolConstraints = contactRollingFrictionConstraintCount;
            var rollingFrictionPool = CollectionsMarshal.AsSpan(tmpSolverContactRollingFrictionConstraintPool);
            var contactPool2 = CollectionsMarshal.AsSpan(tmpSolverContactConstraintPool);
            var solverBodies2 = CollectionsMarshal.AsSpan(tmpSolverBodyPool);
            for (int j = 0; j < numRollingFrictionPoolConstraints; j++)
            {
                SolverConstraint rollingFrictionConstraint = rollingFrictionPool[j];
                btScalar totalImpulse = contactPool2[rollingFrictionConstraint.frictionIndex].appliedImpulse;
                if (totalImpulse > 0)
                {
                    btScalar rollingFrictionMagnitude = rollingFrictionConstraint.friction * totalImpulse;
                    if (rollingFrictionMagnitude > rollingFrictionConstraint.friction)
                        rollingFrictionMagnitude = rollingFrictionConstraint.friction;

                    rollingFrictionConstraint.lowerLimit = -rollingFrictionMagnitude;
                    rollingFrictionConstraint.upperLimit = rollingFrictionMagnitude;

                    btScalar residual = ContactSolverStatics.ResolveSingleConstraintRowGenericScalarReference(
                        solverBodies2[rollingFrictionConstraint.solverBodyIdA],
                        solverBodies2[rollingFrictionConstraint.solverBodyIdB], rollingFrictionConstraint);
                    leastSquaresResidual = BtMath.Max(leastSquaresResidual, residual * residual);
                }
            }
        }
        return leastSquaresResidual;
    }

    /// <summary>btSequentialImpulseConstraintSolver::solveGroupCacheFriendlySplitImpulseIterations</summary>
    private void SolveGroupCacheFriendlySplitImpulseIterations(List<CollisionObject> bodies, int numBodies,
        List<PersistentManifold> manifoldPtr, int numManifolds,
        List<TypedConstraint> constraints, int numConstraints,
        ContactSolverInfo infoGlobal, IDebugDraw debugDrawer)
    {
        if (infoGlobal.splitImpulse && hasSplitImpulsePenetration)
        {
            int iteration;
            for (iteration = 0; iteration < infoGlobal.numIterations; iteration++)
            {
                btScalar splitResidual = 0;
                {
                    int numPoolConstraints = contactConstraintCount;
                    var contactPool = CollectionsMarshal.AsSpan(tmpSolverContactConstraintPool);
                    var solverBodies = CollectionsMarshal.AsSpan(tmpSolverBodyPool);
                    var orderTmp = orderTmpConstraintPool.AsSpan();
                    for (int j = 0; j < numPoolConstraints; j++)
                    {
                        SolverConstraint solveManifold = contactPool[orderTmp[j]];

                        btScalar residual = ContactSolverStatics.ResolveSplitPenetrationImpulseScalarReference(
                            solverBodies[solveManifold.solverBodyIdA], solverBodies[solveManifold.solverBodyIdB], solveManifold);
                        splitResidual = BtMath.Max(splitResidual, residual * residual);
                    }
                }
                if (splitResidual <= infoGlobal.leastSquaresResidualThreshold || iteration >= infoGlobal.numIterations - 1)
                {
                    break;
                }
            }
        }
    }

    /// <summary>btSequentialImpulseConstraintSolver::solveGroupCacheFriendlyIterations</summary>
    public override btScalar SolveGroupCacheFriendlyIterations(List<CollisionObject> bodies, int numBodies,
        List<PersistentManifold> manifoldPtr, int numManifolds,
        List<TypedConstraint> constraints, int numConstraints,
        ContactSolverInfo infoGlobal, IDebugDraw debugDrawer)
    {
        {
            // 解决穿透的特殊步骤（仅 contacts）
            SolveGroupCacheFriendlySplitImpulseIterations(bodies, numBodies, manifoldPtr, numManifolds,
                constraints, numConstraints, infoGlobal, debugDrawer);

            int maxIterations = maxOverrideNumSolverIterations > infoGlobal.numIterations
                ? maxOverrideNumSolverIterations
                : infoGlobal.numIterations;

            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                leastSquaresResidual = SolveSingleIteration(iteration, bodies, numBodies, manifoldPtr, numManifolds,
                    constraints, numConstraints, infoGlobal, debugDrawer);

                if (leastSquaresResidual <= infoGlobal.leastSquaresResidualThreshold || iteration >= maxIterations - 1)
                {
                    break;
                }
            }
        }
        return 0;
    }

    /// <summary>btSequentialImpulseConstraintSolver::writeBackContacts</summary>
    private void WriteBackContacts(int iBegin, int iEnd, ContactSolverInfo infoGlobal)
    {
        for (int j = iBegin; j < iEnd; j++)
        {
            SolverConstraint solveManifold = tmpSolverContactConstraintPool[j];
            var pt = (ManifoldPoint)solveManifold.originalContactPoint;
            pt.appliedImpulse = solveManifold.appliedImpulse;
            pt.appliedImpulseLateral1 = tmpSolverContactFrictionConstraintPool[solveManifold.frictionIndex].appliedImpulse;
            if ((infoGlobal.solverMode & SolverModes.Use2FrictionDirections) != 0)
            {
                pt.appliedImpulseLateral2 = tmpSolverContactFrictionConstraintPool[solveManifold.frictionIndex + 1].appliedImpulse;
            }
            // 此处可添加回调
        }
    }

    /// <summary>btSequentialImpulseConstraintSolver::writeBackJoints</summary>
    private void WriteBackJoints(int iBegin, int iEnd, ContactSolverInfo infoGlobal)
    {
        for (int j = iBegin; j < iEnd; j++)
        {
            SolverConstraint solverConstr = tmpSolverNonContactConstraintPool[j];
            var constr = (TypedConstraint)solverConstr.originalContactPoint;
            JointFeedback fb = constr.JointFeedback;
            if (fb != null)
            {
                fb.appliedForceBodyA += solverConstr.contactNormal1 * solverConstr.appliedImpulse * constr.rigidBodyA.linearFactor / infoGlobal.timeStep;
                fb.appliedForceBodyB += solverConstr.contactNormal2 * solverConstr.appliedImpulse * constr.rigidBodyB.linearFactor / infoGlobal.timeStep;
                fb.appliedTorqueBodyA += solverConstr.relpos1CrossNormal * constr.rigidBodyA.angularFactor * solverConstr.appliedImpulse / infoGlobal.timeStep;
                fb.appliedTorqueBodyB += solverConstr.relpos2CrossNormal * constr.rigidBodyB.angularFactor * solverConstr.appliedImpulse / infoGlobal.timeStep;
            }

            constr.InternalSetAppliedImpulse(solverConstr.appliedImpulse);
            if (BtMath.Abs(solverConstr.appliedImpulse) >= constr.BreakingImpulseThreshold)
            {
                constr.IsEnabled = false;
            }
        }
    }

    /// <summary>btSequentialImpulseConstraintSolver::writeBackBodies</summary>
    private void WriteBackBodies(int iBegin, int iEnd, ContactSolverInfo infoGlobal)
    {
        for (int i = iBegin; i < iEnd; i++)
        {
            RigidBody body = tmpSolverBodyPool[i].originalBody;
            if (body != null)
            {
                if (infoGlobal.splitImpulse)
                    tmpSolverBodyPool[i].WritebackVelocityAndTransform(infoGlobal.timeStep, infoGlobal.splitImpulseTurnErp);
                else
                    tmpSolverBodyPool[i].WritebackVelocity();

                tmpSolverBodyPool[i].originalBody.linearVelocity =
                    tmpSolverBodyPool[i].linearVelocity + tmpSolverBodyPool[i].externalForceImpulse;

                tmpSolverBodyPool[i].originalBody.angularVelocity =
                    tmpSolverBodyPool[i].angularVelocity + tmpSolverBodyPool[i].externalTorqueImpulse;

                if (infoGlobal.splitImpulse)
                    tmpSolverBodyPool[i].originalBody.WorldTransformInternal = tmpSolverBodyPool[i].worldTransform;

                tmpSolverBodyPool[i].originalBody.companionId = -1;
            }
        }
    }

    /// <summary>btSequentialImpulseConstraintSolver::solveGroupCacheFriendlyFinish</summary>
    public override btScalar SolveGroupCacheFriendlyFinish(List<CollisionObject> bodies, int numBodies, ContactSolverInfo infoGlobal)
    {
        if ((infoGlobal.solverMode & SolverModes.UseWarmStarting) != 0)
        {
            WriteBackContacts(0, contactConstraintCount, infoGlobal);
        }

        WriteBackJoints(0, nonContactConstraintCount, infoGlobal);
        WriteBackBodies(0, solverBodyCount, infoGlobal);

        contactConstraintCount = 0;
        nonContactConstraintCount = 0;
        contactFrictionConstraintCount = 0;
        contactRollingFrictionConstraintCount = 0;

        solverBodyCount = 0;
        return 0;
    }

    /// <summary>btSequentialImpulseConstraintSolver::solveGroup（顺序应用冲量）</summary>
    public override btScalar SolveGroup(List<CollisionObject> bodies, int numBodies,
        List<PersistentManifold> manifold, int numManifolds,
        List<TypedConstraint> constraints, int numConstraints,
        ContactSolverInfo info, IDebugDraw debugDrawer, Dispatcher dispatcher)
    {
        // 至少需要提供一些 bodies
        SolveGroupCacheFriendlySetup(bodies, numBodies, manifold, numManifolds, constraints, numConstraints, info, debugDrawer);

        SolveGroupCacheFriendlyIterations(bodies, numBodies, manifold, numManifolds, constraints, numConstraints, info, debugDrawer);

        SolveGroupCacheFriendlyFinish(bodies, numBodies, info);

        return 0;
    }

    /// <summary>btSequentialImpulseConstraintSolver::reset</summary>
    public override void Reset()
    {
        btSeed2 = 0;
    }

    public override ConstraintSolverType SolverType => ConstraintSolverType.SequentialImpulseSolver;
}

/// <summary>
/// btTypedConstraint::solveConstraintObsolete 的替代接口。
/// C# 的 TypedConstraint 未声明该虚方法；需要旧式求解路径的约束
/// （如 Point2PointConstraint）实现此接口，由求解器通过类型匹配调用。
/// </summary>
internal interface ISolveConstraintObsolete
{
    void SolveConstraintObsolete(SolverBody bodyA, SolverBody bodyB, btScalar timeStep);
}
