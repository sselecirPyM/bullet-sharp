using BulletSharp.Math;

namespace BulletSharp;

/// <summary>btSliderFlags</summary>
[Flags]
public enum SliderFlags
{
    None = 0,
    CfmDirLin = 1 << 0,
    ErpDirLin = 1 << 1,
    CfmDirAng = 1 << 2,
    ErpDirAng = 1 << 3,
    CfmOrtLin = 1 << 4,
    ErpOrtLin = 1 << 5,
    CfmOrtAng = 1 << 6,
    ErpOrtAng = 1 << 7,
    CfmLimLin = 1 << 8,
    ErpLimLin = 1 << 9,
    CfmLimAng = 1 << 10,
    ErpLimAng = 1 << 11,
}

/// <summary>
/// Ported from btSliderConstraint: a slider constraint (slides along and rotates about
/// the X axis of the constraint frame).
/// The softness/restitution/damping parameters are distinguished by scenario:
/// DirLin - moving within the linear limits; LimLin - hitting the linear limits;
/// DirAng - moving within the angular limits; LimAng - hitting the angular limits;
/// OrthoLin, OrthoAng - perpendicular to the constraint axis.
/// </summary>
public class SliderConstraint : TypedConstraint
{
    // SLIDER_CONSTRAINT_DEF_*
    private const btScalar DefSoftness = 1;
    private const btScalar DefDamping = 1;
    private const btScalar DefRestitution = (btScalar)0.7;
    private const btScalar DefCfm = 0;

    /// <summary>Backward-compatibility flag during the transition to getInfo/getInfo2</summary>
    protected bool useSolveConstraintObsolete;
    internal bool useOffsetForConstraintFrame;
    internal Transform frameInA;
    internal Transform frameInB;
    // When true, the limits are defined using frameA
    internal bool useLinearReferenceFrameA;
    // Linear limits
    internal btScalar lowerLinLimit;
    internal btScalar upperLinLimit;
    // Angular limits
    internal btScalar lowerAngLimit;
    internal btScalar upperAngLimit;
    // softness, restitution and damping for the different scenarios
    internal btScalar softnessDirLin;
    internal btScalar restitutionDirLin;
    internal btScalar dampingDirLin;
    internal btScalar cfmDirLin;

    internal btScalar softnessDirAng;
    internal btScalar restitutionDirAng;
    internal btScalar dampingDirAng;
    internal btScalar cfmDirAng;

    internal btScalar softnessLimLin;
    internal btScalar restitutionLimLin;
    internal btScalar dampingLimLin;
    internal btScalar cfmLimLin;

    internal btScalar softnessLimAng;
    internal btScalar restitutionLimAng;
    internal btScalar dampingLimAng;
    internal btScalar cfmLimAng;

    internal btScalar softnessOrthoLin;
    internal btScalar restitutionOrthoLin;
    internal btScalar dampingOrthoLin;
    internal btScalar cfmOrthoLin;

    internal btScalar softnessOrthoAng;
    internal btScalar restitutionOrthoAng;
    internal btScalar dampingOrthoAng;
    internal btScalar cfmOrthoAng;

    // Internal use
    internal bool solveLinLim;
    internal bool solveAngLim;

    internal int flags;

    internal JacobianEntry[] jacLin = new JacobianEntry[3];
    internal btScalar[] jacLinDiagABInv = new btScalar[3];

    internal JacobianEntry[] jacAng = new JacobianEntry[3];

    internal Transform calculatedTransformA;
    internal Transform calculatedTransformB;

    internal Vector3 sliderAxis;
    internal Vector3 realPivotAInW;
    internal Vector3 realPivotBInW;
    internal Vector3 projPivotInW;
    internal Vector3 delta;
    internal Vector3 depth = Vector3.Zero;

    internal btScalar linPos;
    internal btScalar angPos;

    internal btScalar angDepth;

    internal bool poweredLinMotor;
    internal btScalar targetLinMotorVelocity;
    internal btScalar maxLinMotorForce;
    internal btScalar accumulatedLinMotorImpulse;

    internal bool poweredAngMotor;
    internal btScalar targetAngMotorVelocity;
    internal btScalar maxAngMotorForce;
    internal btScalar accumulatedAngMotorImpulse;

    /// <summary>btSliderConstraint::btSliderConstraint(rbA, rbB, frameInA, frameInB, useLinearReferenceFrameA)</summary>
    public SliderConstraint(RigidBody rbA, RigidBody rbB, Matrix frameInA, Matrix frameInB, bool useLinearReferenceFrameA)
        : base(TypedConstraintType.Slider, rbA, rbB)
    {
        useSolveConstraintObsolete = false;
        this.frameInA = Transform.FromMatrix(in frameInA);
        this.frameInB = Transform.FromMatrix(in frameInB);
        this.useLinearReferenceFrameA = useLinearReferenceFrameA;
        InitParams();
    }

    /// <summary>btSliderConstraint::btSliderConstraint(rbB, frameInB, useLinearReferenceFrameA)</summary>
    public SliderConstraint(RigidBody rbB, Matrix frameInB, bool useLinearReferenceFrameA)
        : base(TypedConstraintType.Slider, GetFixedBody(), rbB)
    {
        useSolveConstraintObsolete = false;
        this.frameInB = Transform.FromMatrix(in frameInB);
        this.useLinearReferenceFrameA = useLinearReferenceFrameA;
        // When body A is not provided, A is implicitly in world space
        frameInA = rbB.worldTransform * this.frameInB;

        InitParams();
    }

    /// <summary>btSliderConstraint::initParams</summary>
    protected void InitParams()
    {
        lowerLinLimit = 1;
        upperLinLimit = -1;
        lowerAngLimit = 0;
        upperAngLimit = 0;
        softnessDirLin = DefSoftness;
        restitutionDirLin = DefRestitution;
        dampingDirLin = 0;
        cfmDirLin = DefCfm;
        softnessDirAng = DefSoftness;
        restitutionDirAng = DefRestitution;
        dampingDirAng = 0;
        cfmDirAng = DefCfm;
        softnessOrthoLin = DefSoftness;
        restitutionOrthoLin = DefRestitution;
        dampingOrthoLin = DefDamping;
        cfmOrthoLin = DefCfm;
        softnessOrthoAng = DefSoftness;
        restitutionOrthoAng = DefRestitution;
        dampingOrthoAng = DefDamping;
        cfmOrthoAng = DefCfm;
        softnessLimLin = DefSoftness;
        restitutionLimLin = DefRestitution;
        dampingLimLin = DefDamping;
        cfmLimLin = DefCfm;
        softnessLimAng = DefSoftness;
        restitutionLimAng = DefRestitution;
        dampingLimAng = DefDamping;
        cfmLimAng = DefCfm;

        poweredLinMotor = false;
        targetLinMotorVelocity = 0;
        maxLinMotorForce = 0;
        accumulatedLinMotorImpulse = 0;

        poweredAngMotor = false;
        targetAngMotorVelocity = 0;
        maxAngMotorForce = 0;
        accumulatedAngMotorImpulse = 0;

        flags = 0;

        useOffsetForConstraintFrame = true; // USE_OFFSET_FOR_CONSTANT_FRAME

        CalculateTransforms(in rigidBodyA.worldTransform, in rigidBodyB.worldTransform);
    }

    internal override void GetInfo1(ConstraintInfo1 info, ContactSolverInfo infoGlobal)
    {
        // btSliderConstraint::getInfo1
        if (useSolveConstraintObsolete)
        {
            info.numConstraintRows = 0;
            info.nub = 0;
        }
        else
        {
            info.numConstraintRows = 4; // fixed 2 linear + 2 angular
            info.nub = 2;
            // Prepare the constraint
            CalculateTransforms(in rigidBodyA.worldTransform, in rigidBodyB.worldTransform);
            TestAngLimits();
            TestLinLimits();
            if (solveLinLim || poweredLinMotor)
            {
                info.numConstraintRows++; // 3rd linear limit row
                info.nub--;
            }
            if (solveAngLim || poweredAngMotor)
            {
                info.numConstraintRows++; // 3rd angular limit row
                info.nub--;
            }
        }
    }

    /// <summary>btSliderConstraint::getInfo1NonVirtual</summary>
    internal void GetInfo1NonVirtual(ConstraintInfo1 info)
    {
        info.numConstraintRows = 6; // fixed 2 linear + 2 angular + 1 limit (even if unused)
        info.nub = 0;
    }

    internal override void GetInfo2(ConstraintInfo2 info, ContactSolverInfo infoGlobal,
        Transform[] body0Trans, Transform[] body1Trans, int body0Index, int body1Index)
    {
        // btSliderConstraint::getInfo2
        GetInfo2NonVirtual(info, rigidBodyA.worldTransform, rigidBodyB.worldTransform,
            rigidBodyA.linearVelocity, rigidBodyB.linearVelocity, rigidBodyA.InvMass, rigidBodyB.InvMass);
    }

    /// <summary>btSliderConstraint::calculateTransforms (internal because Transform is an internal type)</summary>
    internal void CalculateTransforms(in Transform transA, in Transform transB)
    {
        if (useLinearReferenceFrameA || !useSolveConstraintObsolete)
        {
            calculatedTransformA = transA * frameInA;
            calculatedTransformB = transB * frameInB;
        }
        else
        {
            calculatedTransformA = transB * frameInB;
            calculatedTransformB = transA * frameInA;
        }
        realPivotAInW = calculatedTransformA.Origin;
        realPivotBInW = calculatedTransformB.Origin;
        sliderAxis = calculatedTransformA.Basis.Column(0); // along the X axis
        if (useLinearReferenceFrameA || useSolveConstraintObsolete)
        {
            delta = realPivotBInW - realPivotAInW;
        }
        else
        {
            delta = realPivotAInW - realPivotBInW;
        }
        projPivotInW = realPivotAInW + sliderAxis.Dot(delta) * sliderAxis;
        Vector3 normalWorld;
        int i;
        // Linear part
        for (i = 0; i < 3; i++)
        {
            normalWorld = calculatedTransformA.Basis.Column(i);
            depth[i] = delta.Dot(normalWorld);
        }
    }

    /// <summary>Public Matrix version of btSliderConstraint::calculateTransforms</summary>
    public void CalculateTransforms(Matrix transA, Matrix transB)
    {
        Transform ta = Transform.FromMatrix(in transA);
        Transform tb = Transform.FromMatrix(in transB);
        CalculateTransforms(in ta, in tb);
    }

    /// <summary>btSliderConstraint::testLinLimits</summary>
    public void TestLinLimits()
    {
        solveLinLim = false;
        linPos = depth[0];
        if (lowerLinLimit <= upperLinLimit)
        {
            if (depth[0] > upperLinLimit)
            {
                depth[0] -= upperLinLimit;
                solveLinLim = true;
            }
            else if (depth[0] < lowerLinLimit)
            {
                depth[0] -= lowerLinLimit;
                solveLinLim = true;
            }
            else
            {
                depth[0] = 0;
            }
        }
        else
        {
            depth[0] = 0;
        }
    }

    /// <summary>btSliderConstraint::testAngLimits</summary>
    public void TestAngLimits()
    {
        angDepth = 0;
        solveAngLim = false;
        if (lowerAngLimit <= upperAngLimit)
        {
            Vector3 axisA0 = calculatedTransformA.Basis.Column(1);
            Vector3 axisA1 = calculatedTransformA.Basis.Column(2);
            Vector3 axisB0 = calculatedTransformB.Basis.Column(1);
            btScalar rot = BtMath.Atan2(axisB0.Dot(axisA1), axisB0.Dot(axisA0));
            rot = ConstraintMath.AdjustAngleToLimits(rot, lowerAngLimit, upperAngLimit);
            angPos = rot;
            if (rot < lowerAngLimit)
            {
                angDepth = rot - lowerAngLimit;
                solveAngLim = true;
            }
            else if (rot > upperAngLimit)
            {
                angDepth = rot - upperAngLimit;
                solveAngLim = true;
            }
        }
    }

    /// <summary>btSliderConstraint::getAncorInA</summary>
    public Vector3 GetAncorInA()
    {
        Vector3 ancorInA = realPivotAInW + (lowerLinLimit + upperLinLimit) * (btScalar)0.5 * sliderAxis;
        ancorInA = rigidBodyA.worldTransform.Inverse() * ancorInA;
        return ancorInA;
    }

    /// <summary>btSliderConstraint::getAncorInB</summary>
    public Vector3 GetAncorInB()
    {
        return frameInB.Origin;
    }

    /// <summary>btSliderConstraint::getInfo2NonVirtual</summary>
    internal void GetInfo2NonVirtual(ConstraintInfo2 info, in Transform transA, in Transform transB,
        in Vector3 linVelA, in Vector3 linVelB, btScalar rbAinvMass, btScalar rbBinvMass)
    {
        Transform trA = calculatedTransformA;
        Transform trB = calculatedTransformB;

        int i, s = info.rowskip;

        btScalar signFact = useLinearReferenceFrameA ? 1 : -1;

        // Difference between the two frames in world space
        Vector3 ofs = trB.Origin - trA.Origin;
        // Mass weighting factor
        btScalar miA = rbAinvMass;
        btScalar miB = rbBinvMass;
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
        Vector3 ax1, p, q;
        Vector3 ax1A = trA.Basis.Column(0);
        Vector3 ax1B = trB.Basis.Column(0);
        if (useOffsetForConstraintFrame)
        {
            // Desired slider axis direction = the X axes of frameA and frameB (world space), mass-weighted
            ax1 = ax1A * factA + ax1B * factB;
            ax1.Normalize();
            // Construct two orthogonal axes to the slider axis
            BtMath.PlaneSpace1(in ax1, out p, out q);
        }
        else
        {
            // Old way - use frameA
            ax1 = trA.Basis.Column(0);
            // The 2 orthogonal axes of the slider axis (Y, Z)
            p = trA.Basis.Column(1);
            q = trA.Basis.Column(2);
        }
        // Make rotations about these orthogonal axes equal: the slider axis should be the only unconstrained rotation axis
        info.j1AngularAxis[0] = p[0];
        info.j1AngularAxis[1] = p[1];
        info.j1AngularAxis[2] = p[2];
        info.j1AngularAxis[s + 0] = q[0];
        info.j1AngularAxis[s + 1] = q[1];
        info.j1AngularAxis[s + 2] = q[2];

        info.j2AngularAxis[0] = -p[0];
        info.j2AngularAxis[1] = -p[1];
        info.j2AngularAxis[2] = -p[2];
        info.j2AngularAxis[s + 0] = -q[0];
        info.j2AngularAxis[s + 1] = -q[1];
        info.j2AngularAxis[s + 2] = -q[2];
        // Right-hand side: angular_velocity = (erp*fps) * (ax1A x ax1B), then projected onto p and q
        btScalar currERP = (flags & (int)SliderFlags.ErpOrtAng) != 0 ? softnessOrthoAng : softnessOrthoAng * info.erp;
        btScalar k = info.fps * currERP;

        Vector3 u = ax1A.Cross(ax1B);
        info.constraintError[0] = k * u.Dot(p);
        info.constraintError[s] = k * u.Dot(q);
        if ((flags & (int)SliderFlags.CfmOrtAng) != 0)
        {
            info.cfm[0] = cfmOrthoAng;
            info.cfm[s] = cfmOrthoAng;
        }

        int nrow = 1; // last filled row
        int srow;
        btScalar limitErr;
        int limit;

        // Next two rows: velA + wA x relA == velB + wB x relB, projected onto the two orthogonal axes of the slider axis
        Transform bodyATrans = transA;
        Transform bodyBTrans = transB;
        nrow++;
        int s2 = nrow * s;
        nrow++;
        int s3 = nrow * s;
        Vector3 tmpA = Vector3.Zero, tmpB = Vector3.Zero, relA = Vector3.Zero, relB = Vector3.Zero, c = Vector3.Zero;
        if (useOffsetForConstraintFrame)
        {
            // Vector from bodyB to frameB in world space
            relB = trB.Origin - bodyBTrans.Origin;
            // Projection onto the slider axis
            Vector3 projB = ax1 * relB.Dot(ax1);
            // Vector from bodyB toward the slider axis (orthogonal to it)
            Vector3 orthoB = relB - projB;
            // Same for bodyA
            relA = trA.Origin - bodyATrans.Origin;
            Vector3 projA = ax1 * relA.Dot(ax1);
            Vector3 orthoA = relA - projA;
            // Desired offset between frames A and B along the slider axis
            btScalar sliderOffs = linPos - depth[0];
            // Desired vector from the projection of bodyA's center to the projection of bodyB's center
            Vector3 totalDist = projA + ax1 * sliderOffs - projB;
            // Obtain the offset vectors relA, relB
            relA = orthoA + totalDist * factA;
            relB = orthoB - totalDist * factB;
            // Choose the average orthogonal axis of the slider axis
            p = orthoB * factA + orthoA * factB;
            btScalar len2 = p.LengthSquared();
            if (len2 > BtMath.SimdEpsilon)
            {
                p /= BtMath.Sqrt(len2);
            }
            else
            {
                p = trA.Basis.Column(1);
            }
            // Construct one more orthogonal axis
            q = ax1.Cross(p);
            // Fill two rows
            tmpA = relA.Cross(p);
            tmpB = relB.Cross(p);
            for (i = 0; i < 3; i++) info.j1AngularAxis[s2 + i] = tmpA[i];
            for (i = 0; i < 3; i++) info.j2AngularAxis[s2 + i] = -tmpB[i];
            tmpA = relA.Cross(q);
            tmpB = relB.Cross(q);
            if (hasStaticBody && solveAngLim)
            {
                // Make constraints between static and dynamic objects stiffer: remove wA (or wB) from the equation when the angular limit is hit
                tmpB *= factB;
                tmpA *= factA;
            }
            for (i = 0; i < 3; i++) info.j1AngularAxis[s3 + i] = tmpA[i];
            for (i = 0; i < 3; i++) info.j2AngularAxis[s3 + i] = -tmpB[i];
            for (i = 0; i < 3; i++) info.j1LinearAxis[s2 + i] = p[i];
            for (i = 0; i < 3; i++) info.j1LinearAxis[s3 + i] = q[i];
            for (i = 0; i < 3; i++) info.j2LinearAxis[s2 + i] = -p[i];
            for (i = 0; i < 3; i++) info.j2LinearAxis[s3 + i] = -q[i];
        }
        else
        {
            // Old way - may be incorrect when the bodies are not on the slider axis
            c = bodyBTrans.Origin - bodyATrans.Origin;
            Vector3 tmp = c.Cross(p);
            for (i = 0; i < 3; i++) info.j1AngularAxis[s2 + i] = factA * tmp[i];
            for (i = 0; i < 3; i++) info.j2AngularAxis[s2 + i] = factB * tmp[i];
            tmp = c.Cross(q);
            for (i = 0; i < 3; i++) info.j1AngularAxis[s3 + i] = factA * tmp[i];
            for (i = 0; i < 3; i++) info.j2AngularAxis[s3 + i] = factB * tmp[i];

            for (i = 0; i < 3; i++) info.j1LinearAxis[s2 + i] = p[i];
            for (i = 0; i < 3; i++) info.j1LinearAxis[s3 + i] = q[i];
            for (i = 0; i < 3; i++) info.j2LinearAxis[s2 + i] = -p[i];
            for (i = 0; i < 3; i++) info.j2LinearAxis[s3 + i] = -q[i];
        }
        // Compute the two elements of the right-hand side
        currERP = (flags & (int)SliderFlags.ErpOrtLin) != 0 ? softnessOrthoLin : softnessOrthoLin * info.erp;
        k = info.fps * currERP;

        btScalar rhs = k * p.Dot(ofs);
        info.constraintError[s2] = rhs;
        rhs = k * q.Dot(ofs);
        info.constraintError[s3] = rhs;
        if ((flags & (int)SliderFlags.CfmOrtLin) != 0)
        {
            info.cfm[s2] = cfmOrthoLin;
            info.cfm[s3] = cfmOrthoLin;
        }

        // C++ relies on the solver to preset lowerLimit/upperLimit to ±SIMD_INFINITY; written explicitly here (rows 0-3)
        for (i = 0; i < 4; i++)
        {
            info.lowerLimit[i * s] = -BtMath.SimdInfinity;
            info.upperLimit[i * s] = BtMath.SimdInfinity;
        }

        // Check the linear limit
        limitErr = 0;
        limit = 0;
        if (solveLinLim)
        {
            limitErr = depth[0] * signFact;
            limit = limitErr > 0 ? 2 : 1;
        }
        bool powered = poweredLinMotor;
        // If the slider has limits or a motor, add an extra row
        if (limit != 0 || powered)
        {
            nrow++;
            srow = nrow * info.rowskip;
            info.j1LinearAxis[srow + 0] = ax1[0];
            info.j1LinearAxis[srow + 1] = ax1[1];
            info.j1LinearAxis[srow + 2] = ax1[2];
            info.j2LinearAxis[srow + 0] = -ax1[0];
            info.j2LinearAxis[srow + 1] = -ax1[1];
            info.j2LinearAxis[srow + 2] = -ax1[2];
            // Linear torque decoupling: the linear constraint forces (±ax1) acting on the two bodies
            // must not form a couple, i.e. their points of application must lie on the same ax1 axis,
            // otherwise a limited slider free body would gain angular momentum.
            if (useOffsetForConstraintFrame)
            {
                // Only needed when both bodyA and bodyB are dynamic
                if (!hasStaticBody)
                {
                    tmpA = relA.Cross(ax1);
                    tmpB = relB.Cross(ax1);
                    info.j1AngularAxis[srow + 0] = tmpA[0];
                    info.j1AngularAxis[srow + 1] = tmpA[1];
                    info.j1AngularAxis[srow + 2] = tmpA[2];
                    info.j2AngularAxis[srow + 0] = -tmpB[0];
                    info.j2AngularAxis[srow + 1] = -tmpB[1];
                    info.j2AngularAxis[srow + 2] = -tmpB[2];
                }
            }
            else
            {
                // Old way - may be incorrect when the bodies are not on the slider axis
                Vector3 ltd = c.Cross(ax1); // Linear Torque Decoupling vector (torque)
                info.j1AngularAxis[srow + 0] = factA * ltd[0];
                info.j1AngularAxis[srow + 1] = factA * ltd[1];
                info.j1AngularAxis[srow + 2] = factA * ltd[2];
                info.j2AngularAxis[srow + 0] = factB * ltd[0];
                info.j2AngularAxis[srow + 1] = factB * ltd[1];
                info.j2AngularAxis[srow + 2] = factB * ltd[2];
            }
            // Right-hand side part
            btScalar lostop = lowerLinLimit;
            btScalar histop = upperLinLimit;
            if (limit != 0 && lostop == histop)
            {
                // Motor disabled
                powered = false;
            }
            info.constraintError[srow] = 0;
            info.lowerLimit[srow] = 0;
            info.upperLimit[srow] = 0;
            currERP = (flags & (int)SliderFlags.ErpLimLin) != 0 ? softnessLimLin : info.erp;
            if (powered)
            {
                if ((flags & (int)SliderFlags.CfmDirLin) != 0)
                {
                    info.cfm[srow] = cfmDirLin;
                }
                btScalar tagVel = targetLinMotorVelocity;
                btScalar motFact = ConstraintMath.GetMotorFactor(linPos, lowerLinLimit, upperLinLimit, tagVel, info.fps * currERP);
                info.constraintError[srow] -= signFact * motFact * targetLinMotorVelocity;
                info.lowerLimit[srow] += -maxLinMotorForce / info.fps;
                info.upperLimit[srow] += maxLinMotorForce / info.fps;
            }
            if (limit != 0)
            {
                k = info.fps * currERP;
                info.constraintError[srow] += k * limitErr;
                if ((flags & (int)SliderFlags.CfmLimLin) != 0)
                {
                    info.cfm[srow] = cfmLimLin;
                }
                if (lostop == histop)
                {
                    // Both low and high limits simultaneously
                    info.lowerLimit[srow] = -BtMath.SimdInfinity;
                    info.upperLimit[srow] = BtMath.SimdInfinity;
                }
                else if (limit == 1)
                {
                    // Low limit
                    info.lowerLimit[srow] = -BtMath.SimdInfinity;
                    info.upperLimit[srow] = 0;
                }
                else
                {
                    // High limit
                    info.lowerLimit[srow] = 0;
                    info.upperLimit[srow] = BtMath.SimdInfinity;
                }
                // Bounce (uses the |1.0 - dampingLimLin| parameter)
                btScalar bounce = BtMath.Abs(1 - dampingLimLin);
                if (bounce > 0)
                {
                    btScalar vel = linVelA.Dot(ax1);
                    vel -= linVelB.Dot(ax1);
                    vel *= signFact;
                    // Apply bounce only when the velocity heads toward the limit and the resulting c[] exceeds the current value
                    if (limit == 1)
                    {
                        // Low limit
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
                        // High limit - the computation is fully reversed
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
                info.constraintError[srow] *= softnessLimLin;
            }
        }
        // Check the angular limit
        limitErr = 0;
        limit = 0;
        if (solveAngLim)
        {
            limitErr = angDepth;
            limit = limitErr > 0 ? 1 : 2;
        }
        // If the slider has a limit or an angular motor, add an extra row
        powered = poweredAngMotor;
        if (limit != 0 || powered)
        {
            nrow++;
            srow = nrow * info.rowskip;
            info.j1AngularAxis[srow + 0] = ax1[0];
            info.j1AngularAxis[srow + 1] = ax1[1];
            info.j1AngularAxis[srow + 2] = ax1[2];

            info.j2AngularAxis[srow + 0] = -ax1[0];
            info.j2AngularAxis[srow + 1] = -ax1[1];
            info.j2AngularAxis[srow + 2] = -ax1[2];

            btScalar lostop = lowerAngLimit;
            btScalar histop = upperAngLimit;
            if (limit != 0 && lostop == histop)
            {
                // Motor disabled
                powered = false;
            }
            currERP = (flags & (int)SliderFlags.ErpLimAng) != 0 ? softnessLimAng : info.erp;
            if (powered)
            {
                if ((flags & (int)SliderFlags.CfmDirAng) != 0)
                {
                    info.cfm[srow] = cfmDirAng;
                }
                btScalar motFact = ConstraintMath.GetMotorFactor(angPos, lowerAngLimit, upperAngLimit, targetAngMotorVelocity, info.fps * currERP);
                info.constraintError[srow] = motFact * targetAngMotorVelocity;
                info.lowerLimit[srow] = -maxAngMotorForce / info.fps;
                info.upperLimit[srow] = maxAngMotorForce / info.fps;
            }
            if (limit != 0)
            {
                k = info.fps * currERP;
                info.constraintError[srow] += k * limitErr;
                if ((flags & (int)SliderFlags.CfmLimAng) != 0)
                {
                    info.cfm[srow] = cfmLimAng;
                }
                if (lostop == histop)
                {
                    // Both low and high limits simultaneously
                    info.lowerLimit[srow] = -BtMath.SimdInfinity;
                    info.upperLimit[srow] = BtMath.SimdInfinity;
                }
                else if (limit == 1)
                {
                    // Low limit
                    info.lowerLimit[srow] = 0;
                    info.upperLimit[srow] = BtMath.SimdInfinity;
                }
                else
                {
                    // High limit
                    info.lowerLimit[srow] = -BtMath.SimdInfinity;
                    info.upperLimit[srow] = 0;
                }
                // Bounce (uses the |1.0 - dampingLimAng| parameter)
                btScalar bounce = BtMath.Abs(1 - dampingLimAng);
                if (bounce > 0)
                {
                    btScalar vel = rigidBodyA.angularVelocity.Dot(ax1);
                    vel -= rigidBodyB.angularVelocity.Dot(ax1);
                    // Apply bounce only when the velocity heads toward the limit and the resulting c[] exceeds the current value
                    if (limit == 1)
                    {
                        // Low limit
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
                        // High limit - the computation is fully reversed
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
                info.constraintError[srow] *= softnessLimAng;
            }
        }
    }

    /// <summary>btSliderConstraint::getCalculatedTransformA</summary>
    public Matrix CalculatedTransformA => calculatedTransformA.ToMatrix();

    /// <summary>btSliderConstraint::getCalculatedTransformB</summary>
    public Matrix CalculatedTransformB => calculatedTransformB.ToMatrix();

    /// <summary>btSliderConstraint::getFrameOffsetA</summary>
    public Matrix FrameOffsetA
    {
        get => frameInA.ToMatrix();
        set => frameInA = Transform.FromMatrix(in value);
    }

    /// <summary>btSliderConstraint::getFrameOffsetB</summary>
    public Matrix FrameOffsetB
    {
        get => frameInB.ToMatrix();
        set => frameInB = Transform.FromMatrix(in value);
    }

    /// <summary>btSliderConstraint::getLowerLinLimit/setLowerLinLimit</summary>
    public btScalar LowerLinLimit
    {
        get => lowerLinLimit;
        set => lowerLinLimit = value;
    }

    /// <summary>btSliderConstraint::getUpperLinLimit/setUpperLinLimit</summary>
    public btScalar UpperLinLimit
    {
        get => upperLinLimit;
        set => upperLinLimit = value;
    }

    /// <summary>btSliderConstraint::getLowerAngLimit/setLowerAngLimit (the angle is normalized before writing)</summary>
    public btScalar LowerAngLimit
    {
        get => lowerAngLimit;
        set => lowerAngLimit = ConstraintMath.NormalizeAngle(value);
    }

    /// <summary>btSliderConstraint::getUpperAngLimit/setUpperAngLimit (the angle is normalized before writing)</summary>
    public btScalar UpperAngLimit
    {
        get => upperAngLimit;
        set => upperAngLimit = ConstraintMath.NormalizeAngle(value);
    }

    /// <summary>btSliderConstraint::getUseLinearReferenceFrameA</summary>
    public bool UseLinearReferenceFrameA => useLinearReferenceFrameA;

    public btScalar SoftnessDirLin { get => softnessDirLin; set => softnessDirLin = value; }
    public btScalar RestitutionDirLin { get => restitutionDirLin; set => restitutionDirLin = value; }
    public btScalar DampingDirLin { get => dampingDirLin; set => dampingDirLin = value; }
    public btScalar SoftnessDirAng { get => softnessDirAng; set => softnessDirAng = value; }
    public btScalar RestitutionDirAng { get => restitutionDirAng; set => restitutionDirAng = value; }
    public btScalar DampingDirAng { get => dampingDirAng; set => dampingDirAng = value; }
    public btScalar SoftnessLimLin { get => softnessLimLin; set => softnessLimLin = value; }
    public btScalar RestitutionLimLin { get => restitutionLimLin; set => restitutionLimLin = value; }
    public btScalar DampingLimLin { get => dampingLimLin; set => dampingLimLin = value; }
    public btScalar SoftnessLimAng { get => softnessLimAng; set => softnessLimAng = value; }
    public btScalar RestitutionLimAng { get => restitutionLimAng; set => restitutionLimAng = value; }
    public btScalar DampingLimAng { get => dampingLimAng; set => dampingLimAng = value; }
    public btScalar SoftnessOrthoLin { get => softnessOrthoLin; set => softnessOrthoLin = value; }
    public btScalar RestitutionOrthoLin { get => restitutionOrthoLin; set => restitutionOrthoLin = value; }
    public btScalar DampingOrthoLin { get => dampingOrthoLin; set => dampingOrthoLin = value; }
    public btScalar SoftnessOrthoAng { get => softnessOrthoAng; set => softnessOrthoAng = value; }
    public btScalar RestitutionOrthoAng { get => restitutionOrthoAng; set => restitutionOrthoAng = value; }
    public btScalar DampingOrthoAng { get => dampingOrthoAng; set => dampingOrthoAng = value; }

    /// <summary>btSliderConstraint::setPoweredLinMotor/getPoweredLinMotor</summary>
    public bool PoweredLinMotor
    {
        get => poweredLinMotor;
        set => poweredLinMotor = value;
    }

    /// <summary>btSliderConstraint::setTargetLinMotorVelocity/getTargetLinMotorVelocity</summary>
    public btScalar TargetLinMotorVelocity
    {
        get => targetLinMotorVelocity;
        set => targetLinMotorVelocity = value;
    }

    /// <summary>btSliderConstraint::setMaxLinMotorForce/getMaxLinMotorForce</summary>
    public btScalar MaxLinMotorForce
    {
        get => maxLinMotorForce;
        set => maxLinMotorForce = value;
    }

    /// <summary>btSliderConstraint::setPoweredAngMotor/getPoweredAngMotor</summary>
    public bool PoweredAngMotor
    {
        get => poweredAngMotor;
        set => poweredAngMotor = value;
    }

    /// <summary>btSliderConstraint::setTargetAngMotorVelocity/getTargetAngMotorVelocity</summary>
    public btScalar TargetAngMotorVelocity
    {
        get => targetAngMotorVelocity;
        set => targetAngMotorVelocity = value;
    }

    /// <summary>btSliderConstraint::setMaxAngMotorForce/getMaxAngMotorForce</summary>
    public btScalar MaxAngMotorForce
    {
        get => maxAngMotorForce;
        set => maxAngMotorForce = value;
    }

    /// <summary>btSliderConstraint::getLinearPos</summary>
    public btScalar LinearPos => linPos;

    /// <summary>btSliderConstraint::getAngularPos</summary>
    public btScalar AngularPos => angPos;

    /// <summary>btSliderConstraint::getSolveLinLimit</summary>
    public bool SolveLinLimit => solveLinLim;

    /// <summary>btSliderConstraint::getLinDepth</summary>
    public btScalar LinDepth => depth[0];

    /// <summary>btSliderConstraint::getSolveAngLimit</summary>
    public bool SolveAngLimit => solveAngLim;

    /// <summary>btSliderConstraint::getAngDepth</summary>
    public btScalar AngDepth => angDepth;

    /// <summary>btSliderConstraint::getAncorInA</summary>
    public Vector3 AncorInA => GetAncorInA();

    /// <summary>btSliderConstraint::getAncorInB</summary>
    public Vector3 AncorInB => GetAncorInB();

    /// <summary>btSliderConstraint::getUseFrameOffset/setUseFrameOffset</summary>
    public bool UseFrameOffset
    {
        get => useOffsetForConstraintFrame;
        set => useOffsetForConstraintFrame = value;
    }

    /// <summary>btSliderConstraint::setFrames</summary>
    public void SetFrames(Matrix frameA, Matrix frameB)
    {
        frameInA = Transform.FromMatrix(in frameA);
        frameInB = Transform.FromMatrix(in frameB);
        CalculateTransforms(in rigidBodyA.worldTransform, in rigidBodyB.worldTransform);
        BuildJacobian();
    }

    /// <summary>btSliderConstraint::setParam (overrides the global default parameters; axis selects the limit type)</summary>
    public override void SetParam(int num, btScalar value, int axis = -1)
    {
        switch ((ConstraintParams)num)
        {
            case ConstraintParams.StopErp:
                if (axis < 1)
                {
                    softnessLimLin = value;
                    flags |= (int)SliderFlags.ErpLimLin;
                }
                else if (axis < 3)
                {
                    softnessOrthoLin = value;
                    flags |= (int)SliderFlags.ErpOrtLin;
                }
                else if (axis == 3)
                {
                    softnessLimAng = value;
                    flags |= (int)SliderFlags.ErpLimAng;
                }
                else if (axis < 6)
                {
                    softnessOrthoAng = value;
                    flags |= (int)SliderFlags.ErpOrtAng;
                }
                else
                {
                    // btAssertConstrParams(0)
                }
                break;
            case ConstraintParams.Cfm:
                if (axis < 1)
                {
                    cfmDirLin = value;
                    flags |= (int)SliderFlags.CfmDirLin;
                }
                else if (axis == 3)
                {
                    cfmDirAng = value;
                    flags |= (int)SliderFlags.CfmDirAng;
                }
                else
                {
                    // btAssertConstrParams(0)
                }
                break;
            case ConstraintParams.StopCfm:
                if (axis < 1)
                {
                    cfmLimLin = value;
                    flags |= (int)SliderFlags.CfmLimLin;
                }
                else if (axis < 3)
                {
                    cfmOrthoLin = value;
                    flags |= (int)SliderFlags.CfmOrtLin;
                }
                else if (axis == 3)
                {
                    cfmLimAng = value;
                    flags |= (int)SliderFlags.CfmLimAng;
                }
                else if (axis < 6)
                {
                    cfmOrthoAng = value;
                    flags |= (int)SliderFlags.CfmOrtAng;
                }
                else
                {
                    // btAssertConstrParams(0)
                }
                break;
        }
    }

    /// <summary>btSliderConstraint::getParam (returns the local parameter value)</summary>
    public override btScalar GetParam(int num, int axis)
    {
        btScalar retVal = BtMath.SimdInfinity;
        switch ((ConstraintParams)num)
        {
            case ConstraintParams.StopErp:
                if (axis < 1)
                {
                    retVal = softnessLimLin;
                }
                else if (axis < 3)
                {
                    retVal = softnessOrthoLin;
                }
                else if (axis == 3)
                {
                    retVal = softnessLimAng;
                }
                else if (axis < 6)
                {
                    retVal = softnessOrthoAng;
                }
                else
                {
                    // btAssertConstrParams(0)
                }
                break;
            case ConstraintParams.Cfm:
                if (axis < 1)
                {
                    retVal = cfmDirLin;
                }
                else if (axis == 3)
                {
                    retVal = cfmDirAng;
                }
                else
                {
                    // btAssertConstrParams(0)
                }
                break;
            case ConstraintParams.StopCfm:
                if (axis < 1)
                {
                    retVal = cfmLimLin;
                }
                else if (axis < 3)
                {
                    retVal = cfmOrthoLin;
                }
                else if (axis == 3)
                {
                    retVal = cfmLimAng;
                }
                else if (axis < 6)
                {
                    retVal = cfmOrthoAng;
                }
                else
                {
                    // btAssertConstrParams(0)
                }
                break;
        }
        return retVal;
    }

    /// <summary>btSliderConstraint::getFlags</summary>
    public int Flags => flags;
}
