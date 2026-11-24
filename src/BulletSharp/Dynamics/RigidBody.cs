using BulletSharp.Math;

namespace BulletSharp;

/// <summary>btRigidBodyFlags</summary>
[Flags]
public enum RigidBodyFlags
{
    None = 0,
    DisableWorldGravity = 1,
    /// <summary>BT_ENABLE_GYROPSCOPIC_FORCE defaults to the equivalent of ImplicitBody (Bullet 2.83+)</summary>
    EnableGyroscopicForceExplicit = 2,
    EnableGyroscopicForceImplicitWorld = 4,
    EnableGyroscopicForceImplicitBody = 8,
    EnableGyropscopicForce = EnableGyroscopicForceImplicitBody,
}

/// <summary>btRigidBody::btRigidBodyConstructionInfo</summary>
public class RigidBodyConstructionInfo
{
    public btScalar Mass;
    public MotionState MotionState;
    public Matrix StartWorldTransform = Matrix.Identity;
    public CollisionShape CollisionShape;
    public Vector3 LocalInertia;
    public btScalar LinearDamping;
    public btScalar AngularDamping;
    public btScalar Friction = (btScalar)0.5;
    public btScalar RollingFriction;
    public btScalar SpinningFriction;
    public btScalar Restitution;
    public btScalar LinearSleepingThreshold = (btScalar)0.8;
    public btScalar AngularSleepingThreshold = 1;
    public bool AdditionalDamping;
    public btScalar AdditionalDampingFactor = (btScalar)0.005;
    public btScalar AdditionalLinearDampingThresholdSqr = (btScalar)0.01;
    public btScalar AdditionalAngularDampingThresholdSqr = (btScalar)0.01;
    public btScalar AdditionalAngularDampingFactor = (btScalar)0.01;

    public RigidBodyConstructionInfo(btScalar mass, MotionState motionState, CollisionShape collisionShape)
        : this(mass, motionState, collisionShape, Vector3.Zero)
    {
    }

    public RigidBodyConstructionInfo(btScalar mass, MotionState motionState, CollisionShape collisionShape, Vector3 localInertia)
    {
        Mass = mass;
        MotionState = motionState;
        CollisionShape = collisionShape;
        LocalInertia = localInertia;
    }
}

/// <summary>
/// Ported from btRigidBody: a rigid body. A body with zero mass is fixed (static).
/// </summary>
public class RigidBody : CollisionObject
{
    /// <summary>Global gDeactivationTime</summary>
    public static btScalar GlobalDeactivationTime = 2;
    /// <summary>Global gDisableDeactivation</summary>
    public static bool GlobalDisableDeactivation;

    internal Matrix3x3 invInertiaTensorWorld;
    internal Vector3 linearVelocity;
    internal Vector3 angularVelocity;
    internal btScalar inverseMass;
    internal Vector3 linearFactor;

    internal Vector3 gravity;
    internal Vector3 gravityAcceleration;
    internal Vector3 invInertiaLocal;
    internal Vector3 totalForce;
    internal Vector3 totalTorque;

    internal btScalar linearDamping;
    internal btScalar angularDamping;

    internal bool additionalDamping;
    internal btScalar additionalDampingFactor;
    internal btScalar additionalLinearDampingThresholdSqr;
    internal btScalar additionalAngularDampingThresholdSqr;
    internal btScalar additionalAngularDampingFactor;

    internal btScalar linearSleepingThreshold;
    internal btScalar angularSleepingThreshold;

    internal MotionState optionalMotionState;

    internal readonly List<TypedConstraint> constraintRefs = new();

    internal int rigidbodyFlags;

    internal int debugBodyId;
    private static int uniqueId;

    // The base CollisionObject does not include interpolation velocities (btCollisionObject members); added here
    internal Vector3 interpolationLinearVelocity;
    internal Vector3 interpolationAngularVelocity;

    /// <summary>btCollisionObject::getInterpolationLinearVelocity</summary>
    public Vector3 InterpolationLinearVelocity
    {
        get => interpolationLinearVelocity;
        set => interpolationLinearVelocity = value;
    }

    /// <summary>btCollisionObject::getInterpolationAngularVelocity</summary>
    public Vector3 InterpolationAngularVelocity
    {
        get => interpolationAngularVelocity;
        set => interpolationAngularVelocity = value;
    }

    internal Vector3 deltaLinearVelocity;
    internal Vector3 deltaAngularVelocity;
    internal Vector3 angularFactor;
    internal Vector3 invMass;
    internal Vector3 pushVelocity;
    internal Vector3 turnVelocity;

    public int contactSolverType;
    public int frictionSolverType;

    /// <summary>btRigidBody::btRigidBody(btRigidBodyConstructionInfo)</summary>
    public RigidBody(RigidBodyConstructionInfo constructionInfo)
    {
        SetupRigidBody(constructionInfo);
    }

    /// <summary>btRigidBody::btRigidBody(mass, motionState, collisionShape, localInertia)</summary>
    public RigidBody(btScalar mass, MotionState motionState, CollisionShape collisionShape)
        : this(mass, motionState, collisionShape, Vector3.Zero)
    {
    }

    public RigidBody(btScalar mass, MotionState motionState, CollisionShape collisionShape, Vector3 localInertia)
    {
        SetupRigidBody(new RigidBodyConstructionInfo(mass, motionState, collisionShape, localInertia));
    }

    /// <summary>btRigidBody::setupRigidBody</summary>
    protected void SetupRigidBody(RigidBodyConstructionInfo constructionInfo)
    {
        internalType = CollisionObjectTypes.RigidBody;

        linearVelocity = Vector3.Zero;
        angularVelocity = Vector3.Zero;
        angularFactor = new Vector3(1, 1, 1);
        linearFactor = new Vector3(1, 1, 1);
        gravity = Vector3.Zero;
        gravityAcceleration = Vector3.Zero;
        totalForce = Vector3.Zero;
        totalTorque = Vector3.Zero;
        SetDamping(constructionInfo.LinearDamping, constructionInfo.AngularDamping);

        linearSleepingThreshold = constructionInfo.LinearSleepingThreshold;
        angularSleepingThreshold = constructionInfo.AngularSleepingThreshold;
        optionalMotionState = constructionInfo.MotionState;
        contactSolverType = 0;
        frictionSolverType = 0;
        additionalDamping = constructionInfo.AdditionalDamping;
        additionalDampingFactor = constructionInfo.AdditionalDampingFactor;
        additionalLinearDampingThresholdSqr = constructionInfo.AdditionalLinearDampingThresholdSqr;
        additionalAngularDampingThresholdSqr = constructionInfo.AdditionalAngularDampingThresholdSqr;
        additionalAngularDampingFactor = constructionInfo.AdditionalAngularDampingFactor;

        if (optionalMotionState != null)
        {
            optionalMotionState.GetWorldTransform(out Matrix m);
            worldTransform = Transform.FromMatrix(in m);
        }
        else
        {
            worldTransform = Transform.FromMatrix(in constructionInfo.StartWorldTransform);
        }

        interpolationWorldTransform = worldTransform;
        interpolationLinearVelocity = Vector3.Zero;
        interpolationAngularVelocity = Vector3.Zero;

        // Moved to btCollisionObject
        friction = constructionInfo.Friction;
        rollingFriction = constructionInfo.RollingFriction;
        spinningFriction = constructionInfo.SpinningFriction;

        restitution = constructionInfo.Restitution;

        CollisionShape = constructionInfo.CollisionShape;
        debugBodyId = uniqueId++;

        SetMassProps(constructionInfo.Mass, constructionInfo.LocalInertia);
        UpdateInertiaTensor();

        rigidbodyFlags = (int)RigidBodyFlags.EnableGyroscopicForceImplicitBody;

        deltaLinearVelocity = Vector3.Zero;
        deltaAngularVelocity = Vector3.Zero;
        invMass = inverseMass * linearFactor;
        pushVelocity = Vector3.Zero;
        turnVelocity = Vector3.Zero;
    }

    /// <summary>btRigidBody::proceedToTransform</summary>
    public void ProceedToTransform(ref Matrix newTrans)
    {
        SetCenterOfMassTransform(ref newTrans);
    }

    internal void ProceedToTransformInternal(in Transform newTrans)
    {
        SetCenterOfMassTransformInternal(in newTrans);
    }

    /// <summary>btRigidBody::upcast</summary>
    public static RigidBody Upcast(CollisionObject colObj)
    {
        if (colObj != null && (colObj.internalType & CollisionObjectTypes.RigidBody) != 0)
            return (RigidBody)colObj;
        return null;
    }

    /// <summary>btRigidBody::predictIntegratedTransform</summary>
    public void PredictIntegratedTransform(btScalar step, out Matrix predictedTransform)
    {
        TransformUtil.IntegrateTransform(in worldTransform, in linearVelocity, in angularVelocity, step, out Transform t);
        predictedTransform = t.ToMatrix();
    }

    internal void PredictIntegratedTransformInternal(btScalar step, out Transform predictedTransform)
    {
        TransformUtil.IntegrateTransform(in worldTransform, in linearVelocity, in angularVelocity, step, out predictedTransform);
    }

    /// <summary>btRigidBody::saveKinematicState</summary>
    public void SaveKinematicState(btScalar step)
    {
        // todo: clamp to a safe minimum time step, clamp the maximum angular/linear velocity
        if (step != 0)
        {
            // When using a motion state to synchronize the world transform, use the latest kinematic world transform
            if (MotionState != null)
            {
                MotionState.GetWorldTransform(out Matrix m);
                worldTransform = Transform.FromMatrix(in m);
            }

            TransformUtil.CalculateVelocity(in interpolationWorldTransform, in worldTransform, step,
                out linearVelocity, out angularVelocity);
            interpolationLinearVelocity = linearVelocity;
            interpolationAngularVelocity = angularVelocity;
            interpolationWorldTransform = worldTransform;
        }
    }

    /// <summary>btRigidBody::applyGravity</summary>
    public void ApplyGravity()
    {
        if (IsStaticOrKinematicObject)
            return;

        ApplyCentralForce(ref gravity);
    }

    /// <summary>btRigidBody::clearGravity</summary>
    public void ClearGravity()
    {
        if (IsStaticOrKinematicObject)
            return;

        Vector3 neg = -gravity;
        ApplyCentralForce(ref neg);
    }

    /// <summary>btRigidBody::setGravity</summary>
    public void SetGravity(ref Vector3 acceleration)
    {
        if (inverseMass != 0)
        {
            gravity = acceleration * (1 / inverseMass);
        }
        gravityAcceleration = acceleration;
    }

    public Vector3 Gravity
    {
        get => gravityAcceleration;
        set => SetGravity(ref value);
    }

    /// <summary>Matrix3x3 subtraction (the base layer does not provide operator-)</summary>
    private static Matrix3x3 Sub(in Matrix3x3 a, in Matrix3x3 b) =>
        new(a.Row0 - b.Row0, a.Row1 - b.Row1, a.Row2 - b.Row2);

    /// <summary>btRigidBody::setDamping (btClamped version)</summary>
    public void SetDamping(btScalar linDamping, btScalar angDamping)
    {
        linearDamping = BtMath.Clamp(linDamping, 0, 1);
        angularDamping = BtMath.Clamp(angDamping, 0, 1);
    }

    public btScalar LinearDamping => linearDamping;

    public btScalar AngularDamping => angularDamping;

    public btScalar LinearSleepingThreshold => linearSleepingThreshold;

    public btScalar AngularSleepingThreshold => angularSleepingThreshold;

    /// <summary>btRigidBody::applyDamping (btPow version)</summary>
    public void ApplyDamping(btScalar timeStep)
    {
        // 阻尼为 0 时 Pow(1, timeStep) == 1，乘法无效果，直接跳过
        if (linearDamping != 0)
        {
            linearVelocity *= BtMath.Pow(1 - linearDamping, timeStep);
        }
        if (angularDamping != 0)
        {
            angularVelocity *= BtMath.Pow(1 - angularDamping, timeStep);
        }

        if (additionalDamping)
        {
            // Additional damping avoids low-pass jitter and helps stabilize ragdolls
            if (angularVelocity.LengthSquared() < additionalAngularDampingThresholdSqr &&
                linearVelocity.LengthSquared() < additionalLinearDampingThresholdSqr)
            {
                angularVelocity *= additionalDampingFactor;
                linearVelocity *= additionalDampingFactor;
            }

            btScalar speed = linearVelocity.Length();
            if (speed < linearDamping)
            {
                btScalar dampVel = (btScalar)0.005;
                if (speed > dampVel)
                {
                    Vector3 dir = linearVelocity.Normalized();
                    linearVelocity -= dir * dampVel;
                }
                else
                {
                    linearVelocity = Vector3.Zero;
                }
            }

            btScalar angSpeed = angularVelocity.Length();
            if (angSpeed < angularDamping)
            {
                btScalar angDampVel = (btScalar)0.005;
                if (angSpeed > angDampVel)
                {
                    Vector3 dir = angularVelocity.Normalized();
                    angularVelocity -= dir * angDampVel;
                }
                else
                {
                    angularVelocity = Vector3.Zero;
                }
            }
        }
    }

    /// <summary>btRigidBody::setMassProps (TypedConstraint.GetFixedBody in the base layer depends on this method)</summary>
    public void SetMassProps(btScalar mass, Vector3 inertia)
    {
        if (mass == 0)
        {
            collisionFlags |= (int)BulletSharp.CollisionFlags.StaticObject;
            inverseMass = 0;
        }
        else
        {
            collisionFlags &= ~(int)BulletSharp.CollisionFlags.StaticObject;
            inverseMass = 1 / mass;
        }

        // Fg = m * a
        gravity = mass * gravityAcceleration;

        invInertiaLocal = new Vector3(
            inertia.X != 0 ? 1 / inertia.X : 0,
            inertia.Y != 0 ? 1 / inertia.Y : 0,
            inertia.Z != 0 ? 1 / inertia.Z : 0);

        invMass = linearFactor * inverseMass;
    }

    public Vector3 LinearFactor
    {
        get => linearFactor;
        set
        {
            linearFactor = value;
            invMass = linearFactor * inverseMass;
        }
    }

    public btScalar InvMass => inverseMass;

    public btScalar Mass => inverseMass == 0 ? 0 : 1 / inverseMass;

    public Matrix InvInertiaTensorWorld
    {
        get
        {
            Matrix m = default;
            m.M11 = invInertiaTensorWorld.Row0.X; m.M12 = invInertiaTensorWorld.Row1.X; m.M13 = invInertiaTensorWorld.Row2.X;
            m.M21 = invInertiaTensorWorld.Row0.Y; m.M22 = invInertiaTensorWorld.Row1.Y; m.M23 = invInertiaTensorWorld.Row2.Y;
            m.M31 = invInertiaTensorWorld.Row0.Z; m.M32 = invInertiaTensorWorld.Row1.Z; m.M33 = invInertiaTensorWorld.Row2.Z;
            m.M44 = 1;
            return m;
        }
    }

    internal ref Matrix3x3 InvInertiaTensorWorldInternal => ref invInertiaTensorWorld;

    /// <summary>btRigidBody::integrateVelocities</summary>
    public void IntegrateVelocities(btScalar step)
    {
        if (IsStaticOrKinematicObject)
            return;

        linearVelocity += totalForce * (inverseMass * step);
        angularVelocity += invInertiaTensorWorld * totalTorque * step;

        // MAX_ANGVEL = SIMD_HALF_PI: collision computations fail when the angular velocity is too large
        btScalar angvel = angularVelocity.Length();
        if (angvel * step > BtMath.SimdHalfPi)
        {
            angularVelocity *= BtMath.SimdHalfPi / step / angvel;
        }
    }

    /// <summary>btRigidBody::setCenterOfMassTransform</summary>
    public void SetCenterOfMassTransform(ref Matrix xform)
    {
        SetCenterOfMassTransformInternal(Transform.FromMatrix(in xform));
    }

    internal void SetCenterOfMassTransformInternal(in Transform xform)
    {
        if (IsKinematicObject)
        {
            interpolationWorldTransform = worldTransform;
        }
        else
        {
            interpolationWorldTransform = xform;
        }
        interpolationLinearVelocity = linearVelocity;
        interpolationAngularVelocity = angularVelocity;
        worldTransform = xform;
        UpdateInertiaTensor();
    }

    /// <summary>btRigidBody::applyCentralForce</summary>
    public void ApplyCentralForce(ref Vector3 force)
    {
        totalForce += force * linearFactor;
    }

    public Vector3 TotalForce => totalForce;

    public Vector3 TotalTorque => totalTorque;

    public Vector3 InvInertiaDiagLocal
    {
        get => invInertiaLocal;
        set => invInertiaLocal = value;
    }

    /// <summary>btRigidBody::setSleepingThresholds</summary>
    public void SetSleepingThresholds(btScalar linear, btScalar angular)
    {
        linearSleepingThreshold = linear;
        angularSleepingThreshold = angular;
    }

    /// <summary>btRigidBody::applyTorque</summary>
    public void ApplyTorque(ref Vector3 torque)
    {
        totalTorque += torque * angularFactor;
    }

    /// <summary>btRigidBody::applyForce</summary>
    public void ApplyForce(ref Vector3 force, ref Vector3 relPos)
    {
        ApplyCentralForce(ref force);
        Vector3 t = relPos.Cross(force * linearFactor);
        ApplyTorque(ref t);
    }

    /// <summary>btRigidBody::applyCentralImpulse</summary>
    public void ApplyCentralImpulse(ref Vector3 impulse)
    {
        linearVelocity += impulse * linearFactor * inverseMass;
    }

    /// <summary>btRigidBody::applyTorqueImpulse</summary>
    public void ApplyTorqueImpulse(ref Vector3 torque)
    {
        angularVelocity += invInertiaTensorWorld * torque * angularFactor;
    }

    /// <summary>btRigidBody::applyImpulse</summary>
    public void ApplyImpulse(ref Vector3 impulse, ref Vector3 relPos)
    {
        if (inverseMass != 0)
        {
            ApplyCentralImpulse(ref impulse);
            if (angularFactor != Vector3.Zero)
            {
                Vector3 t = relPos.Cross(impulse * linearFactor);
                ApplyTorqueImpulse(ref t);
            }
        }
    }

    /// <summary>btRigidBody::applyPushImpulse (split impulse positional correction)</summary>
    public void ApplyPushImpulse(ref Vector3 impulse, ref Vector3 relPos)
    {
        if (inverseMass != 0)
        {
            ApplyCentralPushImpulse(ref impulse);
            if (angularFactor != Vector3.Zero)
            {
                Vector3 t = relPos.Cross(impulse * linearFactor);
                ApplyTorqueTurnImpulse(ref t);
            }
        }
    }

    public Vector3 PushVelocity
    {
        get => pushVelocity;
        set => pushVelocity = value;
    }

    public Vector3 TurnVelocity
    {
        get => turnVelocity;
        set => turnVelocity = value;
    }

    /// <summary>btRigidBody::applyCentralPushImpulse</summary>
    public void ApplyCentralPushImpulse(ref Vector3 impulse)
    {
        pushVelocity += impulse * linearFactor * inverseMass;
    }

    /// <summary>btRigidBody::applyTorqueTurnImpulse</summary>
    public void ApplyTorqueTurnImpulse(ref Vector3 torque)
    {
        turnVelocity += invInertiaTensorWorld * torque * angularFactor;
    }

    /// <summary>btRigidBody::clearForces</summary>
    public void ClearForces()
    {
        totalForce = Vector3.Zero;
        totalTorque = Vector3.Zero;
    }

    /// <summary>btRigidBody::updateInertiaTensor</summary>
    public void UpdateInertiaTensor()
    {
        invInertiaTensorWorld = worldTransform.Basis.Scaled(invInertiaLocal) * worldTransform.Basis.Transpose();
    }

    public Vector3 CenterOfMassPosition => worldTransform.Origin;

    /// <summary>btRigidBody::getOrientation</summary>
    public Quaternion Orientation => worldTransform.Basis.GetRotation();

    public Matrix CenterOfMassTransform
    {
        get => worldTransform.ToMatrix();
        set => SetCenterOfMassTransform(ref value);
    }

    public Vector3 LinearVelocity
    {
        get => linearVelocity;
        set
        {
            UpdateRevisionInternal();
            linearVelocity = value;
        }
    }

    public Vector3 AngularVelocity
    {
        get => angularVelocity;
        set
        {
            UpdateRevisionInternal();
            angularVelocity = value;
        }
    }

    /// <summary>btRigidBody::getVelocityInLocalPoint</summary>
    public Vector3 GetVelocityInLocalPoint(ref Vector3 relPos)
    {
        // Linear/angular velocities are also computed for kinematic objects
        return linearVelocity + angularVelocity.Cross(relPos);
    }

    /// <summary>btRigidBody::getPushVelocityInLocalPoint</summary>
    public Vector3 GetPushVelocityInLocalPoint(ref Vector3 relPos)
    {
        return pushVelocity + turnVelocity.Cross(relPos);
    }

    /// <summary>btRigidBody::translate</summary>
    public void Translate(ref Vector3 v)
    {
        worldTransform.Origin += v;
    }

    /// <summary>btRigidBody::getAabb</summary>
    public void GetAabb(out Vector3 aabbMin, out Vector3 aabbMax)
    {
        CollisionShape.GetAabb(in worldTransform, out aabbMin, out aabbMax);
    }

    /// <summary>btRigidBody::computeImpulseDenominator</summary>
    public btScalar ComputeImpulseDenominator(Vector3 pos, Vector3 normal)
    {
        Vector3 r0 = pos - CenterOfMassPosition;
        Vector3 c0 = r0.Cross(normal);
        Vector3 vec = (c0 * invInertiaTensorWorld).Cross(r0);
        return inverseMass + normal.Dot(vec);
    }

    /// <summary>btRigidBody::computeAngularImpulseDenominator</summary>
    public btScalar ComputeAngularImpulseDenominator(Vector3 axis)
    {
        Vector3 vec = axis * invInertiaTensorWorld;
        return axis.Dot(vec);
    }

    /// <summary>btRigidBody::updateDeactivation</summary>
    internal void UpdateDeactivation(btScalar timeStep)
    {
        if (activationState1 == ActivationState.IslandSleeping ||
            activationState1 == ActivationState.DisableDeactivation)
            return;

        if (linearVelocity.LengthSquared() < linearSleepingThreshold * linearSleepingThreshold &&
            angularVelocity.LengthSquared() < angularSleepingThreshold * angularSleepingThreshold)
        {
            deactivationTime += timeStep;
        }
        else
        {
            deactivationTime = 0;
            activationState1 = (ActivationState)0;
        }
    }

    /// <summary>btRigidBody::wantsSleeping</summary>
    internal bool WantsSleeping()
    {
        if (activationState1 == ActivationState.DisableDeactivation)
            return false;

        // Globally disable deactivation
        if (GlobalDisableDeactivation || GlobalDeactivationTime == 0)
            return false;

        if (activationState1 == ActivationState.IslandSleeping ||
            activationState1 == ActivationState.WantsDeactivation)
            return true;

        if (deactivationTime > GlobalDeactivationTime)
        {
            return true;
        }
        return false;
    }

    public BroadphaseProxy BroadphaseProxy => broadphaseHandle;

    internal void SetNewBroadphaseProxy(BroadphaseProxy broadphaseProxy)
    {
        broadphaseHandle = broadphaseProxy;
    }

    /// <summary>btRigidBody::setMotionState (initializes the world transform from the motion state when set)</summary>
    public MotionState MotionState
    {
        get => optionalMotionState;
        set
        {
            optionalMotionState = value;
            if (optionalMotionState != null)
            {
                optionalMotionState.GetWorldTransform(out Matrix m);
                worldTransform = Transform.FromMatrix(in m);
            }
        }
    }

    public Vector3 AngularFactor
    {
        get => angularFactor;
        set
        {
            UpdateRevisionInternal();
            angularFactor = value;
        }
    }

    public void SetAngularFactor(btScalar angFac)
    {
        UpdateRevisionInternal();
        angularFactor = new Vector3(angFac, angFac, angFac);
    }

    /// <summary>btRigidBody::isInWorld (whether it has been added to a world/broadphase)</summary>
    public bool IsInWorld => broadphaseHandle != null;

    /// <summary>btRigidBody::addConstraintRef (also disables collision between the linked bodies)</summary>
    public void AddConstraintRef(TypedConstraint c)
    {
        int index = constraintRefs.IndexOf(c);
        // Do not add an already-referenced constraint again
        if (index == -1)
        {
            constraintRefs.Add(c);
            CollisionObject colObjA = c.rigidBodyA;
            CollisionObject colObjB = c.rigidBodyB;
            if (colObjA == this)
            {
                colObjA.SetIgnoreCollisionCheck(colObjB, true);
            }
            else
            {
                colObjB.SetIgnoreCollisionCheck(colObjA, true);
            }
        }
    }

    /// <summary>btRigidBody::removeConstraintRef</summary>
    public void RemoveConstraintRef(TypedConstraint c)
    {
        int index = constraintRefs.IndexOf(c);
        // Constraints that are not referenced are not removed
        if (index >= 0 && index < constraintRefs.Count)
        {
            constraintRefs.RemoveAt(index);
            CollisionObject colObjA = c.rigidBodyA;
            CollisionObject colObjB = c.rigidBodyB;
            if (colObjA == this)
            {
                colObjA.SetIgnoreCollisionCheck(colObjB, false);
            }
            else
            {
                colObjB.SetIgnoreCollisionCheck(colObjA, false);
            }
        }
    }

    public TypedConstraint GetConstraintRef(int index) => constraintRefs[index];

    public int NumConstraintRefs => constraintRefs.Count;

    public int Flags
    {
        get => rigidbodyFlags;
        set => rigidbodyFlags = value;
    }

    /// <summary>btRigidBody::getLocalInertia</summary>
    public Vector3 LocalInertia
    {
        get
        {
            Vector3 inertia = invInertiaLocal;
            return new Vector3(
                inertia.X != 0 ? 1 / inertia.X : 0,
                inertia.Y != 0 ? 1 / inertia.Y : 0,
                inertia.Z != 0 ? 1 / inertia.Z : 0);
        }
    }

    /// <summary>btMatrix3x3::solve33 (not provided by the base layer; private implementation)</summary>
    private static Vector3 Solve33(in Matrix3x3 m, in Vector3 b)
    {
        Vector3 col1 = m.Column(0);
        Vector3 col2 = m.Column(1);
        Vector3 col3 = m.Column(2);

        btScalar det = col1.Dot(col2.Cross(col3));
        if (BtMath.Abs(det) > BtMath.SimdEpsilon)
        {
            det = 1 / det;
        }
        else
        {
            det = 0;
        }
        return new Vector3(
            det * b.Dot(col2.Cross(col3)),
            det * col1.Dot(b.Cross(col3)),
            det * col1.Dot(col2.Cross(b)));
    }

    /// <summary>evalEulerEqn</summary>
    private static Vector3 EvalEulerEqn(in Vector3 w1, in Vector3 w0, in Vector3 t, btScalar dt, in Matrix3x3 i)
    {
        return i * w1 + w1.Cross(i * w1) * dt - (t * dt + i * w0);
    }

    /// <summary>evalEulerEqnDeriv</summary>
    private static Matrix3x3 EvalEulerEqnDeriv(in Vector3 w1, btScalar dt, in Matrix3x3 i)
    {
        Vector3 iwi = i * w1;
        Matrix3x3 w1X = SkewSymmetric(w1);
        Matrix3x3 iwiX = SkewSymmetric(iwi);
        return i + Sub(w1X * i, iwiX) * dt;
    }

    /// <summary>Skew-symmetric matrix generated by btVector3::getSkewSymmetricMatrix</summary>
    private static Matrix3x3 SkewSymmetric(in Vector3 v) => new(
        0, -v.Z, v.Y,
        v.Z, 0, -v.X,
        -v.Y, v.X, 0);

    /// <summary>btRigidBody::computeGyroscopicForceExplicit (the explicit version adds energy; avoid when possible)</summary>
    public Vector3 ComputeGyroscopicForceExplicit(btScalar maxGyroscopicForce)
    {
        Vector3 inertiaLocal = LocalInertia;
        Matrix3x3 inertiaTensorWorld = worldTransform.Basis.Scaled(inertiaLocal) * worldTransform.Basis.Transpose();
        Vector3 tmp = inertiaTensorWorld * angularVelocity;
        Vector3 gf = angularVelocity.Cross(tmp);
        btScalar l2 = gf.LengthSquared();
        if (l2 > maxGyroscopicForce * maxGyroscopicForce)
        {
            gf *= 1 / BtMath.Sqrt(l2) * maxGyroscopicForce;
        }
        return gf;
    }

    /// <summary>btRigidBody::computeGyroscopicImpulseImplicit_Body (implicit, body frame)</summary>
    public Vector3 ComputeGyroscopicImpulseImplicitBody(btScalar step)
    {
        Vector3 idl = LocalInertia;
        Vector3 omega1 = angularVelocity;
        Quaternion q = worldTransform.GetRotation();

        // Convert to body frame
        Vector3 omegab = Quaternion.Rotate(q.Inverse(), omega1);
        var ib = new Matrix3x3(
            idl.X, 0, 0,
            0, idl.Y, 0,
            0, 0, idl.Z);

        Vector3 ibo = ib * omegab;

        // Residual vector
        Vector3 f = step * omegab.Cross(ibo);

        Matrix3x3 skew0 = SkewSymmetric(omegab);
        Vector3 om = ib * omegab;
        Matrix3x3 skew1 = SkewSymmetric(om);

        // Jacobian
        Matrix3x3 j = ib + Sub(skew0 * ib, skew1) * step;

        Vector3 omegaDiv = Solve33(in j, in f);

        // Single Newton-Raphson update
        omegab -= omegaDiv;
        // Back to world frame
        Vector3 omega2 = Quaternion.Rotate(q, omegab);
        return omega2 - omega1;
    }

    /// <summary>btRigidBody::computeGyroscopicImpulseImplicit_World (implicit, world frame)</summary>
    public Vector3 ComputeGyroscopicImpulseImplicitWorld(btScalar step)
    {
        // Uses the full Newton-Euler equation; an implicit Euler step guarantees stability
        Vector3 inertiaLocal = LocalInertia;
        Vector3 w0 = angularVelocity;

        Matrix3x3 i = worldTransform.Basis.Scaled(inertiaLocal) * worldTransform.Basis.Transpose();

        // Implicitly solve for the new angular velocity w' with Newton's method
        // f(w') = -(T*step + Iw) + Iw' + w'xIw'*step = 0
        // df/dw' = I + 1xIw'*step + w'xI*step
        Vector3 w1 = w0;

        // One Newton step
        {
            Vector3 fw = EvalEulerEqn(in w1, in w0, in Vector3.Zero, step, in i);
            Matrix3x3 dfw = EvalEulerEqnDeriv(in w1, step, in i);

            Vector3 dw = Solve33(in dfw, in fw);
            w1 -= dw;
        }

        return w1 - w0;
    }
}
