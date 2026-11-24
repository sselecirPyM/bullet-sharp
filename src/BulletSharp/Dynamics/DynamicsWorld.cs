using BulletSharp.Math;

namespace BulletSharp;

/// <summary>btDynamicsWorldType</summary>
public enum DynamicsWorldType
{
    Simple = 1,
    Discrete = 2,
    Continuous = 3,
    SoftRigid = 4,
    Gpu = 5,
    SoftMultiBody = 6,
    DeformableMultiBody = 7,
}

/// <summary>
/// btDynamicsWorld / BulletSharp DynamicsWorld abstract class.
/// </summary>
public abstract class DynamicsWorld : CollisionWorld
{
    protected DynamicsWorld(Dispatcher dispatcher, BroadphaseInterface broadphase, CollisionConfiguration collisionConfiguration)
        : base(dispatcher, broadphase, collisionConfiguration)
    {
    }

    public abstract void AddConstraint(TypedConstraint constraint, bool disableCollisionsBetweenLinkedBodies = false);

    public abstract void RemoveConstraint(TypedConstraint constraint);

    public abstract void AddRigidBody(RigidBody body);

    public abstract void AddRigidBody(RigidBody body, CollisionFilterGroups group, CollisionFilterGroups mask);

    public void AddRigidBody(RigidBody body, int group, int mask) =>
        AddRigidBody(body, (CollisionFilterGroups)group, (CollisionFilterGroups)mask);

    public abstract void RemoveRigidBody(RigidBody body);

    public abstract int NumConstraints { get; }

    public abstract TypedConstraint GetConstraint(int index);

    public abstract int StepSimulation(btScalar timeStep, int maxSubSteps = 1, btScalar fixedTimeStep = (btScalar)(1.0 / 60.0));

    public abstract void SynchronizeMotionStates();

    public abstract void SetGravity(ref Vector3 gravity);

    public abstract Vector3 Gravity { get; set; }

    public abstract void ClearForces();

    public abstract ConstraintSolver Solver { get; set; }

    public abstract object WorldUserInfo { get; set; }

    public virtual DynamicsWorldType WorldType => DynamicsWorldType.Discrete;

    /// <summary>Provides a double convenience overload in float-precision builds (compatible with BulletSharp.x64's StepSimulation(double))</summary>
#if !BT_USE_DOUBLE_PRECISION
    public int StepSimulation(double timeStep, int maxSubSteps = 1, double fixedTimeStep = 1.0 / 60.0) =>
        StepSimulation((btScalar)timeStep, maxSubSteps, (btScalar)fixedTimeStep);
#endif
}
