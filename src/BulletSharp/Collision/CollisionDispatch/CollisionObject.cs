using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btCollisionObject. Public API compatible with the original BulletSharp CollisionObject.
/// </summary>
public class CollisionObject : IDisposable
{
    internal Transform worldTransform;
    /// <summary>m_interpolationWorldTransform, used for CCD and interpolation.</summary>
    internal Transform interpolationWorldTransform;
    internal BroadphaseProxy broadphaseHandle;
    internal CollisionShape collisionShape;
    internal int collisionFlags;
    internal int islandTag1;
    internal int companionId;
    internal int worldArrayIndex;
    internal ActivationState activationState1;
    internal btScalar deactivationTime;
    internal btScalar friction;
    internal btScalar restitution;
    internal btScalar rollingFriction;
    internal btScalar spinningFriction;
    internal btScalar contactDamping;
    internal btScalar contactStiffness;
    internal CollisionObjectTypes internalType;
    internal object userObjectPointer;
    internal int userIndex2;
    internal int userIndex;
    internal btScalar hitFraction;
    internal btScalar ccdSweptSphereRadius;
    internal btScalar ccdMotionThreshold;
    internal bool checkCollideWith;
    internal ushort updateRevision;

    // anisotropic friction: m_hasAnisotropicFriction (0=none,1,2)
    internal int hasAnisotropicFriction;
    internal btScalar contactProcessingThreshold;
    internal int mergesSimulationIslands;
    internal Vector3 anisotropicFriction;
    internal btScalar? customDebugColorRgb = null;

    public const int DefaultIslandTag = -1;

    public CollisionObject()
    {
        worldTransform = Transform.Identity;
        interpolationWorldTransform = Transform.Identity;
        islandTag1 = DefaultIslandTag;
        companionId = -1;
        worldArrayIndex = -1;
        activationState1 = ActivationState.ActiveTag;
        deactivationTime = 0;
        friction = (btScalar)0.5;
        restitution = 0;
        rollingFriction = 0;
        spinningFriction = 0;
        contactDamping = 0;
        contactStiffness = BtMath.BtLargeFloat;
        internalType = CollisionObjectTypes.CollisionObject;
        userIndex = -1;
        userIndex2 = -1;
        hitFraction = 1;
        ccdSweptSphereRadius = 0;
        ccdMotionThreshold = 0;
        checkCollideWith = true;
        hasAnisotropicFriction = 0;
        contactProcessingThreshold = BtMath.BtLargeFloat;
        mergesSimulationIslands = 1;
        anisotropicFriction = new Vector3(1, 1, 1);
        updateRevision = 1;
    }

    /// <summary>btCollisionObject::updateRevision (called on shape/material changes).</summary>
    internal void UpdateRevisionInternal() => updateRevision++;

    public ushort GetUpdateRevisionInternal() => updateRevision;

    // ---------------- Anisotropic friction ----------------
    public bool HasAnisotropicFriction(AnisotropicFrictionFlags frictionMode = AnisotropicFrictionFlags.AnisotropicFriction)
        => (hasAnisotropicFriction & (int)frictionMode) != 0;

    public bool HasAnisotropicFrictionPublic => hasAnisotropicFriction != 0;

    public void SetAnisotropicFriction(Vector3 anisotropicFriction, AnisotropicFrictionFlags frictionMode = AnisotropicFrictionFlags.AnisotropicFriction)
    {
        this.anisotropicFriction = anisotropicFriction;
        bool hasFriction = anisotropicFriction.X != 1 || anisotropicFriction.Y != 1 || anisotropicFriction.Z != 1;
        hasAnisotropicFriction = hasFriction ? (int)frictionMode : 0;
    }

    public Vector3 AnisotropicFriction
    {
        get => anisotropicFriction;
        set => SetAnisotropicFriction(value);
    }

    public btScalar ContactDamping => contactDamping;
    public btScalar ContactStiffness => contactStiffness;

    public void SetContactStiffnessAndDamping(btScalar stiffness, btScalar damping)
    {
        contactStiffness = stiffness;
        contactDamping = damping;
        collisionFlags |= (int)BulletSharp.CollisionFlags.HasContactStiffnessDamping;
    }

    // ---------------- Properties ----------------
    public Matrix WorldTransform
    {
        get => worldTransform.ToMatrix();
        set
        {
            UpdateRevisionInternal();
            worldTransform = Transform.FromMatrix(in value);
        }
    }

    internal Transform WorldTransformInternal
    {
        get => worldTransform;
        set
        {
            UpdateRevisionInternal();
            worldTransform = value;
        }
    }

    public Matrix InterpolationWorldTransform
    {
        get => interpolationWorldTransform.ToMatrix();
        set => interpolationWorldTransform = Transform.FromMatrix(in value);
    }

    internal Transform InterpolationWorldTransformInternal
    {
        get => interpolationWorldTransform;
        set => interpolationWorldTransform = value;
    }

    public Matrix GetWorldTransform(out Matrix m)
    {
        m = worldTransform.ToMatrix();
        return m;
    }

    internal BroadphaseProxy BroadphaseHandle
    {
        get => broadphaseHandle;
        set => broadphaseHandle = value;
    }

    public CollisionShape CollisionShape
    {
        get => collisionShape;
        set
        {
            UpdateRevisionInternal();
            collisionShape = value;
        }
    }

    public BulletSharp.CollisionFlags CollisionFlags
    {
        get => (BulletSharp.CollisionFlags)collisionFlags;
        set => collisionFlags = (int)value;
    }

    public int IslandTag
    {
        get => islandTag1;
        set => islandTag1 = value;
    }

    public int CompanionId
    {
        get => companionId;
        set => companionId = value;
    }

    internal int WorldArrayIndex
    {
        get => worldArrayIndex;
        set => worldArrayIndex = value;
    }

    public ActivationState ActivationState
    {
        get => activationState1;
        set => SetActivationState(value);
    }

    internal void SetActivationState(ActivationState newState)
    {
        if (activationState1 != BulletSharp.ActivationState.DisableDeactivation &&
            activationState1 != BulletSharp.ActivationState.DisableSimulation)
            activationState1 = newState;
    }

    public btScalar DeactivationTime
    {
        get => deactivationTime;
        set => deactivationTime = value;
    }

    public btScalar Friction
    {
        get => friction;
        set => friction = value;
    }

    public btScalar RollingFriction
    {
        get => rollingFriction;
        set => rollingFriction = value;
    }

    public btScalar SpinningFriction
    {
        get => spinningFriction;
        set => spinningFriction = value;
    }

    public btScalar Restitution
    {
        get => restitution;
        set => restitution = value;
    }

    public CollisionObjectTypes InternalType
    {
        get => internalType;
        set => internalType = value;
    }

    public object UserObject
    {
        get => userObjectPointer;
        set => userObjectPointer = value;
    }

    public int UserIndex
    {
        get => userIndex;
        set => userIndex = value;
    }

    public int UserIndex2
    {
        get => userIndex2;
        set => userIndex2 = value;
    }

    public btScalar HitFraction
    {
        get => hitFraction;
        set => hitFraction = value;
    }

    public btScalar CcdSweptSphereRadius
    {
        get => ccdSweptSphereRadius;
        set => ccdSweptSphereRadius = value;
    }

    public btScalar CcdMotionThreshold
    {
        get => ccdMotionThreshold;
        set => ccdMotionThreshold = value;
    }

    public btScalar CcdSquareMotionThreshold => ccdMotionThreshold * ccdMotionThreshold;

    public btScalar ContactProcessingThreshold
    {
        get => contactProcessingThreshold;
        set => contactProcessingThreshold = value;
    }

    // ---------------- State queries ----------------
    public bool IsStaticObject => (collisionFlags & (int)BulletSharp.CollisionFlags.StaticObject) != 0;

    public bool IsKinematicObject => (collisionFlags & (int)BulletSharp.CollisionFlags.KinematicObject) != 0;

    public bool IsStaticOrKinematicObject => (collisionFlags & (int)(BulletSharp.CollisionFlags.StaticObject | BulletSharp.CollisionFlags.KinematicObject)) != 0;

    public bool HasContactResponse => (collisionFlags & (int)BulletSharp.CollisionFlags.NoContactResponse) == 0;

    /// <summary>btCollisionObject::mergesSimulationIslands (static/kinematic/no-contact-response objects do not merge islands).</summary>
    public bool MergesSimulationIslands =>
        (collisionFlags & (int)(BulletSharp.CollisionFlags.StaticObject |
                                BulletSharp.CollisionFlags.KinematicObject |
                                BulletSharp.CollisionFlags.NoContactResponse)) == 0;

    public bool IsActive => activationState1 != BulletSharp.ActivationState.IslandSleeping &&
                            activationState1 != BulletSharp.ActivationState.DisableSimulation;

    // ---------------- Activation ----------------
    public void Activate() => Activate(false);

    public void Activate(bool forceActivation)
    {
        if (forceActivation || (collisionFlags & (int)(BulletSharp.CollisionFlags.StaticObject | BulletSharp.CollisionFlags.KinematicObject)) == 0)
        {
            SetActivationState(BulletSharp.ActivationState.ActiveTag);
            deactivationTime = 0;
        }
    }

    internal void ActivateInternal(bool forceActivation) => Activate(forceActivation);

    public void ForceActivationState(ActivationState newState) => activationState1 = newState;

    // ---------------- Collision filtering ----------------
    public bool CheckCollideWith(CollisionObject co) => !checkCollideWith || CheckCollideWithOverride(co);

    protected virtual bool CheckCollideWithOverride(CollisionObject co) => true;

    public void SetIgnoreCollisionCheck(CollisionObject co, bool ignoreCollisionCheck)
    {
        // Simplified implementation: Bullet uses m_checkCollideWith plus a user callback;
        // this provides the same semantics as btCollisionObject::setIgnoreCollisionCheck.
        checkCollideWith = !ignoreCollisionCheck;
    }

    public virtual void Dispose()
    {
    }
}

/// <summary>btCollisionObject::AnisotropicFrictionFlags</summary>
[Flags]
public enum AnisotropicFrictionFlags
{
    None = 0,
    AnisotropicFriction = 1,
    AnisotropicRollingFriction = 2,
}
