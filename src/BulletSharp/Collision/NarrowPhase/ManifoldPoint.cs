using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btManifoldPoint. A reference type so PersistentManifold can replace entries in place.
/// Internal field names map one-to-one to btManifoldPoint members (camelCase).
/// </summary>
public class ManifoldPoint
{
    internal Vector3 localPointA;
    internal Vector3 localPointB;
    internal Vector3 positionWorldOnB;
    internal Vector3 positionWorldOnA;
    internal Vector3 normalWorldOnB;

    internal btScalar distance1;
    internal btScalar combinedFriction;
    internal btScalar combinedRollingFriction;
    internal btScalar combinedSpinningFriction;
    internal btScalar combinedRestitution;

    internal int partId0 = -1;
    internal int partId1 = -1;
    internal int index0 = -1;
    internal int index1 = -1;

    internal object userPersistentData;
    internal ContactPointFlags contactPointFlags;

    internal btScalar appliedImpulse;
    internal btScalar prevRhs;
    internal btScalar appliedImpulseLateral1;
    internal btScalar appliedImpulseLateral2;
    internal btScalar contactMotion1;
    internal btScalar contactMotion2;

    // union { m_contactCFM, m_combinedContactStiffness1 }
    internal btScalar contactCfm;
    // union { m_contactERP, m_combinedContactDamping1 }
    internal btScalar contactErp;
    internal btScalar frictionCfm;

    internal int lifeTime;

    internal Vector3 lateralFrictionDir1;
    internal Vector3 lateralFrictionDir2;

    public ManifoldPoint()
    {
    }

    public ManifoldPoint(Vector3 pointA, Vector3 pointB, Vector3 normal, btScalar distance)
    {
        Set(pointA, pointB, normal, distance);
    }

    /// <summary>Fully resets to a newly constructed state (for scratch reuse; equivalent to reconstructing on the stack in C++).</summary>
    internal void Set(Vector3 pointA, Vector3 pointB, Vector3 normal, btScalar distance)
    {
        localPointA = pointA;
        localPointB = pointB;
        positionWorldOnB = Vector3.Zero;
        positionWorldOnA = Vector3.Zero;
        normalWorldOnB = normal;
        distance1 = distance;
        combinedFriction = 0;
        combinedRollingFriction = 0;
        combinedSpinningFriction = 0;
        combinedRestitution = 0;
        partId0 = -1;
        partId1 = -1;
        index0 = -1;
        index1 = -1;
        userPersistentData = null;
        contactPointFlags = 0;
        appliedImpulse = 0;
        prevRhs = 0;
        appliedImpulseLateral1 = 0;
        appliedImpulseLateral2 = 0;
        contactMotion1 = 0;
        contactMotion2 = 0;
        contactCfm = 0;
        contactErp = 0;
        frictionCfm = 0;
        lifeTime = 0;
        lateralFrictionDir1 = Vector3.Zero;
        lateralFrictionDir2 = Vector3.Zero;
    }

    /// <summary>Copies by value (replaces the C++ assignment operator)</summary>
    internal void CopyFrom(ManifoldPoint other)
    {
        localPointA = other.localPointA;
        localPointB = other.localPointB;
        positionWorldOnB = other.positionWorldOnB;
        positionWorldOnA = other.positionWorldOnA;
        normalWorldOnB = other.normalWorldOnB;
        distance1 = other.distance1;
        combinedFriction = other.combinedFriction;
        combinedRollingFriction = other.combinedRollingFriction;
        combinedSpinningFriction = other.combinedSpinningFriction;
        combinedRestitution = other.combinedRestitution;
        partId0 = other.partId0;
        partId1 = other.partId1;
        index0 = other.index0;
        index1 = other.index1;
        userPersistentData = other.userPersistentData;
        contactPointFlags = other.contactPointFlags;
        appliedImpulse = other.appliedImpulse;
        prevRhs = other.prevRhs;
        appliedImpulseLateral1 = other.appliedImpulseLateral1;
        appliedImpulseLateral2 = other.appliedImpulseLateral2;
        contactMotion1 = other.contactMotion1;
        contactMotion2 = other.contactMotion2;
        contactCfm = other.contactCfm;
        contactErp = other.contactErp;
        frictionCfm = other.frictionCfm;
        lifeTime = other.lifeTime;
        lateralFrictionDir1 = other.lateralFrictionDir1;
        lateralFrictionDir2 = other.lateralFrictionDir2;
    }

    internal btScalar CombinedContactStiffness1 { get => contactCfm; set => contactCfm = value; }
    internal btScalar CombinedContactDamping1 { get => contactErp; set => contactErp = value; }

    public btScalar Distance
    {
        get => distance1;
        set => distance1 = value;
    }

    public int LifeTime => lifeTime;

    public Vector3 PositionWorldOnA => positionWorldOnA;

    public Vector3 PositionWorldOnB => positionWorldOnB;

    public Vector3 NormalWorldOnB
    {
        get => normalWorldOnB;
        set => normalWorldOnB = value;
    }

    public Vector3 LocalPointA => localPointA;

    public Vector3 LocalPointB => localPointB;

    public btScalar AppliedImpulse => appliedImpulse;

    public btScalar AppliedImpulseLateral1 => appliedImpulseLateral1;

    public btScalar AppliedImpulseLateral2 => appliedImpulseLateral2;

    public btScalar CombinedFriction => combinedFriction;

    public btScalar CombinedRestitution => combinedRestitution;

    public btScalar CombinedRollingFriction => combinedRollingFriction;

    public Vector3 LateralFrictionDir1 => lateralFrictionDir1;

    public Vector3 LateralFrictionDir2 => lateralFrictionDir2;

    public int PartId0 => partId0;
    public int PartId1 => partId1;
    public int Index0 => index0;
    public int Index1 => index1;

    public object UserPersistentData
    {
        get => userPersistentData;
        set => userPersistentData = value;
    }

    internal bool IsLateralFrictionInitialized =>
        (contactPointFlags & ContactPointFlags.LateralFrictionInitialized) != 0;
}
