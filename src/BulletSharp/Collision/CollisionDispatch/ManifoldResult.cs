using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btManifoldResult: bridge between collision algorithms and PersistentManifold.
/// </summary>
public class ManifoldResult : DiscreteCollisionDetectorInterface.Result
{
    internal PersistentManifold manifoldPtr;
    internal CollisionObjectWrapper body0Wrap;
    internal CollisionObjectWrapper body1Wrap;
    internal CollisionObject rootObjectA;
    internal CollisionObject rootObjectB;
    internal int partId0 = -1;
    internal int partId1 = -1;
    internal int index0 = -1;
    internal int index1 = -1;

    /// <summary>btManifoldResult::setShapeIdentifiersA</summary>
    public override void SetShapeIdentifiersA(int partId0, int index0)
    {
        this.partId0 = partId0;
        this.index0 = index0;
    }

    /// <summary>btManifoldResult::setShapeIdentifiersB</summary>
    public override void SetShapeIdentifiersB(int partId1, int index1)
    {
        this.partId1 = partId1;
        this.index1 = index1;
    }

    public ManifoldResult()
    {
    }

    public ManifoldResult(CollisionObject body0, CollisionObject body1)
    {
        rootObjectA = body0;
        rootObjectB = body1;
        body0Wrap = new CollisionObjectWrapper(null, body0.CollisionShape, body0, body0.worldTransform, -1, -1);
        body1Wrap = new CollisionObjectWrapper(null, body1.CollisionShape, body1, body1.worldTransform, -1, -1);
    }

    internal ManifoldResult(CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap)
    {
        Reset(body0Wrap, body1Wrap);
    }

    /// <summary>In-place reuse (equivalent to the stack object in C++ defaultNearCallback).</summary>
    internal void Reset(CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap)
    {
        this.body0Wrap = body0Wrap;
        this.body1Wrap = body1Wrap;
        rootObjectA = body0Wrap.collisionObject;
        rootObjectB = body1Wrap.collisionObject;
        manifoldPtr = null;
        partId0 = -1;
        partId1 = -1;
        index0 = -1;
        index1 = -1;
    }

    public PersistentManifold PersistentManifold
    {
        get => manifoldPtr;
        set => manifoldPtr = value;
    }

    internal void SetPersistentManifold(PersistentManifold manifold) => manifoldPtr = manifold;

    public CollisionObjectWrapper Body0Wrap => body0Wrap;
    public CollisionObjectWrapper Body1Wrap => body1Wrap;
    public CollisionObject Body0Internal => rootObjectA;
    public CollisionObject Body1Internal => rootObjectB;

    /// <summary>Calculate the combined friction coefficient of two bodies (clamped to ±10).</summary>
    public static btScalar CalculateCombinedFriction(CollisionObject body0, CollisionObject body1)
    {
        btScalar friction = body0.friction * body1.friction;
        const btScalar maxFriction = 10;
        if (friction < -maxFriction) friction = -maxFriction;
        if (friction > maxFriction) friction = maxFriction;
        return friction;
    }

    public static btScalar CalculateCombinedRollingFriction(CollisionObject body0, CollisionObject body1) =>
        body0.rollingFriction * body1.rollingFriction;

    public static btScalar CalculateCombinedSpinningFriction(CollisionObject body0, CollisionObject body1) =>
        body0.spinningFriction * body1.spinningFriction;

    public static btScalar CalculateCombinedRestitution(CollisionObject body0, CollisionObject body1) =>
        body0.restitution * body1.restitution;

    public static btScalar CalculateCombinedContactDamping(CollisionObject body0, CollisionObject body1)
    {
        btScalar damping = body0.contactDamping + body1.contactDamping;
        if (damping < 0) damping = 0;
        else if (damping > 1) damping = 1;
        return damping;
    }

    public static btScalar CalculateCombinedContactStiffness(CollisionObject body0, CollisionObject body1)
    {
        btScalar stiffness = body0.contactStiffness + body1.contactStiffness;
        if (stiffness < 0) stiffness = 0;
        return stiffness;
    }

    // Scratch contact point: reused to avoid allocating per contact point per step (a stack object in C++).
    private readonly ManifoldPoint scratchPoint = new();

    public override void AddContactPoint(Vector3 normalOnBInWorld, Vector3 pointInWorld, btScalar depth)
    {
        if (depth > manifoldPtr.contactBreakingThreshold)
            return;

        bool isSwapped = manifoldPtr.body0 != body0Wrap.collisionObject;

        Vector3 pointA = pointInWorld + normalOnBInWorld * depth;

        Vector3 localA;
        Vector3 localB;
        if (isSwapped)
        {
            localA = body1Wrap.collisionObject.worldTransform.InverseXForm(pointA);
            localB = body0Wrap.collisionObject.worldTransform.InverseXForm(pointInWorld);
        }
        else
        {
            localA = body0Wrap.collisionObject.worldTransform.InverseXForm(pointA);
            localB = body1Wrap.collisionObject.worldTransform.InverseXForm(pointInWorld);
        }

        ManifoldPoint newPt = scratchPoint;
        newPt.Set(localA, localB, normalOnBInWorld, depth);
        newPt.positionWorldOnA = pointA;
        newPt.positionWorldOnB = pointInWorld;

        int insertIndex = manifoldPtr.GetCacheEntry(newPt);

        newPt.combinedFriction = CalculateCombinedFriction(body0Wrap.collisionObject, body1Wrap.collisionObject);
        newPt.combinedRestitution = CalculateCombinedRestitution(body0Wrap.collisionObject, body1Wrap.collisionObject);
        newPt.combinedRollingFriction = CalculateCombinedRollingFriction(body0Wrap.collisionObject, body1Wrap.collisionObject);
        newPt.combinedSpinningFriction = CalculateCombinedSpinningFriction(body0Wrap.collisionObject, body1Wrap.collisionObject);

        if ((body0Wrap.collisionObject.collisionFlags & (int)CollisionFlags.HasContactStiffnessDamping) != 0 ||
            (body1Wrap.collisionObject.collisionFlags & (int)CollisionFlags.HasContactStiffnessDamping) != 0)
        {
            newPt.contactErp = CalculateCombinedContactDamping(body0Wrap.collisionObject, body1Wrap.collisionObject);
            newPt.contactCfm = CalculateCombinedContactStiffness(body0Wrap.collisionObject, body1Wrap.collisionObject);
            newPt.contactPointFlags |= ContactPointFlags.ContactStiffnessDamping;
        }

        if ((body0Wrap.collisionObject.collisionFlags & (int)CollisionFlags.HasFrictionAnchor) != 0 ||
            (body1Wrap.collisionObject.collisionFlags & (int)CollisionFlags.HasFrictionAnchor) != 0)
        {
            newPt.contactPointFlags |= ContactPointFlags.FrictionAnchor;
        }

        BtMath.PlaneSpace1(newPt.normalWorldOnB, out newPt.lateralFrictionDir1, out newPt.lateralFrictionDir2);

        if (isSwapped)
        {
            newPt.partId0 = partId1;
            newPt.partId1 = partId0;
            newPt.index0 = index1;
            newPt.index1 = index0;
        }
        else
        {
            newPt.partId0 = partId0;
            newPt.partId1 = partId1;
            newPt.index0 = index0;
            newPt.index1 = index1;
        }

        if (insertIndex >= 0)
        {
            manifoldPtr.ReplaceContactPoint(newPt, insertIndex);
        }
        else
        {
            manifoldPtr.AddManifoldPoint(newPt);
        }
    }

    /// <summary>Refresh the world coordinates of the manifold's contact points (selects the correct transforms based on the swap state).</summary>
    public void RefreshContactPoints()
    {
        if (manifoldPtr == null || manifoldPtr.NumContacts == 0)
            return;

        bool isSwapped = manifoldPtr.body0 != body0Wrap.collisionObject;
        if (isSwapped)
        {
            manifoldPtr.RefreshContactPoints(body1Wrap.collisionObject.worldTransform, body0Wrap.collisionObject.worldTransform);
        }
        else
        {
            manifoldPtr.RefreshContactPoints(body0Wrap.collisionObject.worldTransform, body1Wrap.collisionObject.worldTransform);
        }
    }
}
