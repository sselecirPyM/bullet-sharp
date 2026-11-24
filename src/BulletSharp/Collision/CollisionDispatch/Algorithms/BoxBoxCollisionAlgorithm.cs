using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btBoxBoxCollisionAlgorithm: box-box collision detection (using the
/// BoxBoxDetector / ODE routine internally).
/// </summary>
public class BoxBoxCollisionAlgorithm : ActivatingCollisionAlgorithm
{
    internal bool ownManifold;
    internal PersistentManifold manifoldPtr;

    // Reused detector and input (stack objects in C++ processCollision; here cached per collision pair)
    private DiscreteCollisionDetectorInterface.ClosestPointInput input;
    private BoxBoxDetector detector;

    public BoxBoxCollisionAlgorithm(ConstructionInfo ci)
        : base(ci)
    {
    }

    public BoxBoxCollisionAlgorithm(PersistentManifold mf, ConstructionInfo ci,
        CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap)
        : base(ci, body0Wrap, body1Wrap)
    {
        manifoldPtr = mf;
        if (manifoldPtr == null && dispatcher.NeedsCollision(body0Wrap.CollisionObject, body1Wrap.CollisionObject))
        {
            manifoldPtr = dispatcher.GetNewManifold(body0Wrap.CollisionObject, body1Wrap.CollisionObject);
            ownManifold = true;
        }
    }

    /// <summary>btBoxBoxCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm</summary>
    public static CollisionAlgorithm Create(ConstructionInfo ci, CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new BoxBoxCollisionAlgorithm(null, ci, body0Wrap, body1Wrap);

    /// <summary>btBoxBoxCollisionAlgorithm::processCollision（USE_PERSISTENT_CONTACTS）</summary>
    public override void ProcessCollision(CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        if (manifoldPtr == null)
            return;

        var box0 = (BoxShape)body0Wrap.CollisionShape;
        var box1 = (BoxShape)body1Wrap.CollisionShape;

        // Report contact points; internally they are persisted and reduced
        resultOut.SetPersistentManifold(manifoldPtr);

        input ??= new DiscreteCollisionDetectorInterface.ClosestPointInput();
        input.MaximumDistanceSquared = BtMath.BtLargeFloat;
        input.transformA = body0Wrap.WorldTransformInternal;
        input.transformB = body1Wrap.WorldTransformInternal;

        if (detector == null || detector.box1 != box0 || detector.box2 != box1)
            detector = new BoxBoxDetector(box0, box1);
        detector.GetClosestPoints(input, resultOut, dispatchInfo.DebugDraw);

        // refreshContactPoints is only needed when using persistent contacts; otherwise all points are new
        if (ownManifold)
        {
            resultOut.RefreshContactPoints();
        }
    }

    public override btScalar CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        // not yet
        return 1;
    }

    public override void GetAllContactManifolds(List<PersistentManifold> manifoldArray)
    {
        if (manifoldPtr != null && ownManifold)
            manifoldArray.Add(manifoldPtr);
    }

    internal override void Cleanup(Dispatcher dispatcher)
    {
        if (ownManifold && manifoldPtr != null)
        {
            dispatcher.ReleaseManifold(manifoldPtr);
            manifoldPtr = null;
        }
    }
}
