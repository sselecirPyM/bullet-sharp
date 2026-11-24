using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btSphereTriangleCollisionAlgorithm: sphere-triangle collision
/// detection, supporting frame-to-frame coherence (persistent contacts) and
/// collision response.
/// </summary>
public class SphereTriangleCollisionAlgorithm : ActivatingCollisionAlgorithm
{
    internal bool ownManifold;
    internal PersistentManifold manifoldPtr;
    internal bool swapped;

    // Reused detection temporaries (stack objects in C++ processCollision)
    private readonly SphereTriangleDetector detector = new(null, null, 0);
    private readonly DiscreteCollisionDetectorInterface.ClosestPointInput input = new();

    public SphereTriangleCollisionAlgorithm(ConstructionInfo ci)
        : base(ci)
    {
    }

    public SphereTriangleCollisionAlgorithm(PersistentManifold mf, ConstructionInfo ci,
        CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap, bool swapped)
        : base(ci, body0Wrap, body1Wrap)
    {
        manifoldPtr = mf;
        this.swapped = swapped;
        if (manifoldPtr == null)
        {
            manifoldPtr = dispatcher.GetNewManifold(body0Wrap.CollisionObject, body1Wrap.CollisionObject);
            ownManifold = true;
        }
    }

    /// <summary>btSphereTriangleCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm (not swapped)</summary>
    public static CollisionAlgorithm Create(ConstructionInfo ci, CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new SphereTriangleCollisionAlgorithm(ci.Manifold, ci, body0Wrap, body1Wrap, false);

    /// <summary>btSphereTriangleCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm (swapped, m_swapped=true)</summary>
    public static CollisionAlgorithm SwappedCreate(ConstructionInfo ci, CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new SphereTriangleCollisionAlgorithm(ci.Manifold, ci, body0Wrap, body1Wrap, true);

    /// <summary>btSphereTriangleCollisionAlgorithm::processCollision</summary>
    public override void ProcessCollision(CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        if (manifoldPtr == null)
            return;

        CollisionObjectWrapper sphereObjWrap = swapped ? body1Wrap : body0Wrap;
        CollisionObjectWrapper triObjWrap = swapped ? body0Wrap : body1Wrap;

        var sphere = (SphereShape)sphereObjWrap.CollisionShape;
        var triangle = (TriangleShape)triObjWrap.CollisionShape;

        // report contact points; internally they are persisted and reduced
        resultOut.SetPersistentManifold(manifoldPtr);
        // (C++ also adds resultOut->m_closestPointDistanceThreshold here; the current base layer has no such field, treated as 0)
        detector.Set(sphere, triangle, manifoldPtr.ContactBreakingThreshold);

        input.transformA = sphereObjWrap.WorldTransformInternal;
        input.transformB = triObjWrap.WorldTransformInternal;
        input.MaximumDistanceSquared = BtMath.BtLargeFloat; ///@todo: tighter bounds

        bool swapResults = swapped;

        detector.GetClosestPoints(input, resultOut, dispatchInfo.DebugDraw, swapResults);

        if (ownManifold)
            resultOut.RefreshContactPoints();
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
