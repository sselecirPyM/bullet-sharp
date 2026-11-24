using BulletSharp.Math;

namespace BulletSharp;

/// <summary>btDispatcher::btDispatcherQueryType</summary>
public enum DispatcherQueryType
{
    ContactPointAlgorithms = 1,
    ClosestConvexAlgorithms = 2,
}

/// <summary>Near callback delegate of btCollisionPairCallback.</summary>
public delegate void NearCallback(BroadphasePair collisionPair, Dispatcher dispatcher, DispatcherInfo dispatchInfo);

/// <summary>
/// Ported from btDispatcher. Abstract base class Dispatcher in BulletSharp.
/// </summary>
public abstract class Dispatcher : IDisposable
{
    /// <summary>btDispatcher::needsCollision default implementation.</summary>
    public virtual bool NeedsCollision(CollisionObject body0, CollisionObject body1)
    {
        bool needsCollision = true;

        if (!body0.IsActive && !body1.IsActive)
            needsCollision = false;
        else if (!body0.CheckCollideWith(body1) || !body1.CheckCollideWith(body0))
            needsCollision = false;

        return needsCollision;
    }

    /// <summary>btDispatcher::needsResponse default implementation.</summary>
    public virtual bool NeedsResponse(CollisionObject body0, CollisionObject body1)
    {
        bool hasResponse = body0.HasContactResponse && body1.HasContactResponse;
        hasResponse = hasResponse && (!body0.IsStaticOrKinematicObject || !body1.IsStaticOrKinematicObject);
        return hasResponse;
    }

    public abstract CollisionAlgorithm FindAlgorithm(CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap,
        PersistentManifold sharedManifold = null, DispatcherQueryType algoType = DispatcherQueryType.ContactPointAlgorithms);

    public abstract PersistentManifold GetNewManifold(CollisionObject body0, CollisionObject body1);

    public abstract void ReleaseManifold(PersistentManifold manifold);

    public virtual void ClearManifold(PersistentManifold manifold) => manifold.ClearManifold();

    public abstract int NumManifolds { get; }

    public abstract PersistentManifold GetManifoldByIndexInternal(int index);

    /// <summary>btCollisionDispatcher::dispatchAllCollisionPairs</summary>
    public virtual void DispatchAllCollisionPairs(OverlappingPairCache pairCache, DispatcherInfo dispatchInfo, Dispatcher dispatcher)
    {
        collisionPairCallback ??= new CollisionPairCallback();
        collisionPairCallback.Set(dispatchInfo, this);
        pairCache.ProcessAllOverlappingPairs(collisionPairCallback, dispatcher);
    }

    private CollisionPairCallback collisionPairCallback;

    private sealed class CollisionPairCallback : IOverlapCallback
    {
        private DispatcherInfo dispatchInfo;
        private Dispatcher dispatcher;

        public void Set(DispatcherInfo dispatchInfo, Dispatcher dispatcher)
        {
            this.dispatchInfo = dispatchInfo;
            this.dispatcher = dispatcher;
        }

        public bool ProcessOverlap(BroadphasePair pair)
        {
            NearCallback nearCallback = dispatcher.NearCallback ?? s_defaultNearCallback;
            nearCallback(pair, dispatcher, dispatchInfo);
            return false;
        }
    }

    // Cache the method-group conversion to avoid allocating a delegate on every ProcessOverlap.
    private static readonly NearCallback s_defaultNearCallback = DefaultNearCallback;

    internal virtual NearCallback NearCallback { get; set; }

    /// <summary>btCollisionDispatcher::defaultNearCallback</summary>
    public static void DefaultNearCallback(BroadphasePair collisionPair, Dispatcher dispatcher, DispatcherInfo dispatchInfo)
    {
        CollisionObject colObj0 = collisionPair.proxy0.clientObject;
        CollisionObject colObj1 = collisionPair.proxy1.clientObject;

        if (dispatcher.NeedsCollision(colObj0, colObj1))
        {
            // Reuse wrapper/result objects (stack temporaries in C++; BulletSharp is single-threaded per world).
            dispatcher.nearCallbackWrapper0 ??= new CollisionObjectWrapper(null, null, null, Transform.Identity, -1, -1);
            dispatcher.nearCallbackWrapper1 ??= new CollisionObjectWrapper(null, null, null, Transform.Identity, -1, -1);
            CollisionObjectWrapper obj0Wrap = dispatcher.nearCallbackWrapper0;
            CollisionObjectWrapper obj1Wrap = dispatcher.nearCallbackWrapper1;
            obj0Wrap.Set(null, colObj0.CollisionShape, colObj0, colObj0.worldTransform, -1, -1);
            obj1Wrap.Set(null, colObj1.CollisionShape, colObj1, colObj1.worldTransform, -1, -1);

            collisionPair.algorithm ??= dispatcher.FindAlgorithm(obj0Wrap, obj1Wrap, null, DispatcherQueryType.ContactPointAlgorithms);

            if (collisionPair.algorithm != null)
            {
                dispatcher.nearCallbackResult ??= new ManifoldResult();
                ManifoldResult contactPointResult = dispatcher.nearCallbackResult;
                contactPointResult.Reset(obj0Wrap, obj1Wrap);

                if (dispatchInfo.dispatchFunc == DispatchFunc.Discrete)
                {
                    collisionPair.algorithm.ProcessCollision(obj0Wrap, obj1Wrap, dispatchInfo, contactPointResult);
                }
                else
                {
                    btScalar toi = collisionPair.algorithm.CalculateTimeOfImpact(colObj0, colObj1, dispatchInfo, contactPointResult);
                    if (dispatchInfo.timeOfImpact > toi)
                        dispatchInfo.timeOfImpact = toi;
                }
            }
        }
    }

    // Reusable temporaries for DefaultNearCallback (stack objects in C++; single-threaded per world, as in Bullet).
    private CollisionObjectWrapper nearCallbackWrapper0;
    private CollisionObjectWrapper nearCallbackWrapper1;
    private ManifoldResult nearCallbackResult;

    internal abstract void FreeCollisionAlgorithm(CollisionAlgorithm algorithm);

    public virtual void Dispose()
    {
    }
}
