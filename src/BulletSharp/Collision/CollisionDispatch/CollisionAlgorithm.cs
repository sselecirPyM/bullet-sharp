namespace BulletSharp;

/// <summary>
/// Ported from btCollisionAlgorithm: base class for narrowphase algorithms handling a collision pair.
/// </summary>
public abstract class CollisionAlgorithm
{
    /// <summary>btCollisionAlgorithm::ConstructionInfo</summary>
    public class ConstructionInfo
    {
        internal Dispatcher dispatcher1;
        internal PersistentManifold manifold;

        public ConstructionInfo()
        {
        }

        public ConstructionInfo(Dispatcher dispatcher, int unused)
        {
            dispatcher1 = dispatcher;
        }

        public Dispatcher Dispatcher
        {
            get => dispatcher1;
            set => dispatcher1 = value;
        }

        public PersistentManifold Manifold
        {
            get => manifold;
            set => manifold = value;
        }
    }

    internal Dispatcher dispatcher;

    protected CollisionAlgorithm()
    {
    }

    protected CollisionAlgorithm(ConstructionInfo ci)
    {
        dispatcher = ci.dispatcher1;
    }

    internal Dispatcher Dispatcher => dispatcher;

    /// <summary>
    /// Process the collision and write results into resultOut (discrete mode).
    /// </summary>
    public abstract void ProcessCollision(CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut);

    /// <summary>
    /// Calculate the time of impact (continuous mode, CCD). Returns TOI in [0, 1]; returns 1 on miss.
    /// </summary>
    public virtual btScalar CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        return 1;
    }

    public virtual void GetAllContactManifolds(List<PersistentManifold> manifoldArray)
    {
    }

    /// <summary>Release resources held by the algorithm (e.g. its own manifold) when the broadphase pair is removed.</summary>
    internal virtual void Cleanup(Dispatcher dispatcher)
    {
    }
}
