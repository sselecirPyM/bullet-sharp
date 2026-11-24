using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btCollisionDispatcher. Named CollisionDispatcher in BulletSharp.
/// </summary>
public class CollisionDispatcher : Dispatcher
{
    internal const int MaxBroadphaseCollisionTypes = (int)BroadphaseNativeType.MaxBroadphaseCollisionTypes;

    private readonly CreateCollisionAlgorithmFunc[,] doubleDispatchContactPoints =
        new CreateCollisionAlgorithmFunc[MaxBroadphaseCollisionTypes, MaxBroadphaseCollisionTypes];
    private readonly CreateCollisionAlgorithmFunc[,] doubleDispatchClosestPoints =
        new CreateCollisionAlgorithmFunc[MaxBroadphaseCollisionTypes, MaxBroadphaseCollisionTypes];

    private readonly List<PersistentManifold> manifoldsPtr = new();
    internal DispatcherFlags dispatcherFlags = DispatcherFlags.UseRelativeContactBreakingThreshold;
    private readonly CollisionConfiguration collisionConfiguration;

    public CollisionDispatcher(CollisionConfiguration collisionConfiguration)
    {
        this.collisionConfiguration = collisionConfiguration;

        for (int i = 0; i < MaxBroadphaseCollisionTypes; i++)
        {
            for (int j = 0; j < MaxBroadphaseCollisionTypes; j++)
            {
                doubleDispatchContactPoints[i, j] =
                    collisionConfiguration.GetCollisionAlgorithmCreateFunc((BroadphaseNativeType)i, (BroadphaseNativeType)j);
                doubleDispatchClosestPoints[i, j] =
                    collisionConfiguration.GetClosestPointsAlgorithmCreateFunc((BroadphaseNativeType)i, (BroadphaseNativeType)j);
            }
        }
    }

    public void RegisterCollisionCreateFunc(BroadphaseNativeType proxyType0, BroadphaseNativeType proxyType1,
        CreateCollisionAlgorithmFunc createFunc)
    {
        doubleDispatchContactPoints[(int)proxyType0, (int)proxyType1] = createFunc;
    }

    public void RegisterClosestPointsCreateFunc(BroadphaseNativeType proxyType0, BroadphaseNativeType proxyType1,
        CreateCollisionAlgorithmFunc createFunc)
    {
        doubleDispatchClosestPoints[(int)proxyType0, (int)proxyType1] = createFunc;
    }

    public DispatcherFlags DispatcherFlags
    {
        get => dispatcherFlags;
        set => dispatcherFlags = value;
    }

    public override PersistentManifold GetNewManifold(CollisionObject body0, CollisionObject body1)
    {
        btScalar contactBreakingThreshold =
            (dispatcherFlags & DispatcherFlags.UseRelativeContactBreakingThreshold) != 0
                ? BtMath.Min(
                    body0.CollisionShape.GetContactBreakingThreshold(PersistentManifold.GlobalContactBreakingThreshold),
                    body1.CollisionShape.GetContactBreakingThreshold(PersistentManifold.GlobalContactBreakingThreshold))
                : PersistentManifold.GlobalContactBreakingThreshold;

        btScalar contactProcessingThreshold =
            BtMath.Min(body0.contactProcessingThreshold, body1.contactProcessingThreshold);

        var manifold = manifoldPool.Count > 0 ? manifoldPool.Pop() : new PersistentManifold();
        manifold.SetBodies(body0, body1);
        manifold.contactBreakingThreshold = contactBreakingThreshold;
        manifold.contactProcessingThreshold = contactProcessingThreshold;
        manifold.index1a = manifoldsPtr.Count;
        manifoldsPtr.Add(manifold);

        return manifold;
    }

    public override void ClearManifold(PersistentManifold manifold) => manifold.ClearManifold();

    public override void ReleaseManifold(PersistentManifold manifold)
    {
        ClearManifold(manifold);

        int findIndex = manifold.index1a;
        // swap-pop
        manifoldsPtr[findIndex] = manifoldsPtr[^1];
        manifoldsPtr[findIndex].index1a = findIndex;
        manifoldsPtr.RemoveAt(manifoldsPtr.Count - 1);

        // Recycle the manifold object (reset to a fresh state and return to the pool,
        // avoiding reallocation when collision pairs churn).
        manifold.SetBodies(null, null);
        manifold.companionIdA = 0;
        manifold.companionIdB = 0;
        manifoldPool.Push(manifold);
    }

    // Pool of released manifolds (in C++ the caller manages memory; reuse here avoids GC pressure).
    private readonly Stack<PersistentManifold> manifoldPool = new();

    // Reusable temporary for FindAlgorithm (a stack object in C++).
    private readonly CollisionAlgorithm.ConstructionInfo constructionInfo = new();

    public override CollisionAlgorithm FindAlgorithm(CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap,
        PersistentManifold sharedManifold = null, DispatcherQueryType algoType = DispatcherQueryType.ContactPointAlgorithms)
    {
        // Reuse ConstructionInfo (consumed only temporarily by the algorithm constructor; a stack object in C++).
        var ci = constructionInfo;
        ci.Dispatcher = this;
        ci.Manifold = sharedManifold;

        int type0 = (int)body0Wrap.collisionShape.ShapeType;
        int type1 = (int)body1Wrap.collisionShape.ShapeType;

        CreateCollisionAlgorithmFunc createFunc = algoType == DispatcherQueryType.ContactPointAlgorithms
            ? doubleDispatchContactPoints[type0, type1]
            : doubleDispatchClosestPoints[type0, type1];

        return createFunc?.Invoke(ci, body0Wrap, body1Wrap);
    }

    public override int NumManifolds => manifoldsPtr.Count;

    public override PersistentManifold GetManifoldByIndexInternal(int index) => manifoldsPtr[index];

    public List<PersistentManifold> InternalManifoldPointer => manifoldsPtr;

    internal override void FreeCollisionAlgorithm(CollisionAlgorithm algorithm)
    {
        // In C# the object is collected by the GC; the algorithm only needs to release
        // the resources it holds, such as its manifold.
        algorithm.Cleanup(this);
    }
}
