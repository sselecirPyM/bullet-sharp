using BulletSharp.Math;

namespace BulletSharp;

/// <summary>btBroadphaseAabbCallback</summary>
public abstract class BroadphaseAabbCallback
{
    /// <summary>Return true to continue traversal.</summary>
    public abstract bool Process(BroadphaseProxy proxy);
}

/// <summary>btBroadphaseRayCallback (set RayDirectionInverse/Signs/LambdaMax before use)</summary>
public abstract class BroadphaseRayCallback
{
    internal Vector3 rayDirectionInverse;
    internal int[] signs = new int[3];
    internal btScalar lambdaMax;

    public Vector3 RayDirectionInverse
    {
        get => rayDirectionInverse;
        set => rayDirectionInverse = value;
    }

    public int[] Signs => signs;

    public btScalar LambdaMax
    {
        get => lambdaMax;
        set => lambdaMax = value;
    }

    /// <summary>Return true to continue traversal.</summary>
    public abstract bool Process(BroadphaseProxy proxy);
}

/// <summary>
/// Ported from btBroadphaseInterface as the abstract BroadphaseInterface class.
/// </summary>
public abstract class BroadphaseInterface : IDisposable
{
    public abstract BroadphaseProxy CreateProxy(Vector3 aabbMin, Vector3 aabbMax, BroadphaseNativeType shapeType,
        CollisionObject userPtr, CollisionFilterGroups collisionFilterGroup, CollisionFilterGroups collisionFilterMask,
        Dispatcher dispatcher, object multiSapProxy = null);

    public abstract void DestroyProxy(BroadphaseProxy proxy, Dispatcher dispatcher);

    public abstract void SetAabb(BroadphaseProxy proxy, Vector3 aabbMin, Vector3 aabbMax, Dispatcher dispatcher);

    public virtual void GetAabb(BroadphaseProxy proxy, out Vector3 aabbMin, out Vector3 aabbMax)
    {
        aabbMin = proxy.aabbMin;
        aabbMax = proxy.aabbMax;
    }

    public abstract void RayTest(Vector3 rayFromWorld, Vector3 rayToWorld, BroadphaseRayCallback rayCallback,
        Vector3 aabbMin = default, Vector3 aabbMax = default);

    public abstract void AabbTest(Vector3 aabbMin, Vector3 aabbMax, BroadphaseAabbCallback callback);

    public abstract void CalculateOverlappingPairs(Dispatcher dispatcher);

    public abstract OverlappingPairCache OverlappingPairCache { get; }

    public abstract void GetBroadphaseAabb(out Vector3 aabbMin, out Vector3 aabbMax);

    public virtual void ResetPool(Dispatcher dispatcher)
    {
    }

    public virtual void PrintStats()
    {
    }

    public virtual void Dispose()
    {
    }
}
