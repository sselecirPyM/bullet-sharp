using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btBroadphaseProxy.
/// </summary>
public class BroadphaseProxy
{
    private static int nextUid;

    internal readonly int uid;
    internal CollisionObject clientObject;
    internal Vector3 aabbMin;
    internal Vector3 aabbMax;
    internal CollisionFilterGroups collisionFilterGroup;
    internal CollisionFilterGroups collisionFilterMask;
    internal object multiSapParentProxy;

    public BroadphaseProxy()
    {
        uid = System.Threading.Interlocked.Increment(ref nextUid);
    }

    public BroadphaseProxy(Vector3 aabbMin, Vector3 aabbMax, CollisionObject userPtr,
        CollisionFilterGroups collisionFilterGroup, CollisionFilterGroups collisionFilterMask)
        : this(aabbMin, aabbMax, userPtr, collisionFilterGroup, collisionFilterMask, null)
    {
    }

    public BroadphaseProxy(Vector3 aabbMin, Vector3 aabbMax, CollisionObject userPtr,
        CollisionFilterGroups collisionFilterGroup, CollisionFilterGroups collisionFilterMask, object multiSapParentProxy)
    {
        uid = System.Threading.Interlocked.Increment(ref nextUid);
        this.aabbMin = aabbMin;
        this.aabbMax = aabbMax;
        clientObject = userPtr;
        this.collisionFilterGroup = collisionFilterGroup;
        this.collisionFilterMask = collisionFilterMask;
        this.multiSapParentProxy = multiSapParentProxy;
    }

    public int Uid => uid;

    public CollisionObject ClientObject
    {
        get => clientObject;
        set => clientObject = value;
    }

    public Vector3 AabbMin
    {
        get => aabbMin;
        set => aabbMin = value;
    }

    public Vector3 AabbMax
    {
        get => aabbMax;
        set => aabbMax = value;
    }

    public CollisionFilterGroups CollisionFilterGroup
    {
        get => collisionFilterGroup;
        set => collisionFilterGroup = value;
    }

    public CollisionFilterGroups CollisionFilterMask
    {
        get => collisionFilterMask;
        set => collisionFilterMask = value;
    }

    // ---- Shape type helpers (btBroadphaseProxy static methods) ----
    public static bool IsPolyhedral(BroadphaseNativeType type) => type < BroadphaseNativeType.ImplicitConvexShapesStartHere;

    public static bool IsConvex(BroadphaseNativeType type) => type < BroadphaseNativeType.ConcaveShapesStartHere;

    public static bool IsNonMoving(BroadphaseNativeType type) =>
        IsConcave(type) && type != BroadphaseNativeType.GImpactShape;

    public static bool IsConcave(BroadphaseNativeType type) =>
        type > BroadphaseNativeType.ConcaveShapesStartHere && type < BroadphaseNativeType.ConcaveShapesEndHere;

    public static bool IsCompound(BroadphaseNativeType type) => type == BroadphaseNativeType.CompoundShape;

    public static bool IsSoftBody(BroadphaseNativeType type) => type == BroadphaseNativeType.SoftBodyShape;

    public static bool IsInfinite(BroadphaseNativeType type) => type == BroadphaseNativeType.StaticPlaneShape;

    public static bool IsConvex2D(BroadphaseNativeType type) =>
        type == BroadphaseNativeType.Box2DShape || type == BroadphaseNativeType.Convex2DShape;
}

/// <summary>
/// Ported from btBroadphasePair.
/// </summary>
public class BroadphasePair
{
    internal BroadphaseProxy proxy0;
    internal BroadphaseProxy proxy1;
    internal CollisionAlgorithm algorithm;
    internal object internalInfo1 = null;
    internal int internalTmpValue = 0;

    public BroadphasePair()
    {
    }

    public BroadphasePair(BroadphaseProxy proxy0, BroadphaseProxy proxy1)
    {
        // Keep proxies sorted by uid
        if (proxy0.uid < proxy1.uid)
        {
            this.proxy0 = proxy0;
            this.proxy1 = proxy1;
        }
        else
        {
            this.proxy0 = proxy1;
            this.proxy1 = proxy0;
        }
    }

    public BroadphaseProxy Proxy0 => proxy0;
    public BroadphaseProxy Proxy1 => proxy1;
    public CollisionAlgorithm Algorithm => algorithm;
}
