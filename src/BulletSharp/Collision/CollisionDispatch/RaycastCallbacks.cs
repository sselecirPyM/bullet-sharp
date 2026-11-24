using BulletSharp.Math;

namespace BulletSharp;

/// <summary>btTriangleRaycastCallback::EFlags</summary>
[Flags]
public enum TriangleRaycastFlags
{
    None = 0,
    FilterBackfaces = 1,
    KeepUnflippedNormal = 2,
    UseSubSimplexConvexCastRaytest = 4,
    UseGjkConvexCastRaytest = 8,
    DisableHeightfieldAccelerator = 16,
}

/// <summary>btCollisionWorld::LocalShapeInfo (value type, equivalent to the C++ stack-based btLocalShapeInfo).</summary>
public struct LocalShapeInfo
{
    internal int shapePart = -1;
    internal int triangleIndex = -1;

    public LocalShapeInfo()
    {
    }

    public int ShapePart
    {
        readonly get => shapePart;
        set => shapePart = value;
    }

    public int TriangleIndex
    {
        readonly get => triangleIndex;
        set => triangleIndex = value;
    }
}

/// <summary>btCollisionWorld::LocalRayResult (value type, equivalent to the C++ stack-based btLocalRayResult).</summary>
public struct LocalRayResult
{
    internal CollisionObject collisionObject;
    internal LocalShapeInfo localShapeInfo;
    internal bool hasShapeInfo;
    internal Vector3 hitNormalLocal;
    internal btScalar hitFraction;

    internal LocalRayResult(CollisionObject collisionObject, LocalShapeInfo? localShapeInfo,
        Vector3 hitNormalLocal, btScalar hitFraction)
    {
        Set(collisionObject, localShapeInfo, hitNormalLocal, hitFraction);
    }

    /// <summary>In-place reuse (the callback contract requires immediate consumption; equivalent to a C++ stack object).</summary>
    internal void Set(CollisionObject collisionObject, LocalShapeInfo? localShapeInfo,
        Vector3 hitNormalLocal, btScalar hitFraction)
    {
        this.collisionObject = collisionObject;
        this.localShapeInfo = localShapeInfo ?? default;
        hasShapeInfo = localShapeInfo.HasValue;
        this.hitNormalLocal = hitNormalLocal;
        this.hitFraction = hitFraction;
    }

    /// <summary>Whether shape info is present (equivalent to the C++ m_localShapeInfo != NULL check).</summary>
    internal readonly bool HasShapeInfo => hasShapeInfo;

    public CollisionObject CollisionObject => collisionObject;

    public LocalShapeInfo LocalShapeInfo
    {
        readonly get => localShapeInfo;
        set
        {
            localShapeInfo = value;
            hasShapeInfo = true;
        }
    }

    public Vector3 HitNormalLocal
    {
        readonly get => hitNormalLocal;
        set => hitNormalLocal = value;
    }

    public btScalar HitFraction => hitFraction;
}

/// <summary>btCollisionWorld::LocalConvexResult (value type, equivalent to the C++ stack-based btLocalConvexResult).</summary>
public struct LocalConvexResult
{
    internal CollisionObject collisionObject;
    internal LocalShapeInfo localShapeInfo;
    internal bool hasShapeInfo;
    internal Vector3 hitNormalLocal;
    internal Vector3 hitPointLocal;
    internal btScalar hitFraction;

    internal LocalConvexResult(CollisionObject collisionObject, LocalShapeInfo? localShapeInfo,
        Vector3 hitNormalLocal, Vector3 hitPointLocal, btScalar hitFraction)
    {
        this.collisionObject = collisionObject;
        this.localShapeInfo = localShapeInfo ?? default;
        hasShapeInfo = localShapeInfo.HasValue;
        this.hitNormalLocal = hitNormalLocal;
        this.hitPointLocal = hitPointLocal;
        this.hitFraction = hitFraction;
    }

    /// <summary>Whether shape info is present (equivalent to the C++ m_localShapeInfo != NULL check).</summary>
    internal readonly bool HasShapeInfo => hasShapeInfo;

    public CollisionObject CollisionObject => collisionObject;

    public LocalShapeInfo LocalShapeInfo
    {
        readonly get => localShapeInfo;
        set
        {
            localShapeInfo = value;
            hasShapeInfo = true;
        }
    }

    public Vector3 HitNormalLocal
    {
        readonly get => hitNormalLocal;
        set => hitNormalLocal = value;
    }

    public Vector3 HitPointLocal
    {
        readonly get => hitPointLocal;
        set => hitPointLocal = value;
    }

    public btScalar HitFraction => hitFraction;
}

/// <summary>btCollisionWorld::RayResultCallback</summary>
public abstract class RayResultCallback
{
    internal btScalar closestHitFraction = 1;
    internal CollisionObject collisionObject;
    internal CollisionFilterGroups collisionFilterGroup = CollisionFilterGroups.DefaultFilter;
    internal CollisionFilterGroups collisionFilterMask = CollisionFilterGroups.AllFilter;
    internal TriangleRaycastFlags flags;

    public btScalar ClosestHitFraction
    {
        get => closestHitFraction;
        set => closestHitFraction = value;
    }

    public CollisionObject CollisionObject => collisionObject;

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

    public TriangleRaycastFlags Flags
    {
        get => flags;
        set => flags = value;
    }

    public bool HasHit => collisionObject != null;

    public virtual bool NeedsCollision(BroadphaseProxy proxy0)
    {
        bool collides = (proxy0.collisionFilterGroup & collisionFilterMask) != 0;
        collides = collides && (collisionFilterGroup & proxy0.collisionFilterMask) != 0;
        return collides;
    }

    public abstract btScalar AddSingleResult(LocalRayResult rayResult, bool normalInWorldSpace);

    public virtual void Dispose()
    {
    }
}

/// <summary>btCollisionWorld::ClosestRayResultCallback</summary>
public class ClosestRayResultCallback : RayResultCallback
{
    internal Vector3 rayFromWorld;
    internal Vector3 rayToWorld;
    internal Vector3 hitNormalWorld;
    internal Vector3 hitPointWorld;

    public ClosestRayResultCallback(Vector3 rayFromWorld, Vector3 rayToWorld)
    {
        this.rayFromWorld = rayFromWorld;
        this.rayToWorld = rayToWorld;
    }

    /// <summary>Reset to reuse the same instance (avoids allocating a new callback per ray;
    /// equivalent to C++ constructing one on the stack per ray).</summary>
    public void Reset(Vector3 rayFromWorld, Vector3 rayToWorld)
    {
        this.rayFromWorld = rayFromWorld;
        this.rayToWorld = rayToWorld;
        hitNormalWorld = Vector3.Zero;
        hitPointWorld = Vector3.Zero;
        collisionObject = null;
        closestHitFraction = 1;
    }

    public Vector3 HitNormalWorld => hitNormalWorld;

    public Vector3 HitPointWorld => hitPointWorld;

    public Vector3 RayFromWorld => rayFromWorld;

    public Vector3 RayToWorld => rayToWorld;

    public override btScalar AddSingleResult(LocalRayResult rayResult, bool normalInWorldSpace)
    {
        // caller already guarantees rayResult.hitFraction <= closestHitFraction
        closestHitFraction = rayResult.hitFraction;
        collisionObject = rayResult.collisionObject;
        if (normalInWorldSpace)
        {
            hitNormalWorld = rayResult.hitNormalLocal;
        }
        else
        {
            hitNormalWorld = collisionObject.worldTransform.Basis * rayResult.hitNormalLocal;
        }
        hitPointWorld = rayFromWorld.Lerp(rayToWorld, rayResult.hitFraction);
        return rayResult.hitFraction;
    }
}

/// <summary>btCollisionWorld::AllHitsRayResultCallback</summary>
public class AllHitsRayResultCallback : RayResultCallback
{
    internal Vector3 rayFromWorld;
    internal Vector3 rayToWorld;

    public AllHitsRayResultCallback(Vector3 rayFromWorld, Vector3 rayToWorld)
    {
        this.rayFromWorld = rayFromWorld;
        this.rayToWorld = rayToWorld;
    }

    public List<CollisionObject> CollisionObjects { get; } = new();
    public List<Vector3> HitNormalWorld { get; } = new();
    public List<Vector3> HitPointWorld { get; } = new();
    public List<btScalar> HitFractions { get; } = new();

    public Vector3 RayFromWorld => rayFromWorld;
    public Vector3 RayToWorld => rayToWorld;

    public override btScalar AddSingleResult(LocalRayResult rayResult, bool normalInWorldSpace)
    {
        collisionObject = rayResult.collisionObject;
        CollisionObjects.Add(rayResult.collisionObject);
        Vector3 hitNormalWorld;
        if (normalInWorldSpace)
        {
            hitNormalWorld = rayResult.hitNormalLocal;
        }
        else
        {
            hitNormalWorld = collisionObject.worldTransform.Basis * rayResult.hitNormalLocal;
        }
        HitNormalWorld.Add(hitNormalWorld);
        HitPointWorld.Add(rayFromWorld.Lerp(rayToWorld, rayResult.hitFraction));
        HitFractions.Add(rayResult.hitFraction);
        return closestHitFraction;
    }
}

/// <summary>btCollisionWorld::ConvexResultCallback</summary>
public abstract class ConvexResultCallback
{
    internal btScalar closestHitFraction = 1;
    internal CollisionFilterGroups collisionFilterGroup = CollisionFilterGroups.DefaultFilter;
    internal CollisionFilterGroups collisionFilterMask = CollisionFilterGroups.AllFilter;

    public btScalar ClosestHitFraction
    {
        get => closestHitFraction;
        set => closestHitFraction = value;
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

    public virtual bool HasHit => false;

    public virtual bool NeedsCollision(BroadphaseProxy proxy0)
    {
        bool collides = (proxy0.collisionFilterGroup & collisionFilterMask) != 0;
        collides = collides && (collisionFilterGroup & proxy0.collisionFilterMask) != 0;
        return collides;
    }

    public abstract btScalar AddSingleResult(LocalConvexResult convexResult, bool normalInWorldSpace);

    public virtual void Dispose()
    {
    }
}

/// <summary>btCollisionWorld::ClosestConvexResultCallback</summary>
public class ClosestConvexResultCallback : ConvexResultCallback
{
    internal Vector3 convexFromWorld;
    internal Vector3 convexToWorld;
    internal Vector3 hitNormalWorld;
    internal Vector3 hitPointWorld;
    internal CollisionObject hitCollisionObject;

    public ClosestConvexResultCallback(Vector3 convexFromWorld, Vector3 convexToWorld)
    {
        this.convexFromWorld = convexFromWorld;
        this.convexToWorld = convexToWorld;
    }

    public Vector3 ConvexFromWorld => convexFromWorld;

    public Vector3 ConvexToWorld => convexToWorld;

    public Vector3 HitNormalWorld => hitNormalWorld;

    public Vector3 HitPointWorld => hitPointWorld;

    public CollisionObject HitCollisionObject => hitCollisionObject;

    public override bool HasHit => hitCollisionObject != null;

    public override btScalar AddSingleResult(LocalConvexResult convexResult, bool normalInWorldSpace)
    {
        closestHitFraction = convexResult.hitFraction;
        hitCollisionObject = convexResult.collisionObject;
        if (normalInWorldSpace)
        {
            hitNormalWorld = convexResult.hitNormalLocal;
        }
        else
        {
            hitNormalWorld = convexResult.collisionObject.worldTransform.Basis * convexResult.hitNormalLocal;
        }
        hitPointWorld = convexResult.hitPointLocal;
        return convexResult.hitFraction;
    }
}

/// <summary>btCollisionWorld::ContactResultCallback</summary>
public abstract class ContactResultCallback
{
    internal CollisionFilterGroups collisionFilterGroup = CollisionFilterGroups.DefaultFilter;
    internal CollisionFilterGroups collisionFilterMask = CollisionFilterGroups.AllFilter;
    internal btScalar closestDistanceThreshold;

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

    public btScalar ClosestDistanceThreshold
    {
        get => closestDistanceThreshold;
        set => closestDistanceThreshold = value;
    }

    public virtual bool NeedsCollision(BroadphaseProxy proxy0)
    {
        bool collides = (proxy0.collisionFilterGroup & collisionFilterMask) != 0;
        collides = collides && (collisionFilterGroup & proxy0.collisionFilterMask) != 0;
        return collides;
    }

    public abstract btScalar AddSingleResult(ManifoldPoint cp,
        CollisionObjectWrapper colObj0Wrap, int partId0, int index0,
        CollisionObjectWrapper colObj1Wrap, int partId1, int index1);

    public virtual void Dispose()
    {
    }
}
