using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btSimpleBroadphase: brute-force O(n²) AABB culling broadphase (mainly for reference testing).
/// The handle pool is a preallocated array plus a free list (replacing C++ placement new).
/// </summary>
public class SimpleBroadphase : BroadphaseInterface
{
    /// <summary>btSimpleBroadphaseProxy</summary>
    public sealed class SimpleBroadphaseProxy : BroadphaseProxy
    {
        internal int nextFree;

        internal void SetNextFree(int next) => nextFree = next;
        internal int GetNextFree() => nextFree;
    }

    protected int numHandles;      // m_numHandles
    protected int maxHandles;      // m_maxHandles
    protected int lastHandleIndex; // m_LastHandleIndex

    protected SimpleBroadphaseProxy[] handles; // m_pHandles
    protected int firstFreeHandle;             // m_firstFreeHandle

    protected readonly OverlappingPairCache pairCache; // m_pairCache
    protected readonly bool ownsPairCache;             // m_ownsPairCache

    protected int invalidPair; // m_invalidPair

    public SimpleBroadphase(int maxProxies = 16384, OverlappingPairCache overlappingPairCache = null)
    {
        pairCache = overlappingPairCache;
        ownsPairCache = false;
        invalidPair = 0;

        if (pairCache == null)
        {
            pairCache = new HashedOverlappingPairCache();
            ownsPairCache = true;
        }

        // Allocate the handle pool and link all handles into the free list
        handles = new SimpleBroadphaseProxy[maxProxies];
        maxHandles = maxProxies;
        numHandles = 0;
        firstFreeHandle = 0;
        lastHandleIndex = -1;

        for (int i = firstFreeHandle; i < maxProxies; i++)
        {
            handles[i] = new SimpleBroadphaseProxy();
            handles[i].SetNextFree(i + 1);
            // C++ sets m_uniqueId to i+2 here; in this port uid is assigned monotonically by the base class, with equivalent semantics
        }
        handles[maxProxies - 1].SetNextFree(0);
    }

    // allocHandle
    private int AllocHandle()
    {
        int freeHandle = firstFreeHandle;
        firstFreeHandle = handles[freeHandle].GetNextFree();
        numHandles++;
        if (freeHandle > lastHandleIndex)
        {
            lastHandleIndex = freeHandle;
        }
        return freeHandle;
    }

    // freeHandle
    private void FreeHandle(SimpleBroadphaseProxy proxy)
    {
        int handle = Array.IndexOf(handles, proxy);
        if (handle == lastHandleIndex)
        {
            lastHandleIndex--;
        }
        proxy.SetNextFree(firstFreeHandle);
        firstFreeHandle = handle;

        proxy.clientObject = null;

        numHandles--;
    }

    // btSimpleBroadphase::createProxy
    public override BroadphaseProxy CreateProxy(Vector3 aabbMin, Vector3 aabbMax, BroadphaseNativeType shapeType,
        CollisionObject userPtr, CollisionFilterGroups collisionFilterGroup, CollisionFilterGroups collisionFilterMask,
        Dispatcher dispatcher, object multiSapProxy = null)
    {
        if (numHandles >= maxHandles)
        {
            return null; // should never happen
        }

        int newHandleIndex = AllocHandle();
        // Reinitialize the handle (replaces placement new)
        SimpleBroadphaseProxy proxy = handles[newHandleIndex];
        proxy.aabbMin = aabbMin;
        proxy.aabbMax = aabbMax;
        proxy.clientObject = userPtr;
        proxy.collisionFilterGroup = collisionFilterGroup;
        proxy.collisionFilterMask = collisionFilterMask;
        proxy.multiSapParentProxy = multiSapProxy;
        return proxy;
    }

    // btSimpleBroadphase::destroyProxy
    public override void DestroyProxy(BroadphaseProxy proxyOrg, Dispatcher dispatcher)
    {
        // See DbvtBroadphase: dispatch RemoveOverlappingPairsContainingProxy explicitly
        if (pairCache is HashedOverlappingPairCache hashed)
        {
            hashed.RemoveOverlappingPairsContainingProxy(proxyOrg, dispatcher);
        }
        else
        {
            pairCache.RemoveOverlappingPairsContainingProxy(proxyOrg, dispatcher);
        }

        var proxy0 = (SimpleBroadphaseProxy)proxyOrg;
        FreeHandle(proxy0);
    }

    // btSimpleBroadphase::getAabb
    public override void GetAabb(BroadphaseProxy proxy, out Vector3 aabbMin, out Vector3 aabbMax)
    {
        var sbp = (SimpleBroadphaseProxy)proxy;
        aabbMin = sbp.aabbMin;
        aabbMax = sbp.aabbMax;
    }

    // btSimpleBroadphase::setAabb
    public override void SetAabb(BroadphaseProxy proxy, Vector3 aabbMin, Vector3 aabbMax, Dispatcher dispatcher)
    {
        var sbp = (SimpleBroadphaseProxy)proxy;
        sbp.aabbMin = aabbMin;
        sbp.aabbMax = aabbMax;
    }

    // btSimpleBroadphase::rayTest (faithful port: C++ does no ray culling and visits all active handles)
    public override void RayTest(Vector3 rayFromWorld, Vector3 rayToWorld, BroadphaseRayCallback rayCallback,
        Vector3 aabbMin = default, Vector3 aabbMax = default)
    {
        for (int i = 0; i <= lastHandleIndex; i++)
        {
            SimpleBroadphaseProxy proxy = handles[i];
            if (proxy.clientObject == null)
            {
                continue;
            }
            rayCallback.Process(proxy);
        }
    }

    // btSimpleBroadphase::aabbTest
    public override void AabbTest(Vector3 aabbMin, Vector3 aabbMax, BroadphaseAabbCallback callback)
    {
        for (int i = 0; i <= lastHandleIndex; i++)
        {
            SimpleBroadphaseProxy proxy = handles[i];
            if (proxy.clientObject == null)
            {
                continue;
            }
            if (AabbUtil.TestAabbAgainstAabb2(aabbMin, aabbMax, proxy.aabbMin, proxy.aabbMax))
            {
                callback.Process(proxy);
            }
        }
    }

    // btSimpleBroadphase::aabbOverlap
    public static bool AabbOverlap(SimpleBroadphaseProxy proxy0, SimpleBroadphaseProxy proxy1) =>
        proxy0.aabbMin.X <= proxy1.aabbMax.X && proxy1.aabbMin.X <= proxy0.aabbMax.X &&
        proxy0.aabbMin.Y <= proxy1.aabbMax.Y && proxy1.aabbMin.Y <= proxy0.aabbMax.Y &&
        proxy0.aabbMin.Z <= proxy1.aabbMax.Z && proxy1.aabbMin.Z <= proxy0.aabbMax.Z;

    internal static bool AabbOverlap(BroadphaseProxy proxy0, BroadphaseProxy proxy1) =>
        AabbOverlap((SimpleBroadphaseProxy)proxy0, (SimpleBroadphaseProxy)proxy1);

    // CheckOverlapCallback (removes pairs that no longer overlap)
    private sealed class CheckOverlapCallback : IOverlapCallback
    {
        public bool ProcessOverlap(BroadphasePair pair) =>
            !AabbOverlap((SimpleBroadphaseProxy)pair.proxy0, (SimpleBroadphaseProxy)pair.proxy1);
    }

    // btSimpleBroadphase::calculateOverlappingPairs
    public override void CalculateOverlappingPairs(Dispatcher dispatcher)
    {
        // First check for new overlapping pairs
        if (numHandles >= 0)
        {
            int newLargestIndex = -1;
            for (int i = 0; i <= lastHandleIndex; i++)
            {
                SimpleBroadphaseProxy proxy0 = handles[i];
                if (proxy0.clientObject == null)
                {
                    continue;
                }
                newLargestIndex = i;
                for (int j = i + 1; j <= lastHandleIndex; j++)
                {
                    SimpleBroadphaseProxy proxy1 = handles[j];
                    if (proxy1.clientObject == null)
                    {
                        continue;
                    }

                    if (AabbOverlap(proxy0, proxy1))
                    {
                        if (pairCache.FindPair(proxy0, proxy1) == null)
                        {
                            pairCache.AddOverlappingPair(proxy0, proxy1);
                        }
                    }
                    else
                    {
                        if (!pairCache.HasDeferredRemoval)
                        {
                            if (pairCache.FindPair(proxy0, proxy1) != null)
                            {
                                pairCache.RemoveOverlappingPair(proxy0, proxy1, dispatcher);
                            }
                        }
                    }
                }
            }

            lastHandleIndex = newLargestIndex;

            if (ownsPairCache && pairCache.HasDeferredRemoval)
            {
                List<BroadphasePair> overlappingPairArray = pairCache.OverlappingPairArray;

                // Sort so duplicate pairs are adjacent and invalid pairs go last
                // (btAlignedObjectArray::quickSort)
                BtQuickSort.QuickSort(overlappingPairArray, BroadphasePairSortCompare);

                overlappingPairArray.RemoveRange(overlappingPairArray.Count - invalidPair, invalidPair);
                invalidPair = 0;

                BroadphaseProxy previous0 = null;
                BroadphaseProxy previous1 = null;

                for (int i = 0; i < overlappingPairArray.Count; i++)
                {
                    BroadphasePair pair = overlappingPairArray[i];

                    bool isDuplicate = pair.proxy0 == previous0 && pair.proxy1 == previous1;
                    previous0 = pair.proxy0;
                    previous1 = pair.proxy1;

                    bool needsRemoval;
                    if (!isDuplicate)
                    {
                        needsRemoval = !TestAabbOverlap(pair.proxy0, pair.proxy1);
                    }
                    else
                    {
                        // Remove the duplicate
                        needsRemoval = true;
                    }

                    if (needsRemoval)
                    {
                        pairCache.CleanOverlappingPair(pair, dispatcher);
                        pair.proxy0 = null;
                        pair.proxy1 = null;
                        invalidPair++;
                    }
                }

                // CLEAN_INVALID_PAIRS: sort again and truncate the invalid pairs
                // (btAlignedObjectArray::quickSort)
                BtQuickSort.QuickSort(overlappingPairArray, BroadphasePairSortCompare);
                overlappingPairArray.RemoveRange(overlappingPairArray.Count - invalidPair, invalidPair);
                invalidPair = 0;
            }
        }
    }

    // btBroadphasePairSortPredicate (descending uid; a null proxy counts as -1 and sorts last)
    private static int BroadphasePairSortCompare(BroadphasePair a, BroadphasePair b)
    {
        int uidA0 = a.proxy0?.uid ?? -1;
        int uidB0 = b.proxy0?.uid ?? -1;
        int uidA1 = a.proxy1?.uid ?? -1;
        int uidB1 = b.proxy1?.uid ?? -1;
        if (uidA0 != uidB0) return uidB0.CompareTo(uidA0);
        return uidB1.CompareTo(uidA1);
    }

    // btSimpleBroadphase::testAabbOverlap
    public bool TestAabbOverlap(BroadphaseProxy proxy0, BroadphaseProxy proxy1) =>
        AabbOverlap((SimpleBroadphaseProxy)proxy0, (SimpleBroadphaseProxy)proxy1);

    public override OverlappingPairCache OverlappingPairCache => pairCache;

    // btSimpleBroadphase::getBroadphaseAabb
    public override void GetBroadphaseAabb(out Vector3 aabbMin, out Vector3 aabbMax)
    {
        aabbMin = new Vector3(-BtMath.BtLargeFloat);
        aabbMax = new Vector3(BtMath.BtLargeFloat);
    }

    // btSimpleBroadphase::resetPool (not implemented in C++)
    public override void ResetPool(Dispatcher dispatcher)
    {
        // not yet
    }

    public override void PrintStats()
    {
        // Empty implementation, as in C++
    }
}
