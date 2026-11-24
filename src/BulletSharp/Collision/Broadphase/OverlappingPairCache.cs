using System.Runtime.CompilerServices;

namespace BulletSharp;

/// <summary>btOverlapCallback: ProcessOverlap returns true to remove the pair.</summary>
public interface IOverlapCallback
{
    bool ProcessOverlap(BroadphasePair pair);
}

/// <summary>btOverlappingPairCallback (used by ghost objects, etc.)</summary>
public abstract class OverlappingPairCallback
{
    public virtual BroadphasePair AddOverlappingPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1) => null;

    public virtual object RemoveOverlappingPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1, Dispatcher dispatcher) => null;

    public virtual void RemoveOverlappingPairsContainingProxy(BroadphaseProxy proxy0, Dispatcher dispatcher)
    {
    }
}

/// <summary>
/// Ported from btOverlappingPairCache.
/// </summary>
public abstract class OverlappingPairCache : OverlappingPairCallback
{
    internal OverlappingPairCallback ghostPairCallback;

    public abstract int NumOverlappingPairs { get; }

    public abstract List<BroadphasePair> OverlappingPairArray { get; }

    public abstract void CleanOverlappingPair(BroadphasePair pair, Dispatcher dispatcher);

    public abstract void CleanProxyFromPairs(BroadphaseProxy proxy, Dispatcher dispatcher);

    public abstract bool NeedsBroadphaseCollision(BroadphaseProxy proxy0, BroadphaseProxy proxy1);

    public abstract BroadphasePair FindPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1);

    public abstract void ProcessAllOverlappingPairs(IOverlapCallback callback, Dispatcher dispatcher);

    public abstract void SortOverlappingPairs(Dispatcher dispatcher);

    public virtual bool HasDeferredRemoval => false;

    public OverlappingPairCallback InternalGhostPairCallback
    {
        get => ghostPairCallback;
        set => ghostPairCallback = value;
    }
}

/// <summary>
/// Ported from btHashedOverlappingPairCache (flat power-of-two int[] hash table + next[] chains).
/// </summary>
public class HashedOverlappingPairCache : OverlappingPairCache
{
    private const int InitialHashCapacity = 256;

    private readonly List<BroadphasePair> overlappingPairArray = new();
    private int[] hashTable = [];
    private int[] next = [];
    private ulong[] pairKeys = [];

    // Pool of removed pair objects (in C++, pairs are value-type array elements with no per-pair heap allocation)
    private readonly Stack<BroadphasePair> pairPool = new();

    public override int NumOverlappingPairs => overlappingPairArray.Count;

    public override List<BroadphasePair> OverlappingPairArray => overlappingPairArray;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong MakeKeyOrdered(int uid0, int uid1) =>
        ((ulong)(uint)uid0 << 32) | (uint)uid1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int HashKey(ulong key, int mask)
    {
        ulong h = key * 0x9E3779B97F4A7C15UL;
        return (int)(h ^ (h >> 32)) & mask;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int FindIndex(ulong key)
    {
        int[] buckets = hashTable;
        if (buckets.Length == 0)
            return -1;

        int index = buckets[HashKey(key, buckets.Length - 1)];
        ulong[] keys = pairKeys;
        int[] nextArr = next;
        while (index != -1 && keys[index] != key)
        {
            index = nextArr[index];
        }
        return index;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private void GrowTables(int existingCount)
    {
        int required = (existingCount + 1) * 2;
        int newCapacity = hashTable.Length == 0 ? InitialHashCapacity : hashTable.Length * 2;
        while (newCapacity < required)
            newCapacity *= 2;

        int[] newBuckets = new int[newCapacity];
        Array.Fill(newBuckets, -1);
        int[] newNext = new int[newCapacity];
        Array.Resize(ref pairKeys, newCapacity);

        ulong[] keys = pairKeys;
        int mask = newCapacity - 1;
        for (int i = 0; i < existingCount; i++)
        {
            int b = HashKey(keys[i], mask);
            newNext[i] = newBuckets[b];
            newBuckets[b] = i;
        }

        hashTable = newBuckets;
        next = newNext;
    }

    public override void CleanOverlappingPair(BroadphasePair pair, Dispatcher dispatcher)
    {
        if (pair.algorithm != null && dispatcher != null)
        {
            dispatcher.FreeCollisionAlgorithm(pair.algorithm);
            pair.algorithm = null;
        }
    }

    public override void CleanProxyFromPairs(BroadphaseProxy proxy, Dispatcher dispatcher)
    {
        var cleanCallback = new CleanPairCallback(proxy, this, dispatcher);
        ProcessAllOverlappingPairs(cleanCallback, dispatcher);
    }

    private sealed class CleanPairCallback : IOverlapCallback
    {
        private readonly BroadphaseProxy cleanProxy;
        private readonly OverlappingPairCache pairCache;
        private readonly Dispatcher dispatcher;

        public CleanPairCallback(BroadphaseProxy cleanProxy, OverlappingPairCache pairCache, Dispatcher dispatcher)
        {
            this.cleanProxy = cleanProxy;
            this.pairCache = pairCache;
            this.dispatcher = dispatcher;
        }

        public bool ProcessOverlap(BroadphasePair pair)
        {
            if (pair.proxy0 == cleanProxy || pair.proxy1 == cleanProxy)
            {
                pairCache.CleanOverlappingPair(pair, dispatcher);
            }
            return false;
        }
    }

    public override void RemoveOverlappingPairsContainingProxy(BroadphaseProxy proxy, Dispatcher dispatcher)
    {
        var removeCallback = new RemovePairCallback(proxy);
        ProcessAllOverlappingPairs(removeCallback, dispatcher);
    }

    private sealed class RemovePairCallback : IOverlapCallback
    {
        private readonly BroadphaseProxy obsoleteProxy;

        public RemovePairCallback(BroadphaseProxy obsoleteProxy) => this.obsoleteProxy = obsoleteProxy;

        public bool ProcessOverlap(BroadphasePair pair) =>
            pair.proxy0 == obsoleteProxy || pair.proxy1 == obsoleteProxy;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool NeedsBroadphaseCollision(BroadphaseProxy proxy0, BroadphaseProxy proxy1)
    {
        return (proxy0.collisionFilterGroup & proxy1.collisionFilterMask) != 0 &&
               (proxy1.collisionFilterGroup & proxy0.collisionFilterMask) != 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    public override BroadphasePair AddOverlappingPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1)
    {
        // btHashedOverlappingPairCache::addOverlappingPair: apply the collision filter first
        if ((proxy0.collisionFilterGroup & proxy1.collisionFilterMask) == 0 ||
            (proxy1.collisionFilterGroup & proxy0.collisionFilterMask) == 0)
            return null;

        int uid0 = proxy0.uid;
        int uid1 = proxy1.uid;
        if (uid0 > uid1)
        {
            (proxy0, proxy1) = (proxy1, proxy0);
            (uid0, uid1) = (uid1, uid0);
        }

        ulong key = MakeKeyOrdered(uid0, uid1);
        int existing = FindIndex(key);
        if (existing != -1)
            return overlappingPairArray[existing];

        ghostPairCallback?.AddOverlappingPair(proxy0, proxy1);

        // Reuse a removed pair object (proxy0.uid < proxy1.uid is already guaranteed here)
        BroadphasePair pair = pairPool.Count > 0 ? pairPool.Pop() : new BroadphasePair();
        pair.proxy0 = proxy0;
        pair.proxy1 = proxy1;

        int newIndex = overlappingPairArray.Count;
        overlappingPairArray.Add(pair);

        if ((newIndex + 1) * 2 > hashTable.Length)
        {
            GrowTables(newIndex);
        }

        int bucket = HashKey(key, hashTable.Length - 1);
        pairKeys[newIndex] = key;
        next[newIndex] = hashTable[bucket];
        hashTable[bucket] = newIndex;
        return pair;
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    public override object RemoveOverlappingPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1, Dispatcher dispatcher)
    {
        if (proxy0.uid > proxy1.uid) (proxy0, proxy1) = (proxy1, proxy0);

        int[] buckets = hashTable;
        if (buckets.Length == 0)
            return null;

        ulong key = MakeKeyOrdered(proxy0.uid, proxy1.uid);
        int mask = buckets.Length - 1;
        int bucket = HashKey(key, mask);
        int pairIndex = buckets[bucket];
        int prev = -1;
        ulong[] keys = pairKeys;
        int[] nextArr = next;
        while (pairIndex != -1 && keys[pairIndex] != key)
        {
            prev = pairIndex;
            pairIndex = nextArr[pairIndex];
        }
        if (pairIndex == -1)
            return null;

        BroadphasePair pair = overlappingPairArray[pairIndex];
        CleanOverlappingPair(pair, dispatcher);
        object userData = pair.internalInfo1;

        ghostPairCallback?.RemoveOverlappingPair(proxy0, proxy1, dispatcher);

        if (prev != -1)
            nextArr[prev] = nextArr[pairIndex];
        else
            buckets[bucket] = nextArr[pairIndex];

        int lastIndex = overlappingPairArray.Count - 1;
        if (pairIndex != lastIndex)
        {
            ulong lastKey = keys[lastIndex];
            int lastBucket = HashKey(lastKey, mask);
            int cur = buckets[lastBucket];
            int lastPrev = -1;
            while (cur != lastIndex)
            {
                lastPrev = cur;
                cur = nextArr[cur];
            }
            if (lastPrev != -1)
                nextArr[lastPrev] = pairIndex;
            else
                buckets[lastBucket] = pairIndex;

            nextArr[pairIndex] = nextArr[lastIndex];
            keys[pairIndex] = lastKey;
            overlappingPairArray[pairIndex] = overlappingPairArray[lastIndex];
        }
        overlappingPairArray.RemoveAt(lastIndex);

        // Recycle the pair object (reset to a fresh state and return it to the pool)
        pair.proxy0 = null;
        pair.proxy1 = null;
        pair.algorithm = null;
        pair.internalInfo1 = null;
        pair.internalTmpValue = 0;
        pairPool.Push(pair);

        return userData;
    }

    public override BroadphasePair FindPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1)
    {
        if (proxy0.uid > proxy1.uid) (proxy0, proxy1) = (proxy1, proxy0);
        int index = FindIndex(MakeKeyOrdered(proxy0.uid, proxy1.uid));
        return index != -1 ? overlappingPairArray[index] : null;
    }

    public override void ProcessAllOverlappingPairs(IOverlapCallback callback, Dispatcher dispatcher)
    {
        for (int i = 0; i < overlappingPairArray.Count;)
        {
            BroadphasePair pair = overlappingPairArray[i];
            if (callback.ProcessOverlap(pair))
            {
                RemoveOverlappingPair(pair.proxy0, pair.proxy1, dispatcher);
            }
            else
            {
                i++;
            }
        }
    }

    private readonly struct PairSortDescendingComparer : IBtLessComparer<BroadphasePair>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Less(in BroadphasePair a, in BroadphasePair b)
        {
            int uidA0 = a.proxy0.uid, uidB0 = b.proxy0.uid;
            if (uidA0 != uidB0) return uidB0 < uidA0;
            return b.proxy1.uid < a.proxy1.uid;
        }
    }

    public override void SortOverlappingPairs(Dispatcher dispatcher)
    {
        // Deterministic sort (btHashedOverlappingPairCache::sortOverlappingPairs,
        // tmpPairs.quickSort(btBroadphasePairSortPredicate()): descending by uid)
        BtQuickSort.QuickSort<BroadphasePair, PairSortDescendingComparer>(overlappingPairArray);

        int count = overlappingPairArray.Count;
        if (count == 0)
        {
            if (hashTable.Length > 0)
                Array.Fill(hashTable, -1);
            return;
        }

        if (count * 2 > hashTable.Length)
        {
            GrowTables(0);
        }
        else
        {
            Array.Fill(hashTable, -1);
        }

        int mask = hashTable.Length - 1;
        for (int i = 0; i < count; i++)
        {
            BroadphasePair pair = overlappingPairArray[i];
            ulong key = MakeKeyOrdered(pair.proxy0.uid, pair.proxy1.uid);
            pairKeys[i] = key;
            int b = HashKey(key, mask);
            next[i] = hashTable[b];
            hashTable[b] = i;
        }
    }
}
