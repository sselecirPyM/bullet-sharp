using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btDbvtBroadphase: dual DBVT tree broadphase (dynamic set + fixed set with deferred stage migration).
/// </summary>
public class DbvtBroadphase : BroadphaseInterface
{
    // btDbvtBroadphase configuration constants
    public const int DynamicSet = 0; // DYNAMIC_SET
    public const int FixedSet = 1;   // FIXED_SET
    public const int StageCount = 2; // STAGECOUNT

    // gDbvtMargin (DBVT_BP_MARGIN, default 0.05)
    public static btScalar DbvtMargin = (btScalar)0.05;

    /// <summary>btDbvtProxy</summary>
    internal sealed class DbvtProxy : BroadphaseProxy
    {
        internal DbvtNode leaf;
        internal readonly DbvtProxy[] links = new DbvtProxy[2];
        internal int stage;

        public DbvtProxy(Vector3 aabbMin, Vector3 aabbMax, CollisionObject userPtr,
            CollisionFilterGroups collisionFilterGroup, CollisionFilterGroups collisionFilterMask)
            : base(aabbMin, aabbMax, userPtr, collisionFilterGroup, collisionFilterMask)
        {
        }
    }

    /// <summary>btDbvtTreeCollider</summary>
    internal sealed class DbvtTreeCollider : Dbvt.ICollide
    {
        internal readonly DbvtBroadphase broadphase;
        internal DbvtProxy proxy;
        public DbvtTreeCollider(DbvtBroadphase p) => broadphase = p;

        public override void Process(DbvtNode na, DbvtNode nb)
        {
            if (na != nb)
            {
                var pa = (DbvtProxy)na.data;
                var pb = (DbvtProxy)nb.data;
                broadphase.pairCache.AddOverlappingPair(pa, pb);
                ++broadphase.newpairs;
            }
        }

        public override void Process(DbvtNode n) => Process(n, proxy.leaf);
    }

    // Fields (btDbvtBroadphase::m_*)

    internal readonly Dbvt[] sets = { new(), new() };   // m_sets[2]
    internal readonly DbvtProxy[] stageRoots = new DbvtProxy[StageCount + 1]; // m_stageRoots
    internal readonly OverlappingPairCache pairCache;   // m_paircache
    internal btScalar prediction;                       // m_prediction
    internal int stageCurrent;                          // m_stageCurrent
    internal int fupdates = 1;                          // m_fupdates
    internal int dupdates;                              // m_dupdates
    internal int cupdates = 10;                         // m_cupdates
    internal int newpairs = 1;                          // m_newpairs
    internal int fixedleft;                             // m_fixedleft
    internal int updatesCall;                           // m_updates_call
    internal int updatesDone;                           // m_updates_done
    internal btScalar updatesRatio;                     // m_updates_ratio
    internal int pid;                                   // m_pid
    internal int cid;                                   // m_cid
    internal int gid;                                   // m_gid (kept for fidelity; uid is already assigned by the base class)
    internal readonly bool releasePairCache;            // m_releasepaircache
    internal bool deferedcollide;                       // m_deferedcollide
    internal bool needcleanup = true;                   // m_needcleanup

    // Reused temporary objects (stack objects in C++; the world is used single-threaded, as in Bullet)
    private readonly DbvtTreeCollider updateCollider;
    private readonly BroadphaseRayTester rayTester = new(null);
    private BroadphaseAabbTester aabbTester;

    public DbvtBroadphase(OverlappingPairCache pairCache = null)
    {
        deferedcollide = false;
        needcleanup = true;
        releasePairCache = pairCache != null ? false : true;
        prediction = 0;
        stageCurrent = 0;
        fixedleft = 0;
        this.pairCache = pairCache ?? new HashedOverlappingPairCache();
        updateCollider = new DbvtTreeCollider(this);
        gid = 0;
        pid = 0;
        cid = 0;
    }

    // ---- Linked list helpers (btDbvtBroadphase.cpp: listappend/listremove) ----

    private static void ListAppend(DbvtProxy item, ref DbvtProxy list)
    {
        item.links[0] = null;
        item.links[1] = list;
        if (list != null) list.links[0] = item;
        list = item;
    }

    private static void ListRemove(DbvtProxy item, ref DbvtProxy list)
    {
        if (item.links[0] != null)
        {
            item.links[0].links[1] = item.links[1];
        }
        else
        {
            list = item.links[1];
        }
        if (item.links[1] != null) item.links[1].links[0] = item.links[0];
    }

    // ---- btBroadphaseInterface implementation ----

    // btDbvtBroadphase::createProxy
    public override BroadphaseProxy CreateProxy(Vector3 aabbMin, Vector3 aabbMax, BroadphaseNativeType shapeType,
        CollisionObject userPtr, CollisionFilterGroups collisionFilterGroup, CollisionFilterGroups collisionFilterMask,
        Dispatcher dispatcher, object multiSapProxy = null)
    {
        var proxy = new DbvtProxy(aabbMin, aabbMax, userPtr, collisionFilterGroup, collisionFilterMask);
        proxy.multiSapParentProxy = multiSapProxy;

        DbvtVolume aabb = DbvtVolume.FromMM(aabbMin, aabbMax);

        proxy.stage = stageCurrent;
        ++gid; // C++: proxy->m_uniqueId = ++m_gid (equivalent semantics; uid is assigned monotonically by the base class)
        proxy.leaf = sets[0].Insert(aabb, proxy);
        ListAppend(proxy, ref stageRoots[stageCurrent]);
        if (!deferedcollide)
        {
            // Reuse the collider (a stack object in C++)
            updateCollider.proxy = proxy;
            sets[0].CollideTV(sets[0].root, aabb, updateCollider);
            sets[1].CollideTV(sets[1].root, aabb, updateCollider);
            updateCollider.proxy = null;
        }
        return proxy;
    }

    // btDbvtBroadphase::destroyProxy
    public override void DestroyProxy(BroadphaseProxy absproxy, Dispatcher dispatcher)
    {
        var proxy = (DbvtProxy)absproxy;
        if (proxy.stage == StageCount)
        {
            sets[1].Remove(proxy.leaf);
        }
        else
        {
            sets[0].Remove(proxy.leaf);
        }
        ListRemove(proxy, ref stageRoots[proxy.stage]);
        RemoveOverlappingPairsContainingProxy(proxy, dispatcher);
        needcleanup = true;
    }

    private void RemoveOverlappingPairsContainingProxy(BroadphaseProxy proxy, Dispatcher dispatcher)
    {
        if (pairCache is HashedOverlappingPairCache hashed)
        {
            hashed.RemoveOverlappingPairsContainingProxy(proxy, dispatcher);
        }
        else
        {
            pairCache.RemoveOverlappingPairsContainingProxy(proxy, dispatcher);
        }
    }

    // btDbvtBroadphase::getAabb
    public override void GetAabb(BroadphaseProxy proxy, out Vector3 aabbMin, out Vector3 aabbMax)
    {
        aabbMin = proxy.aabbMin;
        aabbMax = proxy.aabbMax;
    }

    // btDbvtBroadphase::setAabb (with velocity prediction update logic)
    public override void SetAabb(BroadphaseProxy absproxy, Vector3 aabbMin, Vector3 aabbMax, Dispatcher dispatcher)
    {
        var proxy = (DbvtProxy)absproxy;
        DbvtVolume aabb = DbvtVolume.FromMM(aabbMin, aabbMax);
        // DBVT_BP_PREVENTFALSEUPDATE=0: no NotEqual check
        bool docollide = false;
        if (proxy.stage == StageCount)
        {
            // fixed -> dynamic set
            sets[1].Remove(proxy.leaf);
            proxy.leaf = sets[0].Insert(aabb, proxy);
            docollide = true;
        }
        else
        {
            // dynamic set
            ++updatesCall;
            if (DbvtVolume.Intersect(proxy.leaf.volume, aabb))
            {
                // Moving
                Vector3 delta = aabbMin - proxy.aabbMin;
                Vector3 velocity = ((proxy.aabbMax - proxy.aabbMin) / 2) * prediction;
                if (delta.X < 0) velocity.X = -velocity.X;
                if (delta.Y < 0) velocity.Y = -velocity.Y;
                if (delta.Z < 0) velocity.Z = -velocity.Z;
                if (sets[0].Update(proxy.leaf, aabb, velocity, DbvtMargin))
                {
                    ++updatesDone;
                    docollide = true;
                }
            }
            else
            {
                // Teleporting
                sets[0].Update(proxy.leaf, aabb);
                ++updatesDone;
                docollide = true;
            }
        }
        ListRemove(proxy, ref stageRoots[proxy.stage]);
        proxy.aabbMin = aabbMin;
        proxy.aabbMax = aabbMax;
        proxy.stage = stageCurrent;
        ListAppend(proxy, ref stageRoots[stageCurrent]);
        if (docollide)
        {
            needcleanup = true;
            if (!deferedcollide)
            {
                // Reuse the collider (a stack object in C++)
                DbvtTreeCollider collider = updateCollider;
                collider.proxy = null;
                sets[1].CollideLeaf(sets[1].root, proxy.leaf, collider);
                sets[0].CollideLeaf(sets[0].root, proxy.leaf, collider);
            }
        }
    }

    // btDbvtBroadphase::setAabbForceUpdate (bypasses the shrink guard, always forces an update)
    public void SetAabbForceUpdate(BroadphaseProxy absproxy, Vector3 aabbMin, Vector3 aabbMax, Dispatcher dispatcher)
    {
        var proxy = (DbvtProxy)absproxy;
        DbvtVolume aabb = DbvtVolume.FromMM(aabbMin, aabbMax);
        bool docollide = false;
        if (proxy.stage == StageCount)
        {
            // fixed -> dynamic set
            sets[1].Remove(proxy.leaf);
            proxy.leaf = sets[0].Insert(aabb, proxy);
            docollide = true;
        }
        else
        {
            // dynamic set / Teleporting
            ++updatesCall;
            sets[0].Update(proxy.leaf, aabb);
            ++updatesDone;
            docollide = true;
        }
        ListRemove(proxy, ref stageRoots[proxy.stage]);
        proxy.aabbMin = aabbMin;
        proxy.aabbMax = aabbMax;
        proxy.stage = stageCurrent;
        ListAppend(proxy, ref stageRoots[stageCurrent]);
        if (docollide)
        {
            needcleanup = true;
            if (!deferedcollide)
            {
                // Reuse the collider (a stack object in C++)
                DbvtTreeCollider collider = updateCollider;
                collider.proxy = null;
                sets[1].CollideLeaf(sets[1].root, proxy.leaf, collider);
                sets[0].CollideLeaf(sets[0].root, proxy.leaf, collider);
            }
        }
    }

    /// <summary>BroadphaseRayTester（btDbvtBroadphase.cpp）</summary>
    private sealed class BroadphaseRayTester : Dbvt.ICollide
    {
        private BroadphaseRayCallback rayCallback;

        public BroadphaseRayTester(BroadphaseRayCallback orgCallback) => rayCallback = orgCallback;

        public void SetCallback(BroadphaseRayCallback orgCallback) => rayCallback = orgCallback;

        public override void Process(DbvtNode leaf)
        {
            var proxy = (DbvtProxy)leaf.data;
            rayCallback.Process(proxy);
        }
    }

    // btDbvtBroadphase::rayTest (single-threaded: uses Dbvt's internal persistent stack)
    public override void RayTest(Vector3 rayFromWorld, Vector3 rayToWorld, BroadphaseRayCallback rayCallback,
        Vector3 aabbMin = default, Vector3 aabbMax = default)
    {
        // Reuse the callback wrapper (a stack object in C++)
        BroadphaseRayTester callback = rayTester;
        callback.SetCallback(rayCallback);
        sets[0].RayTestInternal(sets[0].root, rayFromWorld, rayToWorld,
            rayCallback.rayDirectionInverse, rayCallback.signs, rayCallback.lambdaMax,
            aabbMin, aabbMax, callback);
        sets[1].RayTestInternal(sets[1].root, rayFromWorld, rayToWorld,
            rayCallback.rayDirectionInverse, rayCallback.signs, rayCallback.lambdaMax,
            aabbMin, aabbMax, callback);
    }

    /// <summary>BroadphaseAabbTester（btDbvtBroadphase.cpp）</summary>
    private sealed class BroadphaseAabbTester : Dbvt.ICollide
    {
        private BroadphaseAabbCallback aabbCallback;

        public BroadphaseAabbTester(BroadphaseAabbCallback orgCallback) => aabbCallback = orgCallback;

        internal void SetCallback(BroadphaseAabbCallback orgCallback) => aabbCallback = orgCallback;

        public override void Process(DbvtNode leaf)
        {
            var proxy = (DbvtProxy)leaf.data;
            aabbCallback.Process(proxy);
        }
    }

    // btDbvtBroadphase::aabbTest
    public override void AabbTest(Vector3 aabbMin, Vector3 aabbMax, BroadphaseAabbCallback callback)
    {
        // Reuse the callback wrapper (a stack object in C++)
        aabbTester ??= new BroadphaseAabbTester(callback);
        aabbTester.SetCallback(callback);
        DbvtVolume bounds = DbvtVolume.FromMM(aabbMin, aabbMax);
        sets[0].CollideTV(sets[0].root, bounds, aabbTester);
        sets[1].CollideTV(sets[1].root, bounds, aabbTester);
    }

    // btDbvtBroadphase::calculateOverlappingPairs
    public override void CalculateOverlappingPairs(Dispatcher dispatcher)
    {
        Collide(dispatcher);
        PerformDeferredRemoval(dispatcher);
    }

    // btDbvtBroadphase::performDeferredRemoval
    public void PerformDeferredRemoval(Dispatcher dispatcher)
    {
        if (pairCache.HasDeferredRemoval)
        {
            List<BroadphasePair> overlappingPairArray = pairCache.OverlappingPairArray;

            // Sort so duplicate pairs are adjacent and invalid pairs go last
            // (btAlignedObjectArray::quickSort)
            BtQuickSort.QuickSort(overlappingPairArray, BroadphasePairSortCompare);

            int invalidPair = 0;

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
                    // AABB check consistent with the broadphase
                    var pa = (DbvtProxy)pair.proxy0;
                    var pb = (DbvtProxy)pair.proxy1;
                    needsRemoval = !DbvtVolume.Intersect(pa.leaf.volume, pb.leaf.volume);
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

            // Sort again to push invalid pairs to the end and truncate
            // (btAlignedObjectArray::quickSort)
            BtQuickSort.QuickSort(overlappingPairArray, BroadphasePairSortCompare);
            overlappingPairArray.RemoveRange(overlappingPairArray.Count - invalidPair, invalidPair);
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

    // btDbvtBroadphase::collide
    public void Collide(Dispatcher dispatcher)
    {
        // optimize
        sets[0].OptimizeIncremental(1 + (sets[0].leaves * dupdates) / 100);
        if (fixedleft != 0)
        {
            int count = 1 + (sets[1].leaves * fupdates) / 100;
            sets[1].OptimizeIncremental(count);
            fixedleft = BtMath.Max(0, fixedleft - count);
        }
        // dynamic -> fixed set (advance the stage and migrate the whole list)
        stageCurrent = (stageCurrent + 1) % StageCount;
        DbvtProxy current = stageRoots[stageCurrent];
        if (current != null)
        {
            // DBVT_BP_ACCURATESLEEPING=0: migrate directly without re-colliding
            do
            {
                DbvtProxy next = current.links[1];
                ListRemove(current, ref stageRoots[current.stage]);
                ListAppend(current, ref stageRoots[StageCount]);
                sets[0].Remove(current.leaf);
                DbvtVolume curAabb = DbvtVolume.FromMM(current.aabbMin, current.aabbMax);
                current.leaf = sets[1].Insert(curAabb, current);
                current.stage = StageCount;
                current = next;
            } while (current != null);
            fixedleft = sets[1].leaves;
            needcleanup = true;
        }
        // collide dynamics
        {
            // Reuse the collider (a stack object in C++)
            DbvtTreeCollider collider = updateCollider;
            collider.proxy = null;
            if (deferedcollide)
            {
                sets[0].CollideTTpersistentStack(sets[0].root, sets[1].root, collider);
            }
            if (deferedcollide)
            {
                sets[0].CollideTTpersistentStack(sets[0].root, sets[0].root, collider);
            }
        }
        // clean up
        if (needcleanup)
        {
            List<BroadphasePair> pairs = pairCache.OverlappingPairArray;
            if (pairs.Count > 0)
            {
                int ni = BtMath.Min(pairs.Count, BtMath.Max(newpairs, (pairs.Count * cupdates) / 100));
                for (int i = 0; i < ni; ++i)
                {
                    BroadphasePair p = pairs[(cid + i) % pairs.Count];
                    var pa = (DbvtProxy)p.proxy0;
                    var pb = (DbvtProxy)p.proxy1;
                    if (!DbvtVolume.Intersect(pa.leaf.volume, pb.leaf.volume))
                    {
                        pairCache.RemoveOverlappingPair(pa, pb, dispatcher);
                        --ni;
                        --i;
                    }
                }
                if (pairs.Count > 0)
                {
                    cid = (cid + ni) % pairs.Count;
                }
                else
                {
                    cid = 0;
                }
            }
        }
        ++pid;
        newpairs = 1;
        needcleanup = false;
        if (updatesCall > 0)
        {
            updatesRatio = updatesDone / (btScalar)updatesCall;
        }
        else
        {
            updatesRatio = 0;
        }
        updatesDone /= 2;
        updatesCall /= 2;
    }

    // btDbvtBroadphase::optimize
    public void Optimize()
    {
        sets[0].OptimizeTopDown();
        sets[1].OptimizeTopDown();
    }

    public override OverlappingPairCache OverlappingPairCache => pairCache;

    // btDbvtBroadphase::getBroadphaseAabb
    public override void GetBroadphaseAabb(out Vector3 aabbMin, out Vector3 aabbMax)
    {
        DbvtVolume bounds;
        if (!sets[0].Empty())
        {
            if (!sets[1].Empty())
            {
                DbvtVolume.Merge(sets[0].root.volume, sets[1].root.volume, out bounds);
            }
            else
            {
                bounds = sets[0].root.volume;
            }
        }
        else if (!sets[1].Empty())
        {
            bounds = sets[1].root.volume;
        }
        else
        {
            bounds = DbvtVolume.FromCR(Vector3.Zero, 0);
        }
        aabbMin = bounds.mi;
        aabbMax = bounds.mx;
    }

    // btDbvtBroadphase::resetPool (resets internal structures only in an empty scene, for determinism)
    public override void ResetPool(Dispatcher dispatcher)
    {
        int totalObjects = sets[0].leaves + sets[1].leaves;
        if (totalObjects == 0)
        {
            sets[0].Clear();
            sets[1].Clear();

            deferedcollide = false;
            needcleanup = true;
            stageCurrent = 0;
            fixedleft = 0;
            fupdates = 1;
            dupdates = 0;
            cupdates = 10;
            newpairs = 1;
            updatesCall = 0;
            updatesDone = 0;
            updatesRatio = 0;

            gid = 0;
            pid = 0;
            cid = 0;
            for (int i = 0; i <= StageCount; ++i)
            {
                stageRoots[i] = null;
            }
        }
    }

    public override void PrintStats()
    {
        // Empty implementation, as in C++
    }

    // btDbvtBroadphase::setVelocityPrediction / getVelocityPrediction
    public btScalar VelocityPrediction
    {
        get => prediction;
        set => prediction = value;
    }

    public btScalar UpdatesRatio => updatesRatio;
}
