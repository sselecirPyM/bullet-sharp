using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btCompoundCompoundCollisionAlgorithm: collision detection between two
/// compound shapes (CompoundShape). Performs a leaf-to-leaf traversal with AABB
/// culling on both sides; hit child-shape pairs are dispatched to sub-algorithms via
/// dispatcher.FindAlgorithm and cached by key (childIndex0, childIndex1) in a
/// HashedSimplePairCache; pairs are removed when they no longer overlap or either
/// compound revision changes.
/// Note: the btDbvt tree-vs-tree traversal (MycollideTT) from C++ master is not
/// ported (CompoundShape does not provide a dynamic AABB tree; see
/// docs/KNOWN_LIMITATIONS.md, Module A note 4 / Module B2 note 1); it is replaced
/// by an equivalent O(n0*n1) leaf-pair double loop (same semantics: AABB culling
/// happens inside the callback).
/// </summary>
public class CompoundCompoundCollisionAlgorithm : CompoundCollisionAlgorithm
{
    /// <summary>gCompoundCompoundChildShapePairCallback (global child-shape-pair filter callback)</summary>
    public static ShapePairCallback CompoundCompoundChildShapePairCallback;

    /// <summary>m_childCollisionAlgorithmCache: leaf pair (childIndex0, childIndex1) → child collision algorithm</summary>
    internal readonly HashedSimplePairCache childCollisionAlgorithmCache = new();

    private readonly List<SimplePair> removePairs = new();
    private readonly List<PersistentManifold> manifoldArray = new();

    /// <summary>m_compoundShapeRevision0/1: track changes of both compound shapes to keep the child algorithm cache valid</summary>
    internal int compoundShapeRevision0;
    internal int compoundShapeRevision1;

    public CompoundCompoundCollisionAlgorithm(ConstructionInfo ci, CollisionObjectWrapper body0Wrap,
        CollisionObjectWrapper body1Wrap, bool isSwapped)
        : base(ci, body0Wrap, body1Wrap, isSwapped)
    {
        var compoundShape0 = (CompoundShape)body0Wrap.CollisionShape;
        compoundShapeRevision0 = compoundShape0.UpdateRevision;

        var compoundShape1 = (CompoundShape)body1Wrap.CollisionShape;
        compoundShapeRevision1 = compoundShape1.UpdateRevision;
    }

    /// <summary>btCompoundCompoundCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm (not swapped)</summary>
    public static new CollisionAlgorithm Create(ConstructionInfo ci, CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new CompoundCompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, false);

    /// <summary>btCompoundCompoundCollisionAlgorithm::SwappedCreateFunc::CreateCollisionAlgorithm</summary>
    public static new CollisionAlgorithm SwappedCreate(ConstructionInfo ci, CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new CompoundCompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, true);

    /// <summary>btCompoundCompoundCollisionAlgorithm::getAllContactManifolds</summary>
    public override void GetAllContactManifolds(List<PersistentManifold> manifoldArray)
    {
        List<SimplePair> pairs = childCollisionAlgorithmCache.OverlappingPairArray;
        for (int i = 0; i < pairs.Count; i++)
        {
            if (pairs[i].UserPointer is CollisionAlgorithm algo)
                algo.GetAllContactManifolds(manifoldArray);
        }
    }

    /// <summary>btCompoundCompoundCollisionAlgorithm::removeChildAlgorithms</summary>
    protected override void RemoveChildAlgorithms()
    {
        List<SimplePair> pairs = childCollisionAlgorithmCache.OverlappingPairArray;
        for (int i = 0; i < pairs.Count; i++)
        {
            if (pairs[i].UserPointer is CollisionAlgorithm algo)
                dispatcher.FreeCollisionAlgorithm(algo);
        }
        childCollisionAlgorithmCache.RemoveAllPairs();
    }

    /// <summary>btCompoundCompoundCollisionAlgorithm::processCollision</summary>
    public override void ProcessCollision(CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        CollisionObjectWrapper col0ObjWrap = body0Wrap;
        CollisionObjectWrapper col1ObjWrap = body1Wrap;

        var compoundShape0 = (CompoundShape)col0ObjWrap.CollisionShape;
        var compoundShape1 = (CompoundShape)col1ObjWrap.CollisionShape;

        // (C++ master checks getDynamicAabbTree() here and falls back to
        //  btCompoundCollisionAlgorithm::processCollision when either tree is null;
        //  CompoundShape has no dynamic AABB tree (see docs/KNOWN_LIMITATIONS.md,
        //  Module A note 4), so this port replaces the MycollideTT tree-vs-tree
        //  traversal with the O(n0*n1) leaf-pair traversal below)

        // btCompoundShape might have changed: make sure the child collision algorithm cache is still valid
        if (compoundShape0.UpdateRevision != compoundShapeRevision0 ||
            compoundShape1.UpdateRevision != compoundShapeRevision1)
        {
            // clear the entire child algorithm cache
            RemoveChildAlgorithms();
            compoundShapeRevision0 = compoundShape0.UpdateRevision;
            compoundShapeRevision1 = compoundShape1.UpdateRevision;
        }

        // refresh the contact points of all child manifolds
        // note: strictly speaking this should recurse into all child shapes
        // (compounds can be nested several levels deep)
        {
            List<SimplePair> pairs = childCollisionAlgorithmCache.OverlappingPairArray;
            for (int i = 0; i < pairs.Count; i++)
            {
                if (pairs[i].UserPointer is not CollisionAlgorithm algo)
                    continue;
                manifoldArray.Clear();
                algo.GetAllContactManifolds(manifoldArray);
                for (int m = 0; m < manifoldArray.Count; m++)
                {
                    if (manifoldArray[m].NumContacts != 0)
                    {
                        resultOut.SetPersistentManifold(manifoldArray[m]);
                        resultOut.RefreshContactPoints();
                        resultOut.SetPersistentManifold(null);
                    }
                }
            }
            manifoldArray.Clear();
        }

        var callback = new CompoundCompoundLeafCallback(col0ObjWrap, col1ObjWrap, dispatcher,
            dispatchInfo, resultOut, childCollisionAlgorithmCache, sharedManifold);

        // replaces C++ MycollideTT(tree0->m_root, tree1->m_root, xform, callback, distanceThreshold):
        // leaf-pair double loop with AABB culling inside the callback
        int numChildren0 = compoundShape0.NumChildren;
        int numChildren1 = compoundShape1.NumChildren;
        for (int i0 = 0; i0 < numChildren0; i0++)
        {
            for (int i1 = 0; i1 < numChildren1; i1++)
            {
                callback.Process(i0, i1);
            }
        }

        // remove child shape pairs that no longer overlap
        {
            List<SimplePair> pairs = childCollisionAlgorithmCache.OverlappingPairArray;
            for (int i = 0; i < pairs.Count; i++)
            {
                if (pairs[i].UserPointer is not CollisionAlgorithm algo)
                    continue;

                CollisionShape childShape0 = compoundShape0.GetChildShape(pairs[i].IndexA);
                Transform childTrans0 = compoundShape0.GetChildTransformInternal(pairs[i].IndexA);
                Transform newChildWorldTrans0 = col0ObjWrap.WorldTransformInternal * childTrans0;
                childShape0.GetAabb(in newChildWorldTrans0, out Vector3 aabbMin0, out Vector3 aabbMax0);

                // (C++ expands the aabb by resultOut->m_closestPointDistanceThreshold here; the base layer has no such field, treated as 0)

                CollisionShape childShape1 = compoundShape1.GetChildShape(pairs[i].IndexB);
                Transform childTrans1 = compoundShape1.GetChildTransformInternal(pairs[i].IndexB);
                Transform newChildWorldTrans1 = col1ObjWrap.WorldTransformInternal * childTrans1;
                childShape1.GetAabb(in newChildWorldTrans1, out Vector3 aabbMin1, out Vector3 aabbMax1);

                if (!AabbUtil.TestAabbAgainstAabb2(aabbMin0, aabbMax0, aabbMin1, aabbMax1))
                {
                    dispatcher.FreeCollisionAlgorithm(algo);
                    removePairs.Add(pairs[i]);
                }
            }
            for (int i = 0; i < removePairs.Count; i++)
            {
                childCollisionAlgorithmCache.RemoveOverlappingPair(removePairs[i].IndexA, removePairs[i].IndexB);
            }
            removePairs.Clear();
        }
    }

    /// <summary>btCompoundCompoundCollisionAlgorithm::calculateTimeOfImpact
    /// (btAssert(0) + return 0 in C++, not implemented; faithfully ported)</summary>
    public override btScalar CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        return 0;
    }
}

/// <summary>
/// Ported from btCompoundCompoundLeafCallback (a btDbvt::ICollide in C++ master; this
/// port has no Dbvt tree traversal — Process takes both child shape indices directly
/// and is called from a double loop).
/// </summary>
internal sealed class CompoundCompoundLeafCallback
{
    internal int numOverlapPairs;

    internal readonly CollisionObjectWrapper compound0ColObjWrap;
    internal readonly CollisionObjectWrapper compound1ColObjWrap;
    internal readonly Dispatcher dispatcher;
    internal readonly DispatcherInfo dispatchInfo;
    internal readonly ManifoldResult resultOut;
    internal readonly HashedSimplePairCache childCollisionAlgorithmCache;
    internal readonly PersistentManifold sharedManifold;

    public CompoundCompoundLeafCallback(CollisionObjectWrapper compound0ObjWrap, CollisionObjectWrapper compound1ObjWrap,
        Dispatcher dispatcher, DispatcherInfo dispatchInfo, ManifoldResult resultOut,
        HashedSimplePairCache childAlgorithmsCache, PersistentManifold sharedManifold)
    {
        // note: the C++ constructor initializer list swaps the two arguments
        // (m_compound0ColObjWrap(compound1ObjWrap)) and relies on the argument order
        // at the call site to cancel it out; here we just assign with correct semantics
        compound0ColObjWrap = compound0ObjWrap;
        compound1ColObjWrap = compound1ObjWrap;
        this.dispatcher = dispatcher;
        this.dispatchInfo = dispatchInfo;
        this.resultOut = resultOut;
        childCollisionAlgorithmCache = childAlgorithmsCache;
        this.sharedManifold = sharedManifold;
    }

    /// <summary>btCompoundCompoundLeafCallback::Process (parameters changed from btDbvtNode to child shape indices)</summary>
    public void Process(int childIndex0, int childIndex1)
    {
        numOverlapPairs++;

        var compoundShape0 = (CompoundShape)compound0ColObjWrap.CollisionShape;
        var compoundShape1 = (CompoundShape)compound1ColObjWrap.CollisionShape;

        CollisionShape childShape0 = compoundShape0.GetChildShape(childIndex0);
        CollisionShape childShape1 = compoundShape1.GetChildShape(childIndex1);

        // backup (in C++, orgTrans is only used to compute newChildWorldTrans)
        Transform childTrans0 = compoundShape0.GetChildTransformInternal(childIndex0);
        Transform newChildWorldTrans0 = compound0ColObjWrap.WorldTransformInternal * childTrans0;

        Transform childTrans1 = compoundShape1.GetChildTransformInternal(childIndex1);
        Transform newChildWorldTrans1 = compound1ColObjWrap.WorldTransformInternal * childTrans1;

        // do the AABB check first
        childShape0.GetAabb(in newChildWorldTrans0, out Vector3 aabbMin0, out Vector3 aabbMax0);
        childShape1.GetAabb(in newChildWorldTrans1, out Vector3 aabbMin1, out Vector3 aabbMax1);

        // (C++ expands aabb0 by resultOut->m_closestPointDistanceThreshold here; the base layer has no such field, treated as 0)

        if (CompoundCompoundCollisionAlgorithm.CompoundCompoundChildShapePairCallback != null &&
            !CompoundCompoundCollisionAlgorithm.CompoundCompoundChildShapePairCallback(childShape0, childShape1))
            return;

        if (!AabbUtil.TestAabbAgainstAabb2(aabbMin0, aabbMax0, aabbMin1, aabbMax1))
            return;

        var compoundWrap0 = new CollisionObjectWrapper(compound0ColObjWrap, childShape0,
            compound0ColObjWrap.CollisionObject, in newChildWorldTrans0, -1, childIndex0);
        var compoundWrap1 = new CollisionObjectWrapper(compound1ColObjWrap, childShape1,
            compound1ColObjWrap.CollisionObject, in newChildWorldTrans1, -1, childIndex1);

        // (in C++ the BT_CLOSEST_POINT_ALGORITHMS are used when closestPointDistanceThreshold>0
        //  and released right after; the base layer has no such field (treated as 0), so we
        //  always take the cached branch below)
        CollisionAlgorithm colAlgo;
        SimplePair pair = childCollisionAlgorithmCache.FindPair(childIndex0, childIndex1);
        if (pair != null)
        {
            colAlgo = (CollisionAlgorithm)pair.UserPointer;
        }
        else
        {
            colAlgo = dispatcher.FindAlgorithm(compoundWrap0, compoundWrap1, sharedManifold,
                DispatcherQueryType.ContactPointAlgorithms);
            if (colAlgo == null)
                return;
            pair = childCollisionAlgorithmCache.AddOverlappingPair(childIndex0, childIndex1);
            pair.UserPointer = colAlgo;
        }

        // temporarily replace the wrappers on both sides of resultOut
        CollisionObjectWrapper tmpWrap0 = resultOut.body0Wrap;
        CollisionObjectWrapper tmpWrap1 = resultOut.body1Wrap;

        resultOut.body0Wrap = compoundWrap0;
        resultOut.body1Wrap = compoundWrap1;

        // setShapeIdentifiersA(-1, childIndex0) / setShapeIdentifiersB(-1, childIndex1)
        resultOut.partId0 = -1;
        resultOut.index0 = childIndex0;
        resultOut.partId1 = -1;
        resultOut.index1 = childIndex1;

        colAlgo.ProcessCollision(compoundWrap0, compoundWrap1, dispatchInfo, resultOut);

        resultOut.body0Wrap = tmpWrap0;
        resultOut.body1Wrap = tmpWrap1;
    }
}
