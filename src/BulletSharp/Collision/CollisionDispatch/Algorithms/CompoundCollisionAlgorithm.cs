using BulletSharp.Math;

namespace BulletSharp;

/// <summary>btShapePairCallback: child-shape-pair filter callback (return false to skip the pair)</summary>
public delegate bool ShapePairCallback(CollisionShape shape0, CollisionShape shape1);

/// <summary>
/// Ported from btCompoundCollisionAlgorithm: collision detection between a compound
/// shape (CompoundShape) and another shape. Iterates the compound's child shapes,
/// culls by AABB, and dispatches a sub-algorithm per child via
/// dispatcher.FindAlgorithm; sub-algorithms are cached by key (childIndex, 0) in a
/// HashedSimplePairCache and removed when they no longer overlap or the compound
/// revision changes.
/// Note: the btDbvt dynamic AABB tree acceleration path from C++ master is not
/// ported (CompoundShape does not provide a dynamic AABB tree; see
/// docs/KNOWN_LIMITATIONS.md, Module A note 4 / Module B2 note 1); this is
/// equivalent to the C++ fallback that iterates all child shapes when
/// getDynamicAabbTree()==null.
/// </summary>
public class CompoundCollisionAlgorithm : ActivatingCollisionAlgorithm
{
    /// <summary>gCompoundChildShapePairCallback (global child-shape-pair filter callback)</summary>
    public static ShapePairCallback CompoundChildShapePairCallback;

    internal bool isSwapped;

    /// <summary>Cache of child shape index → child collision algorithm (btHashedSimplePairCache)</summary>
    internal readonly HashedSimplePairCache childAlgorithmsCache = new();

    internal PersistentManifold sharedManifold;
    internal bool ownsManifold;

    /// <summary>m_compoundShapeRevision: tracks compound shape changes to keep the child algorithm cache valid</summary>
    internal int compoundShapeRevision;

    // Reused temporary buffers (correspond to C++ local btManifoldArray / m_removePairs)
    private readonly List<PersistentManifold> manifoldArray = new();
    private readonly List<SimplePair> removePairs = new();

    public CompoundCollisionAlgorithm(ConstructionInfo ci, CollisionObjectWrapper body0Wrap,
        CollisionObjectWrapper body1Wrap, bool isSwapped)
        : base(ci, body0Wrap, body1Wrap)
    {
        this.isSwapped = isSwapped;
        sharedManifold = ci.Manifold;
        ownsManifold = false;

        CollisionObjectWrapper colObjWrap = isSwapped ? body1Wrap : body0Wrap;
        var compoundShape = (CompoundShape)colObjWrap.CollisionShape;
        compoundShapeRevision = compoundShape.UpdateRevision;
    }

    /// <summary>btCompoundCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm (not swapped)</summary>
    public static CollisionAlgorithm Create(ConstructionInfo ci, CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new CompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, false);

    /// <summary>btCompoundCollisionAlgorithm::SwappedCreateFunc::CreateCollisionAlgorithm</summary>
    public static CollisionAlgorithm SwappedCreate(ConstructionInfo ci, CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new CompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, true);

    /// <summary>btCompoundCollisionAlgorithm::getChildAlgorithm (cached child algorithm by child shape index)</summary>
    public CollisionAlgorithm GetChildAlgorithm(int childIndex) =>
        childAlgorithmsCache.FindPair(childIndex, 0)?.UserPointer as CollisionAlgorithm;

    /// <summary>btCompoundCollisionAlgorithm::removeChildAlgorithms</summary>
    protected virtual void RemoveChildAlgorithms()
    {
        List<SimplePair> pairs = childAlgorithmsCache.OverlappingPairArray;
        for (int i = 0; i < pairs.Count; i++)
        {
            if (pairs[i].UserPointer is CollisionAlgorithm algo)
                dispatcher.FreeCollisionAlgorithm(algo);
        }
        childAlgorithmsCache.RemoveAllPairs();
    }

    /// <summary>btCompoundCollisionAlgorithm::getAllContactManifolds</summary>
    public override void GetAllContactManifolds(List<PersistentManifold> manifoldArray)
    {
        List<SimplePair> pairs = childAlgorithmsCache.OverlappingPairArray;
        for (int i = 0; i < pairs.Count; i++)
        {
            if (pairs[i].UserPointer is CollisionAlgorithm algo)
                algo.GetAllContactManifolds(manifoldArray);
        }
    }

    /// <summary>btCompoundCollisionAlgorithm::processCollision</summary>
    public override void ProcessCollision(CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        CollisionObjectWrapper colObjWrap = isSwapped ? body1Wrap : body0Wrap;
        CollisionObjectWrapper otherObjWrap = isSwapped ? body0Wrap : body1Wrap;

        var compoundShape = (CompoundShape)colObjWrap.CollisionShape;

        // btCompoundShape might have changed: make sure the child collision algorithm cache is still valid
        if (compoundShape.UpdateRevision != compoundShapeRevision)
        {
            // clear the entire child algorithm cache
            RemoveChildAlgorithms();
            compoundShapeRevision = compoundShape.UpdateRevision;
        }

        // refresh the contact points of all child manifolds (stale contacts are culled
        // by the breaking threshold)
        // note: strictly speaking this should recurse into all child shapes
        // (compounds can be nested several levels deep)
        {
            List<SimplePair> pairs = childAlgorithmsCache.OverlappingPairArray;
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

        // C++ master prefers compoundShape->getDynamicAabbTree() for tree pruning here;
        // CompoundShape has no dynamic AABB tree (see docs/KNOWN_LIMITATIONS.md,
        // Module A note 4), so take the "no tree" fallback: iterate all child shapes,
        // with the AABB check inside ProcessChildShape
        var callback = new CompoundLeafCallback(colObjWrap, otherObjWrap, dispatcher, dispatchInfo,
            resultOut, childAlgorithmsCache, sharedManifold);
        int numChildren = compoundShape.NumChildren;
        for (int i = 0; i < numChildren; i++)
        {
            callback.ProcessChildShape(compoundShape.GetChildShape(i), i);
        }

        // remove child shape pairs that no longer overlap
        {
            List<SimplePair> pairs = childAlgorithmsCache.OverlappingPairArray;
            for (int i = 0; i < pairs.Count; i++)
            {
                if (pairs[i].UserPointer is not CollisionAlgorithm algo)
                    continue;

                CollisionShape childShape = compoundShape.GetChildShape(pairs[i].IndexA);
                Transform childTrans = compoundShape.GetChildTransformInternal(pairs[i].IndexA);
                Transform newChildWorldTrans = colObjWrap.WorldTransformInternal * childTrans;

                // do the AABB check first
                childShape.GetAabb(in newChildWorldTrans, out Vector3 aabbMin0, out Vector3 aabbMax0);
                otherObjWrap.CollisionShape.GetAabb(otherObjWrap.WorldTransformInternal, out Vector3 aabbMin1, out Vector3 aabbMax1);

                if (!AabbUtil.TestAabbAgainstAabb2(aabbMin0, aabbMax0, aabbMin1, aabbMax1))
                {
                    dispatcher.FreeCollisionAlgorithm(algo);
                    removePairs.Add(pairs[i]);
                }
            }
            for (int i = 0; i < removePairs.Count; i++)
            {
                childAlgorithmsCache.RemoveOverlappingPair(removePairs[i].IndexA, removePairs[i].IndexB);
            }
            removePairs.Clear();
        }
    }

    /// <summary>btCompoundCollisionAlgorithm::calculateTimeOfImpact
    /// (has a btAssert(0) in C++: implemented by temporarily modifying the colObj
    /// world transform; unfinished — this port faithfully reproduces that behavior)</summary>
    public override btScalar CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        CollisionObject colObj = isSwapped ? body1 : body0;
        CollisionObject otherObj = isSwapped ? body0 : body1;

        var compoundShape = (CompoundShape)colObj.CollisionShape;

        btScalar hitFraction = 1;

        List<SimplePair> pairs = childAlgorithmsCache.OverlappingPairArray;
        for (int i = 0; i < pairs.Count; i++)
        {
            if (pairs[i].UserPointer is not CollisionAlgorithm algo)
                continue;

            // back up the world transform
            Transform orgTrans = colObj.WorldTransformInternal;
            Transform childTrans = compoundShape.GetChildTransformInternal(pairs[i].IndexA);
            colObj.WorldTransformInternal = orgTrans * childTrans;

            btScalar frac = algo.CalculateTimeOfImpact(colObj, otherObj, dispatchInfo, resultOut);
            if (frac < hitFraction)
                hitFraction = frac;

            // restore
            colObj.WorldTransformInternal = orgTrans;
        }
        return hitFraction;
    }

    internal override void Cleanup(Dispatcher dispatcher)
    {
        RemoveChildAlgorithms();
        if (ownsManifold && sharedManifold != null)
        {
            dispatcher.ReleaseManifold(sharedManifold);
            sharedManifold = null;
        }
    }
}

/// <summary>
/// Ported from btCompoundLeafCallback (a btDbvt::ICollide in C++ master; this port
/// has no Dbvt tree traversal and keeps only ProcessChildShape, which ProcessCollision
/// calls for every child shape).
/// </summary>
internal sealed class CompoundLeafCallback
{
    internal readonly CollisionObjectWrapper compoundColObjWrap;
    internal readonly CollisionObjectWrapper otherObjWrap;
    internal readonly Dispatcher dispatcher;
    internal readonly DispatcherInfo dispatchInfo;
    internal readonly ManifoldResult resultOut;
    internal readonly HashedSimplePairCache childAlgorithmsCache;
    internal readonly PersistentManifold sharedManifold;

    public CompoundLeafCallback(CollisionObjectWrapper compoundObjWrap, CollisionObjectWrapper otherObjWrap,
        Dispatcher dispatcher, DispatcherInfo dispatchInfo, ManifoldResult resultOut,
        HashedSimplePairCache childAlgorithmsCache, PersistentManifold sharedManifold)
    {
        compoundColObjWrap = compoundObjWrap;
        this.otherObjWrap = otherObjWrap;
        this.dispatcher = dispatcher;
        this.dispatchInfo = dispatchInfo;
        this.resultOut = resultOut;
        this.childAlgorithmsCache = childAlgorithmsCache;
        this.sharedManifold = sharedManifold;
    }

    /// <summary>btCompoundLeafCallback::ProcessChildShape</summary>
    public void ProcessChildShape(CollisionShape childShape, int index)
    {
        var compoundShape = (CompoundShape)compoundColObjWrap.CollisionShape;

        if (CompoundCollisionAlgorithm.CompoundChildShapePairCallback != null &&
            !CompoundCollisionAlgorithm.CompoundChildShapePairCallback(otherObjWrap.CollisionShape, childShape))
            return;

        Transform childTrans = compoundShape.GetChildTransformInternal(index);
        Transform newChildWorldTrans = compoundColObjWrap.WorldTransformInternal * childTrans;

        // do the AABB check first
        childShape.GetAabb(in newChildWorldTrans, out Vector3 aabbMin0, out Vector3 aabbMax0);
        // (C++ also expands the aabb by resultOut->m_closestPointDistanceThreshold here; the base layer has no such field, treated as 0)
        otherObjWrap.CollisionShape.GetAabb(otherObjWrap.WorldTransformInternal, out Vector3 aabbMin1, out Vector3 aabbMax1);

        if (!AabbUtil.TestAabbAgainstAabb2(aabbMin0, aabbMax0, aabbMin1, aabbMax1))
            return;

        // (C++ master also composes the wrapper's preTransform here; the C# CollisionObjectWrapper has no such member, omitted)
        var compoundWrap = new CollisionObjectWrapper(compoundColObjWrap, childShape,
            compoundColObjWrap.CollisionObject, in newChildWorldTrans, -1, index);

        // (in C++ the BT_CLOSEST_POINT_ALGORITHMS are used when closestPointDistanceThreshold>0
        //  and released right after; the base layer has no such field (treated as 0), so we
        //  always take the cached branch below)
        CollisionAlgorithm algo;
        SimplePair pair = childAlgorithmsCache.FindPair(index, 0);
        if (pair != null)
        {
            algo = (CollisionAlgorithm)pair.UserPointer;
        }
        else
        {
            algo = dispatcher.FindAlgorithm(compoundWrap, otherObjWrap, sharedManifold,
                DispatcherQueryType.ContactPointAlgorithms);
            if (algo == null)
                return;
            pair = childAlgorithmsCache.AddOverlappingPair(index, 0);
            pair.UserPointer = algo;
        }

        // detect the swapped case and temporarily replace the wrapper on the compound side in resultOut
        CollisionObjectWrapper tmpWrap;
        bool body0IsCompound = resultOut.Body0Internal == compoundColObjWrap.CollisionObject;
        if (body0IsCompound)
        {
            tmpWrap = resultOut.body0Wrap;
            resultOut.body0Wrap = compoundWrap;
            // setShapeIdentifiersA(-1, index)
            resultOut.partId0 = -1;
            resultOut.index0 = index;
        }
        else
        {
            tmpWrap = resultOut.body1Wrap;
            resultOut.body1Wrap = compoundWrap;
            // setShapeIdentifiersB(-1, index)
            resultOut.partId1 = -1;
            resultOut.index1 = index;
        }

        algo.ProcessCollision(compoundWrap, otherObjWrap, dispatchInfo, resultOut);

        if (body0IsCompound)
            resultOut.body0Wrap = tmpWrap;
        else
            resultOut.body1Wrap = tmpWrap;
    }
}
