using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btConvexConcaveCollisionAlgorithm: collision detection between a
/// convex body and a concave body (triangle mesh etc.). For each triangle of the
/// concave shape overlapping the convex body's AABB, a temporary TriangleShape is
/// built and a sub-algorithm is dispatched via dispatcher.FindAlgorithm
/// (typically ConvexConvexAlgorithm / GJK).
/// Note: the SDF branch (btSdfCollisionShape) and the
/// BT_ENABLE_CONVEX_CONCAVE_EARLY_OUT early-out optimization from C++ are not
/// ported (see docs/KNOWN_LIMITATIONS.md, Module B2 note 5).
/// </summary>
public class ConvexConcaveCollisionAlgorithm : ActivatingCollisionAlgorithm
{
    /// <summary>btConvexConcaveCollisionAlgorithm::m_btConvexTriangleCallback</summary>
    internal readonly ConvexTriangleCallback convexTriangleCallback;
    internal bool isSwapped;

    public ConvexConcaveCollisionAlgorithm(ConstructionInfo ci, CollisionObjectWrapper body0Wrap,
        CollisionObjectWrapper body1Wrap, bool isSwapped)
        : base(ci, body0Wrap, body1Wrap)
    {
        convexTriangleCallback = new ConvexTriangleCallback(ci.Dispatcher, body0Wrap, body1Wrap, isSwapped);
        this.isSwapped = isSwapped;
    }

    /// <summary>btConvexConcaveCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm (not swapped)</summary>
    public static CollisionAlgorithm Create(ConstructionInfo ci, CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new ConvexConcaveCollisionAlgorithm(ci, body0Wrap, body1Wrap, false);

    /// <summary>btConvexConcaveCollisionAlgorithm::SwappedCreateFunc::CreateCollisionAlgorithm</summary>
    public static CollisionAlgorithm SwappedCreate(ConstructionInfo ci, CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new ConvexConcaveCollisionAlgorithm(ci, body0Wrap, body1Wrap, true);

    /// <summary>btConvexConcaveCollisionAlgorithm::getAllContactManifolds</summary>
    public override void GetAllContactManifolds(List<PersistentManifold> manifoldArray)
    {
        if (convexTriangleCallback.manifoldPtr != null)
            manifoldArray.Add(convexTriangleCallback.manifoldPtr);
    }

    /// <summary>btConvexConcaveCollisionAlgorithm::clearCache</summary>
    public void ClearCache() => convexTriangleCallback.ClearCache();

    /// <summary>btConvexConcaveCollisionAlgorithm::processCollision</summary>
    public override void ProcessCollision(CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        CollisionObjectWrapper convexBodyWrap = isSwapped ? body1Wrap : body0Wrap;
        CollisionObjectWrapper triBodyWrap = isSwapped ? body0Wrap : body1Wrap;

        if (triBodyWrap.CollisionShape.IsConcave)
        {
            // C++ checks SDF_SHAPE_PROXYTYPE here and takes the btSdfCollisionShape::queryPoint
            // branch; not ported (see docs/KNOWN_LIMITATIONS.md, Module B2 note 5), so this
            // takes the regular concave branch.
            var concaveShape = (ConcaveShape)triBodyWrap.CollisionShape;

            if (convexBodyWrap.CollisionShape.IsConvex)
            {
                btScalar collisionMarginTriangle = concaveShape.Margin;

                resultOut.SetPersistentManifold(convexTriangleCallback.manifoldPtr);
                convexTriangleCallback.SetTimeStepAndCounters(collisionMarginTriangle, dispatchInfo,
                    convexBodyWrap, triBodyWrap, resultOut);

                convexTriangleCallback.manifoldPtr.SetBodies(convexBodyWrap.CollisionObject, triBodyWrap.CollisionObject);

                concaveShape.ProcessAllTriangles(convexTriangleCallback,
                    convexTriangleCallback.AabbMin, convexTriangleCallback.AabbMax);

                resultOut.RefreshContactPoints();

                convexTriangleCallback.ClearWrapperData();
            }
        }
    }

    /// <summary>btConvexConcaveCollisionAlgorithm::calculateTimeOfImpact
    /// (approximate CCD with a swept sphere against triangles; port of LocalTriangleSphereCastCallback)</summary>
    public override btScalar CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        CollisionObject convexBody = isSwapped ? body1 : body0;
        CollisionObject triBody = isSwapped ? body0 : body1;

        // only perform CCD if the motion exceeds the threshold, to avoid being stuck
        // in blocking CCD (linear velocity is halved each frame while hitFraction<1)
        btScalar squareMot0 = (convexBody.InterpolationWorldTransformInternal.Origin - convexBody.WorldTransformInternal.Origin).LengthSquared();
        if (squareMot0 < convexBody.CcdSquareMotionThreshold)
            return 1;

        Transform triInv = triBody.WorldTransformInternal.Inverse();
        Transform convexFromLocal = triInv * convexBody.WorldTransformInternal;
        Transform convexToLocal = triInv * convexBody.InterpolationWorldTransformInternal;

        if (triBody.CollisionShape.IsConcave)
        {
            Vector3 rayAabbMin = convexFromLocal.Origin;
            rayAabbMin.SetMin(convexToLocal.Origin);
            Vector3 rayAabbMax = convexFromLocal.Origin;
            rayAabbMax.SetMax(convexToLocal.Origin);
            btScalar ccdRadius0 = convexBody.CcdSweptSphereRadius;
            rayAabbMin -= new Vector3(ccdRadius0);
            rayAabbMax += new Vector3(ccdRadius0);

            btScalar curHitFraction = 1;
            localSphereCastCallback ??= new LocalTriangleSphereCastCallback(in convexFromLocal, in convexToLocal,
                convexBody.CcdSweptSphereRadius, curHitFraction);
            localSphereCastCallback.Set(in convexFromLocal, in convexToLocal,
                convexBody.CcdSweptSphereRadius, convexBody.HitFraction);

            var triangleMesh = (ConcaveShape)triBody.CollisionShape;
            triangleMesh.ProcessAllTriangles(localSphereCastCallback, rayAabbMin, rayAabbMax);

            if (localSphereCastCallback.hitFraction < convexBody.HitFraction)
            {
                convexBody.HitFraction = localSphereCastCallback.hitFraction;
                return localSphereCastCallback.hitFraction;
            }
        }

        return 1;
    }

    internal override void Cleanup(Dispatcher dispatcher)
    {
        // ~btConvexTriangleCallback: clear the cache and release the owned manifold
        if (convexTriangleCallback.manifoldPtr != null)
        {
            convexTriangleCallback.ClearCache();
            dispatcher.ReleaseManifold(convexTriangleCallback.manifoldPtr);
            convexTriangleCallback.manifoldPtr = null;
        }
    }

    // Reused CCD sweep callback (a stack object in C++ calculateTimeOfImpact)
    private LocalTriangleSphereCastCallback localSphereCastCallback;

    /// <summary>
    /// Ported from LocalTriangleSphereCastCallback in btConvexConcaveCollisionAlgorithm.cpp
    /// (used inside calculateTimeOfImpact: runs a swept-sphere SubSimplexConvexCast
    /// against each triangle).
    /// </summary>
    private sealed class LocalTriangleSphereCastCallback : TriangleCallback
    {
        internal Transform ccdSphereFromTrans;
        internal Transform ccdSphereToTrans;
        internal btScalar ccdSphereRadius;
        internal btScalar hitFraction;

        // Reused per-triangle temporaries (stack objects in C++ ProcessTriangle)
        private readonly CastResult castResult = new();
        private readonly SphereShape pointShape = new(0);
        private readonly TriangleShape triShape = new();
        private readonly VoronoiSimplexSolver simplexSolver = new();
        private readonly SubSimplexConvexCast convexCaster;

        public LocalTriangleSphereCastCallback(in Transform from, in Transform to, btScalar ccdSphereRadius, btScalar hitFraction)
        {
            convexCaster = new SubSimplexConvexCast(pointShape, triShape, simplexSolver);
            Set(in from, in to, ccdSphereRadius, hitFraction);
        }

        public void Set(in Transform from, in Transform to, btScalar ccdSphereRadius, btScalar hitFraction)
        {
            ccdSphereFromTrans = from;
            ccdSphereToTrans = to;
            this.ccdSphereRadius = ccdSphereRadius;
            this.hitFraction = hitFraction;
        }

        public override void ProcessTriangle(Vector3[] triangle, int partId, int triangleIndex)
        {
            // swept sphere only for now (reuses the temporaries, equivalent to the C++ stack objects)
            Transform ident = Transform.Identity;
            castResult.Reset(hitFraction);
            pointShape.Margin = ccdSphereRadius;
            triShape.vertices1[0] = triangle[0];
            triShape.vertices1[1] = triangle[1];
            triShape.vertices1[2] = triangle[2];
            convexCaster.SetShapes(pointShape, triShape);

            if (convexCaster.CalcTimeOfImpact(in ccdSphereFromTrans, in ccdSphereToTrans, in ident, in ident, castResult))
            {
                if (hitFraction > castResult.Fraction)
                    hitFraction = castResult.Fraction;
            }
        }
    }
}

/// <summary>
/// Ported from btConvexTriangleCallback: for each triangle of the concave body
/// overlapping the convex body's AABB, builds a temporary TriangleShape child wrapper
/// and dispatches a convex-triangle sub-algorithm via the dispatcher.
/// (Inherits btTriangleCallback in C++; this port likewise inherits TriangleCallback
/// to match the signature of ConcaveShape.ProcessAllTriangles.)
/// </summary>
internal class ConvexTriangleCallback : TriangleCallback
{
    internal Vector3 aabbMin;
    internal Vector3 aabbMax;

    internal CollisionObjectWrapper convexBodyWrap;
    internal CollisionObjectWrapper triBodyWrap;

    internal ManifoldResult resultOut;
    internal readonly Dispatcher dispatcher;
    internal DispatcherInfo dispatchInfoPtr;
    internal btScalar collisionMarginTriangle;

    internal PersistentManifold manifoldPtr;

    // Reused per-triangle temporaries (stack objects in C++ processTriangle)
    private readonly TriangleShape tm = new();
    private readonly CollisionObjectWrapper triObWrap = new(null, null, null, Transform.Identity, -1, -1);

    public ConvexTriangleCallback(Dispatcher dispatcher, CollisionObjectWrapper body0Wrap,
        CollisionObjectWrapper body1Wrap, bool isSwapped)
    {
        this.dispatcher = dispatcher;
        convexBodyWrap = isSwapped ? body1Wrap : body0Wrap;
        triBodyWrap = isSwapped ? body0Wrap : body1Wrap;

        // create a manifold from the dispatcher's manifold pool
        manifoldPtr = dispatcher.GetNewManifold(convexBodyWrap.CollisionObject, triBodyWrap.CollisionObject);

        ClearCache();
    }

    /// <summary>btConvexTriangleCallback::setTimeStepAndCounters</summary>
    public void SetTimeStepAndCounters(btScalar collisionMarginTriangle, DispatcherInfo dispatchInfo,
        CollisionObjectWrapper convexBodyWrap, CollisionObjectWrapper triBodyWrap, ManifoldResult resultOut)
    {
        this.convexBodyWrap = convexBodyWrap;
        this.triBodyWrap = triBodyWrap;

        dispatchInfoPtr = dispatchInfo;
        this.collisionMarginTriangle = collisionMarginTriangle;
        this.resultOut = resultOut;

        // recompute the aabb (the convex body's AABB in concave local space)
        Transform convexInTriangleSpace = triBodyWrap.WorldTransformInternal.Inverse() * convexBodyWrap.WorldTransformInternal;
        CollisionShape convexShape = convexBodyWrap.CollisionShape;
        convexShape.GetAabb(in convexInTriangleSpace, out aabbMin, out aabbMax);
        // (C++ also adds resultOut->m_closestPointDistanceThreshold here; the current base layer has no such field, treated as 0)
        btScalar extraMargin = collisionMarginTriangle;

        var extra = new Vector3(extraMargin);

        aabbMax += extra;
        aabbMin -= extra;
    }

    /// <summary>btConvexTriangleCallback::clearWrapperData</summary>
    public void ClearWrapperData()
    {
        convexBodyWrap = null;
        triBodyWrap = null;
    }

    /// <summary>btConvexTriangleCallback::clearCache</summary>
    public void ClearCache() => dispatcher.ClearManifold(manifoldPtr);

    /// <summary>btConvexTriangleCallback::getAabbMin</summary>
    public Vector3 AabbMin => aabbMin;

    /// <summary>btConvexTriangleCallback::getAabbMax</summary>
    public Vector3 AabbMax => aabbMax;

    /// <summary>btConvexTriangleCallback::processTriangle</summary>
    public override void ProcessTriangle(Vector3[] triangle, int partId, int triangleIndex)
    {
        if (!AabbUtil.TestTriangleAgainstAabb2(triangle[0], triangle[1], triangle[2], aabbMin, aabbMax))
            return;

        if (convexBodyWrap.CollisionShape.IsConvex)
        {
            // reuse the TriangleShape and child wrapper (stack objects in C++)
            tm.vertices1[0] = triangle[0];
            tm.vertices1[1] = triangle[1];
            tm.vertices1[2] = triangle[2];
            tm.Margin = collisionMarginTriangle;

            triObWrap.Set(triBodyWrap, tm, triBodyWrap.CollisionObject,
                triBodyWrap.WorldTransformInternal, partId, triangleIndex);

            // (in C++ the BT_CLOSEST_POINT_ALGORITHMS are used when closestPointDistanceThreshold>0;
            //  the base layer has no such field (treated as 0), so we always take the contact point algorithms)
            CollisionAlgorithm colAlgo = dispatcher.FindAlgorithm(convexBodyWrap, triObWrap,
                manifoldPtr, DispatcherQueryType.ContactPointAlgorithms);
            if (colAlgo == null)
                return;

            CollisionObjectWrapper tmpWrap;
            bool isTriBody0 = resultOut.Body0Internal == triBodyWrap.CollisionObject;
            if (isTriBody0)
            {
                tmpWrap = resultOut.body0Wrap;
                resultOut.body0Wrap = triObWrap;
                // setShapeIdentifiersA(partId, triangleIndex)
                resultOut.partId0 = partId;
                resultOut.index0 = triangleIndex;
            }
            else
            {
                tmpWrap = resultOut.body1Wrap;
                resultOut.body1Wrap = triObWrap;
                // setShapeIdentifiersB(partId, triangleIndex)
                resultOut.partId1 = partId;
                resultOut.index1 = triangleIndex;
            }

            colAlgo.ProcessCollision(convexBodyWrap, triObWrap, dispatchInfoPtr, resultOut);

            if (isTriBody0)
                resultOut.body0Wrap = tmpWrap;
            else
                resultOut.body1Wrap = tmpWrap;

            dispatcher.FreeCollisionAlgorithm(colAlgo);
        }
    }
}
