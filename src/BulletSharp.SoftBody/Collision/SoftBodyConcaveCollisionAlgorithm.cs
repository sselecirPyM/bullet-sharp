using System.Diagnostics;
using BulletSharp.Math;

namespace BulletSharp.SoftBody;

/// <summary>
/// Packed (partId, triangleIndex) key and cached extruded child shape for concave mesh collision.
/// Ported from BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.h (btTriIndex).
/// </summary>
public struct TriIndex
{
    internal int partIdTriangleIndex;
    internal CollisionShape childShape;

    public TriIndex(int partId, int triangleIndex, CollisionShape shape)
    {
        partIdTriangleIndex = (partId << (31 - QuantizedBvh.MaxNumPartsInBits)) | triangleIndex;
        childShape = shape;
    }

    public readonly int PartIdTriangleIndex => partIdTriangleIndex;

    public CollisionShape ChildShape
    {
        readonly get => childShape;
        set => childShape = value;
    }

    public readonly int TriangleIndex =>
        partIdTriangleIndex & ~(~0 << (31 - QuantizedBvh.MaxNumPartsInBits));

    public readonly int PartId =>
        partIdTriangleIndex >> (31 - QuantizedBvh.MaxNumPartsInBits);

    public readonly int Uid => partIdTriangleIndex;
}

/// <summary>
/// Processes concave mesh triangles overlapping a soft body's AABB, caching extruded hull shapes per triangle.
/// Ported from BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.{h,cpp} (btSoftBodyTriangleCallback).
/// </summary>
public class SoftBodyTriangleCallback : TriangleCallback
{
    private const btScalar SoftBodyTriangleExtrusion = (btScalar)0.06;

    internal SoftBody softBody;
    internal CollisionObject triBody;

    internal Vector3 aabbMin;
    internal Vector3 aabbMax;

    internal ManifoldResult resultOut;

    internal readonly Dispatcher dispatcher;
    internal DispatcherInfo dispatchInfoPtr;
    internal btScalar collisionMarginTriangle;

    internal readonly Dictionary<int, TriIndex> shapeCache = new();

    private readonly CollisionObjectWrapper softBodyWrap = new(null, null, null, Transform.Identity, -1, -1);
    private readonly CollisionObjectWrapper triBodyWrap = new(null, null, null, Transform.Identity, -1, -1);

    public int TriangleCount;

    public SoftBodyTriangleCallback(Dispatcher dispatcher, CollisionObjectWrapper body0Wrap,
        CollisionObjectWrapper body1Wrap, bool isSwapped)
    {
        this.dispatcher = dispatcher;
        dispatchInfoPtr = null;
        softBody = isSwapped
            ? (SoftBody)body1Wrap.CollisionObject
            : (SoftBody)body0Wrap.CollisionObject;
        triBody = isSwapped ? body0Wrap.CollisionObject : body1Wrap.CollisionObject;

        ClearCache();
    }

    public Vector3 AabbMin => aabbMin;

    public Vector3 AabbMax => aabbMax;

    public void SetTimeStepAndCounters(btScalar collisionMarginTriangle, CollisionObjectWrapper triObjWrap,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        dispatchInfoPtr = dispatchInfo;
        this.collisionMarginTriangle = collisionMarginTriangle + SoftBodyTriangleExtrusion;
        this.resultOut = resultOut;

        softBody.GetAabb(out Vector3 aabbWorldSpaceMin, out Vector3 aabbWorldSpaceMax);
        Vector3 halfExtents = (aabbWorldSpaceMax - aabbWorldSpaceMin) * (btScalar)0.5;
        Vector3 softBodyCenter = (aabbWorldSpaceMax + aabbWorldSpaceMin) * (btScalar)0.5;

        Transform softTransform = new(Matrix3x3.Identity, softBodyCenter);

        Transform convexInTriangleSpace = triObjWrap.WorldTransformInternal.Inverse() * softTransform;
        AabbUtil.TransformAabb(in halfExtents, this.collisionMarginTriangle, in convexInTriangleSpace, out aabbMin, out aabbMax);
    }

    public override void ProcessTriangle(Vector3[] triangle, int partId, int triangleIndex)
    {
        // Debug drawing of the overlapping triangles
        if (dispatchInfoPtr != null && dispatchInfoPtr.debugDraw != null &&
            (dispatchInfoPtr.debugDraw.DebugMode & DebugDrawModes.DrawWireframe) != 0)
        {
            Vector3 color = new(1, 1, 0);
            Transform tr = triBody.WorldTransformInternal;
            Vector3 v0 = tr * triangle[0];
            Vector3 v1 = tr * triangle[1];
            Vector3 v2 = tr * triangle[2];
            dispatchInfoPtr.debugDraw.DrawLine(ref v0, ref v1, ref color);
            dispatchInfoPtr.debugDraw.DrawLine(ref v1, ref v2, ref color);
            dispatchInfoPtr.debugDraw.DrawLine(ref v2, ref v0, ref color);
        }

        var triIndex = new TriIndex(partId, triangleIndex, null);
        int triKey = triIndex.Uid;

        if (shapeCache.TryGetValue(triKey, out TriIndex shapeIndex))
        {
            CollisionShape cachedShape = shapeIndex.childShape;
            Debug.Assert(cachedShape != null);

            softBodyWrap.Set(null, softBody.CollisionShape, softBody, softBody.WorldTransformInternal, -1, -1);
            triBodyWrap.Set(null, cachedShape, triBody, triBody.WorldTransformInternal, partId, triangleIndex);

            CollisionAlgorithm colAlgo = dispatcher.FindAlgorithm(softBodyWrap, triBodyWrap,
                null, DispatcherQueryType.ContactPointAlgorithms);
            if (colAlgo != null)
            {
                colAlgo.ProcessCollision(softBodyWrap, triBodyWrap, dispatchInfoPtr, resultOut);
                dispatcher.FreeCollisionAlgorithm(colAlgo);
            }
            return;
        }

        Vector3 normal = (triangle[1] - triangle[0]).Cross(triangle[2] - triangle[0]).Normalized();
        normal *= SoftBodyTriangleExtrusion;
        Vector3[] pts =
        {
            triangle[0] + normal,
            triangle[1] + normal,
            triangle[2] + normal,
            triangle[0] - normal,
            triangle[1] - normal,
            triangle[2] - normal,
        };

        var tm = new ConvexHullShape(pts);

        softBodyWrap.Set(null, softBody.CollisionShape, softBody, softBody.WorldTransformInternal, -1, -1);
        triBodyWrap.Set(null, tm, triBody, triBody.WorldTransformInternal, partId, triangleIndex);

        CollisionAlgorithm colAlgoNew = dispatcher.FindAlgorithm(softBodyWrap, triBodyWrap,
            null, DispatcherQueryType.ContactPointAlgorithms);
        if (colAlgoNew != null)
        {
            colAlgoNew.ProcessCollision(softBodyWrap, triBodyWrap, dispatchInfoPtr, resultOut);
            dispatcher.FreeCollisionAlgorithm(colAlgoNew);
        }

        triIndex.childShape = tm;
        shapeCache[triKey] = triIndex;
    }

    public void ClearCache()
    {
        foreach (TriIndex tmp in shapeCache.Values)
        {
            if (tmp.childShape != null)
            {
                softBody?.worldInfo?.sparseSdf.RemoveReferences(tmp.childShape);
            }
        }
        shapeCache.Clear();
    }
}

/// <summary>
/// Supports collision between soft body shapes and concave shapes (triangle meshes, static planes, etc.).
/// Ported from BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.{h,cpp}.
/// </summary>
public class SoftBodyConcaveCollisionAlgorithm : CollisionAlgorithm, IDisposable
{
    internal bool isSwapped;
    internal readonly SoftBodyTriangleCallback softBodyTriangleCallback;

    private LocalTriangleSphereCastCallback localSphereCastCallback;

    public SoftBodyConcaveCollisionAlgorithm(ConstructionInfo ci, CollisionObjectWrapper body0Wrap,
        CollisionObjectWrapper body1Wrap, bool isSwapped)
        : base(ci)
    {
        this.isSwapped = isSwapped;
        softBodyTriangleCallback = new SoftBodyTriangleCallback(ci.Dispatcher, body0Wrap, body1Wrap, isSwapped);
    }

    public bool IsSwapped => isSwapped;

    public SoftBodyTriangleCallback TriangleCallback => softBodyTriangleCallback;

    /// <summary>btSoftBodyConcaveCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm (not swapped: soft vs concave)</summary>
    public static CollisionAlgorithm Create(ConstructionInfo ci,
        CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new SoftBodyConcaveCollisionAlgorithm(ci, body0Wrap, body1Wrap, false);

    /// <summary>btSoftBodyConcaveCollisionAlgorithm::SwappedCreateFunc::CreateCollisionAlgorithm (swapped: concave vs soft)</summary>
    public static CollisionAlgorithm SwappedCreate(ConstructionInfo ci,
        CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new SoftBodyConcaveCollisionAlgorithm(ci, body0Wrap, body1Wrap, true);

    public void ClearCache() => softBodyTriangleCallback.ClearCache();

    public override void ProcessCollision(CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        CollisionObjectWrapper triBody = isSwapped ? body0Wrap : body1Wrap;

        if (triBody.CollisionShape.IsConcave)
        {
            var concaveShape = (ConcaveShape)triBody.CollisionShape;
            btScalar collisionMarginTriangle = concaveShape.Margin;

            softBodyTriangleCallback.SetTimeStepAndCounters(collisionMarginTriangle, triBody, dispatchInfo, resultOut);
            concaveShape.ProcessAllTriangles(softBodyTriangleCallback,
                softBodyTriangleCallback.AabbMin, softBodyTriangleCallback.AabbMax);
        }
    }

    public override btScalar CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        CollisionObject convexBody = isSwapped ? body1 : body0;
        CollisionObject triBody = isSwapped ? body0 : body1;

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

    public override void GetAllContactManifolds(List<PersistentManifold> manifoldArray)
    {
        // we don't add any manifolds
    }

    internal override void Cleanup(Dispatcher dispatcher)
    {
        softBodyTriangleCallback.ClearCache();
    }

    public void Dispose()
    {
        Cleanup(dispatcher);
        GC.SuppressFinalize(this);
    }

    private sealed class LocalTriangleSphereCastCallback : TriangleCallback
    {
        internal Transform ccdSphereFromTrans;
        internal Transform ccdSphereToTrans;
        internal btScalar ccdSphereRadius;
        internal btScalar hitFraction;

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
