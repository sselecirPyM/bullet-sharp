using System.Runtime.CompilerServices;
using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btCollisionWorld. Named CollisionWorld in BulletSharp.
/// </summary>
public class CollisionWorld : IDisposable
{
    internal readonly List<CollisionObject> collisionObjects = new();
    internal Dispatcher dispatcher1;
    internal readonly DispatcherInfo dispatcherInfo = new();
    internal BroadphaseInterface broadphasePairCache;
    internal IDebugDraw debugDrawer;
    internal bool forceUpdateAllAabbs = true;

    public CollisionWorld(Dispatcher dispatcher, BroadphaseInterface broadphasePairCache, CollisionConfiguration collisionConfiguration)
    {
        dispatcher1 = dispatcher;
        this.broadphasePairCache = broadphasePairCache;
    }

    // ---------------- Object management ----------------

    public virtual void AddCollisionObject(CollisionObject collisionObject,
        CollisionFilterGroups collisionFilterGroup = CollisionFilterGroups.DefaultFilter,
        CollisionFilterGroups collisionFilterMask = CollisionFilterGroups.AllFilter)
    {
        collisionObject.WorldArrayIndex = collisionObjects.Count;
        collisionObjects.Add(collisionObject);

        Transform trans = collisionObject.worldTransform;
        collisionObject.CollisionShape.GetAabb(in trans, out Vector3 minAabb, out Vector3 maxAabb);

        BroadphaseNativeType type = collisionObject.CollisionShape.ShapeType;
        collisionObject.BroadphaseHandle = broadphasePairCache.CreateProxy(minAabb, maxAabb, type,
            collisionObject, collisionFilterGroup, collisionFilterMask, dispatcher1, null);
    }

    public virtual void RemoveCollisionObject(CollisionObject collisionObject)
    {
        BroadphaseProxy bp = collisionObject.BroadphaseHandle;
        if (bp != null)
        {
            broadphasePairCache.OverlappingPairCache.CleanProxyFromPairs(bp, dispatcher1);
            broadphasePairCache.DestroyProxy(bp, dispatcher1);
            collisionObject.BroadphaseHandle = null;
        }

        int iObj = collisionObject.WorldArrayIndex;
        if (iObj >= 0 && iObj < collisionObjects.Count)
        {
            collisionObjects[iObj] = collisionObjects[^1];
            collisionObjects[iObj].WorldArrayIndex = iObj;
            collisionObjects.RemoveAt(collisionObjects.Count - 1);
        }
        collisionObject.WorldArrayIndex = -1;
    }

    public int NumCollisionObjects => collisionObjects.Count;

    public List<CollisionObject> CollisionObjectArray => collisionObjects;

    // ---------------- Dispatching ----------------

    public virtual void PerformDiscreteCollisionDetection()
    {
        UpdateAabbs();

        ComputeOverlappingPairs();

        Dispatcher dispatcher = Dispatcher;
        dispatcher?.DispatchAllCollisionPairs(broadphasePairCache.OverlappingPairCache, dispatcherInfo, dispatcher1);
    }

    public void ComputeOverlappingPairs()
    {
        broadphasePairCache.CalculateOverlappingPairs(dispatcher1);
    }

    public Dispatcher Dispatcher
    {
        get => dispatcher1;
        set => dispatcher1 = value;
    }

    public DispatcherInfo DispatchInfo => dispatcherInfo;

    public BroadphaseInterface Broadphase
    {
        get => broadphasePairCache;
        set => broadphasePairCache = value;
    }

    public bool ForceUpdateAllAabbs
    {
        get => forceUpdateAllAabbs;
        set => forceUpdateAllAabbs = value;
    }

    public IDebugDraw DebugDrawer
    {
        get => debugDrawer;
        set => debugDrawer = value;
    }

    public virtual void DebugDrawWorld()
    {
        if (debugDrawer == null)
            return;

        if ((debugDrawer.DebugMode & DebugDrawModes.DrawAabb) != 0)
        {
            for (int i = 0; i < collisionObjects.Count; i++)
            {
                CollisionObject colObj = collisionObjects[i];
                if (colObj.CollisionShape == null)
                    continue;

                Vector3 color = new((btScalar)1, (btScalar)0, (btScalar)0);
                switch (colObj.ActivationState)
                {
                    case ActivationState.ActiveTag:
                        color = new Vector3((btScalar)1, (btScalar)1, (btScalar)1);
                        break;
                    case ActivationState.IslandSleeping:
                        color = new Vector3((btScalar)0, (btScalar)1, (btScalar)0);
                        break;
                    case ActivationState.WantsDeactivation:
                        color = new Vector3((btScalar)0, (btScalar)1, (btScalar)1);
                        break;
                    case ActivationState.DisableDeactivation:
                        color = new Vector3((btScalar)1, (btScalar)0, (btScalar)0);
                        break;
                    case ActivationState.DisableSimulation:
                        color = new Vector3((btScalar)1, (btScalar)1, (btScalar)0);
                        break;
                }

                colObj.CollisionShape.GetAabb(in colObj.worldTransform, out Vector3 minAabb, out Vector3 maxAabb);
                debugDrawer.DrawAabb(ref minAabb, ref maxAabb, ref color);
            }
        }
    }

    // ---------------- AABB updates ----------------

    public virtual void UpdateSingleAabb(CollisionObject colObj)
    {
        colObj.CollisionShape.GetAabb(in colObj.worldTransform, out Vector3 minAabb, out Vector3 maxAabb);
        Vector3 contactThreshold = new(PersistentManifold.GlobalContactBreakingThreshold);
        minAabb -= contactThreshold;
        maxAabb += contactThreshold;

        if (dispatcherInfo.useContinuous && colObj.internalType == CollisionObjectTypes.RigidBody &&
            !colObj.IsStaticOrKinematicObject)
        {
            colObj.CollisionShape.GetAabb(in colObj.interpolationWorldTransform, out Vector3 minAabb2, out Vector3 maxAabb2);
            minAabb2 -= contactThreshold;
            maxAabb2 += contactThreshold;
            minAabb.SetMin(minAabb2);
            maxAabb.SetMax(maxAabb2);
        }

        if (colObj.BroadphaseHandle != null)
        {
            broadphasePairCache.SetAabb(colObj.BroadphaseHandle, minAabb, maxAabb, dispatcher1);
        }
    }

    public virtual void UpdateAabbs()
    {
        for (int i = 0; i < collisionObjects.Count; i++)
        {
            CollisionObject colObj = collisionObjects[i];
            if (colObj.IsActive)
            {
                UpdateSingleAabb(colObj);
            }
        }
    }

    // ---------------- Ray casting ----------------

    public virtual void RayTest(Vector3 rayFromWorld, Vector3 rayToWorld, RayResultCallback resultCallback)
    {
        // Reuse the callback object (a stack object in C++; single-threaded per world, as in Bullet).
        singleRayCallback ??= new SingleRayCallback();
        singleRayCallback.Set(rayFromWorld, rayToWorld, this, resultCallback);
        broadphasePairCache.RayTest(rayFromWorld, rayToWorld, singleRayCallback);
    }

    private SingleRayCallback singleRayCallback;

    // Reusable ray-cast temporaries (stack objects in C++ RayTestSingleInternal).
    private CastResult rayCastResult;
    private SphereShape rayPointShape;
    private VoronoiSimplexSolver raySimplexSolver;
    private SubSimplexConvexCast raySubSimplexCast;
    private GjkConvexCast rayGjkConvexCast;
    private LocalRayResult rayLocalRayResult;

    // Reusable sweep-test temporaries (stack objects in C++ ObjectQuerySingleInternal).
    private CastResult sweepCastResult;
    private VoronoiSimplexSolver sweepSimplexSolver;
    private SubSimplexConvexCast sweepSubSimplexCast;
    private ContinuousConvexCollision sweepPlaneCaster;

    // Reusable callback/bridge objects (stack objects in C++; single-threaded per world).
    private SingleSweepCallback singleSweepCallback;
    private BridgeTriangleRaycastCallback bridgeTriangleRaycastCallback;
    private BridgeTriangleConvexcastCallback bridgeTriangleConvexcastCallback;
    private LocalInfoAdder localInfoAdder;
    private LocalInfoAdderConvex localInfoAdderConvex;

    // Reusable contact-test objects (stack objects in C++ ContactPairTest).
    private CollisionObjectWrapper contactTestWrapA;
    private CollisionObjectWrapper contactTestWrapB;
    private BridgedManifoldResult contactPairResult;

    private sealed class SingleRayCallback : BroadphaseRayCallback
    {
        private Vector3 rayFromWorld;
        private Vector3 rayToWorld;
        private CollisionWorld world;
        private RayResultCallback resultCallback;
        internal Transform rayFromTrans;
        internal Transform rayToTrans;

        public void Set(Vector3 rayFromWorld, Vector3 rayToWorld, CollisionWorld world, RayResultCallback resultCallback)
        {
            this.rayFromWorld = rayFromWorld;
            this.rayToWorld = rayToWorld;
            this.world = world;
            this.resultCallback = resultCallback;

            rayFromTrans = Transform.Identity;
            rayFromTrans.Origin = rayFromWorld;
            rayToTrans = Transform.Identity;
            rayToTrans.Origin = rayToWorld;

            Vector3 rayDir = rayToWorld - rayFromWorld;
            rayDirectionInverse = new Vector3(
                rayDir.X == 0 ? BtMath.BtLargeFloat : 1 / rayDir.X,
                rayDir.Y == 0 ? BtMath.BtLargeFloat : 1 / rayDir.Y,
                rayDir.Z == 0 ? BtMath.BtLargeFloat : 1 / rayDir.Z);
            signs[0] = rayDirectionInverse.X < 0 ? 1 : 0;
            signs[1] = rayDirectionInverse.Y < 0 ? 1 : 0;
            signs[2] = rayDirectionInverse.Z < 0 ? 1 : 0;

            lambdaMax = rayDir.Length();
        }

        public override bool Process(BroadphaseProxy proxy)
        {
            CollisionObject collisionObject = proxy.clientObject;
            if (resultCallback.NeedsCollision(collisionObject.BroadphaseHandle))
            {
                world.RayTestSingleInternal(rayFromTrans, rayToTrans, collisionObject, resultCallback);
            }
            return true;
        }
    }

    public virtual void RayTestSingle(Matrix rayFromWorld, Matrix rayToWorld, CollisionObject collisionObject,
        CollisionShape collisionShape, Matrix colObjWorldTransform, RayResultCallback resultCallback)
    {
        RayTestSingleInternal(Transform.FromMatrix(in rayFromWorld), Transform.FromMatrix(in rayToWorld),
            collisionObject, collisionShape, Transform.FromMatrix(in colObjWorldTransform), resultCallback);
    }

    internal void RayTestSingleInternal(in Transform rayFromTrans, in Transform rayToTrans,
        CollisionObject collisionObject, RayResultCallback resultCallback)
    {
        RayTestSingleInternal(in rayFromTrans, in rayToTrans, collisionObject, collisionObject.CollisionShape,
            collisionObject.worldTransform, resultCallback);
    }

    /// <summary>
    /// btCollisionWorld::rayTestSingle. AggressiveOptimization: called once per ray-candidate
    /// with only a zero/trivial-iteration compound loop inside, it never trips OSR and stays
    /// at instrumented Tier0 under default tiering (measured ~2x slowdown on raycast).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    internal virtual void RayTestSingleInternal(in Transform rayFromTrans, in Transform rayToTrans,
        CollisionObject collisionObject, CollisionShape collisionShape, in Transform colObjWorldTransform,
        RayResultCallback resultCallback)
    {
        if (collisionShape.IsConvex)
        {
            // Reuse convex-cast temporaries (stack objects in C++).
            rayCastResult ??= new CastResult();
            rayPointShape ??= new SphereShape(0) { Margin = 0 };
            raySimplexSolver ??= new VoronoiSimplexSolver();
            CastResult castResult = rayCastResult;
            castResult.Reset(resultCallback.closestHitFraction);
            ConvexShape convexShape = (ConvexShape)collisionShape;
            ConvexCast convexCaster;
            if ((resultCallback.flags & TriangleRaycastFlags.UseGjkConvexCastRaytest) != 0)
            {
                rayGjkConvexCast ??= new GjkConvexCast(rayPointShape, convexShape, raySimplexSolver);
                rayGjkConvexCast.SetShapes(rayPointShape, convexShape);
                convexCaster = rayGjkConvexCast;
            }
            else
            {
                raySubSimplexCast ??= new SubSimplexConvexCast(rayPointShape, convexShape, raySimplexSolver);
                raySubSimplexCast.SetShapes(rayPointShape, convexShape);
                convexCaster = raySubSimplexCast;
            }

            if (convexCaster.CalcTimeOfImpact(in rayFromTrans, in rayToTrans, in colObjWorldTransform, in colObjWorldTransform, castResult))
            {
                if (castResult.normal.LengthSquared() > (btScalar)0.0001)
                {
                    if (castResult.fraction < resultCallback.closestHitFraction)
                    {
                        castResult.normal.Normalize();
                        // Reuse the result value (struct, zero allocation; the callback must consume it immediately).
                        rayLocalRayResult.Set(collisionObject, null, castResult.normal, castResult.fraction);
                        resultCallback.AddSingleResult(rayLocalRayResult, true);
                    }
                }
            }
        }
        else if (collisionShape.IsConcave)
        {
            var concaveShape = (ConcaveShape)collisionShape;
            Transform worldTocollisionObject = colObjWorldTransform.Inverse();
            Vector3 rayFromLocal = worldTocollisionObject * rayFromTrans.Origin;
            Vector3 rayToLocal = worldTocollisionObject * rayToTrans.Origin;

            bridgeTriangleRaycastCallback ??= new BridgeTriangleRaycastCallback(rayFromLocal, rayToLocal, resultCallback, collisionObject, in colObjWorldTransform);
            bridgeTriangleRaycastCallback.Set(rayFromLocal, rayToLocal, resultCallback, collisionObject, in colObjWorldTransform, resultCallback.closestHitFraction);

            Vector3 rayAabbMinLocal = rayFromLocal;
            rayAabbMinLocal.SetMin(rayToLocal);
            Vector3 rayAabbMaxLocal = rayFromLocal;
            rayAabbMaxLocal.SetMax(rayToLocal);

            concaveShape.ProcessAllTriangles(bridgeTriangleRaycastCallback, rayAabbMinLocal, rayAabbMaxLocal);
        }
        else if (collisionShape.IsCompound)
        {
            var compoundShape = (CompoundShape)collisionShape;
            localInfoAdder ??= new LocalInfoAdder(resultCallback);
            for (int i = 0, n = compoundShape.NumChildren; i < n; i++)
            {
                Transform childTrans = compoundShape.GetChildTransformInternal(i);
                CollisionShape childCollisionShape = compoundShape.GetChildShape(i);
                Transform childWorldTrans = colObjWorldTransform * childTrans;

                localInfoAdder.Set(i, resultCallback);
                RayTestSingleInternal(in rayFromTrans, in rayToTrans, collisionObject, childCollisionShape, childWorldTrans, localInfoAdder);
            }
        }
    }

    private sealed class LocalInfoAdder : RayResultCallback
    {
        private RayResultCallback userCallback;
        private int i;

        public LocalInfoAdder(RayResultCallback user)
        {
            Set(0, user);
        }

        public void Set(int i, RayResultCallback user)
        {
            this.i = i;
            userCallback = user;
            closestHitFraction = user.closestHitFraction;
            collisionFilterGroup = user.collisionFilterGroup;
            collisionFilterMask = user.collisionFilterMask;
            flags = user.flags;
        }

        public override bool NeedsCollision(BroadphaseProxy p) => userCallback.NeedsCollision(p);

        public override btScalar AddSingleResult(LocalRayResult r, bool normalInWorldSpace)
        {
            // Equivalent to C++: if (r.m_localShapeInfo == NULL) r.m_localShapeInfo = &shapeInfo;
            if (!r.HasShapeInfo)
                r.LocalShapeInfo = new LocalShapeInfo { shapePart = -1, triangleIndex = i };
            btScalar result = userCallback.AddSingleResult(r, normalInWorldSpace);
            closestHitFraction = userCallback.closestHitFraction;
            return result;
        }
    }

    private sealed class BridgeTriangleRaycastCallback : TriangleRaycastCallback
    {
        private RayResultCallback resultCallback;
        private CollisionObject collisionObject;
        private Transform colObjWorldTransform;

        public BridgeTriangleRaycastCallback(Vector3 from, Vector3 to, RayResultCallback resultCallback,
            CollisionObject collisionObject, in Transform colObjWorldTransform)
            : base(from, to, resultCallback.flags)
        {
            this.resultCallback = resultCallback;
            this.collisionObject = collisionObject;
            this.colObjWorldTransform = colObjWorldTransform;
        }

        /// <summary>In-place reuse (equivalent to reconstructing on the stack per ray in C++).</summary>
        public void Set(Vector3 from, Vector3 to, RayResultCallback resultCallback,
            CollisionObject collisionObject, in Transform colObjWorldTransform, btScalar hitFraction)
        {
            this.from = from;
            this.to = to;
            flags = resultCallback.flags;
            this.hitFraction = hitFraction;
            this.resultCallback = resultCallback;
            this.collisionObject = collisionObject;
            this.colObjWorldTransform = colObjWorldTransform;
        }

        public override btScalar ReportHit(Vector3 hitNormalLocal, btScalar hitFraction, int partId, int triangleIndex)
        {
            // Struct value type, zero allocation (equivalent to a C++ stack object).
            var shapeInfo = new LocalShapeInfo { shapePart = partId, triangleIndex = triangleIndex };
            Vector3 hitNormalWorld = colObjWorldTransform.Basis * hitNormalLocal;
            var rayResult = new LocalRayResult(collisionObject, shapeInfo, hitNormalWorld, hitFraction);
            return resultCallback.AddSingleResult(rayResult, true);
        }
    }

    // ---------------- Convex sweep ----------------

    public void ConvexSweepTest(ConvexShape castShape, Matrix convexFromWorld, Matrix convexToWorld,
        ConvexResultCallback resultCallback, btScalar allowedCcdPenetration = 0)
    {
        Transform convexFromTrans = Transform.FromMatrix(in convexFromWorld);
        Transform convexToTrans = Transform.FromMatrix(in convexToWorld);

        castShape.GetAabb(Transform.Identity, out Vector3 castShapeAabbMin, out Vector3 castShapeAabbMax);

        singleSweepCallback ??= new SingleSweepCallback(castShape, convexFromWorld, convexToWorld, this, resultCallback, allowedCcdPenetration);
        singleSweepCallback.Set(castShape, convexFromWorld, convexToWorld, this, resultCallback, allowedCcdPenetration);
        broadphasePairCache.RayTest(convexFromTrans.Origin, convexToTrans.Origin, singleSweepCallback, castShapeAabbMin, castShapeAabbMax);
    }

    private sealed class SingleSweepCallback : BroadphaseRayCallback
    {
        private ConvexShape castShape;
        private Transform convexFromTrans;
        private Transform convexToTrans;
        private CollisionWorld world;
        private ConvexResultCallback resultCallback;
        private btScalar allowedCcdPenetration;

        public SingleSweepCallback(ConvexShape castShape, Matrix convexFromWorld, Matrix convexToWorld,
            CollisionWorld world, ConvexResultCallback resultCallback, btScalar allowedCcdPenetration)
        {
            Set(castShape, convexFromWorld, convexToWorld, world, resultCallback, allowedCcdPenetration);
        }

        /// <summary>In-place reuse (equivalent to reconstructing on the stack per sweep in C++).</summary>
        public void Set(ConvexShape castShape, Matrix convexFromWorld, Matrix convexToWorld,
            CollisionWorld world, ConvexResultCallback resultCallback, btScalar allowedCcdPenetration)
        {
            this.castShape = castShape;
            convexFromTrans = Transform.FromMatrix(in convexFromWorld);
            convexToTrans = Transform.FromMatrix(in convexToWorld);
            this.world = world;
            this.resultCallback = resultCallback;
            this.allowedCcdPenetration = allowedCcdPenetration;

            Vector3 unNormalizedRayDir = convexToTrans.Origin - convexFromTrans.Origin;
            rayDirectionInverse = new Vector3(
                unNormalizedRayDir.X == 0 ? BtMath.BtLargeFloat : 1 / unNormalizedRayDir.X,
                unNormalizedRayDir.Y == 0 ? BtMath.BtLargeFloat : 1 / unNormalizedRayDir.Y,
                unNormalizedRayDir.Z == 0 ? BtMath.BtLargeFloat : 1 / unNormalizedRayDir.Z);
            signs[0] = rayDirectionInverse.X < 0 ? 1 : 0;
            signs[1] = rayDirectionInverse.Y < 0 ? 1 : 0;
            signs[2] = rayDirectionInverse.Z < 0 ? 1 : 0;
            lambdaMax = unNormalizedRayDir.Length();
        }

        public override bool Process(BroadphaseProxy proxy)
        {
            CollisionObject collisionObject = proxy.clientObject;
            if (resultCallback.NeedsCollision(collisionObject.BroadphaseHandle))
            {
                world.ObjectQuerySingleInternal(castShape, in convexFromTrans, in convexToTrans,
                    collisionObject, resultCallback, allowedCcdPenetration);
            }
            return true;
        }
    }

    internal void ObjectQuerySingleInternal(ConvexShape castShape, in Transform convexFromTrans, in Transform convexToTrans,
        CollisionObject collisionObject, ConvexResultCallback resultCallback, btScalar allowedPenetration)
    {
        ObjectQuerySingleInternal(castShape, in convexFromTrans, in convexToTrans, collisionObject,
            collisionObject.CollisionShape, collisionObject.worldTransform, resultCallback, allowedPenetration);
    }

    internal void ObjectQuerySingleInternal(ConvexShape castShape, in Transform convexFromTrans, in Transform convexToTrans,
        CollisionObject collisionObject, CollisionShape collisionShape, in Transform colObjWorldTransform,
        ConvexResultCallback resultCallback, btScalar allowedPenetration)
    {
        if (collisionShape.IsConvex)
        {
            // Reuse sweep temporaries (stack objects in C++).
            sweepCastResult ??= new CastResult();
            sweepSimplexSolver ??= new VoronoiSimplexSolver();
            sweepSubSimplexCast ??= new SubSimplexConvexCast(castShape, (ConvexShape)collisionShape, sweepSimplexSolver);
            CastResult castResult = sweepCastResult;
            castResult.Reset(resultCallback.closestHitFraction);
            castResult.AllowedPenetration = allowedPenetration;
            var convexShape = (ConvexShape)collisionShape;
            sweepSubSimplexCast.SetShapes(castShape, convexShape);

            if (sweepSubSimplexCast.CalcTimeOfImpact(in convexFromTrans, in convexToTrans, in colObjWorldTransform, in colObjWorldTransform, castResult))
            {
                if (castResult.normal.LengthSquared() > (btScalar)0.0001)
                {
                    if (castResult.fraction < resultCallback.closestHitFraction)
                    {
                        castResult.normal.Normalize();
                        // Struct value type, zero allocation.
                        var localConvexResult = new LocalConvexResult(collisionObject, null, castResult.normal, castResult.hitPoint, castResult.fraction);
                        resultCallback.AddSingleResult(localConvexResult, true);
                    }
                }
            }
        }
        else if (collisionShape.IsConcave)
        {
            if (collisionShape.ShapeType == BroadphaseNativeType.StaticPlaneShape)
            {
                sweepCastResult ??= new CastResult();
                sweepPlaneCaster ??= new ContinuousConvexCollision(castShape, (StaticPlaneShape)collisionShape);
                CastResult castResult = sweepCastResult;
                castResult.Reset(resultCallback.closestHitFraction);
                castResult.AllowedPenetration = allowedPenetration;
                var planeShape = (StaticPlaneShape)collisionShape;
                sweepPlaneCaster.SetShapes(castShape, planeShape);

                if (sweepPlaneCaster.CalcTimeOfImpact(in convexFromTrans, in convexToTrans, in colObjWorldTransform, in colObjWorldTransform, castResult))
                {
                    if (castResult.normal.LengthSquared() > (btScalar)0.0001)
                    {
                        if (castResult.fraction < resultCallback.closestHitFraction)
                        {
                            castResult.normal.Normalize();
                            var localConvexResult = new LocalConvexResult(collisionObject, null, castResult.normal, castResult.hitPoint, castResult.fraction);
                            resultCallback.AddSingleResult(localConvexResult, true);
                        }
                    }
                }
            }
            else
            {
                var concaveShape = (ConcaveShape)collisionShape;
                Transform worldTocollisionObject = colObjWorldTransform.Inverse();
                Vector3 convexFromLocal = worldTocollisionObject * convexFromTrans.Origin;
                Vector3 convexToLocal = worldTocollisionObject * convexToTrans.Origin;
                Transform rotationXform = new(worldTocollisionObject.Basis * convexToTrans.Basis, Vector3.Zero);

                bridgeTriangleConvexcastCallback ??= new BridgeTriangleConvexcastCallback(castShape, in convexFromTrans, in convexToTrans,
                    resultCallback, collisionObject, concaveShape, in colObjWorldTransform);
                bridgeTriangleConvexcastCallback.Set(castShape, in convexFromTrans, in convexToTrans,
                    resultCallback, collisionObject, concaveShape, in colObjWorldTransform,
                    resultCallback.closestHitFraction, allowedPenetration);
                castShape.GetAabb(in rotationXform, out Vector3 boxMinLocal, out Vector3 boxMaxLocal);

                Vector3 rayAabbMinLocal = convexFromLocal;
                rayAabbMinLocal.SetMin(convexToLocal);
                Vector3 rayAabbMaxLocal = convexFromLocal;
                rayAabbMaxLocal.SetMax(convexToLocal);
                rayAabbMinLocal += boxMinLocal;
                rayAabbMaxLocal += boxMaxLocal;
                concaveShape.ProcessAllTriangles(bridgeTriangleConvexcastCallback, rayAabbMinLocal, rayAabbMaxLocal);
            }
        }
        else if (collisionShape.IsCompound)
        {
            var compoundShape = (CompoundShape)collisionShape;
            localInfoAdderConvex ??= new LocalInfoAdderConvex(resultCallback);
            for (int i = 0, n = compoundShape.NumChildren; i < n; i++)
            {
                Transform childTrans = compoundShape.GetChildTransformInternal(i);
                CollisionShape childCollisionShape = compoundShape.GetChildShape(i);
                Transform childWorldTrans = colObjWorldTransform * childTrans;

                localInfoAdderConvex.Set(i, resultCallback);
                ObjectQuerySingleInternal(castShape, in convexFromTrans, in convexToTrans,
                    collisionObject, childCollisionShape, childWorldTrans, localInfoAdderConvex, allowedPenetration);
            }
        }
    }

    private sealed class LocalInfoAdderConvex : ConvexResultCallback
    {
        private ConvexResultCallback userCallback;
        private int i;

        public LocalInfoAdderConvex(ConvexResultCallback user)
        {
            Set(0, user);
        }

        public void Set(int i, ConvexResultCallback user)
        {
            this.i = i;
            userCallback = user;
            closestHitFraction = user.closestHitFraction;
            collisionFilterGroup = user.collisionFilterGroup;
            collisionFilterMask = user.collisionFilterMask;
        }

        public override bool NeedsCollision(BroadphaseProxy p) => userCallback.NeedsCollision(p);

        public override btScalar AddSingleResult(LocalConvexResult r, bool normalInWorldSpace)
        {
            // Equivalent to C++: if (r.m_localShapeInfo == NULL) r.m_localShapeInfo = &shapeInfo;
            if (!r.HasShapeInfo)
                r.LocalShapeInfo = new LocalShapeInfo { shapePart = -1, triangleIndex = i };
            btScalar result = userCallback.AddSingleResult(r, normalInWorldSpace);
            closestHitFraction = userCallback.closestHitFraction;
            return result;
        }
    }

    private sealed class BridgeTriangleConvexcastCallback : TriangleConvexcastCallback
    {
        private ConvexResultCallback resultCallback;
        private CollisionObject collisionObject;
        private Transform colObjWorldTransform;

        public BridgeTriangleConvexcastCallback(ConvexShape castShape, in Transform from, in Transform to,
            ConvexResultCallback resultCallback, CollisionObject collisionObject, ConcaveShape triangleMesh,
            in Transform triangleToWorld)
            : base(castShape, in from, in to, in triangleToWorld, triangleMesh.Margin)
        {
            this.resultCallback = resultCallback;
            this.collisionObject = collisionObject;
            colObjWorldTransform = triangleToWorld;
        }

        /// <summary>In-place reuse (equivalent to reconstructing on the stack per sweep in C++).</summary>
        public void Set(ConvexShape castShape, in Transform from, in Transform to,
            ConvexResultCallback resultCallback, CollisionObject collisionObject, ConcaveShape triangleMesh,
            in Transform triangleToWorld, btScalar hitFraction, btScalar allowedPenetration)
        {
            convexShape = castShape;
            convexShapeFrom = from;
            convexShapeTo = to;
            this.triangleToWorld = triangleToWorld;
            triangleCollisionMargin = triangleMesh.Margin;
            this.hitFraction = hitFraction;
            this.allowedPenetration = allowedPenetration;
            this.resultCallback = resultCallback;
            this.collisionObject = collisionObject;
            colObjWorldTransform = triangleToWorld;
        }

        public override btScalar ReportHit(Vector3 hitNormalLocal, Vector3 hitPointLocal, btScalar hitFraction, int partId, int triangleIndex)
        {
            if (hitFraction <= resultCallback.closestHitFraction)
            {
                // Struct value type, zero allocation.
                var shapeInfo = new LocalShapeInfo { shapePart = partId, triangleIndex = triangleIndex };
                var convexResult = new LocalConvexResult(collisionObject, shapeInfo, hitNormalLocal, hitPointLocal, hitFraction);
                return resultCallback.AddSingleResult(convexResult, true);
            }
            return hitFraction;
        }
    }

    // ---------------- Contact testing ----------------

    public void ContactTest(CollisionObject colObj, ContactResultCallback resultCallback)
    {
        for (int i = 0; i < collisionObjects.Count; i++)
        {
            CollisionObject other = collisionObjects[i];
            if (other == colObj) continue;
            ContactPairTest(colObj, other, resultCallback);
        }
    }

    public void ContactPairTest(CollisionObject colObjA, CollisionObject colObjB, ContactResultCallback resultCallback)
    {
        // Reuse wrapper and result objects (stack objects in C++).
        contactTestWrapA ??= new CollisionObjectWrapper(null, null, null, Transform.Identity, -1, -1);
        contactTestWrapB ??= new CollisionObjectWrapper(null, null, null, Transform.Identity, -1, -1);
        contactTestWrapA.Set(null, colObjA.CollisionShape, colObjA, colObjA.worldTransform, -1, -1);
        contactTestWrapB.Set(null, colObjB.CollisionShape, colObjB, colObjB.worldTransform, -1, -1);

        CollisionAlgorithm algorithm = dispatcher1.FindAlgorithm(contactTestWrapA, contactTestWrapB, null, DispatcherQueryType.ContactPointAlgorithms);
        if (algorithm != null)
        {
            contactPairResult ??= new BridgedManifoldResult(contactTestWrapA, contactTestWrapB, resultCallback);
            contactPairResult.Set(contactTestWrapA, contactTestWrapB, resultCallback);
            algorithm.ProcessCollision(contactTestWrapA, contactTestWrapB, dispatcherInfo, contactPairResult);
            dispatcher1.FreeCollisionAlgorithm(algorithm);
        }
    }

    private sealed class BridgedManifoldResult : ManifoldResult
    {
        private ContactResultCallback resultCallback;
        // Scratch contact point (same reuse pattern as ManifoldResult.scratchPoint in the base class).
        private readonly ManifoldPoint scratchPoint = new();

        public BridgedManifoldResult(CollisionObjectWrapper obj0Wrap, CollisionObjectWrapper obj1Wrap, ContactResultCallback resultCallback)
            : base(obj0Wrap, obj1Wrap)
        {
            this.resultCallback = resultCallback;
        }

        /// <summary>In-place reuse (equivalent to reconstructing on the stack per test in C++).</summary>
        public void Set(CollisionObjectWrapper obj0Wrap, CollisionObjectWrapper obj1Wrap, ContactResultCallback resultCallback)
        {
            Reset(obj0Wrap, obj1Wrap);
            this.resultCallback = resultCallback;
        }

        public override void AddContactPoint(Vector3 normalOnBInWorld, Vector3 pointInWorld, btScalar depth)
        {
            bool isSwapped = manifoldPtr.body0 != body0Wrap.collisionObject;

            Vector3 pointA = pointInWorld + normalOnBInWorld * depth;

            Vector3 localA;
            Vector3 localB;
            if (isSwapped)
            {
                localA = body1Wrap.collisionObject.worldTransform.InverseXForm(pointA);
                localB = body0Wrap.collisionObject.worldTransform.InverseXForm(pointInWorld);
            }
            else
            {
                localA = body0Wrap.collisionObject.worldTransform.InverseXForm(pointA);
                localB = body1Wrap.collisionObject.worldTransform.InverseXForm(pointInWorld);
            }

            // Reuse the scratch contact point (equivalent to C++ stack construction;
            // the callback contract requires immediate consumption).
            var newPt = scratchPoint;
            newPt.Set(localA, localB, normalOnBInWorld, depth);
            newPt.positionWorldOnA = pointA;
            newPt.positionWorldOnB = pointInWorld;

            newPt.combinedFriction = CalculateCombinedFriction(body0Wrap.collisionObject, body1Wrap.collisionObject);
            newPt.combinedRestitution = CalculateCombinedRestitution(body0Wrap.collisionObject, body1Wrap.collisionObject);
            newPt.combinedRollingFriction = CalculateCombinedRollingFriction(body0Wrap.collisionObject, body1Wrap.collisionObject);
            newPt.combinedSpinningFriction = CalculateCombinedSpinningFriction(body0Wrap.collisionObject, body1Wrap.collisionObject);

            if (isSwapped)
            {
                newPt.partId0 = partId1;
                newPt.partId1 = partId0;
                newPt.index0 = index1;
                newPt.index1 = index0;
            }
            else
            {
                newPt.partId0 = partId0;
                newPt.partId1 = partId1;
                newPt.index0 = index0;
                newPt.index1 = index1;
            }

            resultCallback.AddSingleResult(newPt,
                body0Wrap, newPt.partId0, newPt.index0,
                body1Wrap, newPt.partId1, newPt.index1);
        }
    }

    public virtual void Dispose()
    {
    }
}
