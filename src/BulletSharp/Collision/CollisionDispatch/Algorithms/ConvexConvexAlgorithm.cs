using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btConvexConvexAlgorithm: convex-convex collision algorithm,
/// implementing TOI, closest-point and penetration-depth computation for convex
/// shapes. Multiple contact points are computed by perturbing the orientation of the
/// smaller object orthogonally to the separating normal (an idea by Gino van den
/// Bergen, posted on the Bullet forum).
/// Note: the SAT polyhedral clipping path (btPolyhedralContactClipping, used when
/// getConvexPolyhedron()!=null) is not ported; the current behavior is equivalent
/// to the C++ fallback when getConvexPolyhedron()==null (pure GJK/EPA + perturbation).
/// </summary>
public class ConvexConvexAlgorithm : ActivatingCollisionAlgorithm
{
    /// <summary>Shared stateless penetration depth solver (btGjkEpaPenetrationDepthSolver)</summary>
    private static readonly GjkEpaPenetrationDepthSolver s_sharedPenetrationDepthSolver = new();

    /// <summary>Global disableCcd from btConvexConvexAlgorithm.cpp</summary>
    public static bool DisableCcd = false;

    internal GjkEpaPenetrationDepthSolver pdSolver;

    internal bool ownManifold;
    internal PersistentManifold manifoldPtr;
    internal bool lowLevelOfDetail;

    internal int numPerturbationIterations;
    internal int minimumPointsPerturbationThreshold;

    // Reused GJK temporaries (stack objects in C++ processCollision)
    private DiscreteCollisionDetectorInterface.ClosestPointInput gjkInput;
    private VoronoiSimplexSolver gjkSimplexSolver;
    private GjkPairDetector gjkPairDetector;

    public ConvexConvexAlgorithm(ConstructionInfo ci, CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap,
        int numPerturbationIterations = 0, int minimumPointsPerturbationThreshold = 2)
        : this(ci.Manifold, ci, body0Wrap, body1Wrap, s_sharedPenetrationDepthSolver,
            numPerturbationIterations, minimumPointsPerturbationThreshold)
    {
    }

    public ConvexConvexAlgorithm(PersistentManifold mf, ConstructionInfo ci,
        CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap,
        GjkEpaPenetrationDepthSolver pdSolver, int numPerturbationIterations = 0, int minimumPointsPerturbationThreshold = 2)
        : base(ci, body0Wrap, body1Wrap)
    {
        this.pdSolver = pdSolver;
        ownManifold = false;
        manifoldPtr = mf;
        lowLevelOfDetail = false;
        this.numPerturbationIterations = numPerturbationIterations;
        this.minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;
    }

    /// <summary>btConvexConvexAlgorithm::CreateFunc::CreateCollisionAlgorithm
    /// (mirrors the CreateFunc defaults: numPerturbationIterations=0,
    /// minimumPointsPerturbationThreshold=3; a new VoronoiSimplexSolver per call; the
    /// penetration depth solver is the shared stateless GjkEpaPenetrationDepthSolver)</summary>
    public static CollisionAlgorithm Create(ConstructionInfo ci, CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new ConvexConvexAlgorithm(ci.Manifold, ci, body0Wrap, body1Wrap, s_sharedPenetrationDepthSolver, 0, 3);

    public void SetLowLevelOfDetail(bool useLowLevel) => lowLevelOfDetail = useLowLevel;

    public PersistentManifold Manifold => manifoldPtr;

    public int NumPerturbationIterations
    {
        get => numPerturbationIterations;
        set => numPerturbationIterations = value;
    }

    public int MinimumPointsPerturbationThreshold
    {
        get => minimumPointsPerturbationThreshold;
        set => minimumPointsPerturbationThreshold = value;
    }

    public override void GetAllContactManifolds(List<PersistentManifold> manifoldArray)
    {
        // should we use ownManifold to avoid duplicates?
        if (manifoldPtr != null && ownManifold)
            manifoldArray.Add(manifoldPtr);
    }

    /// <summary>btPerturbedContactResult: result callback for perturbed queries</summary>
    internal class PerturbedContactResult : ManifoldResult
    {
        internal readonly ManifoldResult originalManifoldResult;
        internal readonly Transform transformA;
        internal readonly Transform transformB;
        internal readonly Transform unPerturbedTransform;
        internal readonly bool perturbA;

        public PerturbedContactResult(ManifoldResult originalResult, in Transform transformA, in Transform transformB,
            in Transform unPerturbedTransform, bool perturbA)
        {
            originalManifoldResult = originalResult;
            this.transformA = transformA;
            this.transformB = transformB;
            this.unPerturbedTransform = unPerturbedTransform;
            this.perturbA = perturbA;
        }

        public override void AddContactPoint(Vector3 normalOnBInWorld, Vector3 pointInWorld, btScalar orgDepth)
        {
            Vector3 startPt;
            btScalar newDepth;

            if (perturbA)
            {
                Vector3 endPtOrg = pointInWorld + normalOnBInWorld * orgDepth;
                Vector3 endPt = unPerturbedTransform * transformA.Inverse() * endPtOrg;
                newDepth = (endPt - pointInWorld).Dot(normalOnBInWorld);
                startPt = endPt - normalOnBInWorld * newDepth;
            }
            else
            {
                Vector3 endPt = pointInWorld + normalOnBInWorld * orgDepth;
                startPt = unPerturbedTransform * transformB.Inverse() * pointInWorld;
                newDepth = (endPt - startPt).Dot(normalOnBInWorld);
            }

            originalManifoldResult.AddContactPoint(normalOnBInWorld, startPt, newDepth);
        }
    }

    /// <summary>segmentsClosestPoints: computes the parameters and offsets of the closest points of two line segments</summary>
    private static void SegmentsClosestPoints(
        out Vector3 ptsVector,
        out Vector3 offsetA,
        out Vector3 offsetB,
        Vector3 translation,
        in Vector3 dirA, btScalar hlenA,
        in Vector3 dirB, btScalar hlenB)
    {
        // compute the parameter of the closest point on each segment
        btScalar dirADotDirB = dirA.Dot(dirB);
        btScalar dirADotTrans = dirA.Dot(translation);
        btScalar dirBDotTrans = dirB.Dot(translation);

        btScalar denom = 1 - dirADotDirB * dirADotDirB;

        btScalar tA;
        if (denom == 0)
        {
            tA = 0;
        }
        else
        {
            tA = (dirADotTrans - dirBDotTrans * dirADotDirB) / denom;
            if (tA < -hlenA)
                tA = -hlenA;
            else if (tA > hlenA)
                tA = hlenA;
        }

        btScalar tB = tA * dirADotDirB - dirBDotTrans;

        if (tB < -hlenB)
        {
            tB = -hlenB;
            tA = tB * dirADotDirB + dirADotTrans;

            if (tA < -hlenA)
                tA = -hlenA;
            else if (tA > hlenA)
                tA = hlenA;
        }
        else if (tB > hlenB)
        {
            tB = hlenB;
            tA = tB * dirADotDirB + dirADotTrans;

            if (tA < -hlenA)
                tA = -hlenA;
            else if (tA > hlenA)
                tA = hlenA;
        }

        // compute the closest points relative to the segment centers
        offsetA = dirA * tA;
        offsetB = dirB * tB;

        ptsVector = translation - offsetA + offsetB;
    }

    /// <summary>capsuleCapsuleDistance: special case for degenerate capsules (capsule-capsule / capsule-sphere)</summary>
    private static btScalar CapsuleCapsuleDistance(
        out Vector3 normalOnB,
        out Vector3 pointOnB,
        btScalar capsuleLengthA,
        btScalar capsuleRadiusA,
        btScalar capsuleLengthB,
        btScalar capsuleRadiusB,
        int capsuleAxisA,
        int capsuleAxisB,
        in Transform transformA,
        in Transform transformB,
        btScalar distanceThreshold)
    {
        Vector3 directionA = transformA.Basis.Column(capsuleAxisA);
        Vector3 translationA = transformA.Origin;
        Vector3 directionB = transformB.Basis.Column(capsuleAxisB);
        Vector3 translationB = transformB.Origin;

        // translation between centers
        Vector3 translation = translationB - translationA;

        // compute the closest points of the capsule line segments
        SegmentsClosestPoints(out Vector3 ptsVector, out _, out Vector3 offsetB, translation,
            in directionA, capsuleLengthA, in directionB, capsuleLengthB);

        btScalar distance = ptsVector.Length() - capsuleRadiusA - capsuleRadiusB;

        if (distance > distanceThreshold)
        {
            normalOnB = Vector3.Zero;
            pointOnB = Vector3.Zero;
            return distance;
        }

        btScalar lenSqr = ptsVector.LengthSquared();
        if (lenSqr <= BtMath.SimdEpsilon * BtMath.SimdEpsilon)
        {
            // degenerate case: the capsules may be at the same location; take a vector tangent to directionA
            BtMath.PlaneSpace1(in directionA, out normalOnB, out _);
        }
        else
        {
            // compute the contact normal
            normalOnB = ptsVector * -BtMath.RecipSqrt(lenSqr);
        }
        pointOnB = transformB.Origin + offsetB + normalOnB * capsuleRadiusB;

        return distance;
    }

    /// <summary>btConvexConvexAlgorithm::processCollision</summary>
    public override void ProcessCollision(CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        if (manifoldPtr == null)
        {
            // swapped?
            manifoldPtr = dispatcher.GetNewManifold(body0Wrap.CollisionObject, body1Wrap.CollisionObject);
            ownManifold = true;
        }
        resultOut.SetPersistentManifold(manifoldPtr);

        var min0 = (ConvexShape)body0Wrap.CollisionShape;
        var min1 = (ConvexShape)body1Wrap.CollisionShape;

        Vector3 normalOnB;
        Vector3 pointOnBWorld;

        // special case for degenerate capsules (BT_DISABLE_CAPSULE_CAPSULE_COLLIDER is not defined)
        if (min0.ShapeType == BroadphaseNativeType.CapsuleShape && min1.ShapeType == BroadphaseNativeType.CapsuleShape)
        {
            var capsuleA = (CapsuleShape)min0;
            var capsuleB = (CapsuleShape)min1;

            // (C++ also adds resultOut->m_closestPointDistanceThreshold here; the current base layer has no such field, treated as 0)
            btScalar threshold = manifoldPtr.ContactBreakingThreshold;

            btScalar dist = CapsuleCapsuleDistance(out normalOnB, out pointOnBWorld,
                capsuleA.HalfHeight, capsuleA.Radius,
                capsuleB.HalfHeight, capsuleB.Radius, capsuleA.UpAxis, capsuleB.UpAxis,
                body0Wrap.WorldTransformInternal, body1Wrap.WorldTransformInternal, threshold);

            if (dist < threshold)
            {
                resultOut.AddContactPoint(normalOnB, pointOnBWorld, dist);
            }
            resultOut.RefreshContactPoints();
            return;
        }

        if (min0.ShapeType == BroadphaseNativeType.CapsuleShape && min1.ShapeType == BroadphaseNativeType.SphereShape)
        {
            var capsuleA = (CapsuleShape)min0;
            var capsuleB = (SphereShape)min1;

            btScalar threshold = manifoldPtr.ContactBreakingThreshold;

            btScalar dist = CapsuleCapsuleDistance(out normalOnB, out pointOnBWorld,
                capsuleA.HalfHeight, capsuleA.Radius,
                0, capsuleB.Radius, capsuleA.UpAxis, 1,
                body0Wrap.WorldTransformInternal, body1Wrap.WorldTransformInternal, threshold);

            if (dist < threshold)
            {
                resultOut.AddContactPoint(normalOnB, pointOnBWorld, dist);
            }
            resultOut.RefreshContactPoints();
            return;
        }

        if (min0.ShapeType == BroadphaseNativeType.SphereShape && min1.ShapeType == BroadphaseNativeType.CapsuleShape)
        {
            var capsuleA = (SphereShape)min0;
            var capsuleB = (CapsuleShape)min1;

            btScalar threshold = manifoldPtr.ContactBreakingThreshold;

            btScalar dist = CapsuleCapsuleDistance(out normalOnB, out pointOnBWorld,
                0, capsuleA.Radius,
                capsuleB.HalfHeight, capsuleB.Radius, 1, capsuleB.UpAxis,
                body0Wrap.WorldTransformInternal, body1Wrap.WorldTransformInternal, threshold);

            if (dist < threshold)
            {
                resultOut.AddContactPoint(normalOnB, pointOnBWorld, dist);
            }
            resultOut.RefreshContactPoints();
            return;
        }

        {
            // reuse the GJK temporaries (stack objects in C++; algorithm instances are cached per collision pair)
            gjkInput ??= new DiscreteCollisionDetectorInterface.ClosestPointInput();
            gjkSimplexSolver ??= new VoronoiSimplexSolver();
            gjkPairDetector ??= new GjkPairDetector(min0, min1, gjkSimplexSolver, pdSolver);
            var input = gjkInput;
            GjkPairDetector detector = gjkPairDetector;
            // TODO: if (dispatchInfo.UseContinuous)
            detector.MinkowskiA = min0;
            detector.MinkowskiB = min1;

            // (C++ also adds resultOut->m_closestPointDistanceThreshold here; the current base layer has no such field, treated as 0)
            input.MaximumDistanceSquared = min0.Margin + min1.Margin + manifoldPtr.ContactBreakingThreshold;
            input.MaximumDistanceSquared *= input.MaximumDistanceSquared;

            input.transformA = body0Wrap.WorldTransformInternal;
            input.transformB = body1Wrap.WorldTransformInternal;

            // note: the SAT/clipping path (btPolyhedralContactClipping, used for
            // isPolyhedral && getConvexPolyhedron()!=null) is not ported; the current
            // behavior matches the C++ fallback for getConvexPolyhedron()==null
            // (continues with GJK).

            gjkPairDetector.GetClosestPoints(input, resultOut, dispatchInfo.DebugDraw);

            // now perform numPerturbationIterations collision queries in perturbed orientations

            // perturbation is performed when the manifold has fewer contacts than minimumPointsPerturbationThreshold
            if (numPerturbationIterations != 0 && resultOut.PersistentManifold.NumContacts < minimumPointsPerturbationThreshold)
            {
                btScalar l2 = gjkPairDetector.CachedSeparatingAxis.LengthSquared();

                if (l2 > BtMath.SimdEpsilon)
                {
                    Vector3 sepNormalWorldSpace = gjkPairDetector.CachedSeparatingAxis * (1 / l2);

                    BtMath.PlaneSpace1(in sepNormalWorldSpace, out Vector3 v0, out _);

                    bool perturbeA;
                    const btScalar angleLimit = (btScalar)0.125 * BtMath.Pi;
                    btScalar perturbeAngle;
                    btScalar radiusA = min0.GetAngularMotionDisc();
                    btScalar radiusB = min1.GetAngularMotionDisc();
                    if (radiusA < radiusB)
                    {
                        perturbeAngle = PersistentManifold.GlobalContactBreakingThreshold / radiusA;
                        perturbeA = true;
                    }
                    else
                    {
                        perturbeAngle = PersistentManifold.GlobalContactBreakingThreshold / radiusB;
                        perturbeA = false;
                    }
                    if (perturbeAngle > angleLimit)
                        perturbeAngle = angleLimit;

                    Transform unPerturbedTransform = perturbeA ? input.transformA : input.transformB;

                    for (int i = 0; i < numPerturbationIterations; i++)
                    {
                        if (v0.LengthSquared() > BtMath.SimdEpsilon)
                        {
                            var perturbeRot = new Quaternion(v0, perturbeAngle);
                            btScalar iterationAngle = i * (BtMath.Simd2Pi / numPerturbationIterations);
                            var rotq = new Quaternion(sepNormalWorldSpace, iterationAngle);

                            if (perturbeA)
                            {
                                input.transformA.Basis = new Matrix3x3(rotq.Inverse() * perturbeRot * rotq) * body0Wrap.WorldTransformInternal.Basis;
                                input.transformB = body1Wrap.WorldTransformInternal;
                            }
                            else
                            {
                                input.transformA = body0Wrap.WorldTransformInternal;
                                input.transformB.Basis = new Matrix3x3(rotq.Inverse() * perturbeRot * rotq) * body1Wrap.WorldTransformInternal.Basis;
                            }

                            var perturbedResultOut = new PerturbedContactResult(resultOut, input.transformA, input.transformB,
                                unPerturbedTransform, perturbeA);
                            gjkPairDetector.GetClosestPoints(input, perturbedResultOut, dispatchInfo.DebugDraw);
                        }
                    }
                }
            }
        }

        if (ownManifold)
        {
            resultOut.RefreshContactPoints();
        }
    }

    /// <summary>btConvexConvexAlgorithm::calculateTimeOfImpact</summary>
    public override btScalar CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        // rather than checking all pairs, compute TOI only when the motion exceeds the threshold

        // the linear motion of one of the objects needs to exceed ccdSquareMotionThreshold
        btScalar resultFraction = 1;

        btScalar squareMot0 = (body0.InterpolationWorldTransformInternal.Origin - body0.WorldTransformInternal.Origin).LengthSquared();
        btScalar squareMot1 = (body1.InterpolationWorldTransformInternal.Origin - body1.WorldTransformInternal.Origin).LengthSquared();

        if (squareMot0 < body0.CcdSquareMotionThreshold &&
            squareMot1 < body1.CcdSquareMotionThreshold)
            return resultFraction;

        if (DisableCcd)
            return 1;

        // a quick way to test continuous collision detection algorithms:
        // approximate one of the objects by a sphere to simplify the problem.
        // TOI should not be reported when starting from a penetrating state.
        // a proper CCD implementation would add better accuracy and "allowed
        // penetration" handling, and the physics main loop should keep a TOI queue
        // (like Brian Mirtich's application of Timewarp to rigid bodies).

        // Convex0 against a swept sphere of Convex1
        {
            var convex0 = (ConvexShape)body0.CollisionShape;

            var sphere1 = new SphereShape(body1.CcdSweptSphereRadius); // todo: allow a non-zero sphere radius for a better approximation
            var result = new CastResult();
            var voronoiSimplex = new VoronoiSimplexSolver();
            // simplification: one object is reduced to a sphere
            var ccd1 = new GjkConvexCast(convex0, sphere1, voronoiSimplex);
            if (ccd1.CalcTimeOfImpact(body0.WorldTransformInternal, body0.InterpolationWorldTransformInternal,
                    body1.WorldTransformInternal, body1.InterpolationWorldTransformInternal, result))
            {
                // store result.Fraction in both objects
                if (body0.HitFraction > result.Fraction)
                    body0.HitFraction = result.Fraction;

                if (body1.HitFraction > result.Fraction)
                    body1.HitFraction = result.Fraction;

                if (resultFraction > result.Fraction)
                    resultFraction = result.Fraction;
            }
        }

        // swept sphere of Convex0 against Convex1
        {
            var convex1 = (ConvexShape)body1.CollisionShape;

            var sphere0 = new SphereShape(body0.CcdSweptSphereRadius);
            var result = new CastResult();
            var voronoiSimplex = new VoronoiSimplexSolver();
            var ccd1 = new GjkConvexCast(sphere0, convex1, voronoiSimplex);
            if (ccd1.CalcTimeOfImpact(body0.WorldTransformInternal, body0.InterpolationWorldTransformInternal,
                    body1.WorldTransformInternal, body1.InterpolationWorldTransformInternal, result))
            {
                if (body0.HitFraction > result.Fraction)
                    body0.HitFraction = result.Fraction;

                if (body1.HitFraction > result.Fraction)
                    body1.HitFraction = result.Fraction;

                if (resultFraction > result.Fraction)
                    resultFraction = result.Fraction;
            }
        }

        return resultFraction;
    }

    internal override void Cleanup(Dispatcher dispatcher)
    {
        if (ownManifold && manifoldPtr != null)
        {
            dispatcher.ReleaseManifold(manifoldPtr);
            manifoldPtr = null;
        }
    }
}
