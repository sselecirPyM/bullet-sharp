using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btContinuousConvexCollision: continuous convex-vs-convex / convex-vs-static-plane TOI computation.
/// </summary>
public class ContinuousConvexCollision : ConvexCast
{
    // Guards against infinite loops on extreme/degenerate input
    private const int MaxIterations = 64;

    internal VoronoiSimplexSolver simplexSolver;
    internal ConvexCast penetrationDepthSolver;
    internal ConvexShape convexA;
    internal ConvexShape convexB1;
    internal StaticPlaneShape planeShape;

    // Reused temporaries (stack objects in C++)
    private readonly PointCollector pointCollector1 = new();
    private readonly PointCollector pointCollectorLoop = new();
    private GjkPairDetector gjkDetector;
    private DiscreteCollisionDetectorInterface.ClosestPointInput closestPointInput;

    public ContinuousConvexCollision(ConvexShape shapeA, ConvexShape shapeB)
        : this(shapeA, shapeB, new VoronoiSimplexSolver(), null)
    {
    }

    public ContinuousConvexCollision(ConvexShape convexA, ConvexShape convexB,
        VoronoiSimplexSolver simplexSolver, ConvexCast penetrationDepthSolver = null)
    {
        this.simplexSolver = simplexSolver;
        this.penetrationDepthSolver = penetrationDepthSolver;
        this.convexA = convexA;
        convexB1 = convexB;
    }

    public ContinuousConvexCollision(ConvexShape convexA, StaticPlaneShape plane)
    {
        simplexSolver = null;
        penetrationDepthSolver = null;
        this.convexA = convexA;
        convexB1 = null;
        planeShape = plane;
    }

    /// <summary>In-place reuse: convex-vs-convex (equivalent to reconstructing on the stack per call in C++)</summary>
    internal void SetShapes(ConvexShape shapeA, ConvexShape shapeB, VoronoiSimplexSolver simplexSolver, ConvexCast penetrationDepthSolver)
    {
        this.simplexSolver = simplexSolver;
        this.penetrationDepthSolver = penetrationDepthSolver;
        convexA = shapeA;
        convexB1 = shapeB;
        planeShape = null;
        gjkDetector = null;
    }

    /// <summary>In-place reuse: convex-vs-plane</summary>
    internal void SetShapes(ConvexShape shapeA, StaticPlaneShape plane)
    {
        simplexSolver = null;
        penetrationDepthSolver = null;
        convexA = shapeA;
        convexB1 = null;
        planeShape = plane;
        gjkDetector = null;
    }

    /// <summary>btContinuousConvexCollision::computeClosestPoints</summary>
    private void ComputeClosestPoints(in Transform transA, in Transform transB, PointCollector pointCollector)
    {
        if (convexB1 != null)
        {
            simplexSolver.Reset();
            gjkDetector ??= new GjkPairDetector(convexA, convexB1,
                (int)convexA.ShapeType, (int)convexB1.ShapeType, convexA.Margin, convexB1.Margin,
                simplexSolver, penetrationDepthSolver);
            // Refresh shape-related fields per call (equivalent to reconstructing btGjkPairDetector on the stack in C++)
            gjkDetector.minkowskiA = convexA;
            gjkDetector.minkowskiB = convexB1;
            gjkDetector.shapeTypeA = (int)convexA.ShapeType;
            gjkDetector.shapeTypeB = (int)convexB1.ShapeType;
            gjkDetector.marginA = convexA.Margin;
            gjkDetector.marginB = convexB1.Margin;
            gjkDetector.simplexSolver = simplexSolver;
            gjkDetector.penetrationDepthSolver = penetrationDepthSolver;
            gjkDetector.cachedSeparatingAxis = new Vector3(0, 1, 0);
            gjkDetector.cachedSeparatingDistance = 0;
            gjkDetector.ignoreMargin = false;

            closestPointInput ??= new DiscreteCollisionDetectorInterface.ClosestPointInput();
            closestPointInput.transformA = transA;
            closestPointInput.transformB = transB;
            closestPointInput.maximumDistanceSquared = BtMath.BtLargeFloat;
            gjkDetector.GetClosestPoints(closestPointInput, pointCollector, null);
        }
        else
        {
            // Convex vs plane
            ConvexShape convexShape = convexA;
            StaticPlaneShape planeShape = this.planeShape;

            Vector3 planeNormal = planeShape.PlaneNormal;
            btScalar planeConstant = planeShape.PlaneConstant;

            Transform convexWorldTransform = transA;
            Transform convexInPlaneTrans = transB.Inverse() * convexWorldTransform;
            Transform planeInConvex = convexWorldTransform.Inverse() * transB;

            Vector3 vtx = convexShape.LocalGetSupportingVertex(planeInConvex.Basis * -planeNormal);

            Vector3 vtxInPlane = convexInPlaneTrans * vtx;
            btScalar distance = planeNormal.Dot(vtxInPlane) - planeConstant;

            Vector3 vtxInPlaneProjected = vtxInPlane - distance * planeNormal;
            Vector3 vtxInPlaneWorld = transB * vtxInPlaneProjected;
            Vector3 normalOnSurfaceB = transB.Basis * planeNormal;

            pointCollector.AddContactPoint(normalOnSurfaceB, vtxInPlaneWorld, distance);
        }
    }

    /// <summary>btContinuousConvexCollision::calcTimeOfImpact</summary>
    public override bool CalcTimeOfImpact(in Transform fromA, in Transform toA, in Transform fromB, in Transform toB, CastResult result)
    {
        // Compute linear and angular velocities over the interval for interpolation
        TransformUtil.CalculateVelocity(in fromA, in toA, 1, out Vector3 linVelA, out Vector3 angVelA);
        TransformUtil.CalculateVelocity(in fromB, in toB, 1, out Vector3 linVelB, out Vector3 angVelB);

        btScalar boundingRadiusA = convexA.GetAngularMotionDisc();
        btScalar boundingRadiusB = convexB1 != null ? convexB1.GetAngularMotionDisc() : 0;

        btScalar maxAngularProjectedVelocity = angVelA.Length() * boundingRadiusA + angVelB.Length() * boundingRadiusB;
        Vector3 relLinVel = linVelB - linVelA;

        btScalar relLinVelocLength = relLinVel.Length();

        if (relLinVelocLength + maxAngularProjectedVelocity == 0)
            return false;

        btScalar lambda = 0;

        Vector3 n = Vector3.Zero;
        Vector3 c = Vector3.Zero;

        btScalar lastLambda = lambda;

        int numIter = 0;
        // First solve using GJK
        btScalar radius = (btScalar)0.001;

        pointCollector1.Reset();
        ComputeClosestPoints(in fromA, in fromB, pointCollector1);

        bool hasResult = pointCollector1.hasResult;
        c = pointCollector1.pointInWorld;

        if (hasResult)
        {
            btScalar dist = pointCollector1.distance + result.AllowedPenetration;
            n = pointCollector1.normalOnBInWorld;
            btScalar projectedLinearVelocity = relLinVel.Dot(n);
            if (projectedLinearVelocity + maxAngularProjectedVelocity <= BtMath.SimdEpsilon)
                return false;

            // Iterate until close enough
            while (dist > radius)
            {
                projectedLinearVelocity = relLinVel.Dot(n);

                // Don't report a TOI when moving away from the contact normal
                if (projectedLinearVelocity + maxAngularProjectedVelocity <= BtMath.SimdEpsilon)
                    return false;

                btScalar dLambda = dist / (projectedLinearVelocity + maxAngularProjectedVelocity);

                lambda += dLambda;

                if (lambda > 1 || lambda < 0)
                    return false;

                // todo: check with relative epsilon
                if (lambda <= lastLambda)
                {
                    return false;
                }
                lastLambda = lambda;

                // Interpolate to the next lambda
                TransformUtil.IntegrateTransform(in fromA, linVelA, angVelA, lambda, out Transform interpolatedTransA);
                TransformUtil.IntegrateTransform(in fromB, linVelB, angVelB, lambda, out Transform interpolatedTransB);

                pointCollectorLoop.Reset();
                ComputeClosestPoints(in interpolatedTransA, in interpolatedTransB, pointCollectorLoop);

                if (pointCollectorLoop.hasResult)
                {
                    dist = pointCollectorLoop.distance + result.AllowedPenetration;
                    c = pointCollectorLoop.pointInWorld;
                    n = pointCollectorLoop.normalOnBInWorld;
                }
                else
                {
                    return false;
                }

                numIter++;
                if (numIter > MaxIterations)
                {
                    return false;
                }
            }

            result.Fraction = lambda;
            result.Normal = n;
            result.HitPoint = c;
            return true;
        }

        return false;
    }
}
