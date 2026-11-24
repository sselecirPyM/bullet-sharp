using System.Runtime.CompilerServices;
using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btGjkConvexCast: convex sweep intersection based on iterative GJK closest points.
/// </summary>
public class GjkConvexCast : ConvexCast
{
#if BT_USE_DOUBLE_PRECISION
    private const int MaxIterations = 64;
#else
    private const int MaxIterations = 32;
#endif

    internal readonly VoronoiSimplexSolver simplexSolver;
    internal ConvexShape convexA;
    internal ConvexShape convexB;

    // Reused GJK temporaries (stack objects in C++ calcTimeOfImpact)
    private readonly PointCollector pointCollector = new();
    private readonly DiscreteCollisionDetectorInterface.ClosestPointInput gjkInput = new();
    private GjkPairDetector gjk;

    public GjkConvexCast(ConvexShape convexA, ConvexShape convexB, VoronoiSimplexSolver simplexSolver)
    {
        this.simplexSolver = simplexSolver;
        this.convexA = convexA;
        this.convexB = convexB;
    }

    /// <summary>In-place reuse (avoids one allocation per convex object during raycasts; equivalent to a C++ stack object)</summary>
    internal void SetShapes(ConvexShape convexA, ConvexShape convexB)
    {
        this.convexA = convexA;
        this.convexB = convexB;
        gjk = null; // shapes changed; rebuild on next use (simplexSolver is still reused)
    }

    public ConvexShape ShapeA => convexA;
    public ConvexShape ShapeB => convexB;

    /// <summary>btGjkConvexCast::calcTimeOfImpact</summary>
    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    public override bool CalcTimeOfImpact(in Transform fromA, in Transform toA, in Transform fromB, in Transform toB, CastResult result)
    {
        simplexSolver.Reset();

        // Compute linear velocities over the interval for interpolation (assumes no rotation/angular velocity)
        Vector3 linVelA = toA.Origin - fromA.Origin;
        Vector3 linVelB = toB.Origin - fromB.Origin;

        btScalar radius = (btScalar)0.001;
        btScalar lambda = 0;
        Vector3 v = new(1, 0, 0);

        int maxIter = MaxIterations;

        Vector3 n = Vector3.Zero;
        Vector3 c = Vector3.Zero;
        Vector3 r = linVelA - linVelB;

        btScalar lastLambda = lambda;

        int numIter = 0;
        // First solve using GJK (reusing cached objects; stack objects in C++)
        PointCollector pointCollector = this.pointCollector;
        pointCollector.distance = BtMath.BtLargeFloat;
        pointCollector.hasResult = false;
        gjk ??= new GjkPairDetector(convexA, convexB, simplexSolver, null);
        gjk.MinkowskiA = convexA;
        gjk.MinkowskiB = convexB;
        var input = gjkInput;
        input.transformA = fromA;
        input.transformB = fromB;
        input.maximumDistanceSquared = BtMath.BtLargeFloat;

        gjk.GetClosestPoints(input, pointCollector, null);

        bool hasResult = pointCollector.hasResult;
        c = pointCollector.pointInWorld;

        if (hasResult)
        {
            btScalar dist = pointCollector.distance;
            n = pointCollector.normalOnBInWorld;

            // Iterate until close enough
            while (dist > radius)
            {
                numIter++;
                if (numIter > maxIter)
                {
                    return false; // todo: report a failure
                }

                btScalar projectedLinearVelocity = r.Dot(n);
                btScalar dLambda = dist / projectedLinearVelocity;

                lambda = lambda - dLambda;

                if (lambda > 1)
                    return false;

                if (lambda < 0)
                    return false;

                // todo: check with relative epsilon
                if (lambda <= lastLambda)
                {
                    return false;
                }
                lastLambda = lambda;

                // Interpolate to the next lambda
                input.transformA.Origin = fromA.Origin.Lerp(toA.Origin, lambda);
                input.transformB.Origin = fromB.Origin.Lerp(toB.Origin, lambda);

                gjk.GetClosestPoints(input, pointCollector, null);
                if (pointCollector.hasResult)
                {
                    if (pointCollector.distance < 0)
                    {
                        result.Fraction = lastLambda;
                        n = pointCollector.normalOnBInWorld;
                        result.Normal = n;
                        result.HitPoint = pointCollector.pointInWorld;
                        return true;
                    }
                    c = pointCollector.pointInWorld;
                    n = pointCollector.normalOnBInWorld;
                    dist = pointCollector.distance;
                }
                else
                {
                    return false;
                }
            }

            // Don't report a TOI when moving away from the contact normal (or it would cause slight penetration)
            if (n.Dot(r) >= -result.AllowedPenetration)
                return false;

            result.Fraction = lambda;
            result.Normal = n;
            result.HitPoint = c;
            return true;
        }

        return false;
    }
}
