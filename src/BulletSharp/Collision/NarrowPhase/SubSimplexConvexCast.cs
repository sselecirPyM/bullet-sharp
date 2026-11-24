using System.Runtime.CompilerServices;
using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btSubsimplexConvexCast: sub-simplex based convex sweep intersection.
/// </summary>
public class SubSimplexConvexCast : ConvexCast
{
    // Default values of btConvexCast::CastResult::m_subSimplexCastMaxIterations / m_subSimplexCastEpsilon
    internal const int SubSimplexCastMaxIterations = 32;
#if BT_USE_DOUBLE_PRECISION
    internal const btScalar SubSimplexCastEpsilon = (btScalar)0.0001;
#else
    internal const btScalar SubSimplexCastEpsilon = (btScalar)0.0001;
#endif

    internal readonly VoronoiSimplexSolver simplexSolver;
    internal ConvexShape convexA;
    internal ConvexShape convexB;

    public SubSimplexConvexCast(ConvexShape convexA, ConvexShape convexB, VoronoiSimplexSolver simplexSolver)
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
    }

    public ConvexShape ShapeA => convexA;
    public ConvexShape ShapeB => convexB;

    /// <summary>btSubsimplexConvexCast::calcTimeOfImpact</summary>
    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    public override bool CalcTimeOfImpact(in Transform fromA, in Transform toA, in Transform fromB, in Transform toB, CastResult result)
    {
        simplexSolver.Reset();

        Vector3 linVelA = toA.Origin - fromA.Origin;
        Vector3 linVelB = toB.Origin - fromB.Origin;

        btScalar lambda = 0;

        Transform interpolatedTransA = fromA;
        Transform interpolatedTransB = fromB;

        // Take the relative motion
        Vector3 r = linVelA - linVelB;

        Vector3 supVertexA = fromA * convexA.LocalGetSupportingVertex(-r * fromA.Basis);
        Vector3 supVertexB = fromB * convexB.LocalGetSupportingVertex(r * fromB.Basis);
        Vector3 v = supVertexA - supVertexB;
        int maxIter = SubSimplexCastMaxIterations;

        Vector3 n = Vector3.Zero;

        btScalar dist2 = v.LengthSquared();

        while (dist2 > SubSimplexCastEpsilon && maxIter-- != 0)
        {
            supVertexA = interpolatedTransA * convexA.LocalGetSupportingVertex(-v * interpolatedTransA.Basis);
            supVertexB = interpolatedTransB * convexB.LocalGetSupportingVertex(v * interpolatedTransB.Basis);
            Vector3 w = supVertexA - supVertexB;

            btScalar VdotW = v.Dot(w);

            if (lambda > 1)
            {
                return false;
            }

            if (VdotW > 0)
            {
                btScalar VdotR = v.Dot(r);

                if (VdotR >= -(BtMath.SimdEpsilon * BtMath.SimdEpsilon))
                    return false;

                lambda = lambda - VdotW / VdotR;
                // Interpolate to the next lambda: x = s + lambda * r
                interpolatedTransA.Origin = fromA.Origin.Lerp(toA.Origin, lambda);
                interpolatedTransB.Origin = fromB.Origin.Lerp(toB.Origin, lambda);
                // check next line
                w = supVertexA - supVertexB;

                n = v;
            }
            // As in regular GJK, only add the vertex if it isn't (approximately) in the simplex, otherwise it causes division by zero/NaN
            if (!simplexSolver.InSimplex(w))
                simplexSolver.AddVertex(w, supVertexA, supVertexB);

            Vector3 closest = v;
            if (simplexSolver.ClosestPoint(ref closest))
            {
                v = closest;
                dist2 = v.LengthSquared();
            }
            else
            {
                dist2 = 0;
            }
        }

        // Don't report a TOI when moving away along the hit normal
        result.Fraction = lambda;
        if (n.LengthSquared() >= BtMath.SimdEpsilon * BtMath.SimdEpsilon)
            result.Normal = n.Normalized();
        else
            result.Normal = Vector3.Zero;

        // Don't report a TOI when moving away from the contact normal (or it would cause slight penetration)
        if (result.Normal.Dot(r) >= -result.AllowedPenetration)
            return false;

        simplexSolver.ComputePoints(out Vector3 hitA, out Vector3 hitB);
        result.HitPoint = hitB;
        return true;
    }
}
