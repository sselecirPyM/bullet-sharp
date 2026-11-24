using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btGjkEpaPenetrationDepthSolver: penetration depth solver based on btGjkEpaSolver2.
/// Stateless and shareable across algorithms. Inherits ConvexCast to stay compatible with the solver
/// parameter types of BulletSharp's ContinuousConvexCollision/GjkPairDetector (its CalcTimeOfImpact is not usable).
/// </summary>
public class GjkEpaPenetrationDepthSolver : ConvexCast, IConvexPenetrationDepthSolver
{
    /// <summary>btVector3::safeNormalize</summary>
    private static Vector3 SafeNormalize(Vector3 v)
    {
        btScalar l2 = v.LengthSquared();
        if (l2 >= BtMath.SimdEpsilon * BtMath.SimdEpsilon)
        {
            v /= BtMath.Sqrt(l2);
        }
        else
        {
            v = new Vector3(1, 0, 0);
        }
        return v;
    }

    /// <summary>btGjkEpaPenetrationDepthSolver::calcPenDepth</summary>
    public bool CalcPenDepth(VoronoiSimplexSolver simplexSolver,
        ConvexShape pConvexA, ConvexShape pConvexB,
        in Transform transformA, in Transform transformB,
        ref Vector3 v, out Vector3 wWitnessOnA, out Vector3 wWitnessOnB)
    {
        Span<Vector3> guessVectors = stackalloc Vector3[9]
        {
            SafeNormalize(transformB.Origin - transformA.Origin),
            SafeNormalize(transformA.Origin - transformB.Origin),
            new Vector3(0, 0, 1),
            new Vector3(0, 1, 0),
            new Vector3(1, 0, 0),
            new Vector3(1, 1, 0),
            new Vector3(1, 1, 1),
            new Vector3(0, 1, 1),
            new Vector3(1, 0, 1),
        };

        int numVectors = guessVectors.Length;

        for (int i = 0; i < numVectors; i++)
        {
            simplexSolver.Reset();
            Vector3 guessVector = guessVectors[i];

            // Value type (on the stack), equivalent to sResults on the stack in C++
            GjkEpaSolver2.Results results = default;

            if (GjkEpaSolver2.Penetration(pConvexA, in transformA, pConvexB, in transformB, guessVector, ref results))
            {
                wWitnessOnA = results.Witness0;
                wWitnessOnB = results.Witness1;
                v = results.Normal;
                return true;
            }
            else
            {
                if (GjkEpaSolver2.Distance(pConvexA, in transformA, pConvexB, in transformB, guessVector, ref results))
                {
                    wWitnessOnA = results.Witness0;
                    wWitnessOnB = results.Witness1;
                    v = results.Normal;
                    return false;
                }
            }
        }

        // Failed to compute distance/penetration
        wWitnessOnA = Vector3.Zero;
        wWitnessOnB = Vector3.Zero;
        v = Vector3.Zero;
        return false;
    }

    /// <summary>ConvexCast interface stub: a penetration solver doesn't support TOI computation</summary>
    public override bool CalcTimeOfImpact(in Transform fromA, in Transform toA, in Transform fromB, in Transform toB, CastResult result) => false;
}
