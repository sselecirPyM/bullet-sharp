namespace BulletSharp.Math;

/// <summary>
/// Ported from LinearMath/btPolarDecomposition.h and btPolarDecomposition.cpp.
/// Computes the polar decomposition A = U * H of a real 3x3 matrix into an orthogonal
/// matrix U and a symmetric positive-definite matrix H.
/// </summary>
public sealed class PolarDecomposition
{
    public const btScalar DefaultTolerance = (btScalar)0.0001;
    public const uint DefaultMaxIterations = 16;

    private static readonly PolarDecomposition s_default = new();

    private readonly btScalar tolerance;
    private readonly uint maxIterations;

    public PolarDecomposition(btScalar tolerance = DefaultTolerance, uint maxIterations = DefaultMaxIterations)
    {
        this.tolerance = tolerance;
        this.maxIterations = maxIterations;
    }

    public uint MaxIterations => maxIterations;

    /// <summary>
    /// Decomposes matrix <paramref name="a"/> into orthogonal <paramref name="u"/> and symmetric <paramref name="h"/>.
    /// If the return value equals <see cref="MaxIterations"/>, the algorithm failed to converge within tolerance.
    /// </summary>
    public uint Decompose(in Matrix3x3 a, out Matrix3x3 u, out Matrix3x3 h)
    {
        // Use the 'u' and 'h' matrices for intermediate calculations
        u = a;
        h = a.Inverse();

        for (uint i = 0; i < maxIterations; ++i)
        {
            btScalar h1 = P1Norm(in h);
            btScalar hInf = PInfNorm(in h);
            btScalar u1 = P1Norm(in u);
            btScalar uInf = PInfNorm(in u);

            btScalar hNorm = h1 * hInf;
            btScalar uNorm = u1 * uInf;

            // The matrix is effectively singular so we cannot invert it
            if (FuzzyZero(hNorm) || FuzzyZero(uNorm))
                break;

            btScalar gamma = BtMath.Pow(hNorm / uNorm, (btScalar)0.25);
            btScalar invGamma = 1 / gamma;

            // Determine the delta to 'u'
            Matrix3x3 delta = (u * (gamma - 2) + h.Transpose() * invGamma) * (btScalar)0.5;

            // Update the matrices
            u = u + delta;
            h = u.Inverse();

            // Check for convergence
            if (P1Norm(in delta) <= tolerance * u1)
            {
                h = u.Transpose() * a;
                h = (h + h.Transpose()) * (btScalar)0.5;
                return i;
            }
        }

        // The algorithm has failed to converge to the specified tolerance, but we
        // want to make sure that the matrices returned are in the right form.
        h = u.Transpose() * a;
        h = (h + h.Transpose()) * (btScalar)0.5;

        return maxIterations;
    }

    /// <summary>btPolarDecomposition.h::polarDecompose</summary>
    public static uint PolarDecompose(in Matrix3x3 a, out Matrix3x3 u, out Matrix3x3 h) =>
        s_default.Decompose(in a, out u, out h);

    private static bool FuzzyZero(btScalar x) => BtMath.Abs(x) <= BtMath.SimdEpsilon;

    private static btScalar AbsColumnSum(in Matrix3x3 a, int i) =>
        BtMath.Abs(a[0, i]) + BtMath.Abs(a[1, i]) + BtMath.Abs(a[2, i]);

    private static btScalar AbsRowSum(in Matrix3x3 a, int i) =>
        BtMath.Abs(a[i, 0]) + BtMath.Abs(a[i, 1]) + BtMath.Abs(a[i, 2]);

    private static btScalar P1Norm(in Matrix3x3 a)
    {
        btScalar sum0 = AbsColumnSum(in a, 0);
        btScalar sum1 = AbsColumnSum(in a, 1);
        btScalar sum2 = AbsColumnSum(in a, 2);
        return BtMath.Max(BtMath.Max(sum0, sum1), sum2);
    }

    private static btScalar PInfNorm(in Matrix3x3 a)
    {
        btScalar sum0 = AbsRowSum(in a, 0);
        btScalar sum1 = AbsRowSum(in a, 1);
        btScalar sum2 = AbsRowSum(in a, 2);
        return BtMath.Max(BtMath.Max(sum0, sum1), sum2);
    }
}
