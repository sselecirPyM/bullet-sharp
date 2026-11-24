using System.Runtime.CompilerServices;

namespace BulletSharp.Math;

/// <summary>
/// Math functions and constants tied to btScalar precision (replacing Bullet's SIMD_* macros).
/// </summary>
public static class BtMath
{
#if BT_USE_DOUBLE_PRECISION
    public const btScalar BtEpsilon = 2.2204460492503131e-16;
    public const btScalar SimdEpsilon = 2.2204460492503131e-16;
    public const btScalar BtLargeFloat = 1e30;
    public const btScalar SimdInfinity = double.MaxValue;
#else
    public const btScalar BtEpsilon = 1.192092896e-7f;
    public const btScalar SimdEpsilon = 1.192092896e-7f;
    public const btScalar BtLargeFloat = 1e18f;
    public const btScalar SimdInfinity = float.MaxValue;
#endif
    public const btScalar Pi = (btScalar)3.14159265358979323846264338327950288;
    public const btScalar Simd2Pi = Pi * 2;
    public const btScalar SimdHalfPi = Pi / 2;
    public const btScalar SimdQuarterPi = Pi / 4;
    public const btScalar SimdRadsPerDeg = Simd2Pi / 360;
    public const btScalar SimdDegsPerRad = 360 / Simd2Pi;
    public const btScalar SimdSqrt12 = (btScalar)0.7071067811865475244008443621048490;
    /// <summary>Bullet's ANGULAR_MOTION_THRESHOLD</summary>
    public const btScalar AngularMotionThreshold = SimdHalfPi / 2;
    /// <summary>CONVEX_DISTANCE_MARGIN; default margin for new convex shapes</summary>
    public const btScalar ConvexDistanceMargin = (btScalar)0.04;
    /// <summary>Default value of gContactBreakingThreshold</summary>
    public const btScalar ContactBreakingThreshold = (btScalar)0.02;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Sqrt(btScalar x)
    {
#if BT_USE_DOUBLE_PRECISION
        return System.Math.Sqrt(x);
#else
        return System.MathF.Sqrt(x);
#endif
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar RecipSqrt(btScalar x) => 1 / Sqrt(x);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Sin(btScalar x)
    {
#if BT_USE_DOUBLE_PRECISION
        return System.Math.Sin(x);
#else
        return System.MathF.Sin(x);
#endif
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Cos(btScalar x)
    {
#if BT_USE_DOUBLE_PRECISION
        return System.Math.Cos(x);
#else
        return System.MathF.Cos(x);
#endif
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Tan(btScalar x)
    {
#if BT_USE_DOUBLE_PRECISION
        return System.Math.Tan(x);
#else
        return System.MathF.Tan(x);
#endif
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Asin(btScalar x)
    {
        if (x < -1) return -SimdHalfPi;
        if (x > 1) return SimdHalfPi;
#if BT_USE_DOUBLE_PRECISION
        return System.Math.Asin(x);
#else
        return System.MathF.Asin(x);
#endif
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Acos(btScalar x)
    {
        if (x < -1) return Pi;
        if (x > 1) return 0;
#if BT_USE_DOUBLE_PRECISION
        return System.Math.Acos(x);
#else
        return System.MathF.Acos(x);
#endif
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Atan(btScalar x)
    {
#if BT_USE_DOUBLE_PRECISION
        return System.Math.Atan(x);
#else
        return System.MathF.Atan(x);
#endif
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Atan2(btScalar y, btScalar x)
    {
#if BT_USE_DOUBLE_PRECISION
        return System.Math.Atan2(y, x);
#else
        return System.MathF.Atan2(y, x);
#endif
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Atan2Fast(btScalar y, btScalar x)
    {
        btScalar coeff1 = Pi / 4;
        btScalar coeff2 = 3 * coeff1;
        btScalar absY = Abs(y);
        btScalar angle;
        if (x >= 0)
        {
            btScalar r = (x - absY) / (x + absY);
            angle = coeff1 - coeff1 * r;
        }
        else
        {
            btScalar r = (x + absY) / (absY - x);
            angle = coeff2 - coeff1 * r;
        }
        return y < 0 ? -angle : angle;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Abs(btScalar x)
    {
#if BT_USE_DOUBLE_PRECISION
        return System.Math.Abs(x);
#else
        return System.MathF.Abs(x);
#endif
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Floor(btScalar x)
    {
#if BT_USE_DOUBLE_PRECISION
        return System.Math.Floor(x);
#else
        return System.MathF.Floor(x);
#endif
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Ceil(btScalar x)
    {
#if BT_USE_DOUBLE_PRECISION
        return System.Math.Ceiling(x);
#else
        return System.MathF.Ceiling(x);
#endif
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Pow(btScalar x, btScalar y)
    {
#if BT_USE_DOUBLE_PRECISION
        return System.Math.Pow(x, y);
#else
        return System.MathF.Pow(x, y);
#endif
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Exp(btScalar x)
    {
#if BT_USE_DOUBLE_PRECISION
        return System.Math.Exp(x);
#else
        return System.MathF.Exp(x);
#endif
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Fmod(btScalar x, btScalar y)
    {
#if BT_USE_DOUBLE_PRECISION
        return x % y;
#else
        return x % y;
#endif
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Max(btScalar a, btScalar b) => a > b ? a : b;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Min(btScalar a, btScalar b) => a < b ? a : b;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Max(int a, int b) => a > b ? a : b;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Min(int a, int b) => a < b ? a : b;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Clamp(btScalar v, btScalar lo, btScalar hi) => v < lo ? lo : (v > hi ? hi : v);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Clamp(int v, int lo, int hi) => v < lo ? lo : (v > hi ? hi : v);

    /// <summary>btFsel: a >= 0 ? b : c</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Fsel(btScalar a, btScalar b, btScalar c) => a >= 0 ? b : c;

    /// <summary>btMachineIsLittleEndian</summary>
    public static bool MachineIsLittleEndian => BitConverter.IsLittleEndian;

    /// <summary>btPlaneSpace1 - generates an orthonormal plane basis for normal n</summary>
    public static void PlaneSpace1(in Vector3 n, out Vector3 p, out Vector3 q)
    {
        if (Abs(n.Z) > SimdSqrt12)
        {
            // choose p in y-z plane
            btScalar a = n.Y * n.Y + n.Z * n.Z;
            btScalar k = RecipSqrt(a);
            p = new Vector3(0, -n.Z * k, n.Y * k);
            // set q = n x p
            q = new Vector3(a * k, -n.X * p.Z, n.X * p.Y);
        }
        else
        {
            // choose p in x-y plane
            btScalar a = n.X * n.X + n.Y * n.Y;
            btScalar k = RecipSqrt(a);
            p = new Vector3(-n.Y * k, n.X * k, 0);
            // set q = n x p
            q = new Vector3(-n.Z * p.Y, n.Z * p.X, a * k);
        }
    }

    /// <summary>btFsels</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Recip(btScalar x) => 1 / x;
}
