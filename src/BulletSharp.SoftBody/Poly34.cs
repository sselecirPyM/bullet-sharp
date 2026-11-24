using System.Runtime.CompilerServices;
using BulletSharp.Math;

namespace BulletSharp.SoftBody;

/// <summary>
/// Ported from BulletSoftBody/poly34.h and poly34.cpp (public domain, S.I. Khashin).
/// Real and complex root solvers for quadratic, cubic, quartic, and quintic polynomials.
/// </summary>
public static class Poly34
{
    private const btScalar TwoPi = (btScalar)6.28318530717958648;
    private const btScalar Eps = BtMath.SimdEpsilon;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static btScalar Root3Positive(btScalar x)
    {
        btScalar s = 1;
        while (x < 1)
        {
            x *= 8;
            s *= (btScalar)0.5;
        }
        while (x > 8)
        {
            x *= (btScalar)0.125;
            s *= 2;
        }
        btScalar r = (btScalar)1.5;
        const btScalar OneThird = (btScalar)(1.0 / 3.0);
        r -= OneThird * (r - x / (r * r));
        r -= OneThird * (r - x / (r * r));
        r -= OneThird * (r - x / (r * r));
        r -= OneThird * (r - x / (r * r));
        r -= OneThird * (r - x / (r * r));
        r -= OneThird * (r - x / (r * r));
        return r * s;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static btScalar Root3(btScalar x)
    {
        if (x > 0)
            return Root3Positive(x);
        if (x < 0)
            return -Root3Positive(-x);
        return 0;
    }

    /// <summary>
    /// Solves x^2 + a*x + b = 0.
    /// Returns 2 for two real roots (x[0], x[1]); returns 0 for complex conjugate roots x[0] +/- i*x[1].
    /// </summary>
    public static int SolveP2(Span<btScalar> x, btScalar a, btScalar b)
    {
        btScalar d = (btScalar)0.25 * a * a - b;
        if (d >= 0)
        {
            d = BtMath.Sqrt(d);
            x[0] = -(btScalar)0.5 * a + d;
            x[1] = -(btScalar)0.5 * a - d;
            return 2;
        }
        x[0] = -(btScalar)0.5 * a;
        x[1] = BtMath.Sqrt(-d);
        return 0;
    }

    /// <summary>
    /// Solves x^3 + a*x^2 + b*x + c = 0.
    /// Returns 3 (three real roots), 2 (two real roots), or 1 (x[0] real, x[1] +/- i*x[2] complex).
    /// </summary>
    public static int SolveP3(Span<btScalar> x, btScalar a, btScalar b, btScalar c)
    {
        btScalar a2 = a * a;
        btScalar q = (a2 - 3 * b) / 9;
        btScalar r = (a * (2 * a2 - 9 * b) + 27 * c) / 54;
        // equation x^3 + q*x + r = 0
        btScalar r2 = r * r;
        btScalar q3 = q * q * q;
        btScalar capA, capB;
        if (r2 <= (q3 + Eps))
        {
            btScalar t = r / BtMath.Sqrt(q3);
            if (t < -1)
                t = -1;
            if (t > 1)
                t = 1;
            t = BtMath.Acos(t);
            a /= 3;
            q = -2 * BtMath.Sqrt(q);
            x[0] = q * BtMath.Cos(t / 3) - a;
            x[1] = q * BtMath.Cos((t + TwoPi) / 3) - a;
            x[2] = q * BtMath.Cos((t - TwoPi) / 3) - a;
            return 3;
        }
        else
        {
            capA = -Root3(BtMath.Abs(r) + BtMath.Sqrt(r2 - q3));
            if (r < 0)
                capA = -capA;
            capB = capA == 0 ? 0 : q / capA;

            a /= 3;
            x[0] = (capA + capB) - a;
            x[1] = -(btScalar)0.5 * (capA + capB) - a;
            x[2] = (btScalar)0.5 * BtMath.Sqrt(3) * (capA - capB);
            if (BtMath.Abs(x[2]) < Eps)
            {
                x[2] = x[1];
                return 2;
            }
            return 1;
        }
    }

    /// <summary>Returns a + i*b = sqrt(x + i*y) with a &gt;= 0.</summary>
    public static void CSqrt(btScalar x, btScalar y, out btScalar a, out btScalar b)
    {
        btScalar r = BtMath.Sqrt(x * x + y * y);
        if (y == 0)
        {
            r = BtMath.Sqrt(r);
            if (x >= 0)
            {
                a = r;
                b = 0;
            }
            else
            {
                a = 0;
                b = r;
            }
        }
        else
        {
            a = BtMath.Sqrt((btScalar)0.5 * (x + r));
            b = (btScalar)0.5 * y / a;
        }
    }

    /// <summary>Solves biquadratic x^4 + b*x^2 + d = 0.</summary>
    public static int SolveP4Bi(Span<btScalar> x, btScalar b, btScalar d)
    {
        btScalar disc = b * b - 4 * d;
        if (disc >= 0)
        {
            btScalar sD = BtMath.Sqrt(disc);
            btScalar x1 = (-b + sD) / 2;
            btScalar x2 = (-b - sD) / 2; // x2 <= x1
            if (x2 >= 0) // 0 <= x2 <= x1, 4 real roots
            {
                btScalar sx1 = BtMath.Sqrt(x1);
                btScalar sx2 = BtMath.Sqrt(x2);
                x[0] = -sx1;
                x[1] = sx1;
                x[2] = -sx2;
                x[3] = sx2;
                return 4;
            }
            if (x1 < 0) // x2 <= x1 < 0, two pairs of imaginary roots
            {
                btScalar sx1 = BtMath.Sqrt(-x1);
                btScalar sx2 = BtMath.Sqrt(-x2);
                x[0] = 0;
                x[1] = sx1;
                x[2] = 0;
                x[3] = sx2;
                return 0;
            }
            // now x2 < 0 <= x1, two real roots and one pair of imaginary roots
            {
                btScalar sx1 = BtMath.Sqrt(x1);
                btScalar sx2 = BtMath.Sqrt(-x2);
                x[0] = -sx1;
                x[1] = sx1;
                x[2] = 0;
                x[3] = sx2;
                return 2;
            }
        }
        else
        {
            btScalar sD2 = (btScalar)0.5 * BtMath.Sqrt(-disc);
            CSqrt(-(btScalar)0.5 * b, sD2, out x[0], out x[1]);
            CSqrt(-(btScalar)0.5 * b, -sD2, out x[2], out x[3]);
            return 0;
        }
    }

    private static void DblSort3(ref btScalar a, ref btScalar b, ref btScalar c)
    {
        btScalar t;
        if (a > b)
        {
            t = b;
            b = a;
            a = t;
        }
        if (c < b)
        {
            t = c;
            c = b;
            b = t;
            if (a > b)
            {
                t = b;
                b = a;
                a = t;
            }
        }
    }

    /// <summary>Solves depressed quartic x^4 + b*x^2 + c*x + d = 0.</summary>
    public static int SolveP4De(Span<btScalar> x, btScalar b, btScalar c, btScalar d)
    {
        if (BtMath.Abs(c) < (btScalar)1e-14 * (BtMath.Abs(b) + BtMath.Abs(d)))
            return SolveP4Bi(x, b, d);

        int res3 = SolveP3(x, 2 * b, b * b - 4 * d, -c * c);
        if (res3 > 1)
        {
            DblSort3(ref x[0], ref x[1], ref x[2]);
            if (x[0] > 0)
            {
                btScalar sz1 = BtMath.Sqrt(x[0]);
                btScalar sz2 = BtMath.Sqrt(x[1]);
                btScalar sz3 = BtMath.Sqrt(x[2]);
                if (c > 0)
                {
                    x[0] = (-sz1 - sz2 - sz3) / 2;
                    x[1] = (-sz1 + sz2 + sz3) / 2;
                    x[2] = (sz1 - sz2 + sz3) / 2;
                    x[3] = (sz1 + sz2 - sz3) / 2;
                    return 4;
                }
                x[0] = (-sz1 - sz2 + sz3) / 2;
                x[1] = (-sz1 + sz2 - sz3) / 2;
                x[2] = (sz1 - sz2 - sz3) / 2;
                x[3] = (sz1 + sz2 + sz3) / 2;
                return 4;
            }
            {
                btScalar sz1 = BtMath.Sqrt(-x[0]);
                btScalar sz2 = BtMath.Sqrt(-x[1]);
                btScalar sz3 = BtMath.Sqrt(x[2]);

                if (c > 0)
                {
                    x[0] = -sz3 / 2;
                    x[1] = (sz1 - sz2) / 2;
                    x[2] = sz3 / 2;
                    x[3] = (-sz1 - sz2) / 2;
                    return 0;
                }
                x[0] = sz3 / 2;
                x[1] = (-sz1 + sz2) / 2;
                x[2] = -sz3 / 2;
                x[3] = (sz1 + sz2) / 2;
                return 0;
            }
        }

        if (x[0] < 0)
            x[0] = 0;
        btScalar sz1Real = BtMath.Sqrt(x[0]);
        CSqrt(x[1], x[2], out btScalar szr, out btScalar szi);
        if (c > 0)
        {
            x[0] = -sz1Real / 2 - szr;
            x[1] = -sz1Real / 2 + szr;
            x[2] = sz1Real / 2;
            x[3] = szi;
            return 2;
        }
        x[0] = sz1Real / 2 - szr;
        x[1] = sz1Real / 2 + szr;
        x[2] = -sz1Real / 2;
        x[3] = szi;
        return 2;
    }

    /// <summary>Performs one Newton step for x^4 + a*x^3 + b*x^2 + c*x + d.</summary>
    public static btScalar N4Step(btScalar x, btScalar a, btScalar b, btScalar c, btScalar d)
    {
        btScalar fxs = ((4 * x + 3 * a) * x + 2 * b) * x + c;
        if (fxs == 0)
            return x;
        btScalar fx = (((x + a) * x + b) * x + c) * x + d;
        return x - fx / fxs;
    }

    /// <summary>
    /// Solves x^4 + a*x^3 + b*x^2 + c*x + d = 0 by Descartes-Euler method.
    /// Returns 4 (four real roots), 2 (x[0], x[1] real and x[2] +/- i*x[3]), or 0 (two complex pairs).
    /// </summary>
    public static int SolveP4(Span<btScalar> x, btScalar a, btScalar b, btScalar c, btScalar d)
    {
        btScalar d1 = d + (btScalar)0.25 * a * ((btScalar)0.25 * b * a - (btScalar)(3.0 / 64.0) * a * a * a - c);
        btScalar c1 = c + (btScalar)0.5 * a * ((btScalar)0.25 * a * a - b);
        btScalar b1 = b - (btScalar)0.375 * a * a;
        int res = SolveP4De(x, b1, c1, d1);
        if (res == 4)
        {
            x[0] -= a / 4;
            x[1] -= a / 4;
            x[2] -= a / 4;
            x[3] -= a / 4;
        }
        else if (res == 2)
        {
            x[0] -= a / 4;
            x[1] -= a / 4;
            x[2] -= a / 4;
        }
        else
        {
            x[0] -= a / 4;
            x[2] -= a / 4;
        }
        if (res > 0)
        {
            x[0] = N4Step(x[0], a, b, c, d);
            x[1] = N4Step(x[1], a, b, c, d);
        }
        if (res > 2)
        {
            x[2] = N4Step(x[2], a, b, c, d);
            x[3] = N4Step(x[3], a, b, c, d);
        }
        return res;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static btScalar F5(btScalar t, btScalar a, btScalar b, btScalar c, btScalar d, btScalar e) =>
        ((((t + a) * t + b) * t + c) * t + d) * t + e;

    /// <summary>Returns one real root of x^5 + a*x^4 + b*x^3 + c*x^2 + d*x + e = 0.</summary>
    public static btScalar SolveP5_1(btScalar a, btScalar b, btScalar c, btScalar d, btScalar e)
    {
        int cnt;
        if (BtMath.Abs(e) < Eps)
            return 0;

        btScalar brd = BtMath.Abs(a);
        if (BtMath.Abs(b) > brd)
            brd = BtMath.Abs(b);
        if (BtMath.Abs(c) > brd)
            brd = BtMath.Abs(c);
        if (BtMath.Abs(d) > brd)
            brd = BtMath.Abs(d);
        if (BtMath.Abs(e) > brd)
            brd = BtMath.Abs(e);
        brd++;

        btScalar x0, f0;
        btScalar x1, f1;
        btScalar x2, f2, f2s;
        btScalar dx = 0;

        if (e < 0)
        {
            x0 = 0;
            x1 = brd;
            f0 = e;
            f1 = F5(x1, a, b, c, d, e);
            x2 = (btScalar)0.01 * brd;
        }
        else
        {
            x0 = -brd;
            x1 = 0;
            f0 = F5(x0, a, b, c, d, e);
            f1 = e;
            x2 = -(btScalar)0.01 * brd;
        }

        if (BtMath.Abs(f0) < Eps)
            return x0;
        if (BtMath.Abs(f1) < Eps)
            return x1;

        for (cnt = 0; cnt < 10; cnt++)
        {
            x2 = (x0 + x1) / 2;
            f2 = F5(x2, a, b, c, d, e);
            if (BtMath.Abs(f2) < Eps)
                return x2;
            if (f2 > 0)
            {
                x1 = x2;
                f1 = f2;
            }
            else
            {
                x0 = x2;
                f0 = f2;
            }
        }

        do
        {
            if (cnt++ > 50)
                break;
            if (x2 <= x0 || x2 >= x1)
                x2 = (x0 + x1) / 2;
            f2 = F5(x2, a, b, c, d, e);
            if (BtMath.Abs(f2) < Eps)
                return x2;
            if (f2 > 0)
            {
                x1 = x2;
                f1 = f2;
            }
            else
            {
                x0 = x2;
                f0 = f2;
            }
            f2s = (((5 * x2 + 4 * a) * x2 + 3 * b) * x2 + 2 * c) * x2 + d;
            if (BtMath.Abs(f2s) < Eps)
            {
#if BT_USE_DOUBLE_PRECISION
                x2 = 1e99;
#else
                x2 = float.PositiveInfinity;
#endif
                continue;
            }
            dx = f2 / f2s;
            x2 -= dx;
        } while (BtMath.Abs(dx) > Eps);
        return x2;
    }

    /// <summary>
    /// Solves x^5 + a*x^4 + b*x^3 + c*x^2 + d*x + e = 0.
    /// Returns 5, 3, or 1 (number of real roots).
    /// </summary>
    public static int SolveP5(Span<btScalar> x, btScalar a, btScalar b, btScalar c, btScalar d, btScalar e)
    {
        btScalar r = x[0] = SolveP5_1(a, b, c, d, e);
        btScalar a1 = a + r, b1 = b + r * a1, c1 = c + r * b1, d1 = d + r * c1;
        return 1 + SolveP4(x.Slice(1), a1, b1, c1, d1);
    }
}
