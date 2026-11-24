using System.Diagnostics;
using System.Runtime.CompilerServices;
using BulletSharp.Math;

namespace BulletSharp.SoftBody;

/// <summary>
/// Ported from BulletSoftBody/btSoftBodyInternals.h (btSymMatrix&lt;T&gt;).
/// Symmetric lower-triangular packed matrix storage.
/// </summary>
internal sealed class SymMatrix<T>
{
    internal T[] store = Array.Empty<T>();
    internal int dim;

    public SymMatrix()
    {
        dim = 0;
    }

    public SymMatrix(int n, T init = default)
    {
        Resize(n, init);
    }

    public int Dim => dim;

    public T[] Store => store;

    public void Resize(int n, T init = default)
    {
        dim = n;
        int newSize = (n * (n + 1)) / 2;
        int oldSize = store.Length;
        if (newSize != oldSize)
        {
            Array.Resize(ref store, newSize);
            for (int i = oldSize; i < newSize; ++i)
            {
                store[i] = init;
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int Index(int c, int r)
    {
        if (c > r)
        {
            (c, r) = (r, c);
        }
        Debug.Assert(c >= 0 && r < dim);
        return (r * (r + 1)) / 2 + c;
    }

    public ref T this[int c, int r]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => ref store[Index(c, r)];
    }
}

/// <summary>
/// Ported from BulletSoftBody/btSoftBodyInternals.h (inline math and geometry helpers).
/// Note: ZeroInitialize/CompLess/CompGreater are omitted (handled natively by CLR zero-init and C# comparers),
/// and btEigen is omitted (unused in classic btSoftBody paths; Matrix3x3.Diagonalize provides Jacobi eigensolve).
/// </summary>
internal static class SoftBodyInternals
{
    [ThreadStatic]
    private static Vector3[] t_facePoints;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Lerp(btScalar a, btScalar b, btScalar t) => a + (b - a) * t;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 Lerp(in Vector3 a, in Vector3 b, btScalar t) => a + (b - a) * t;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix3x3 Lerp(in Matrix3x3 a, in Matrix3x3 b, btScalar t) => new(
        Lerp(in a.Row0, in b.Row0, t),
        Lerp(in a.Row1, in b.Row1, t),
        Lerp(in a.Row2, in b.Row2, t));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar InvLerp(btScalar a, btScalar b, btScalar t) =>
        (b + a * t - b * t) / (a * b);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 Clamp(in Vector3 v, btScalar maxlength)
    {
        btScalar sql = v.LengthSquared();
        if (sql > maxlength * maxlength)
        {
            return (v * maxlength) / BtMath.Sqrt(sql);
        }
        return v;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Clamp(btScalar x, btScalar l, btScalar h) =>
        x < l ? l : (x > h ? h : x);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Clamp(int x, int l, int h) =>
        x < l ? l : (x > h ? h : x);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Sq(btScalar x) => x * x;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Sq(int x) => x * x;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Cube(btScalar x) => x * x * x;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Cube(int x) => x * x * x;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Sign(btScalar x) => x < 0 ? (btScalar)(-1) : (btScalar)1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Sign(int x) => x < 0 ? -1 : 1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool SameSign(btScalar x, btScalar y) => (x * y) > 0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar ClusterMetric(in Vector3 x, in Vector3 y)
    {
        Vector3 d = x - y;
        return BtMath.Abs(d.X) + BtMath.Abs(d.Y) + BtMath.Abs(d.Z);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix3x3 ScaleAlongAxis(in Vector3 a, btScalar s)
    {
        btScalar xx = a.X * a.X;
        btScalar yy = a.Y * a.Y;
        btScalar zz = a.Z * a.Z;
        btScalar xy = a.X * a.Y;
        btScalar yz = a.Y * a.Z;
        btScalar zx = a.Z * a.X;
        return new Matrix3x3(
            new Vector3(1 - xx + xx * s, xy * s - xy, zx * s - zx),
            new Vector3(xy * s - xy, 1 - yy + yy * s, yz * s - yz),
            new Vector3(zx * s - zx, yz * s - yz, 1 - zz + zz * s));
    }

    /// <summary>Skew-symmetric cross-product matrix [v]_x such that Cross(v) * u == v x u.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix3x3 Cross(in Vector3 v) => new(
        new Vector3(0, -v.Z, v.Y),
        new Vector3(v.Z, 0, -v.X),
        new Vector3(-v.Y, v.X, 0));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix3x3 Diagonal(btScalar x) => new(
        new Vector3(x, 0, 0),
        new Vector3(0, x, 0),
        new Vector3(0, 0, x));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix3x3 Diagonal(in Vector3 v) => new(
        new Vector3(v.X, 0, 0),
        new Vector3(0, v.Y, 0),
        new Vector3(0, 0, v.Z));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Dot(btScalar[] a, btScalar[] b, int ndof)
    {
        btScalar result = 0;
        for (int i = 0; i < ndof; ++i)
        {
            result += a[i] * b[i];
        }
        return result;
    }

    public static Matrix3x3 OuterProduct(
        btScalar[] v1, btScalar[] v2, btScalar[] v3,
        btScalar[] u1, btScalar[] u2, btScalar[] u3,
        int ndof)
    {
        btScalar a11 = Dot(v1, u1, ndof);
        btScalar a12 = Dot(v1, u2, ndof);
        btScalar a13 = Dot(v1, u3, ndof);

        btScalar a21 = Dot(v2, u1, ndof);
        btScalar a22 = Dot(v2, u2, ndof);
        btScalar a23 = Dot(v2, u3, ndof);

        btScalar a31 = Dot(v3, u1, ndof);
        btScalar a32 = Dot(v3, u2, ndof);
        btScalar a33 = Dot(v3, u3, ndof);

        return new Matrix3x3(
            new Vector3(a11, a12, a13),
            new Vector3(a21, a22, a23),
            new Vector3(a31, a32, a33));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix3x3 OuterProduct(in Vector3 v1, in Vector3 v2)
    {
        btScalar a11 = v1.X * v2.X;
        btScalar a12 = v1.X * v2.Y;
        btScalar a13 = v1.X * v2.Z;

        btScalar a21 = v1.Y * v2.X;
        btScalar a22 = v1.Y * v2.Y;
        btScalar a23 = v1.Y * v2.Z;

        btScalar a31 = v1.Z * v2.X;
        btScalar a32 = v1.Z * v2.Y;
        btScalar a33 = v1.Z * v2.Z;

        return new Matrix3x3(
            new Vector3(a11, a12, a13),
            new Vector3(a21, a22, a23),
            new Vector3(a31, a32, a33));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix3x3 Add(in Matrix3x3 a, in Matrix3x3 b) => new(
        a.Row0 + b.Row0,
        a.Row1 + b.Row1,
        a.Row2 + b.Row2);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix3x3 Sub(in Matrix3x3 a, in Matrix3x3 b) => new(
        a.Row0 - b.Row0,
        a.Row1 - b.Row1,
        a.Row2 - b.Row2);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix3x3 Mul(in Matrix3x3 a, btScalar b) => new(
        a.Row0 * b,
        a.Row1 * b,
        a.Row2 * b);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Orthogonalize(ref Matrix3x3 m)
    {
        m.Row2 = Vector3.Cross(m.Row0, m.Row1).Normalized();
        m.Row1 = Vector3.Cross(m.Row2, m.Row0).Normalized();
        m.Row0 = Vector3.Cross(m.Row1, m.Row2).Normalized();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix3x3 MassMatrix(btScalar im, in Matrix3x3 iwi, in Vector3 r)
    {
        Matrix3x3 cr = Cross(in r);
        return Sub(Diagonal(im), cr * iwi * cr);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix3x3 ImpulseMatrix(
        btScalar dt,
        btScalar ima,
        btScalar imb,
        in Matrix3x3 iwi,
        in Vector3 r) =>
        Diagonal(1 / dt) * Add(Diagonal(ima), MassMatrix(imb, in iwi, in r)).Inverse();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix3x3 ImpulseMatrix(
        btScalar dt,
        in Matrix3x3 effectiveMassInv,
        btScalar imb,
        in Matrix3x3 iwi,
        in Vector3 r) =>
        Diagonal(1 / dt) * Add(in effectiveMassInv, MassMatrix(imb, in iwi, in r)).Inverse();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix3x3 ImpulseMatrix(
        btScalar ima, in Matrix3x3 iia, in Vector3 ra,
        btScalar imb, in Matrix3x3 iib, in Vector3 rb) =>
        Add(MassMatrix(ima, in iia, in ra), MassMatrix(imb, in iib, in rb)).Inverse();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix3x3 AngularImpulseMatrix(in Matrix3x3 iia, in Matrix3x3 iib) =>
        Add(in iia, in iib).Inverse();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 ProjectOnAxis(in Vector3 v, in Vector3 a) =>
        a * Vector3.Dot(v, a);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 ProjectOnPlane(in Vector3 v, in Vector3 a) =>
        v - ProjectOnAxis(in v, in a);

    // Note: btSoftBodyInternals.h defines exactly two ProjectOrigin overloads (segment and triangle);
    // the 3-overload simplex version returning barycentric weights/mask lives in GjkEpa2.
    public static void ProjectOrigin(
        in Vector3 a,
        in Vector3 b,
        ref Vector3 prj,
        ref btScalar sqd)
    {
        Vector3 d = b - a;
        btScalar m2 = d.LengthSquared();
        if (m2 > BtMath.SimdEpsilon)
        {
            btScalar t = Clamp(-Vector3.Dot(a, d) / m2, (btScalar)0, (btScalar)1);
            Vector3 p = a + d * t;
            btScalar l2 = p.LengthSquared();
            if (l2 < sqd)
            {
                prj = p;
                sqd = l2;
            }
        }
    }

    public static void ProjectOrigin(
        in Vector3 a,
        in Vector3 b,
        in Vector3 c,
        ref Vector3 prj,
        ref btScalar sqd)
    {
        Vector3 q = Vector3.Cross(b - a, c - a);
        btScalar m2 = q.LengthSquared();
        if (m2 > BtMath.SimdEpsilon)
        {
            Vector3 n = q / BtMath.Sqrt(m2);
            btScalar k = Vector3.Dot(a, n);
            btScalar k2 = k * k;
            if (k2 < sqd)
            {
                Vector3 p = n * k;
                if (Vector3.Dot(Vector3.Cross(a - p, b - p), q) > 0 &&
                    Vector3.Dot(Vector3.Cross(b - p, c - p), q) > 0 &&
                    Vector3.Dot(Vector3.Cross(c - p, a - p), q) > 0)
                {
                    prj = p;
                    sqd = k2;
                }
                else
                {
                    ProjectOrigin(in a, in b, ref prj, ref sqd);
                    ProjectOrigin(in b, in c, ref prj, ref sqd);
                    ProjectOrigin(in c, in a, ref prj, ref sqd);
                }
            }
        }
    }

    public static bool RayIntersectsTriangle(
        in Vector3 origin,
        in Vector3 dir,
        in Vector3 v0,
        in Vector3 v1,
        in Vector3 v2,
        out btScalar t)
    {
        t = 0;
        Vector3 e1 = v1 - v0;
        Vector3 e2 = v2 - v0;
        Vector3 h = dir.Cross(e2);
        btScalar a = e1.Dot(h);

        if (a > (btScalar)(-0.00001) && a < (btScalar)0.00001)
        {
            return false;
        }

        btScalar f = (btScalar)1 / a;
        Vector3 s = origin - v0;
        btScalar u = f * s.Dot(h);

        if (u < 0 || u > 1)
        {
            return false;
        }

        Vector3 q = s.Cross(e1);
        btScalar v = f * dir.Dot(q);
        if (v < 0 || u + v > 1)
        {
            return false;
        }

        t = f * e2.Dot(q);
        return t > 0;
    }

    public static bool LineIntersectsTriangle(
        in Vector3 rayStart,
        in Vector3 rayEnd,
        in Vector3 p1,
        in Vector3 p2,
        in Vector3 p3,
        out Vector3 sect,
        out Vector3 normal)
    {
        sect = Vector3.Zero;
        normal = Vector3.Zero;

        Vector3 dir = rayEnd - rayStart;
        btScalar dirNorm = dir.Length();
        if (dirNorm < BtMath.SimdEpsilon)
        {
            return false;
        }

        dir.Normalize();
        bool ret = RayIntersectsTriangle(in rayStart, in dir, in p1, in p2, in p3, out btScalar t);
        if (ret)
        {
            if (t <= dirNorm)
            {
                sect = rayStart + dir * t;
            }
            else
            {
                ret = false;
            }
        }

        if (ret)
        {
            Vector3 n = (p3 - p1).Cross(p2 - p1);
            // btVector3::safeNormalize()
            btScalar l2 = n.LengthSquared();
            if (l2 >= BtMath.SimdEpsilon * BtMath.SimdEpsilon)
            {
                n /= BtMath.Sqrt(l2);
            }
            else
            {
                n = new Vector3(1, 0, 0);
            }

            normal = n.Dot(dir) < 0 ? n : -n;
        }

        return ret;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar BaryEval(btScalar a, btScalar b, btScalar c, in Vector3 coord) =>
        a * coord.X + b * coord.Y + c * coord.Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 BaryEval(in Vector3 a, in Vector3 b, in Vector3 c, in Vector3 coord) =>
        a * coord.X + b * coord.Y + c * coord.Z;

    public static Vector3 BaryCoord(
        in Vector3 a,
        in Vector3 b,
        in Vector3 c,
        in Vector3 p)
    {
        btScalar w0 = Vector3.Cross(a - p, b - p).Length();
        btScalar w1 = Vector3.Cross(b - p, c - p).Length();
        btScalar w2 = Vector3.Cross(c - p, a - p).Length();
        btScalar isum = 1 / (w0 + w1 + w2);
        return new Vector3(w1 * isum, w2 * isum, w0 * isum);
    }

    public static btScalar ImplicitSolve(
        ImplicitFn fn,
        in Vector3 a,
        in Vector3 b,
        btScalar accuracy,
        int maxiterations = 256)
    {
        btScalar span0 = 0;
        btScalar span1 = 1;
        btScalar value0 = fn.Eval(in a);
        btScalar value1 = fn.Eval(in b);
        if (value0 > value1)
        {
            (span0, span1) = (span1, span0);
            (value0, value1) = (value1, value0);
        }
        if (value0 > -accuracy)
        {
            return -1;
        }
        if (value1 < accuracy)
        {
            return -1;
        }
        for (int i = 0; i < maxiterations; ++i)
        {
            btScalar t = Lerp(span0, span1, value0 / (value0 - value1));
            Vector3 pos = Lerp(in a, in b, t);
            btScalar v = fn.Eval(in pos);
            if (t <= 0 || t >= 1)
            {
                break;
            }
            if (BtMath.Abs(v) < accuracy)
            {
                return t;
            }
            if (v < 0)
            {
                span0 = t;
                value0 = v;
            }
            else
            {
                span1 = t;
                value1 = v;
            }
        }
        return -1;
    }

    public static void EvaluateMedium(
        SoftBodyWorldInfo wfi,
        in Vector3 x,
        out sMedium medium)
    {
        medium.velocity = Vector3.Zero;
        medium.pressure = 0;
        medium.density = wfi.airDensity;
        if (wfi.waterDensity > 0)
        {
            btScalar depth = -(Vector3.Dot(x, wfi.waterNormal) + wfi.waterOffset);
            if (depth > 0)
            {
                medium.density = wfi.waterDensity;
                medium.pressure = depth * wfi.waterDensity * wfi.gravity.Length();
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 NormalizeAny(in Vector3 v)
    {
        btScalar l = v.Length();
        if (l > BtMath.SimdEpsilon)
        {
            return v / l;
        }
        return Vector3.Zero;
    }

    public static DbvtVolume VolumeOf(Face f, btScalar margin)
    {
        Vector3[] pts = t_facePoints ??= new Vector3[3];
        pts[0] = f.n[0].x;
        pts[1] = f.n[1].x;
        pts[2] = f.n[2].x;
        DbvtVolume vol = DbvtVolume.FromPoints(pts, 3);
        vol.Expand(new Vector3(margin, margin, margin));
        return vol;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 CenterOf(Face f) =>
        (f.n[0].x + f.n[1].x + f.n[2].x) / 3;

    /// <summary>Returns |(x1 - x0) x (x2 - x0)| (twice the geometric triangle area, matching btSoftBodyInternals.h::AreaOf).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar AreaOf(in Vector3 x0, in Vector3 x1, in Vector3 x2)
    {
        Vector3 a = x1 - x0;
        Vector3 b = x2 - x0;
        Vector3 cr = Vector3.Cross(a, b);
        btScalar area = cr.Length();
        return area;
    }

    /// <summary>Returns (x1 - x0) . ((x2 - x0) x (x3 - x0)) (six times signed tetrahedron volume, matching btSoftBodyInternals.h::VolumeOf).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar VolumeOf(in Vector3 x0, in Vector3 x1, in Vector3 x2, in Vector3 x3)
    {
        Vector3 a = x1 - x0;
        Vector3 b = x2 - x0;
        Vector3 c = x3 - x0;
        return Vector3.Dot(a, Vector3.Cross(b, c));
    }

    public static void ApplyClampedForce(Node n, in Vector3 f, btScalar dt)
    {
        btScalar dtim = dt * n.im;
        if ((f * dtim).LengthSquared() > n.v.LengthSquared())
        {
            // Clamp
            n.f -= ProjectOnAxis(in n.v, f.Normalized()) / dtim;
        }
        else
        {
            // Apply
            n.f += f;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int MatchEdge(Node a, Node b, Node ma, Node mb)
    {
        if (ReferenceEquals(a, ma) && ReferenceEquals(b, mb))
        {
            return 0;
        }
        if (ReferenceEquals(a, mb) && ReferenceEquals(b, ma))
        {
            return 1;
        }
        return -1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int PolarDecompose(in Matrix3x3 m, out Matrix3x3 q, out Matrix3x3 s) =>
        (int)PolarDecomposition.PolarDecompose(in m, out q, out s);
}
