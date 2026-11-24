using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace BulletSharp.Math;

/// <summary>
/// Port of Bullet's SSE infrastructure (the BT_USE_SSE sections of btVector3.h /
/// btQuaternion.h) on top of System.Runtime.Intrinsics.
///
/// Bullet only enables these paths for single precision on x86/x64 (BT_USE_SSE is
/// undefined for BT_USE_DOUBLE_PRECISION, and ARM uses NEON instead), so this class
/// exists only in float builds; call sites fall back to the scalar code otherwise
/// (the #else branches of the original C++).
///
/// Numerical fidelity: the operations marked "bit-exact" produce bitwise-identical
/// results to the scalar fallback (same IEEE roundings in the same order), so they are
/// enabled whenever SSE is available. The operations that reorder or approximate
/// arithmetic (rsqrt-based normalize, the SSE quaternion product and dot) are gated
/// behind the BT_USE_SSE_IN_API compilation symbol, mirroring Bullet's own opt-in
/// switch, because they change results in the last ulp.
///
/// Shuffle controls are BT_SHUFFLE(x, y, z, w) = (w &lt;&lt; 6 | z &lt;&lt; 4 | y &lt;&lt; 2 | x) &amp; 0xff,
/// written as literals so they stay compile-time constants, e.g.
/// BT_SHUFFLE(1, 2, 0, 3) = 0xC9 = the (Y Z X 0) lane permutation.
/// </summary>
#if !BT_USE_DOUBLE_PRECISION
internal static class BtSimd
{
    /// <summary>True when the BT_USE_SSE paths can run (x86/x64, single precision).</summary>
    public static bool IsSupported => Sse.IsSupported;

    // ------------------------------------------------------------------
    // Constant masks (btVector3.h / btQuaternion.h; _mm_set_epi32 takes the
    // highest lane first, Vector128.Create the lowest, so these read reversed)
    // ------------------------------------------------------------------

    /// <summary>btv3AbsiMask / btv3AbsfMask: (0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0)</summary>
    public static readonly Vector128<float> Btv3AbsfMask =
        Vector128.Create(0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x00000000).AsSingle();

    /// <summary>btvFFF0iMask / btvFFF0fMask: (0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0)</summary>
    public static readonly Vector128<float> BtvFFF0fMask =
        Vector128.Create(unchecked((int)0xFFFFFFFF), unchecked((int)0xFFFFFFFF), unchecked((int)0xFFFFFFFF), 0x00000000).AsSingle();

    /// <summary>btvMzeroMask: (-0.0f, -0.0f, -0.0f, -0.0f) — flip all sign bits</summary>
    public static readonly Vector128<float> BtvMzeroMask = Vector128.Create(-0.0f, -0.0f, -0.0f, -0.0f);

    /// <summary>v1110: (1, 1, 1, 0)</summary>
    public static readonly Vector128<float> V1110 = Vector128.Create(1.0f, 1.0f, 1.0f, 0.0f);

    /// <summary>vHalf: (0.5, 0.5, 0.5, 0.5)</summary>
    public static readonly Vector128<float> VHalf = Vector128.Create(0.5f, 0.5f, 0.5f, 0.5f);

    /// <summary>v1_5: (1.5, 1.5, 1.5, 1.5)</summary>
    public static readonly Vector128<float> V1_5 = Vector128.Create(1.5f, 1.5f, 1.5f, 1.5f);

    /// <summary>vOnes (btQuaternion.h): (1, 1, 1, 1)</summary>
    public static readonly Vector128<float> VOnes = Vector128.Create(1.0f, 1.0f, 1.0f, 1.0f);

    /// <summary>vQInv (btQuaternion.h): (-0, -0, -0, +0) — negate x/y/z, keep w (inverse)</summary>
    public static readonly Vector128<float> VQInv = Vector128.Create(-0.0f, -0.0f, -0.0f, 0.0f);

    /// <summary>vPPPM (btQuaternion.h): (+0, +0, +0, -0) — negate the last element (w)</summary>
    public static readonly Vector128<float> VPPPM = Vector128.Create(0.0f, 0.0f, 0.0f, -0.0f);

    // ------------------------------------------------------------------
    // Vector reinterpretation — the C# analogue of btVector3's
    // union { btSimdFloat4 mVec128; btScalar m_floats[4]; }
    // (Vector3/Quaternion keep their unused w lane at 0 so the bits always match.)
    // ------------------------------------------------------------------

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> LoadReadonly(in Vector3 v) =>
        Unsafe.As<Vector3, Vector128<float>>(ref Unsafe.AsRef(in v));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> Load(ref Vector3 v) =>
        Unsafe.As<Vector3, Vector128<float>>(ref v);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> LoadReadonly(in Quaternion q) =>
        Unsafe.As<Quaternion, Vector128<float>>(ref Unsafe.AsRef(in q));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> Load(ref Quaternion q) =>
        Unsafe.As<Quaternion, Vector128<float>>(ref q);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Store(ref Vector3 v, Vector128<float> r) =>
        Unsafe.As<Vector3, Vector128<float>>(ref v) = r;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Store(ref Quaternion q, Vector128<float> r) =>
        Unsafe.As<Quaternion, Vector128<float>>(ref q) = r;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 ToVector3(Vector128<float> v) =>
        Unsafe.As<Vector128<float>, Vector3>(ref v);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Quaternion ToQuaternion(Vector128<float> v) =>
        Unsafe.As<Vector128<float>, Quaternion>(ref v);

    // ------------------------------------------------------------------
    // btVector3 SSE operations
    // ------------------------------------------------------------------

    /// <summary>
    /// btVector3::dot — horizontal add of lanes 0..2. Bit-exact with the scalar
    /// x*x + y*y + z*z (same (x+y)+z rounding order).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Dot(Vector128<float> a, Vector128<float> b)
    {
        Vector128<float> vd = Sse.Multiply(a, b);
        Vector128<float> z = Sse.MoveHighToLow(vd, vd);
        Vector128<float> y = Sse.Shuffle(vd, vd, 0x55);
        vd = Sse.AddScalar(vd, y);
        vd = Sse.AddScalar(vd, z);
        return vd.GetElement(0);
    }

    /// <summary>btVector3::cross — (Y Z X 0) shuffle form. Bit-exact with the scalar formula.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> Cross(Vector128<float> a, Vector128<float> b)
    {
        Vector128<float> t = Sse.Shuffle(a, a, 0xC9);  // BT_SHUFFLE(1, 2, 0, 3): (Y Z X 0)
        Vector128<float> v = Sse.Shuffle(b, b, 0xC9);  // (Y Z X 0)

        v = Sse.Multiply(v, a);
        t = Sse.Multiply(t, b);
        v = Sse.Subtract(v, t);

        return Sse.Shuffle(v, v, 0xC9);
    }

    /// <summary>btVector3::triple — this . (v1 x v2). Bit-exact with the scalar formula.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Triple(Vector128<float> a, Vector128<float> v1, Vector128<float> v2)
    {
        // cross:
        Vector128<float> t = Sse.Shuffle(v1, v1, 0xC9);  // BT_SHUFFLE(1, 2, 0, 3): (Y Z X 0)
        Vector128<float> v = Sse.Shuffle(v2, v2, 0xC9);  // (Y Z X 0)

        v = Sse.Multiply(v, v1);
        t = Sse.Multiply(t, v2);
        v = Sse.Subtract(v, t);

        v = Sse.Shuffle(v, v, 0xC9);

        // dot:
        v = Sse.Multiply(v, a);
        Vector128<float> z = Sse.MoveHighToLow(v, v);
        Vector128<float> y = Sse.Shuffle(v, v, 0x55);
        v = Sse.AddScalar(v, y);
        v = Sse.AddScalar(v, z);
        return v.GetElement(0);
    }

    /// <summary>btVector3::absolute — and with btv3AbsfMask (also zeroes the w lane).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> Abs3(Vector128<float> v) => Sse.And(v, Btv3AbsfMask);

    /// <summary>operator- on btVector3 — xor with btvMzeroMask.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> Negate3(Vector128<float> v) => Sse.Xor(v, BtvMzeroMask);

    /// <summary>operator- (v1 - v2) on btVector3 — sub, then and with btvFFF0fMask.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> Subtract3(Vector128<float> a, Vector128<float> b) =>
        Sse.And(Sse.Subtract(a, b), BtvFFF0fMask);

    /// <summary>btVector3 operator*= (scalar) — splat to (S S S 0) so w stays zero.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> MultiplyScalar3(Vector128<float> v, float s)
    {
        Vector128<float> vs = Vector128.CreateScalar(s);   // (S 0 0 0)
        vs = Sse.Shuffle(vs, vs, 0x80);                    // (S S S 0.0)
        return Sse.Multiply(v, vs);
    }

    /// <summary>btVector3::lerp — v + (v1 - v) * t (w lane stays 0).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> Lerp(Vector128<float> v, Vector128<float> v1, float t)
    {
        Vector128<float> vt = Vector128.CreateScalar(t);   // (t 0 0 0)
        vt = Sse.Shuffle(vt, vt, 0x80);                    // (t t t 0.0)
        Vector128<float> vl = Sse.Subtract(v1, v);
        vl = Sse.Multiply(vl, vt);
        return Sse.Add(vl, v);
    }

    /// <summary>
    /// btVector3::normalize (BT_USE_SSE path) — reciprocal-sqrt estimate plus one
    /// Newton-Raphson step, multiplied through all lanes. Approximate: differs from
    /// the exact 1/sqrt in the last ulp (BT_USE_SSE_IN_API only).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> Normalize(Vector128<float> v)
    {
        // dot product first
        Vector128<float> vd = Sse.Multiply(v, v);
        Vector128<float> z = Sse.MoveHighToLow(vd, vd);
        Vector128<float> y = Sse.Shuffle(vd, vd, 0x55);
        vd = Sse.AddScalar(vd, y);
        vd = Sse.AddScalar(vd, z);

        // NR step 1/sqrt(x) - vd is x, y is output
        y = Sse.ReciprocalSqrtScalar(vd);  // estimate

        // one step NR
        z = V1_5;
        vd = Sse.MultiplyScalar(vd, VHalf);  // vd * 0.5
        vd = Sse.MultiplyScalar(vd, y);      // vd * 0.5 * y0
        vd = Sse.MultiplyScalar(vd, y);      // vd * 0.5 * y0 * y0
        z = Sse.SubtractScalar(z, vd);       // 1.5 - vd * 0.5 * y0 * y0

        y = Sse.MultiplyScalar(y, z);        // y0 * (1.5 - vd * 0.5 * y0 * y0)

        y = Sse.Shuffle(y, y, 0x80);         // bt_splat_ps(y, 0x80)
        return Sse.Multiply(v, y);
    }

    // ------------------------------------------------------------------
    // btQuaternion SSE operations
    // ------------------------------------------------------------------

    /// <summary>btQuaternion::dot — horizontal add of all four lanes (BT_USE_SSE_IN_API).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Dot4(Vector128<float> a, Vector128<float> b)
    {
        Vector128<float> vd = Sse.Multiply(a, b);

        Vector128<float> t = Sse.MoveHighToLow(vd, vd);
        vd = Sse.Add(vd, t);
        t = Sse.Shuffle(vd, vd, 0x55);
        vd = Sse.AddScalar(vd, t);

        return vd.GetElement(0);
    }

    /// <summary>
    /// btQuaternion multiplication (operator*= SSE path). The lane grouping differs
    /// from the scalar formula, so results may differ in the last ulp
    /// (BT_USE_SSE_IN_API only).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> MultiplyQuaternion(Vector128<float> a, Vector128<float> b)
    {
        Vector128<float> a1 = Sse.Shuffle(a, a, 0x24);  // BT_SHUFFLE(0, 1, 2, 0)
        Vector128<float> b1 = Sse.Shuffle(b, b, 0x3F);  // BT_SHUFFLE(3, 3, 3, 0): (W W W X)

        a1 = Sse.Multiply(a1, b1);

        Vector128<float> a2 = Sse.Shuffle(a, a, 0x49);  // BT_SHUFFLE(1, 2, 0, 1)
        Vector128<float> b2 = Sse.Shuffle(b, b, 0x52);  // BT_SHUFFLE(2, 0, 1, 1)

        a2 = Sse.Multiply(a2, b2);

        b1 = Sse.Shuffle(a, a, 0x92);  // BT_SHUFFLE(2, 0, 1, 2)
        b2 = Sse.Shuffle(b, b, 0x89);  // BT_SHUFFLE(1, 2, 0, 2)

        b1 = Sse.Multiply(b1, b2);              // A3 *= B3

        Vector128<float> r = Sse.Shuffle(a, a, 0xFF);  // BT_SHUFFLE(3, 3, 3, 3): splat w (A0)
        r = Sse.Multiply(r, b);                 // A0 * B0

        a1 = Sse.Add(a1, a2);                   // AB12
        r = Sse.Subtract(r, b1);                // AB03 = AB0 - AB3
        a1 = Sse.Xor(a1, VPPPM);                // change sign of the last element
        r = Sse.Add(r, a1);                     // AB03 + AB12

        return r;
    }

    /// <summary>
    /// btQuaternion::normalize (SSE path) — 1/sqrt(len2) splatted and multiplied
    /// through (BT_USE_SSE_IN_API; depends on the SSE dot grouping).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> NormalizeQuaternion(Vector128<float> q)
    {
        Vector128<float> vd = Sse.Multiply(q, q);

        Vector128<float> t = Sse.MoveHighToLow(vd, vd);
        vd = Sse.Add(vd, t);
        t = Sse.Shuffle(vd, vd, 0x55);
        vd = Sse.AddScalar(vd, t);

        vd = Sse.SqrtScalar(vd);
        vd = Sse.DivideScalar(VOnes, vd);
        vd = Sse.Shuffle(vd, vd, 0x00);         // bt_pshufd_ps(vd, 0): splat
        return Sse.Multiply(q, vd);
    }

    /// <summary>btQuaternion operator*= (scalar) — splat to (S S S S).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> MultiplyScalar4(Vector128<float> q, float s)
    {
        Vector128<float> vs = Vector128.CreateScalar(s);   // (S 0 0 0)
        vs = Sse.Shuffle(vs, vs, 0x00);                    // (S S S S)
        return Sse.Multiply(q, vs);
    }
}
#endif
