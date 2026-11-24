using System.Globalization;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace BulletSharp.Math;

/// <summary>
/// Ported from btVector3; the component type depends on the compile-time precision (btScalar).
///
/// Like btVector3 under BT_USE_SIMD_VECTOR3, the struct carries an unused fourth
/// component (<see cref="W"/>, always kept 0) so that in single-precision builds it
/// exactly fills one 128-bit SSE register — the C# analogue of Bullet's
/// union { btSimdFloat4 mVec128; btScalar m_floats[4]; } with ATTRIBUTE_ALIGNED16.
/// All operators that can use SSE do so (see BtSimd); scalar fallbacks match Bullet's
/// non-SSE branches.
/// </summary>
public struct Vector3 : IEquatable<Vector3>
{
    public btScalar X;
    public btScalar Y;
    public btScalar Z;

    /// <summary>
    /// Unused fourth component (btVector3's m_floats[3]): kept 0 to preserve the SSE
    /// lane invariant, like Bullet's setValue/masks do. Not part of equality or hashing.
    /// </summary>
    public btScalar W;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3(btScalar x, btScalar y, btScalar z)
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            this = Unsafe.BitCast<Vector128<float>, Vector3>(Vector128.Create(x, y, z, 0f));
            return;
        }
#endif
        X = x;
        Y = y;
        Z = z;
        W = 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3(btScalar value)
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            this = Unsafe.BitCast<Vector128<float>, Vector3>(Vector128.Create(value, value, value, 0f));
            return;
        }
#endif
        X = value;
        Y = value;
        Z = value;
        W = 0;
    }

    public btScalar this[int index]
    {
        readonly get => index switch
        {
            0 => X,
            1 => Y,
            2 => Z,
            _ => throw new ArgumentOutOfRangeException(nameof(index)),
        };
        set
        {
            switch (index)
            {
                case 0: X = value; break;
                case 1: Y = value; break;
                case 2: Z = value; break;
                default: throw new ArgumentOutOfRangeException(nameof(index));
            }
        }
    }

    public static readonly Vector3 Zero = new(0, 0, 0);
    public static readonly Vector3 One = new(1, 1, 1);
    public static readonly Vector3 UnitX = new(1, 0, 0);
    public static readonly Vector3 UnitY = new(0, 1, 0);
    public static readonly Vector3 UnitZ = new(0, 0, 1);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly btScalar Length() => BtMath.Sqrt(LengthSquared());

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly btScalar LengthSquared() => Dot(this);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly btScalar Distance(Vector3 v) => (v - this).Length();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly btScalar DistanceSquared(Vector3 v) => (v - this).LengthSquared();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly btScalar Dot(Vector3 v)
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            return BtSimd.Dot(
                Unsafe.BitCast<Vector3, Vector128<float>>(this),
                Unsafe.BitCast<Vector3, Vector128<float>>(v));
        }
#endif
        return X * v.X + Y * v.Y + Z * v.Z;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Dot(Vector3 a, Vector3 b) => a.Dot(b);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Vector3 Cross(Vector3 v)
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            return Unsafe.BitCast<Vector128<float>, Vector3>(BtSimd.Cross(
                Unsafe.BitCast<Vector3, Vector128<float>>(this),
                Unsafe.BitCast<Vector3, Vector128<float>>(v)));
        }
#endif
        return new Vector3(
            Y * v.Z - Z * v.Y,
            Z * v.X - X * v.Z,
            X * v.Y - Y * v.X);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 Cross(Vector3 a, Vector3 b) => a.Cross(b);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly btScalar Triple(Vector3 v1, Vector3 v2)
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            return BtSimd.Triple(
                Unsafe.BitCast<Vector3, Vector128<float>>(this),
                Unsafe.BitCast<Vector3, Vector128<float>>(v1),
                Unsafe.BitCast<Vector3, Vector128<float>>(v2));
        }
#endif
        return X * (v1.Y * v2.Z - v1.Z * v2.Y) +
            Y * (v1.Z * v2.X - v1.X * v2.Z) +
            Z * (v1.X * v2.Y - v1.Y * v2.X);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Normalize()
    {
        btScalar len = Length();
        if (len > BtMath.SimdEpsilon)
        {
#if BT_USE_SSE_IN_API && !BT_USE_DOUBLE_PRECISION
            if (Sse.IsSupported)
            {
                // btVector3::normalize SSE path: rsqrt estimate + one Newton-Raphson step
                this = Unsafe.BitCast<Vector128<float>, Vector3>(BtSimd.Normalize(Unsafe.BitCast<Vector3, Vector128<float>>(this)));
                return;
            }
#endif
            btScalar inv = 1 / len;
#if !BT_USE_DOUBLE_PRECISION
            if (Sse.IsSupported)
            {
                this = Unsafe.BitCast<Vector128<float>, Vector3>(
                    BtSimd.MultiplyScalar3(Unsafe.BitCast<Vector3, Vector128<float>>(this), inv));
                return;
            }
#endif
            X *= inv;
            Y *= inv;
            Z *= inv;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Vector3 Normalized()
    {
        Vector3 v = this;
        v.Normalize();
        return v;
    }

    /// <summary>btVector3::rotate - rotates around wAxis by angle (Rodrigues' formula).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Vector3 Rotate(Vector3 wAxis, btScalar angle)
    {
        Vector3 o = wAxis * wAxis.Dot(this);
        Vector3 x = this - o;
        Vector3 y = wAxis.Cross(this);
        return o + x * BtMath.Cos(angle) + y * BtMath.Sin(angle);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Vector3 Absolute()
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            return Unsafe.BitCast<Vector128<float>, Vector3>(
                BtSimd.Abs3(Unsafe.BitCast<Vector3, Vector128<float>>(this)));
        }
#endif
        return new Vector3(BtMath.Abs(X), BtMath.Abs(Y), BtMath.Abs(Z));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly int MinAxis() => X < Y ? (X < Z ? 0 : 2) : (Y < Z ? 1 : 2);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly int MaxAxis() => X < Y ? (Y < Z ? 2 : 1) : (X < Z ? 2 : 0);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly int FurthestAxis() => Absolute().MinAxis();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly int ClosestAxis() => Absolute().MaxAxis();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Vector3 Lerp(Vector3 v, btScalar t)
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            return Unsafe.BitCast<Vector128<float>, Vector3>(BtSimd.Lerp(
                Unsafe.BitCast<Vector3, Vector128<float>>(this),
                Unsafe.BitCast<Vector3, Vector128<float>>(v), t));
        }
#endif
        return new Vector3(
            X + (v.X - X) * t,
            Y + (v.Y - Y) * t,
            Z + (v.Z - Z) * t);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly btScalar MaxComponent() => BtMath.Max(X, BtMath.Max(Y, Z));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly btScalar MinComponent() => BtMath.Min(X, BtMath.Min(Y, Z));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetMin(Vector3 v)
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            this = Unsafe.BitCast<Vector128<float>, Vector3>(Sse.Min(
                Unsafe.BitCast<Vector3, Vector128<float>>(this),
                Unsafe.BitCast<Vector3, Vector128<float>>(v)));
            return;
        }
#endif
        if (v.X < X) X = v.X;
        if (v.Y < Y) Y = v.Y;
        if (v.Z < Z) Z = v.Z;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetMax(Vector3 v)
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            this = Unsafe.BitCast<Vector128<float>, Vector3>(Sse.Max(
                Unsafe.BitCast<Vector3, Vector128<float>>(this),
                Unsafe.BitCast<Vector3, Vector128<float>>(v)));
            return;
        }
#endif
        if (v.X > X) X = v.X;
        if (v.Y > Y) Y = v.Y;
        if (v.Z > Z) Z = v.Z;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Vector3 Min(Vector3 v)
    {
        Vector3 r = this;
        r.SetMin(v);
        return r;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Vector3 Max(Vector3 v)
    {
        Vector3 r = this;
        r.SetMax(v);
        return r;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetValue(btScalar x, btScalar y, btScalar z)
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            this = Unsafe.BitCast<Vector128<float>, Vector3>(Vector128.Create(x, y, z, 0f));
            return;
        }
#endif
        X = x;
        Y = y;
        Z = z;
        W = 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetZero()
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            this = Unsafe.BitCast<Vector128<float>, Vector3>(Vector128<float>.Zero);
            return;
        }
#endif
        X = 0;
        Y = 0;
        Z = 0;
        W = 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool IsZero() => X == 0 && Y == 0 && Z == 0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool FuzzyZero() => LengthSquared() < BtMath.SimdEpsilon * BtMath.SimdEpsilon;

    // -------- Operators --------
    public static Vector3 operator +(Vector3 a, Vector3 b)
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            return Unsafe.BitCast<Vector128<float>, Vector3>(Sse.Add(
                Unsafe.BitCast<Vector3, Vector128<float>>(a),
                Unsafe.BitCast<Vector3, Vector128<float>>(b)));
        }
#endif
        return new Vector3(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
    }

    public static Vector3 operator -(Vector3 a, Vector3 b)
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            return Unsafe.BitCast<Vector128<float>, Vector3>(BtSimd.Subtract3(
                Unsafe.BitCast<Vector3, Vector128<float>>(a),
                Unsafe.BitCast<Vector3, Vector128<float>>(b)));
        }
#endif
        return new Vector3(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
    }

    public static Vector3 operator -(Vector3 v)
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            return Unsafe.BitCast<Vector128<float>, Vector3>(BtSimd.Negate3(
                Unsafe.BitCast<Vector3, Vector128<float>>(v)));
        }
#endif
        return new Vector3(-v.X, -v.Y, -v.Z);
    }

    public static Vector3 operator *(Vector3 v, btScalar s)
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            return Unsafe.BitCast<Vector128<float>, Vector3>(BtSimd.MultiplyScalar3(
                Unsafe.BitCast<Vector3, Vector128<float>>(v), s));
        }
#endif
        return new Vector3(v.X * s, v.Y * s, v.Z * s);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 operator *(btScalar s, Vector3 v) => v * s;

    public static Vector3 operator *(Vector3 a, Vector3 b)
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            return Unsafe.BitCast<Vector128<float>, Vector3>(Sse.Multiply(
                Unsafe.BitCast<Vector3, Vector128<float>>(a),
                Unsafe.BitCast<Vector3, Vector128<float>>(b)));
        }
#endif
        return new Vector3(a.X * b.X, a.Y * b.Y, a.Z * b.Z);
    }

    public static Vector3 operator /(Vector3 v, btScalar s)
    {
        btScalar inv = 1 / s;
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            return Unsafe.BitCast<Vector128<float>, Vector3>(BtSimd.MultiplyScalar3(
                Unsafe.BitCast<Vector3, Vector128<float>>(v), inv));
        }
#endif
        return new Vector3(v.X * inv, v.Y * inv, v.Z * inv);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 operator /(Vector3 a, Vector3 b) => new(a.X / b.X, a.Y / b.Y, a.Z / b.Z);

    public static bool operator ==(Vector3 a, Vector3 b) => a.X == b.X && a.Y == b.Y && a.Z == b.Z;
    public static bool operator !=(Vector3 a, Vector3 b) => !(a == b);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool Equals(Vector3 other) => this == other;
    public override readonly bool Equals(object obj) => obj is Vector3 v && Equals(v);
    public override readonly int GetHashCode() => HashCode.Combine(X, Y, Z);

    public override readonly string ToString() =>
        string.Format(CultureInfo.InvariantCulture, "({0}, {1}, {2})", X, Y, Z);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal readonly void SerializeOut(out btScalar x, out btScalar y, out btScalar z)
    {
        x = X; y = Y; z = Z;
    }
}
