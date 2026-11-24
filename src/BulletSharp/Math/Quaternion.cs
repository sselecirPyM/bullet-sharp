using System.Globalization;
using System.Runtime.Intrinsics.X86;

namespace BulletSharp.Math;

/// <summary>
/// Ported from btQuaternion; the component type depends on the compile-time precision (btScalar).
/// In single-precision builds the four components exactly fill one SSE register, so the
/// BT_USE_SSE paths of btQuaternion.h are ported via BtSimd (see Vector3 as well).
/// </summary>
public struct Quaternion : IEquatable<Quaternion>
{
    public btScalar X;
    public btScalar Y;
    public btScalar Z;
    public btScalar W;

    public Quaternion(btScalar x, btScalar y, btScalar z, btScalar w)
    {
        X = x;
        Y = y;
        Z = z;
        W = w;
    }

    /// <summary>Axis-angle construction (btQuaternion(axis, angle)).</summary>
    public Quaternion(Vector3 axis, btScalar angle)
    {
        btScalar d = axis.Length();
        btScalar s = BtMath.Sin(angle / 2) / d;
        X = axis.X * s;
        Y = axis.Y * s;
        Z = axis.Z * s;
        W = BtMath.Cos(angle / 2);
    }

    public Quaternion(Vector3 axisX, Vector3 axisY, Vector3 axisZ)
    {
        var m = new Matrix3x3(axisX, axisY, axisZ);
        this = m.GetRotation();
    }

    public static readonly Quaternion Identity = new(0, 0, 0, 1);

    public readonly btScalar Length() => BtMath.Sqrt(LengthSquared());

    public readonly btScalar LengthSquared() => Dot(this);

    public readonly btScalar Dot(Quaternion q)
    {
#if BT_USE_SSE_IN_API && !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            return BtSimd.Dot4(BtSimd.LoadReadonly(in this), BtSimd.Load(ref q));
        }
#endif
        return X * q.X + Y * q.Y + Z * q.Z + W * q.W;
    }

    public static btScalar Dot(Quaternion a, Quaternion b) => a.Dot(b);

    public void Normalize()
    {
        btScalar len = Length();
        if (len > BtMath.SimdEpsilon)
        {
#if BT_USE_SSE_IN_API && !BT_USE_DOUBLE_PRECISION
            if (Sse.IsSupported)
            {
                // btQuaternion::normalize SSE path: splat 1/sqrt(len2) and multiply through
                BtSimd.Store(ref this, BtSimd.NormalizeQuaternion(BtSimd.Load(ref this)));
                return;
            }
#endif
            btScalar inv = 1 / len;
            X *= inv;
            Y *= inv;
            Z *= inv;
            W *= inv;
        }
    }

    public readonly Quaternion Normalized()
    {
        Quaternion q = this;
        q.Normalize();
        return q;
    }

    public readonly btScalar Angle(Quaternion q)
    {
        btScalar s = BtMath.Sqrt(LengthSquared() * q.LengthSquared());
        return BtMath.Acos(Dot(q) / s);
    }

    public readonly btScalar AngleShortestPath(Quaternion q)
    {
        btScalar s = BtMath.Sqrt(LengthSquared() * q.LengthSquared());
        btScalar d = Dot(q);
        if (d < 0)
            return BtMath.Acos(-d / s) * 2;
        return BtMath.Acos(d / s) * 2;
    }

    /// <summary>btQuaternion::getAngle</summary>
    public readonly btScalar GetAngle()
    {
        return 2 * BtMath.Acos(W);
    }

    /// <summary>btQuaternion::getAxis</summary>
    public readonly Vector3 GetAxis()
    {
        btScalar sSquared = 1 - W * W;
        if (sSquared < 10 * BtMath.SimdEpsilon)
            return new Vector3(1, 0, 0);
        btScalar s = 1 / BtMath.Sqrt(sSquared);
        return new Vector3(X * s, Y * s, Z * s);
    }

    public readonly Quaternion Inverse()
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            // btQuaternion::inverse: xor with vQInv (-0, -0, -0, +0) flips x/y/z signs
            return BtSimd.ToQuaternion(Sse.Xor(BtSimd.LoadReadonly(in this), BtSimd.VQInv));
        }
#endif
        return new Quaternion(-X, -Y, -Z, W);
    }

    public readonly Quaternion Farthest(Quaternion qd)
    {
        Quaternion diff = this - qd;
        Quaternion sum = this + qd;
        return diff.Dot(diff) > sum.Dot(sum) ? qd : -qd;
    }

    public readonly Quaternion Nearest(Quaternion qd)
    {
        Quaternion diff = this - qd;
        Quaternion sum = this + qd;
        return diff.Dot(diff) < sum.Dot(sum) ? qd : -qd;
    }

    /// <summary>btSlerp</summary>
    public static Quaternion Slerp(Quaternion q1, Quaternion q2, btScalar t)
    {
        btScalar theta = q1.AngleShortestPath(q2) / 2;
        if (theta != 0)
        {
            btScalar d = 1 / BtMath.Sin(theta);
            btScalar s0 = BtMath.Sin((1 - t) * theta);
            btScalar s1 = BtMath.Sin(t * theta);
            btScalar sign = q1.Dot(q2) < 0 ? -1 : 1;
            return new Quaternion(
                (q1.X * s0 + q2.X * s1 * sign) * d,
                (q1.Y * s0 + q2.Y * s1 * sign) * d,
                (q1.Z * s0 + q2.Z * s1 * sign) * d,
                (q1.W * s0 + q2.W * s1 * sign) * d);
        }
        return q1;
    }

    public static Quaternion FromAxisAngle(Vector3 axis, btScalar angle) => new(axis, angle);

    /// <summary>Inverse of btMatrix3x3::getRotation (constructs from a rotation matrix).</summary>
    public static Quaternion FromRotationMatrix(in Matrix3x3 m) => m.GetRotation();

    public static Quaternion FromMatrix(in Matrix m) => new Matrix3x3(in m).GetRotation();

    /// <summary>btQuatRotate: rotates a vector by a quaternion.</summary>
    public static Vector3 Rotate(Quaternion rotation, Vector3 v)
    {
        Quaternion q = new(
            rotation.W * v.X + rotation.Y * v.Z - rotation.Z * v.Y,
            rotation.W * v.Y + rotation.Z * v.X - rotation.X * v.Z,
            rotation.W * v.Z + rotation.X * v.Y - rotation.Y * v.X,
            -rotation.X * v.X - rotation.Y * v.Y - rotation.Z * v.Z);
        return new Vector3(
            q.W * -rotation.X + q.X * rotation.W - q.Y * rotation.Z + q.Z * rotation.Y,
            q.W * -rotation.Y + q.Y * rotation.W - q.Z * rotation.X + q.X * rotation.Z,
            q.W * -rotation.Z + q.Z * rotation.W - q.X * rotation.Y + q.Y * rotation.X);
    }

    /// <summary>btShortestArcQuat</summary>
    public static Quaternion ShortestArc(Vector3 v0, Vector3 v1)
    {
        v0.Normalize();
        v1.Normalize();
        Vector3 c = v0.Cross(v1);
        btScalar d = v0.Dot(v1);
        if (d < -1 + BtMath.SimdEpsilon)
        {
            // Opposite directions: pick any orthogonal axis and rotate 180 degrees.
            Vector3 n;
            BtMath.PlaneSpace1(v0, out n, out _);
            return new Quaternion(n.X, n.Y, n.Z, 0);
        }
        btScalar s = BtMath.Sqrt((1 + d) * 2);
        btScalar rs = 1 / s;
        return new Quaternion(c.X * rs, c.Y * rs, c.Z * rs, s / 2);
    }

    // -------- Operators --------
    public static Quaternion operator +(Quaternion a, Quaternion b)
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            return BtSimd.ToQuaternion(Sse.Add(BtSimd.Load(ref a), BtSimd.Load(ref b)));
        }
#endif
        return new Quaternion(a.X + b.X, a.Y + b.Y, a.Z + b.Z, a.W + b.W);
    }

    public static Quaternion operator -(Quaternion a, Quaternion b)
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            return BtSimd.ToQuaternion(Sse.Subtract(BtSimd.Load(ref a), BtSimd.Load(ref b)));
        }
#endif
        return new Quaternion(a.X - b.X, a.Y - b.Y, a.Z - b.Z, a.W - b.W);
    }

    public static Quaternion operator -(Quaternion q)
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            return BtSimd.ToQuaternion(Sse.Xor(BtSimd.Load(ref q), BtSimd.BtvMzeroMask));
        }
#endif
        return new Quaternion(-q.X, -q.Y, -q.Z, -q.W);
    }

    public static Quaternion operator *(Quaternion q, btScalar s)
    {
#if !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            return BtSimd.ToQuaternion(BtSimd.MultiplyScalar4(BtSimd.Load(ref q), s));
        }
#endif
        return new Quaternion(q.X * s, q.Y * s, q.Z * s, q.W * s);
    }

    public static Quaternion operator *(btScalar s, Quaternion q) => q * s;

    public static Quaternion operator /(Quaternion q, btScalar s) => q * (1 / s);

    public static Quaternion operator *(Quaternion a, Quaternion b)
    {
#if BT_USE_SSE_IN_API && !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            return BtSimd.ToQuaternion(BtSimd.MultiplyQuaternion(BtSimd.Load(ref a), BtSimd.Load(ref b)));
        }
#endif
        return new Quaternion(
            a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
            a.W * b.Y + a.Y * b.W + a.Z * b.X - a.X * b.Z,
            a.W * b.Z + a.Z * b.W + a.X * b.Y - a.Y * b.X,
            a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z);
    }

    /// <summary>btQuaternion's q * v (returns a quaternion product).</summary>
    public static Quaternion operator *(Quaternion q, Vector3 w)
    {
#if BT_USE_SSE_IN_API && !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            // The C++ version masks the vector's w lane to 0 and runs the quaternion product
            return BtSimd.ToQuaternion(BtSimd.MultiplyQuaternion(
                BtSimd.Load(ref q), Sse.And(BtSimd.Load(ref w), BtSimd.BtvFFF0fMask)));
        }
#endif
        return new Quaternion(
            q.W * w.X + q.Y * w.Z - q.Z * w.Y,
            q.W * w.Y + q.Z * w.X - q.X * w.Z,
            q.W * w.Z + q.X * w.Y - q.Y * w.X,
            -q.X * w.X - q.Y * w.Y - q.Z * w.Z);
    }

    public static Quaternion operator *(Vector3 w, Quaternion q)
    {
#if BT_USE_SSE_IN_API && !BT_USE_DOUBLE_PRECISION
        if (Sse.IsSupported)
        {
            return BtSimd.ToQuaternion(BtSimd.MultiplyQuaternion(
                Sse.And(BtSimd.Load(ref w), BtSimd.BtvFFF0fMask), BtSimd.Load(ref q)));
        }
#endif
        return new Quaternion(
            w.X * q.W + w.Y * q.Z - w.Z * q.Y,
            w.Y * q.W + w.Z * q.X - w.X * q.Z,
            w.Z * q.W + w.X * q.Y - w.Y * q.X,
            -w.X * q.X - w.Y * q.Y - w.Z * q.Z);
    }

    public static bool operator ==(Quaternion a, Quaternion b) =>
        a.X == b.X && a.Y == b.Y && a.Z == b.Z && a.W == b.W;

    public static bool operator !=(Quaternion a, Quaternion b) => !(a == b);

    public readonly bool Equals(Quaternion other) => this == other;
    public override readonly bool Equals(object obj) => obj is Quaternion q && Equals(q);
    public override readonly int GetHashCode() => HashCode.Combine(X, Y, Z, W);

    public override readonly string ToString() =>
        string.Format(CultureInfo.InvariantCulture, "({0}, {1}, {2}, {3})", X, Y, Z, W);
}
