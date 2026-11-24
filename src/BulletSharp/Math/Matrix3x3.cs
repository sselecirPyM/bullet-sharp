using System.Runtime.CompilerServices;

namespace BulletSharp.Math;

/// <summary>
/// Ported from btMatrix3x3 (row-major, rows are basis vectors). For internal engine use.
/// </summary>
public struct Matrix3x3
{
    public Vector3 Row0;
    public Vector3 Row1;
    public Vector3 Row2;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Matrix3x3(Vector3 row0, Vector3 row1, Vector3 row2)
    {
        Row0 = row0;
        Row1 = row1;
        Row2 = row2;
    }

    public Matrix3x3(
        btScalar xx, btScalar xy, btScalar xz,
        btScalar yx, btScalar yy, btScalar yz,
        btScalar zx, btScalar zy, btScalar zz)
    {
        Row0 = new Vector3(xx, xy, xz);
        Row1 = new Vector3(yx, yy, yz);
        Row2 = new Vector3(zx, zy, zz);
    }

    /// <summary>Constructs from a quaternion (btMatrix3x3::setRotation).</summary>
    public Matrix3x3(Quaternion q)
    {
        btScalar d = q.LengthSquared();
        btScalar s = 2 / d;
        btScalar xs = q.X * s, ys = q.Y * s, zs = q.Z * s;
        btScalar wx = q.W * xs, wy = q.W * ys, wz = q.W * zs;
        btScalar xx = q.X * xs, xy = q.X * ys, xz = q.X * zs;
        btScalar yy = q.Y * ys, yz = q.Y * zs, zz = q.Z * zs;
        Row0 = new Vector3(1 - (yy + zz), xy - wz, xz + wy);
        Row1 = new Vector3(xy + wz, 1 - (xx + zz), yz - wx);
        Row2 = new Vector3(xz - wy, yz + wx, 1 - (xx + yy));
    }

    /// <summary>Extracts the 3x3 basis from the public Matrix (BulletSharp.Matrix uses a transposed layout).</summary>
    public Matrix3x3(in Matrix m)
    {
        Row0 = new Vector3(m.M11, m.M21, m.M31);
        Row1 = new Vector3(m.M12, m.M22, m.M32);
        Row2 = new Vector3(m.M13, m.M23, m.M33);
    }

    public static readonly Matrix3x3 Identity = new(
        new Vector3(1, 0, 0),
        new Vector3(0, 1, 0),
        new Vector3(0, 0, 1));

    public btScalar this[int row, int col]
    {
        readonly get => row switch
        {
            0 => Row0[col],
            1 => Row1[col],
            _ => Row2[col],
        };
        set
        {
            switch (row)
            {
                case 0: Row0[col] = value; break;
                case 1: Row1[col] = value; break;
                default: Row2[col] = value; break;
            }
        }
    }

    public readonly Vector3 Row(int i) => i switch
    {
        0 => Row0,
        1 => Row1,
        _ => Row2,
    };

    public readonly Vector3 Column(int i) => new(Row0[i], Row1[i], Row2[i]);

    public void SetIdentity()
    {
        Row0 = new Vector3(1, 0, 0);
        Row1 = new Vector3(0, 1, 0);
        Row2 = new Vector3(0, 0, 1);
    }

    /// <summary>btMatrix3x3::getRotation</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Quaternion GetRotation()
    {
        btScalar m00 = Row0.X, m01 = Row0.Y, m02 = Row0.Z;
        btScalar m10 = Row1.X, m11 = Row1.Y, m12 = Row1.Z;
        btScalar m20 = Row2.X, m21 = Row2.Y, m22 = Row2.Z;

        btScalar trace = m00 + m11 + m22;

        btScalar x, y, z, w;
        if (trace > 0)
        {
            btScalar s = BtMath.Sqrt(trace + 1);
            w = s * (btScalar)0.5;
            s = (btScalar)0.5 / s;
            x = (m21 - m12) * s;
            y = (m02 - m20) * s;
            z = (m10 - m01) * s;
        }
        else
        {
            // i = 对角元最大者的下标（平局规则与 C++ 一致）
            int i = m00 < m11
                ? (m11 < m22 ? 2 : 1)
                : (m00 < m22 ? 2 : 0);

            // 无堆分配版本（原 C++ 使用栈上 btScalar[9]/btScalar[4]）
            switch (i)
            {
                case 0: // j = 1, k = 2
                {
                    btScalar s = BtMath.Sqrt(m00 - m11 - m22 + 1);
                    x = s * (btScalar)0.5;
                    s = (btScalar)0.5 / s;
                    y = (m10 + m01) * s;
                    z = (m20 + m02) * s;
                    w = (m21 - m12) * s;
                    break;
                }
                case 1: // j = 2, k = 0
                {
                    btScalar s = BtMath.Sqrt(m11 - m22 - m00 + 1);
                    y = s * (btScalar)0.5;
                    s = (btScalar)0.5 / s;
                    z = (m21 + m12) * s;
                    x = (m01 + m10) * s;
                    w = (m02 - m20) * s;
                    break;
                }
                default: // i = 2, j = 0, k = 1
                {
                    btScalar s = BtMath.Sqrt(m22 - m00 - m11 + 1);
                    z = s * (btScalar)0.5;
                    s = (btScalar)0.5 / s;
                    x = (m02 + m20) * s;
                    y = (m12 + m21) * s;
                    w = (m10 - m01) * s;
                    break;
                }
            }
        }
        return new Quaternion(x, y, z, w);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Matrix3x3 Transpose() => new(
        new Vector3(Row0.X, Row1.X, Row2.X),
        new Vector3(Row0.Y, Row1.Y, Row2.Y),
        new Vector3(Row0.Z, Row1.Z, Row2.Z));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly btScalar Determinant() => Row0.Triple(Row1, Row2);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly btScalar Cofac(int r1, int c1, int r2, int c2) =>
        this[r1, c1] * this[r2, c2] - this[r1, c2] * this[r2, c1];

    /// <summary>btMatrix3x3::inverse (adjugate / determinant)</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Matrix3x3 Inverse()
    {
        Vector3 co = new(Cofac(1, 1, 2, 2), Cofac(1, 2, 2, 0), Cofac(1, 0, 2, 1));
        btScalar det = Row0.Dot(co);
        btScalar s = 1 / det;
        return new Matrix3x3(
            co.X * s, Cofac(0, 2, 2, 1) * s, Cofac(0, 1, 1, 2) * s,
            co.Y * s, Cofac(0, 0, 2, 2) * s, Cofac(0, 2, 1, 0) * s,
            co.Z * s, Cofac(0, 1, 2, 0) * s, Cofac(0, 0, 1, 1) * s);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Matrix3x3 Absolute() => new(Row0.Absolute(), Row1.Absolute(), Row2.Absolute());

    /// <summary>btMatrix3x3::scaled（按列缩放：result[i][j] = m[i][j] * s[j]）</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Matrix3x3 Scaled(Vector3 s) => new(
        Row0.X * s.X, Row0.Y * s.Y, Row0.Z * s.Z,
        Row1.X * s.X, Row1.Y * s.Y, Row1.Z * s.Z,
        Row2.X * s.X, Row2.Y * s.Y, Row2.Z * s.Z);

    /// <summary>Dot product of the X column with v (btMatrix3x3::tdotx).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly btScalar TDotX(Vector3 v) => Row0.X * v.X + Row1.X * v.Y + Row2.X * v.Z;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly btScalar TDotY(Vector3 v) => Row0.Y * v.X + Row1.Y * v.Y + Row2.Y * v.Z;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly btScalar TDotZ(Vector3 v) => Row0.Z * v.X + Row1.Z * v.Y + Row2.Z * v.Z;

    /// <summary>this * m.Transpose() (btMatrix3x3::transposeTimes).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Matrix3x3 TransposeTimes(in Matrix3x3 m) => new(
        Row0.X * m.Row0.X + Row0.Y * m.Row0.Y + Row0.Z * m.Row0.Z,
        Row0.X * m.Row1.X + Row0.Y * m.Row1.Y + Row0.Z * m.Row1.Z,
        Row0.X * m.Row2.X + Row0.Y * m.Row2.Y + Row0.Z * m.Row2.Z,
        Row1.X * m.Row0.X + Row1.Y * m.Row0.Y + Row1.Z * m.Row0.Z,
        Row1.X * m.Row1.X + Row1.Y * m.Row1.Y + Row1.Z * m.Row1.Z,
        Row1.X * m.Row2.X + Row1.Y * m.Row2.Y + Row1.Z * m.Row2.Z,
        Row2.X * m.Row0.X + Row2.Y * m.Row0.Y + Row2.Z * m.Row0.Z,
        Row2.X * m.Row1.X + Row2.Y * m.Row1.Y + Row2.Z * m.Row1.Z,
        Row2.X * m.Row2.X + Row2.Y * m.Row2.Y + Row2.Z * m.Row2.Z);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Matrix3x3 TimesTranspose(in Matrix3x3 m) => Transpose().TransposeTimes(m).Transpose();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix3x3 operator *(in Matrix3x3 m1, in Matrix3x3 m2) => new(
        m2.TDotX(m1.Row0), m2.TDotY(m1.Row0), m2.TDotZ(m1.Row0),
        m2.TDotX(m1.Row1), m2.TDotY(m1.Row1), m2.TDotZ(m1.Row1),
        m2.TDotX(m1.Row2), m2.TDotY(m1.Row2), m2.TDotZ(m1.Row2));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 operator *(in Matrix3x3 m, Vector3 v) => new(
        m.Row0.Dot(v), m.Row1.Dot(v), m.Row2.Dot(v));

    /// <summary>v * m (i.e. m.Transpose() * v).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 operator *(Vector3 v, in Matrix3x3 m) => new(
        m.TDotX(v), m.TDotY(v), m.TDotZ(v));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix3x3 operator *(in Matrix3x3 m, btScalar s) => new(m.Row0 * s, m.Row1 * s, m.Row2 * s);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix3x3 operator +(in Matrix3x3 a, in Matrix3x3 b) => new(a.Row0 + b.Row0, a.Row1 + b.Row1, a.Row2 + b.Row2);

    public static bool operator ==(in Matrix3x3 a, in Matrix3x3 b) =>
        a.Row0 == b.Row0 && a.Row1 == b.Row1 && a.Row2 == b.Row2;

    /// <summary>
    /// Diagonalizes this symmetric matrix in-place by the Jacobi method (btMatrix3x3::diagonalize):
    /// old_this = rot * new_this * rot^T.
    /// </summary>
    public void Diagonalize(out Matrix3x3 rot, btScalar threshold, int maxSteps)
    {
        rot = Identity;
        for (int step = maxSteps; step > 0; step--)
        {
            // find off-diagonal element [p][q] with largest magnitude
            int p = 0;
            int q = 1;
            int r = 2;
            btScalar max = BtMath.Abs(Row0.Y);
            btScalar v = BtMath.Abs(Row0.Z);
            if (v > max)
            {
                q = 2;
                r = 1;
                max = v;
            }
            v = BtMath.Abs(Row1.Z);
            if (v > max)
            {
                p = 1;
                q = 2;
                r = 0;
                max = v;
            }

            btScalar t = threshold * (BtMath.Abs(Row0.X) + BtMath.Abs(Row1.Y) + BtMath.Abs(Row2.Z));
            if (max <= t)
            {
                if (max <= BtMath.SimdEpsilon * t)
                {
                    return;
                }
                step = 1;
            }

            // compute Jacobi rotation J which leads to a zero for element [p][q]
            btScalar mpq = this[p, q];
            btScalar theta = (this[q, q] - this[p, p]) / (2 * mpq);
            btScalar theta2 = theta * theta;
            btScalar cos;
            btScalar sin;
            if (theta2 * theta2 < (btScalar)(10 / BtMath.SimdEpsilon))
            {
                t = (theta >= 0)
                    ? 1 / (theta + BtMath.Sqrt(1 + theta2))
                    : 1 / (theta - BtMath.Sqrt(1 + theta2));
                cos = 1 / BtMath.Sqrt(1 + t * t);
                sin = cos * t;
            }
            else
            {
                // approximation for large theta-value, i.e., a nearly diagonal matrix
                t = 1 / (theta * (2 + (btScalar)0.5 / theta2));
                cos = 1 - (btScalar)0.5 * t * t;
                sin = cos * t;
            }

            // apply rotation to matrix (this = J^T * this * J)
            this[p, q] = 0;
            this[q, p] = 0;
            this[p, p] -= t * mpq;
            this[q, q] += t * mpq;
            btScalar mrp = this[r, p];
            btScalar mrq = this[r, q];
            btScalar rpNew = cos * mrp - sin * mrq;
            btScalar rqNew = cos * mrq + sin * mrp;
            this[r, p] = rpNew;
            this[p, r] = rpNew;
            this[r, q] = rqNew;
            this[q, r] = rqNew;

            // apply rotation to rot (rot = rot * J)
            for (int i = 0; i < 3; i++)
            {
                mrp = rot[i, p];
                mrq = rot[i, q];
                rot[i, p] = cos * mrp - sin * mrq;
                rot[i, q] = cos * mrq + sin * mrp;
            }
        }
    }

    public static bool operator !=(in Matrix3x3 a, in Matrix3x3 b) => !(a == b);

    public override readonly bool Equals(object obj) => obj is Matrix3x3 m && this == m;
    public override readonly int GetHashCode() => HashCode.Combine(Row0, Row1, Row2);
}
