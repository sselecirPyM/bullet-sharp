using System.Globalization;
using System.Runtime.CompilerServices;

namespace BulletSharp.Math;

/// <summary>
/// 4x4 row-major matrix with the same layout as the original BulletSharp.Math.Matrix / SharpDX.Matrix
/// (for rigid transforms the translation lives in M41, M42, M43; points are row vectors multiplied on the left).
/// The component type depends on the compile-time precision (btScalar).
/// </summary>
public struct Matrix : IEquatable<Matrix>
{
    public btScalar M11, M12, M13, M14;
    public btScalar M21, M22, M23, M24;
    public btScalar M31, M32, M33, M34;
    public btScalar M41, M42, M43, M44;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Matrix(
        btScalar m11, btScalar m12, btScalar m13, btScalar m14,
        btScalar m21, btScalar m22, btScalar m23, btScalar m24,
        btScalar m31, btScalar m32, btScalar m33, btScalar m34,
        btScalar m41, btScalar m42, btScalar m43, btScalar m44)
    {
        M11 = m11; M12 = m12; M13 = m13; M14 = m14;
        M21 = m21; M22 = m22; M23 = m23; M24 = m24;
        M31 = m31; M32 = m32; M33 = m33; M34 = m34;
        M41 = m41; M42 = m42; M43 = m43; M44 = m44;
    }

    /// <summary>Constructs from a rotation quaternion and a translation.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Matrix(Quaternion rotation, Vector3 translation)
    {
        this = new Transform(rotation, translation).ToMatrix();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Matrix(Quaternion rotation) : this(rotation, Vector3.Zero) { }

    public static readonly Matrix Identity = new Matrix(
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);

    public static readonly Matrix Zero = default;

    /// <summary>Translation component (first 3 columns of row 4).</summary>
    public Vector3 Origin
    {
        readonly get => new(M41, M42, M43);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set { M41 = value.X; M42 = value.Y; M43 = value.Z; }
    }

    /// <summary>Access to the 3x3 rotation basis with transposed layout (rows = basis vectors).</summary>
    internal Matrix3x3 BasisInternal
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        readonly get => new(in this);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set
        {
            M11 = value.Row0.X; M21 = value.Row0.Y; M31 = value.Row0.Z;
            M12 = value.Row1.X; M22 = value.Row1.Y; M32 = value.Row1.Z;
            M13 = value.Row2.X; M23 = value.Row2.Y; M33 = value.Row2.Z;
        }
    }

    /// <summary>Rotation quaternion.</summary>
    public Quaternion Rotation
    {
        readonly get => BasisInternal.GetRotation();
        set => BasisInternal = new Matrix3x3(value);
    }

    public void SetIdentity() => this = Identity;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix Translation(Vector3 v)
    {
        Matrix m = Identity;
        m.Origin = v;
        return m;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix RotationQuaternion(Quaternion q)
    {
        Matrix m = Identity;
        m.Rotation = q;
        return m;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix FromQuaternion(Quaternion q) => RotationQuaternion(q);

    /// <summary>Rigid (affine) inverse transform; equivalent to btTransform::inverse.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Matrix Inverse() => Transform.FromMatrix(in this).Inverse().ToMatrix();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Matrix Transpose() => new(
        M11, M21, M31, M41,
        M12, M22, M32, M42,
        M13, M23, M33, M43,
        M14, M24, M34, M44);

    /// <summary>Determinant of the 3x3 basis.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly btScalar Determinant() => BasisInternal.Determinant();

    /// <summary>point * m (row-vector convention, including translation).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 TransformCoordinate(Vector3 point, in Matrix m) => new(
        point.X * m.M11 + point.Y * m.M21 + point.Z * m.M31 + m.M41,
        point.X * m.M12 + point.Y * m.M22 + point.Z * m.M32 + m.M42,
        point.X * m.M13 + point.Y * m.M23 + point.Z * m.M33 + m.M43);

    /// <summary>Rotation only (no translation).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 TransformNormal(Vector3 normal, in Matrix m) => new(
        normal.X * m.M11 + normal.Y * m.M21 + normal.Z * m.M31,
        normal.X * m.M12 + normal.Y * m.M22 + normal.Z * m.M32,
        normal.X * m.M13 + normal.Y * m.M23 + normal.Z * m.M33);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix Multiply(in Matrix a, in Matrix b)
    {
        Matrix r = default;
        r.M11 = a.M11 * b.M11 + a.M12 * b.M21 + a.M13 * b.M31 + a.M14 * b.M41;
        r.M12 = a.M11 * b.M12 + a.M12 * b.M22 + a.M13 * b.M32 + a.M14 * b.M42;
        r.M13 = a.M11 * b.M13 + a.M12 * b.M23 + a.M13 * b.M33 + a.M14 * b.M43;
        r.M14 = a.M11 * b.M14 + a.M12 * b.M24 + a.M13 * b.M34 + a.M14 * b.M44;
        r.M21 = a.M21 * b.M11 + a.M22 * b.M21 + a.M23 * b.M31 + a.M24 * b.M41;
        r.M22 = a.M21 * b.M12 + a.M22 * b.M22 + a.M23 * b.M32 + a.M24 * b.M42;
        r.M23 = a.M21 * b.M13 + a.M22 * b.M23 + a.M23 * b.M33 + a.M24 * b.M43;
        r.M24 = a.M21 * b.M14 + a.M22 * b.M24 + a.M23 * b.M34 + a.M24 * b.M44;
        r.M31 = a.M31 * b.M11 + a.M32 * b.M21 + a.M33 * b.M31 + a.M34 * b.M41;
        r.M32 = a.M31 * b.M12 + a.M32 * b.M22 + a.M33 * b.M32 + a.M34 * b.M42;
        r.M33 = a.M31 * b.M13 + a.M32 * b.M23 + a.M33 * b.M33 + a.M34 * b.M43;
        r.M34 = a.M31 * b.M14 + a.M32 * b.M24 + a.M33 * b.M34 + a.M34 * b.M44;
        r.M41 = a.M41 * b.M11 + a.M42 * b.M21 + a.M43 * b.M31 + a.M44 * b.M41;
        r.M42 = a.M41 * b.M12 + a.M42 * b.M22 + a.M43 * b.M32 + a.M44 * b.M42;
        r.M43 = a.M41 * b.M13 + a.M42 * b.M23 + a.M43 * b.M33 + a.M44 * b.M43;
        r.M44 = a.M41 * b.M14 + a.M42 * b.M24 + a.M43 * b.M34 + a.M44 * b.M44;
        return r;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Matrix operator *(in Matrix a, in Matrix b) => Multiply(in a, in b);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 operator *(Vector3 v, in Matrix m) => TransformCoordinate(v, in m);

    public static bool operator ==(in Matrix a, in Matrix b) =>
        a.M11 == b.M11 && a.M12 == b.M12 && a.M13 == b.M13 && a.M14 == b.M14 &&
        a.M21 == b.M21 && a.M22 == b.M22 && a.M23 == b.M23 && a.M24 == b.M24 &&
        a.M31 == b.M31 && a.M32 == b.M32 && a.M33 == b.M33 && a.M34 == b.M34 &&
        a.M41 == b.M41 && a.M42 == b.M42 && a.M43 == b.M43 && a.M44 == b.M44;

    public static bool operator !=(in Matrix a, in Matrix b) => !(a == b);

    public readonly bool Equals(Matrix other) => this == other;
    public override readonly bool Equals(object obj) => obj is Matrix m && Equals(m);

    public override readonly int GetHashCode()
    {
        var h = new HashCode();
        h.Add(M11); h.Add(M12); h.Add(M13); h.Add(M14);
        h.Add(M21); h.Add(M22); h.Add(M23); h.Add(M24);
        h.Add(M31); h.Add(M32); h.Add(M33); h.Add(M34);
        h.Add(M41); h.Add(M42); h.Add(M43); h.Add(M44);
        return h.ToHashCode();
    }

    public override readonly string ToString() => string.Format(CultureInfo.InvariantCulture,
        "[{0}, {1}, {2}, {3}; {4}, {5}, {6}, {7}; {8}, {9}, {10}, {11}; {12}, {13}, {14}, {15}]",
        M11, M12, M13, M14, M21, M22, M23, M24, M31, M32, M33, M34, M41, M42, M43, M44);
}
