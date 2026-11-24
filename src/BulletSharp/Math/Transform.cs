namespace BulletSharp.Math;

/// <summary>
/// Ported from btTransform (3x3 basis + origin rigid transform). Used internally by the engine;
/// converted to/from <see cref="Matrix"/> at the public API boundary.
/// </summary>
public struct Transform
{
    public Matrix3x3 Basis;
    public Vector3 Origin;

    public Transform(Matrix3x3 basis, Vector3 origin)
    {
        Basis = basis;
        Origin = origin;
    }

    public Transform(Quaternion rotation, Vector3 origin)
    {
        Basis = new Matrix3x3(rotation);
        Origin = origin;
    }

    public static readonly Transform Identity = new(Matrix3x3.Identity, Vector3.Zero);

    public static Transform FromMatrix(in Matrix m) => new(
        new Matrix3x3(in m),
        new Vector3(m.M41, m.M42, m.M43));

    public readonly Matrix ToMatrix()
    {
        Matrix m = default;
        m.M11 = Basis.Row0.X; m.M12 = Basis.Row1.X; m.M13 = Basis.Row2.X; m.M14 = 0;
        m.M21 = Basis.Row0.Y; m.M22 = Basis.Row1.Y; m.M23 = Basis.Row2.Y; m.M24 = 0;
        m.M31 = Basis.Row0.Z; m.M32 = Basis.Row1.Z; m.M33 = Basis.Row2.Z; m.M34 = 0;
        m.M41 = Origin.X; m.M42 = Origin.Y; m.M43 = Origin.Z; m.M44 = 1;
        return m;
    }

    /// <summary>btTransform::inverse</summary>
    public readonly Transform Inverse()
    {
        Matrix3x3 inv = Basis.Transpose();
        return new Transform(inv, inv * -Origin);
    }

    /// <summary>btTransform::inverseTimes</summary>
    public readonly Transform InverseTimes(in Transform t)
    {
        Vector3 v = t.Origin - Origin;
        return new Transform(Basis.TransposeTimes(t.Basis), v * Basis);
    }

    public void SetRotation(Quaternion q) => Basis = new Matrix3x3(q);

    public readonly Quaternion GetRotation() => Basis.GetRotation();

    public static Vector3 operator *(in Transform t, Vector3 v) =>
        new(t.Basis.Row0.Dot(v) + t.Origin.X,
            t.Basis.Row1.Dot(v) + t.Origin.Y,
            t.Basis.Row2.Dot(v) + t.Origin.Z);

    /// <summary>t1 * t2: applies t2 first, then t1.</summary>
    public static Transform operator *(in Transform t1, in Transform t2) =>
        new(t1.Basis * t2.Basis, t1 * t2.Origin);

    /// <summary>btTransform::inverseXForm</summary>
    public readonly Vector3 InverseXForm(Vector3 v)
    {
        Vector3 d = v - Origin;
        return Basis.Transpose() * d;
    }

    public static bool operator ==(in Transform a, in Transform b) =>
        a.Basis == b.Basis && a.Origin == b.Origin;

    public static bool operator !=(in Transform a, in Transform b) => !(a == b);

    public override readonly bool Equals(object obj) => obj is Transform t && this == t;
    public override readonly int GetHashCode() => HashCode.Combine(Basis, Origin);
}
