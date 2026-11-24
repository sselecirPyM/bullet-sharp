using BulletSharp.Math;

namespace BulletSharp;

/// <summary>btConvexCast::CastResult</summary>
public class CastResult
{
    internal btScalar fraction;
    internal Vector3 normal;
    internal Vector3 hitPoint;
    internal btScalar allowedPenetration;

    public CastResult()
    {
        fraction = BtMath.BtLargeFloat;
    }

    public btScalar Fraction
    {
        get => fraction;
        set => fraction = value;
    }

    public Vector3 Normal
    {
        get => normal;
        set => normal = value;
    }

    public Vector3 HitPoint
    {
        get => hitPoint;
        set => hitPoint = value;
    }

    public btScalar AllowedPenetration
    {
        get => allowedPenetration;
        set => allowedPenetration = value;
    }

    /// <summary>In-place reuse (equivalent to reconstructing on the stack in C++)</summary>
    internal void Reset(btScalar fraction)
    {
        this.fraction = fraction;
        normal = Vector3.Zero;
        hitPoint = Vector3.Zero;
        allowedPenetration = 0;
    }
}

/// <summary>
/// Ported from btConvexCast: base class for convex sweep intersection tests.
/// </summary>
public abstract class ConvexCast
{
    /// <summary>Computes the time of first contact of fromA→toA relative to fromB→toB. Returns true and fills result on hit.</summary>
    public abstract bool CalcTimeOfImpact(in Transform fromA, in Transform toA, in Transform fromB, in Transform toB, CastResult result);

    public virtual void Dispose()
    {
    }
}
