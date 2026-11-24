using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Common AABB/ray utilities ported from LinearMath/btAabbUtil2.h.
/// </summary>
internal static class AabbUtil
{
    public static Vector3 Dot3(in Vector3 v0, in Vector3 v1, in Vector3 v2, in Vector3 v) =>
        new(v0.Dot(v), v1.Dot(v), v2.Dot(v));

    /// <summary>btTransformAabb(halfExtents, margin, t, min, max)</summary>
    public static void TransformAabb(in Vector3 halfExtents, btScalar margin, in Transform t, out Vector3 aabbMin, out Vector3 aabbMax)
    {
        Vector3 halfExtentsWithMargin = halfExtents + new Vector3(margin);
        Matrix3x3 absB = t.Basis.Absolute();
        Vector3 center = t.Origin;
        Vector3 extent = Dot3(absB.Row0, absB.Row1, absB.Row2, halfExtentsWithMargin);
        aabbMin = center - extent;
        aabbMax = center + extent;
    }

    /// <summary>btTransformAabb(localAabbMin, localAabbMax, margin, t, min, max)</summary>
    public static void TransformAabb(in Vector3 localAabbMin, in Vector3 localAabbMax, btScalar margin, in Transform trans, out Vector3 aabbMin, out Vector3 aabbMax)
    {
        Vector3 localHalfExtents = (localAabbMax - localAabbMin) / 2;
        Vector3 localCenter = (localAabbMax + localAabbMin) / 2;
        Matrix3x3 absB = trans.Basis.Absolute();
        Vector3 center = trans * localCenter;
        Vector3 extent = Dot3(absB.Row0, absB.Row1, absB.Row2, localHalfExtents) + new Vector3(margin);
        aabbMin = center - extent;
        aabbMax = center + extent;
    }

    /// <summary>TestPointAgainstAabb2</summary>
    public static bool TestPointAgainstAabb2(in Vector3 aabbMin1, in Vector3 aabbMax1, in Vector3 point)
    {
        bool overlap = true;
        overlap = (aabbMin1.X > point.X || aabbMax1.X < point.X) ? false : overlap;
        overlap = (aabbMin1.Z > point.Z || aabbMax1.Z < point.Z) ? false : overlap;
        overlap = (aabbMin1.Y > point.Y || aabbMax1.Y < point.Y) ? false : overlap;
        return overlap;
    }

    /// <summary>TestAabbAgainstAabb2 (conservative AABB overlap test)</summary>
    public static bool TestAabbAgainstAabb2(in Vector3 aabbMin1, in Vector3 aabbMax1, in Vector3 aabbMin2, in Vector3 aabbMax2)
    {
        bool overlap = true;
        overlap = (aabbMin1.X > aabbMax2.X || aabbMax1.X < aabbMin2.X) ? false : overlap;
        overlap = (aabbMin1.Z > aabbMax2.Z || aabbMax1.Z < aabbMin2.Z) ? false : overlap;
        overlap = (aabbMin1.Y > aabbMax2.Y || aabbMax1.Y < aabbMin2.Y) ? false : overlap;
        return overlap;
    }

    /// <summary>TestTriangleAgainstAabb2</summary>
    public static bool TestTriangleAgainstAabb2(in Vector3 p1, in Vector3 p2, in Vector3 p3, in Vector3 aabbMin, in Vector3 aabbMax)
    {
        if (BtMath.Min(BtMath.Min(p1.X, p2.X), p3.X) > aabbMax.X) return false;
        if (BtMath.Max(BtMath.Max(p1.X, p2.X), p3.X) < aabbMin.X) return false;

        if (BtMath.Min(BtMath.Min(p1.Z, p2.Z), p3.Z) > aabbMax.Z) return false;
        if (BtMath.Max(BtMath.Max(p1.Z, p2.Z), p3.Z) < aabbMin.Z) return false;

        if (BtMath.Min(BtMath.Min(p1.Y, p2.Y), p3.Y) > aabbMax.Y) return false;
        if (BtMath.Max(BtMath.Max(p1.Y, p2.Y), p3.Y) < aabbMin.Y) return false;
        return true;
    }

    /// <summary>btOutcode</summary>
    public static int Outcode(in Vector3 p, in Vector3 halfExtent)
    {
        return (p.X < -halfExtent.X ? 0x01 : 0x0) |
               (p.X > halfExtent.X ? 0x08 : 0x0) |
               (p.Y < -halfExtent.Y ? 0x02 : 0x0) |
               (p.Y > halfExtent.Y ? 0x10 : 0x0) |
               (p.Z < -halfExtent.Z ? 0x04 : 0x0) |
               (p.Z > halfExtent.Z ? 0x20 : 0x0);
    }

    /// <summary>
    /// btRayAabb2 (branch-free scalar form).
    /// The C++ original indexes bounds[s[i]] / bounds[1-s[i]] through a stack array of two
    /// btVector3 pointers; the span-based port left ~10 un-eliminable bounds checks per node
    /// visit. Passing the two bounds and three signs as scalars restores the zero-check form
    /// (the 0/1 selects compile to cmovs) and keeps the method small enough to inline.
    /// </summary>
    public static bool RayAabb2(in Vector3 rayFrom, in Vector3 rayInvDirection,
        int signX, int signY, int signZ, in Vector3 bounds0, in Vector3 bounds1,
        ref btScalar tmin, btScalar lambdaMin, btScalar lambdaMax)
    {
        tmin = ((signX == 0 ? bounds0.X : bounds1.X) - rayFrom.X) * rayInvDirection.X;
        btScalar tmax = ((signX != 0 ? bounds0.X : bounds1.X) - rayFrom.X) * rayInvDirection.X;
        btScalar tymin = ((signY == 0 ? bounds0.Y : bounds1.Y) - rayFrom.Y) * rayInvDirection.Y;
        btScalar tymax = ((signY != 0 ? bounds0.Y : bounds1.Y) - rayFrom.Y) * rayInvDirection.Y;

        if (tmin > tymax || tymin > tmax)
            return false;

        if (tymin > tmin) tmin = tymin;
        if (tymax < tmax) tmax = tymax;

        btScalar tzmin = ((signZ == 0 ? bounds0.Z : bounds1.Z) - rayFrom.Z) * rayInvDirection.Z;
        btScalar tzmax = ((signZ != 0 ? bounds0.Z : bounds1.Z) - rayFrom.Z) * rayInvDirection.Z;

        if (tmin > tzmax || tzmin > tmax)
            return false;
        if (tzmin > tmin) tmin = tzmin;
        if (tzmax < tmax) tmax = tzmax;
        return tmin < lambdaMax && tmax > lambdaMin;
    }

    /// <summary>btRayAabb (param is the maximum lambda on input, the hit lambda on output)</summary>
    public static bool RayAabb(in Vector3 rayFrom, in Vector3 rayTo, in Vector3 aabbMin, in Vector3 aabbMax,
        ref btScalar param, ref Vector3 normal)
    {
        Vector3 aabbHalfExtent = (aabbMax - aabbMin) / 2;
        Vector3 aabbCenter = (aabbMax + aabbMin) / 2;
        Vector3 source = rayFrom - aabbCenter;
        Vector3 target = rayTo - aabbCenter;
        int sourceOutcode = Outcode(source, aabbHalfExtent);
        int targetOutcode = Outcode(target, aabbHalfExtent);
        if ((sourceOutcode & targetOutcode) == 0x0)
        {
            btScalar lambdaEnter = 0;
            btScalar lambdaExit = param;
            Vector3 r = target - source;
            btScalar normSign = 1;
            Vector3 hitNormal = Vector3.Zero;
            int bit = 1;

            for (int j = 0; j < 2; j++)
            {
                for (int i = 0; i != 3; ++i)
                {
                    if ((sourceOutcode & bit) != 0)
                    {
                        btScalar lambda = (-source[i] - aabbHalfExtent[i] * normSign) / r[i];
                        if (lambdaEnter <= lambda)
                        {
                            lambdaEnter = lambda;
                            hitNormal.SetZero();
                            hitNormal[i] = normSign;
                        }
                    }
                    else if ((targetOutcode & bit) != 0)
                    {
                        btScalar lambda = (-source[i] - aabbHalfExtent[i] * normSign) / r[i];
                        if (lambda < lambdaExit) lambdaExit = lambda;
                    }
                    bit <<= 1;
                }
                normSign = -1;
            }
            if (lambdaEnter <= lambdaExit)
            {
                param = lambdaEnter;
                normal = hitNormal;
                return true;
            }
        }
        return false;
    }
}
