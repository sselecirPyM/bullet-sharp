using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btTriangleConvexcastCallback (btRaycastCallback.h/.cpp):
/// callback base class that performs a convex sweep intersection against triangles.
/// </summary>
public abstract class TriangleConvexcastCallback : TriangleCallback
{
    // Stateless penetration solver, shareable globally
    private static readonly GjkEpaPenetrationDepthSolver s_penetrationSolver = new();

    internal ConvexShape convexShape;
    internal Transform convexShapeFrom;
    internal Transform convexShapeTo;
    internal Transform triangleToWorld;
    internal btScalar triangleCollisionMargin;

    // Reused per-triangle temporaries (stack objects in C++ ProcessTriangle)
    private readonly TriangleShape triShape = new();
    private readonly VoronoiSimplexSolver simplexSolver = new();
    private readonly ContinuousConvexCollision convexCaster;
    private readonly CastResult castResult = new();

    /// <summary>Current hit fraction (btTriangleConvexcastCallback::m_hitFraction)</summary>
    public btScalar hitFraction;

    /// <summary>Allowed penetration amount (btTriangleConvexcastCallback::m_allowedPenetration)</summary>
    public btScalar allowedPenetration;

    protected TriangleConvexcastCallback(ConvexShape convexShape, in Transform convexShapeFrom, in Transform convexShapeTo,
        in Transform triangleToWorld, btScalar triangleCollisionMargin)
    {
        convexCaster = new ContinuousConvexCollision(convexShape, triShape, simplexSolver, s_penetrationSolver);
        Set(convexShape, in convexShapeFrom, in convexShapeTo, in triangleToWorld, triangleCollisionMargin, 1, 0);
    }

    /// <summary>In-place reuse (equivalent to reconstructing on the stack per call in C++)</summary>
    internal void Set(ConvexShape convexShape, in Transform convexShapeFrom, in Transform convexShapeTo,
        in Transform triangleToWorld, btScalar triangleCollisionMargin, btScalar hitFraction, btScalar allowedPenetration)
    {
        this.convexShape = convexShape;
        this.convexShapeFrom = convexShapeFrom;
        this.convexShapeTo = convexShapeTo;
        this.triangleToWorld = triangleToWorld;
        this.triangleCollisionMargin = triangleCollisionMargin;
        this.hitFraction = hitFraction;
        this.allowedPenetration = allowedPenetration;
    }

    /// <summary>btTriangleConvexcastCallback::processTriangle</summary>
    public override void ProcessTriangle(Vector3[] triangle, int partId, int triangleIndex)
    {
        // Reuse the TriangleShape / solver / caster / result (stack objects in C++)
        triShape.vertices1[0] = triangle[0];
        triShape.vertices1[1] = triangle[1];
        triShape.vertices1[2] = triangle[2];
        triShape.Margin = triangleCollisionMargin;

        convexCaster.SetShapes(convexShape, triShape, simplexSolver, s_penetrationSolver);

        castResult.Reset(1);
        castResult.AllowedPenetration = allowedPenetration;
        if (convexCaster.CalcTimeOfImpact(in convexShapeFrom, in convexShapeTo, in triangleToWorld, in triangleToWorld, castResult))
        {
            // Add a hit
            if (castResult.Normal.LengthSquared() > (btScalar)0.0001)
            {
                if (castResult.Fraction < hitFraction)
                {
                    // The btContinuousConvexCast normal is already in world space
                    castResult.Normal.Normalize();

                    ReportHit(castResult.Normal,
                        castResult.HitPoint,
                        castResult.Fraction,
                        partId,
                        triangleIndex);
                }
            }
        }
    }

    /// <summary>btTriangleConvexcastCallback::reportHit</summary>
    public abstract btScalar ReportHit(Vector3 hitNormalLocal, Vector3 hitPointLocal, btScalar hitFraction, int partId, int triangleIndex);
}
