using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btTriangleRaycastCallback (btRaycastCallback.h/.cpp):
/// callback base class that raycasts against triangles (in local space).
/// </summary>
public abstract class TriangleRaycastCallback : TriangleCallback
{
    // Input
    internal Vector3 from;
    internal Vector3 to;
    internal TriangleRaycastFlags flags;

    /// <summary>Current hit fraction (btTriangleRaycastCallback::m_hitFraction)</summary>
    public btScalar hitFraction;

    protected TriangleRaycastCallback(Vector3 from, Vector3 to, TriangleRaycastFlags flags = TriangleRaycastFlags.None)
    {
        this.from = from;
        this.to = to;
        this.flags = flags;
        hitFraction = 1;
    }

    public Vector3 From => from;
    public Vector3 To => to;

    public TriangleRaycastFlags Flags
    {
        get => flags;
        set => flags = value;
    }

    /// <summary>btTriangleRaycastCallback::processTriangle</summary>
    public override void ProcessTriangle(Vector3[] triangle, int partId, int triangleIndex)
    {
        Vector3 vert0 = triangle[0];
        Vector3 vert1 = triangle[1];
        Vector3 vert2 = triangle[2];

        Vector3 v10 = vert1 - vert0;
        Vector3 v20 = vert2 - vert0;

        Vector3 triangleNormal = v10.Cross(v20);

        btScalar dist = vert0.Dot(triangleNormal);
        btScalar distA = triangleNormal.Dot(from) - dist;
        btScalar distB = triangleNormal.Dot(to) - dist;

        if (distA * distB >= 0)
        {
            return; // same sign, the ray doesn't cross the triangle's plane
        }

        if ((flags & TriangleRaycastFlags.FilterBackfaces) != 0 && distA <= 0)
        {
            // Back face, skip the check
            return;
        }

        btScalar projLength = distA - distB;
        btScalar distance = distA / projLength;
        // We have the ray-plane intersection; check whether it lies inside the triangle.
        // Add an epsilon tolerance scaled by the triangle's size in case the ray hits exactly on an edge.

        if (distance < hitFraction)
        {
            btScalar edgeTolerance = triangleNormal.LengthSquared();
            edgeTolerance *= -(btScalar)0.0001;
            Vector3 point = from.Lerp(to, distance);
            {
                Vector3 v0p = vert0 - point;
                Vector3 v1p = vert1 - point;
                Vector3 cp0 = v0p.Cross(v1p);

                if (cp0.Dot(triangleNormal) >= edgeTolerance)
                {
                    Vector3 v2p = vert2 - point;
                    Vector3 cp1 = v1p.Cross(v2p);
                    if (cp1.Dot(triangleNormal) >= edgeTolerance)
                    {
                        Vector3 cp2 = v2p.Cross(v0p);

                        if (cp2.Dot(triangleNormal) >= edgeTolerance)
                        {
                            // triangleNormal hasn't been normalized up to this point
                            triangleNormal.Normalize();

                            // Allow returning an unflipped normal when the ray hits a back face
                            if ((flags & TriangleRaycastFlags.KeepUnflippedNormal) == 0 && distA <= 0)
                            {
                                hitFraction = ReportHit(-triangleNormal, distance, partId, triangleIndex);
                            }
                            else
                            {
                                hitFraction = ReportHit(triangleNormal, distance, partId, triangleIndex);
                            }
                        }
                    }
                }
            }
        }
    }

    /// <summary>btTriangleRaycastCallback::reportHit (returns the new hitFraction)</summary>
    public abstract btScalar ReportHit(Vector3 hitNormalLocal, btScalar hitFraction, int partId, int triangleIndex);
}
