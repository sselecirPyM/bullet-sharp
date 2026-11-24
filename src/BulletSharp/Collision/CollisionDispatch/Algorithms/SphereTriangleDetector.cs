using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from SphereTriangleDetector: sphere-triangle detector, adapted to btDiscreteCollisionDetectorInterface.
/// </summary>
public class SphereTriangleDetector : DiscreteCollisionDetectorInterface
{
    internal SphereShape sphere;
    internal TriangleShape triangle;
    internal btScalar contactBreakingThreshold;

    public SphereTriangleDetector(SphereShape sphere, TriangleShape triangle, btScalar contactBreakingThreshold)
    {
        Set(sphere, triangle, contactBreakingThreshold);
    }

    /// <summary>In-place reuse (equivalent to C++ reconstructing the object on the stack per call)</summary>
    internal void Set(SphereShape sphere, TriangleShape triangle, btScalar contactBreakingThreshold)
    {
        this.sphere = sphere;
        this.triangle = triangle;
        this.contactBreakingThreshold = contactBreakingThreshold;
    }

    /// <summary>SphereTriangleDetector::getClosestPoints</summary>
    public override void GetClosestPoints(ClosestPointInput input, Result output, IDebugDraw debugDraw, bool swapResults = false)
    {
        Transform transformA = input.transformA;
        Transform transformB = input.transformB;

        // move the sphere into triangle space
        Transform sphereInTr = transformB.InverseTimes(in transformA);

        if (Collide(sphereInTr.Origin, out Vector3 point, out Vector3 normal, out btScalar depth, contactBreakingThreshold))
        {
            if (swapResults)
            {
                Vector3 normalOnB = transformB.Basis * normal;
                Vector3 normalOnA = -normalOnB;
                Vector3 pointOnA = transformB * point + normalOnB * depth;
                output.AddContactPoint(normalOnA, pointOnA, depth);
            }
            else
            {
                output.AddContactPoint(transformB.Basis * normal, transformB * point, depth);
            }
        }
    }

    /// <summary>SegmentSqrDistance (squared distance from a point to a segment, outputs the nearest point)</summary>
    internal static btScalar SegmentSqrDistance(in Vector3 from, in Vector3 to, in Vector3 p, out Vector3 nearest)
    {
        Vector3 diff = p - from;
        Vector3 v = to - from;
        btScalar t = v.Dot(diff);

        if (t > 0)
        {
            btScalar dotVV = v.Dot(v);
            if (t < dotVV)
            {
                t /= dotVV;
                diff -= t * v;
            }
            else
            {
                t = 1;
                diff -= v;
            }
        }
        else
        {
            t = 0;
        }

        nearest = from + t * v;
        return diff.Dot(diff);
    }

    private bool FaceContains(in Vector3 p, Vector3[] vertices, in Vector3 normal) =>
        PointInTriangle(vertices, in normal, in p);

    /// <summary>SphereTriangleDetector::collide</summary>
    public bool Collide(Vector3 sphereCenter, out Vector3 point, out Vector3 resultNormal, out btScalar depth,
        btScalar contactBreakingThreshold)
    {
        point = Vector3.Zero;
        resultNormal = Vector3.Zero;
        depth = 0;

        Vector3[] vertices = triangle.Vertices1;

        btScalar radius = sphere.Radius;
        btScalar radiusWithThreshold = radius + contactBreakingThreshold;

        Vector3 normal = (vertices[1] - vertices[0]).Cross(vertices[2] - vertices[0]);

        btScalar l2 = normal.LengthSquared();
        bool hasContact = false;
        Vector3 contactPoint = Vector3.Zero;

        if (l2 >= BtMath.SimdEpsilon * BtMath.SimdEpsilon)
        {
            normal /= BtMath.Sqrt(l2);

            Vector3 p1ToCentre = sphereCenter - vertices[0];
            btScalar distanceFromPlane = p1ToCentre.Dot(normal);

            if (distanceFromPlane < 0)
            {
                // triangle facing the other way
                distanceFromPlane *= -1;
                normal *= -1;
            }

            bool isInsideContactPlane = distanceFromPlane < radiusWithThreshold;

            // check for contact / intersection
            if (isInsideContactPlane)
            {
                if (FaceContains(in sphereCenter, vertices, in normal))
                {
                    // inside the contact wedge — point on the contact shell plane
                    hasContact = true;
                    contactPoint = sphereCenter - normal * distanceFromPlane;
                }
                else
                {
                    // could be inside one of the contact capsules
                    btScalar contactCapsuleRadiusSqr = radiusWithThreshold * radiusWithThreshold;
                    btScalar minDistSqr = contactCapsuleRadiusSqr;
                    for (int i = 0; i < triangle.NumEdges; i++)
                    {
                        triangle.GetEdge(i, out Vector3 pa, out Vector3 pb);

                        btScalar distanceSqr = SegmentSqrDistance(in pa, in pb, in sphereCenter, out Vector3 nearestOnEdge);
                        if (distanceSqr < minDistSqr)
                        {
                            // indeed inside a capsule: record the capsule with the smallest distance
                            minDistSqr = distanceSqr;
                            hasContact = true;
                            contactPoint = nearestOnEdge;
                        }
                    }
                }
            }
        }

        if (hasContact)
        {
            Vector3 contactToCentre = sphereCenter - contactPoint;
            btScalar distanceSqr = contactToCentre.LengthSquared();

            if (distanceSqr < radiusWithThreshold * radiusWithThreshold)
            {
                if (distanceSqr > BtMath.SimdEpsilon)
                {
                    btScalar distance = BtMath.Sqrt(distanceSqr);
                    resultNormal = contactToCentre;
                    resultNormal.Normalize();
                    point = contactPoint;
                    depth = -(radius - distance);
                }
                else
                {
                    resultNormal = normal;
                    point = contactPoint;
                    depth = -radius;
                }
                return true;
            }
        }

        return false;
    }

    /// <summary>SphereTriangleDetector::pointInTriangle</summary>
    private static bool PointInTriangle(Vector3[] vertices, in Vector3 normal, in Vector3 p)
    {
        Vector3 p1 = vertices[0];
        Vector3 p2 = vertices[1];
        Vector3 p3 = vertices[2];

        Vector3 edge1 = p2 - p1;
        Vector3 edge2 = p3 - p2;
        Vector3 edge3 = p1 - p3;

        Vector3 p1ToP = p - p1;
        Vector3 p2ToP = p - p2;
        Vector3 p3ToP = p - p3;

        Vector3 edge1Normal = edge1.Cross(normal);
        Vector3 edge2Normal = edge2.Cross(normal);
        Vector3 edge3Normal = edge3.Cross(normal);

        btScalar r1 = edge1Normal.Dot(p1ToP);
        btScalar r2 = edge2Normal.Dot(p2ToP);
        btScalar r3 = edge3Normal.Dot(p3ToP);
        if ((r1 > 0 && r2 > 0 && r3 > 0) ||
            (r1 <= 0 && r2 <= 0 && r3 <= 0))
            return true;
        return false;
    }
}
