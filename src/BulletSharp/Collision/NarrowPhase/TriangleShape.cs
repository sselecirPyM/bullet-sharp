using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btTriangleShape: a single triangle shape (polyhedral convex shape).
/// Note: it lives in the NarrowPhase directory because it is mainly used internally by
/// narrow-phase algorithms (SphereTriangleDetector, ConvexConvexAlgorithm, TriangleConvexcastCallback, etc.).
/// </summary>
public class TriangleShape : PolyhedralConvexShape
{
    internal readonly Vector3[] vertices1 = new Vector3[3];

    public TriangleShape()
    {
        shapeType = BroadphaseNativeType.TriangleShape;
    }

    public TriangleShape(Vector3 p0, Vector3 p1, Vector3 p2)
    {
        shapeType = BroadphaseNativeType.TriangleShape;
        vertices1[0] = p0;
        vertices1[1] = p1;
        vertices1[2] = p2;
    }

    public TriangleShape(Vector3[] xyz)
        : this(xyz[0], xyz[1], xyz[2])
    {
    }

    /// <summary>btTriangleShape::m_vertices1</summary>
    public Vector3[] Vertices1 => vertices1;

    /// <summary>btPolyhedralConvexShape::getNumVertices</summary>
    public int NumVertices => 3;

    /// <summary>btPolyhedralConvexShape::getVertex</summary>
    public void GetVertex(int index, out Vector3 vert) => vert = vertices1[index];

    /// <summary>btPolyhedralConvexShape::getNumEdges</summary>
    public int NumEdges => 3;

    /// <summary>btPolyhedralConvexShape::getEdge</summary>
    public void GetEdge(int i, out Vector3 pa, out Vector3 pb)
    {
        GetVertex(i, out pa);
        GetVertex((i + 1) % 3, out pb);
    }

    /// <summary>btTriangleShape::getAabb (uses the slower generic implementation)</summary>
    internal override void GetAabb(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax) =>
        GetAabbSlow(in t, out aabbMin, out aabbMax);

    /// <summary>btTriangleShape::localGetSupportingVertexWithoutMargin</summary>
    public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 dir)
    {
        Vector3 dots = new(dir.Dot(vertices1[0]), dir.Dot(vertices1[1]), dir.Dot(vertices1[2]));
        return vertices1[dots.MaxAxis()];
    }

    /// <summary>btPolyhedralConvexShape::getPlane</summary>
    public void GetPlane(int i, out Vector3 planeNormal, out Vector3 planeSupport) =>
        GetPlaneEquation(i, out planeNormal, out planeSupport);

    /// <summary>btPolyhedralConvexShape::getNumPlanes</summary>
    public int NumPlanes => 1;

    /// <summary>btTriangleShape::calcNormal</summary>
    public void CalcNormal(out Vector3 normal)
    {
        normal = (vertices1[1] - vertices1[0]).Cross(vertices1[2] - vertices1[0]);
        normal.Normalize();
    }

    /// <summary>btTriangleShape::getPlaneEquation</summary>
    public void GetPlaneEquation(int i, out Vector3 planeNormal, out Vector3 planeSupport)
    {
        CalcNormal(out planeNormal);
        planeSupport = vertices1[0];
    }

    /// <summary>btTriangleShape::calculateLocalInertia (a triangle has no volume inertia; returns zero)</summary>
    public override void CalculateLocalInertia(btScalar mass, out Vector3 inertia) =>
        inertia = Vector3.Zero;

    /// <summary>btTriangleShape::isInside</summary>
    public bool IsInside(Vector3 pt, btScalar tolerance)
    {
        CalcNormal(out Vector3 normal);
        // Distance to the plane
        btScalar dist = pt.Dot(normal);
        btScalar planeconst = vertices1[0].Dot(normal);
        dist -= planeconst;
        if (dist >= -tolerance && dist <= tolerance)
        {
            // Check that the point is inside all edge planes
            for (int i = 0; i < 3; i++)
            {
                GetEdge(i, out Vector3 pa, out Vector3 pb);
                Vector3 edge = pb - pa;
                Vector3 edgeNormal = edge.Cross(normal);
                edgeNormal.Normalize();
                btScalar dist2 = pt.Dot(edgeNormal);
                btScalar edgeConst = pa.Dot(edgeNormal);
                dist2 -= edgeConst;
                if (dist2 < -tolerance)
                    return false;
            }

            return true;
        }

        return false;
    }

    /// <summary>btConvexShape::getNumPreferredPenetrationDirections</summary>
    public override int NumPreferredPenetrationDirections => 2;

    /// <summary>btConvexShape::getPreferredPenetrationDirection</summary>
    public override void GetPreferredPenetrationDirection(int index, out Vector3 penetrationVector)
    {
        CalcNormal(out penetrationVector);
        if (index != 0)
            penetrationVector *= -1;
    }

    /// <summary>btPolyhedralConvexShape::initializePolyhedralFeatures (a single triangle needs no hull features; no-op)</summary>
    protected override void InitializePolyhedralFeatures()
    {
    }
}
