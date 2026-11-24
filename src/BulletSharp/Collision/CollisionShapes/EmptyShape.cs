using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btEmptyShape: an empty shape with no actual collision geometry
/// (can be replaced by another shape at runtime).
/// </summary>
public class EmptyShape : ConcaveShape
{
    /// <summary>btEmptyShape::btEmptyShape</summary>
    public EmptyShape()
    {
        shapeType = BroadphaseNativeType.EmptyShape;
    }

    /// <summary>btEmptyShape::getAabb (origin +/- margin)</summary>
    internal override void GetAabb(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax)
    {
        Vector3 margin = new(Margin);

        aabbMin = t.Origin - margin;
        aabbMax = t.Origin + margin;
    }

    /// <summary>btEmptyShape::processAllTriangles (no triangles)</summary>
    public override void ProcessAllTriangles(TriangleCallback callback, Vector3 aabbMin, Vector3 aabbMax)
    {
    }

    /// <summary>btEmptyShape::calculateLocalInertia (btAssert(0) in C++; here it returns zero inertia)</summary>
    public override void CalculateLocalInertia(btScalar mass, out Vector3 inertia)
    {
        inertia = Vector3.Zero;
    }
}
