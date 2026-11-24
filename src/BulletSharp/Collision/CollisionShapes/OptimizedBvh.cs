using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btOptimizedBvh: quantized BVH built from a StridingMeshInterface.
/// </summary>
public class OptimizedBvh : QuantizedBvh
{
    /// <summary>btOptimizedBvh::build (instance version inherited from QuantizedBvh.Build)</summary>
    public new static OptimizedBvh Build(StridingMeshInterface triangles, bool useQuantizedAabbCompression, Vector3 bvhAabbMin, Vector3 bvhAabbMax)
    {
        OptimizedBvh bvh = new();
        ((QuantizedBvh)bvh).Build(triangles, useQuantizedAabbCompression, bvhAabbMin, bvhAabbMax);
        return bvh;
    }
}
