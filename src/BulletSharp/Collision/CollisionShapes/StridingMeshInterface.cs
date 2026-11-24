using BulletSharp.Math;

namespace BulletSharp;

/// <summary>PHY_ScalarType (btConcaveShape.h): data type for mesh vertex/index/heightfield data.</summary>
public enum PhyScalarType
{
    PHY_FLOAT,
    PHY_DOUBLE,
    PHY_INTEGER,
    PHY_SHORT,
    PHY_FIXEDPOINT88,
    PHY_UCHAR,
}

/// <summary>
/// Ported from btStridingMeshInterface: generic interface for accessing high-performance
/// triangle meshes, used together with shapes such as btBvhTriangleMeshShape.
/// Note: the C# port does not use raw pointer/stride locking semantics; mesh data is stored
/// as managed arrays in IndexedMesh.
/// </summary>
public abstract class StridingMeshInterface : IDisposable
{
    internal Vector3 scaling = Vector3.One;

    /// <summary>btStridingMeshInterface::InternalProcessAllTriangles</summary>
    public abstract void InternalProcessAllTriangles(InternalTriangleIndexCallback callback, Vector3 aabbMin, Vector3 aabbMax);

    /// <summary>Returns the triangle at (subPart, triangleIndex) with Scaling applied to the vertices; used by BVH callbacks.</summary>
    internal abstract void GetTriangle(int subPart, int triangleIndex, Vector3[] triangle);

    /// <summary>btStridingMeshInterface::getScaling / setScaling</summary>
    public Vector3 Scaling
    {
        get => scaling;
        set => scaling = value;
    }

    /// <summary>btStridingMeshInterface::preallocateVertices (optional, empty by default)</summary>
    public virtual void PreallocateVertices(int numverts)
    {
    }

    /// <summary>btStridingMeshInterface::preallocateIndices (optional, empty by default)</summary>
    public virtual void PreallocateIndices(int numindices)
    {
    }

    /// <summary>btStridingMeshInterface::hasPremadeAabb (false by default)</summary>
    public virtual bool HasPremadeAabb => false;

    /// <summary>btStridingMeshInterface::setPremadeAabb (empty by default)</summary>
    public virtual void SetPremadeAabb(Vector3 aabbMin, Vector3 aabbMax)
    {
    }

    /// <summary>btStridingMeshInterface::getPremadeAabb (empty by default)</summary>
    public virtual void GetPremadeAabb(out Vector3 aabbMin, out Vector3 aabbMax)
    {
        aabbMin = Vector3.Zero;
        aabbMax = Vector3.Zero;
    }

    /// <summary>btStridingMeshInterface::calculateAabbBruteForce (brute-force AABB computation)</summary>
    public void CalculateAabbBruteForce(out Vector3 aabbMin, out Vector3 aabbMax)
    {
        AabbCalculationCallback aabbCallback = new();
        InternalProcessAllTriangles(aabbCallback,
            new Vector3(-BtMath.BtLargeFloat), new Vector3(BtMath.BtLargeFloat));

        aabbMin = aabbCallback.aabbMin;
        aabbMax = aabbCallback.aabbMax;
    }

    private sealed class AabbCalculationCallback : InternalTriangleIndexCallback
    {
        public Vector3 aabbMin = new(BtMath.BtLargeFloat);
        public Vector3 aabbMax = new(-BtMath.BtLargeFloat);

        public override void InternalProcessTriangleIndex(Vector3[] triangle, int partId, int triangleIndex)
        {
            aabbMin.SetMin(triangle[0]);
            aabbMax.SetMax(triangle[0]);
            aabbMin.SetMin(triangle[1]);
            aabbMax.SetMax(triangle[1]);
            aabbMin.SetMin(triangle[2]);
            aabbMax.SetMax(triangle[2]);
        }
    }

    public virtual void Dispose()
    {
    }
}
