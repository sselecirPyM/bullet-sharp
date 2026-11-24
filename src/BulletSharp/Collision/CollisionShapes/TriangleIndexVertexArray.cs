using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btTriangleIndexVertexArray: triangle mesh composed of several IndexedMesh sub-parts.
/// Graphics and collision mesh data can be shared directly (alternatively, derive your own
/// StridingMeshInterface).
/// </summary>
public class TriangleIndexVertexArray : StridingMeshInterface
{
    internal readonly List<IndexedMesh> indexedMeshes = new();

    // Premade aabb (m_hasAabb being an int is deliberate for C++ serialization compatibility)
    internal int hasAabb;
    internal Vector3 premadeAabbMin;
    internal Vector3 premadeAabbMax;

    public TriangleIndexVertexArray()
    {
    }

    /// <summary>Constructs from a set of IndexedMesh parts (equivalent to calling addIndexedMesh for each in C++)</summary>
    public TriangleIndexVertexArray(IndexedMesh[] meshes)
    {
        foreach (IndexedMesh mesh in meshes)
        {
            AddIndexedMesh(mesh);
        }
    }

    /// <summary>btTriangleIndexVertexArray (convenience constructor for a single mesh)</summary>
    public TriangleIndexVertexArray(int numTriangles, int[] triangleIndexBase, int numVertices, btScalar[] vertexBase)
    {
        IndexedMesh mesh = new()
        {
            numTriangles = numTriangles,
            triangleIndexBase = triangleIndexBase,
            numVertices = numVertices,
            vertexBase = vertexBase,
            vertexStride = 3,
        };
        AddIndexedMesh(mesh);
    }

    /// <summary>btTriangleIndexVertexArray::addIndexedMesh</summary>
    public void AddIndexedMesh(IndexedMesh mesh) => indexedMeshes.Add(mesh);

    /// <summary>btTriangleIndexVertexArray::getIndexedMeshArray</summary>
    public List<IndexedMesh> IndexedMeshArray => indexedMeshes;

    /// <summary>btStridingMeshInterface::InternalProcessAllTriangles (iterates the index array of each sub-mesh; vertices have Scaling applied)</summary>
    public override void InternalProcessAllTriangles(InternalTriangleIndexCallback callback, Vector3 aabbMin, Vector3 aabbMax)
    {
        Vector3 meshScaling = Scaling;
        Vector3[] triangle = new Vector3[3];

        for (int part = 0; part < indexedMeshes.Count; part++)
        {
            IndexedMesh mesh = indexedMeshes[part];
            int[] indices = mesh.triangleIndexBase;
            btScalar[] vertices = mesh.vertexBase;
            int stride = mesh.vertexStride;

            for (int gfxIndex = 0; gfxIndex < mesh.numTriangles; gfxIndex++)
            {
                for (int j = 0; j < 3; j++)
                {
                    int graphicsIndex = indices[gfxIndex * 3 + j];
                    int offset = graphicsIndex * stride;
                    triangle[j] = new Vector3(
                        vertices[offset] * meshScaling.X,
                        vertices[offset + 1] * meshScaling.Y,
                        vertices[offset + 2] * meshScaling.Z);
                }
                callback.InternalProcessTriangleIndex(triangle, part, gfxIndex);
            }
        }
    }

    /// <summary>Returns the triangle at (subPart, triangleIndex) with Scaling applied to the vertices</summary>
    internal override void GetTriangle(int subPart, int triangleIndex, Vector3[] triangle)
    {
        IndexedMesh mesh = indexedMeshes[subPart];
        int[] indices = mesh.triangleIndexBase;
        btScalar[] vertices = mesh.vertexBase;
        int stride = mesh.vertexStride;
        Vector3 meshScaling = Scaling;

        for (int j = 2; j >= 0; j--)
        {
            int graphicsIndex = indices[triangleIndex * 3 + j];
            int offset = graphicsIndex * stride;
            triangle[j] = new Vector3(
                vertices[offset] * meshScaling.X,
                vertices[offset + 1] * meshScaling.Y,
                vertices[offset + 2] * meshScaling.Z);
        }
    }

    /// <summary>btTriangleIndexVertexArray::hasPremadeAabb</summary>
    public override bool HasPremadeAabb => hasAabb == 1;

    /// <summary>btTriangleIndexVertexArray::setPremadeAabb</summary>
    public override void SetPremadeAabb(Vector3 aabbMin, Vector3 aabbMax)
    {
        premadeAabbMin = aabbMin;
        premadeAabbMax = aabbMax;
        hasAabb = 1; // Deliberately an int; see the C++ header comment
    }

    /// <summary>btTriangleIndexVertexArray::getPremadeAabb</summary>
    public override void GetPremadeAabb(out Vector3 aabbMin, out Vector3 aabbMax)
    {
        aabbMin = premadeAabbMin;
        aabbMax = premadeAabbMax;
    }
}
