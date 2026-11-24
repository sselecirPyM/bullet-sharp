using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btTriangleMesh: convenient triangle mesh storage class that can be used as the
/// data source for btBvhTriangleMeshShape.
/// Note: the C# port always uses 32-bit int indices and 3-component vertices (corresponding to
/// the C++ storage layout with use32bitIndices=true and use4componentVertices=false).
/// By default AddTriangle does not weld duplicate vertices (welding is very slow for large
/// meshes); it is generally recommended to use TriangleIndexVertexArray directly.
/// </summary>
public class TriangleMesh : TriangleIndexVertexArray
{
    private readonly List<btScalar> vertices3Component = new();
    private readonly List<int> indices32Bit = new();

    // Single-element cache for GetIndexedMeshArray (array copies are lazily rebuilt after data changes)
    private bool meshArraysDirty = true;
    private btScalar[] cachedVertices;
    private int[] cachedIndices;

    /// <summary>btTriangleMesh::m_weldingThreshold (welding threshold for duplicate vertices, compared against length2)</summary>
    public btScalar WeldingThreshold { get; set; }

    public TriangleMesh()
    {
        WeldingThreshold = 0;
        indexedMeshes.Add(new IndexedMesh
        {
            numTriangles = 0,
            numVertices = 0,
            triangleIndexBase = null,
            vertexBase = null,
            vertexStride = 3,
        });
    }

    /// <summary>btTriangleMesh::addIndex (internal method; prefer AddTriangle)</summary>
    public void AddIndex(int index)
    {
        indices32Bit.Add(index);
        meshArraysDirty = true;
    }

    /// <summary>btTriangleMesh::addTriangleIndices (indices must refer to previously added vertices)</summary>
    public void AddTriangleIndices(int index1, int index2, int index3)
    {
        indexedMeshes[0].numTriangles++;
        AddIndex(index1);
        AddIndex(index2);
        AddIndex(index3);
    }

    /// <summary>btTriangleMesh::findOrAddVertex (internal method; prefer AddTriangle)</summary>
    public int FindOrAddVertex(Vector3 vertex, bool removeDuplicateVertices)
    {
        // Returns the index of a new or existing vertex
        if (removeDuplicateVertices)
        {
            for (int i = 0; i < vertices3Component.Count; i += 3)
            {
                Vector3 vtx = new(vertices3Component[i], vertices3Component[i + 1], vertices3Component[i + 2]);
                if ((vtx - vertex).LengthSquared() <= WeldingThreshold)
                {
                    return i / 3;
                }
            }
        }
        vertices3Component.Add(vertex.X);
        vertices3Component.Add(vertex.Y);
        vertices3Component.Add(vertex.Z);
        indexedMeshes[0].numVertices++;
        meshArraysDirty = true;
        return vertices3Component.Count / 3 - 1;
    }

    /// <summary>btTriangleMesh::addTriangle</summary>
    public void AddTriangle(Vector3 vertex0, Vector3 vertex1, Vector3 vertex2, bool removeDuplicateVertices = false)
    {
        indexedMeshes[0].numTriangles++;
        AddIndex(FindOrAddVertex(vertex0, removeDuplicateVertices));
        AddIndex(FindOrAddVertex(vertex1, removeDuplicateVertices));
        AddIndex(FindOrAddVertex(vertex2, removeDuplicateVertices));
    }

    /// <summary>btTriangleMesh::getNumTriangles</summary>
    public int NumTriangles => indices32Bit.Count / 3;

    /// <summary>btTriangleMesh::preallocateVertices</summary>
    public override void PreallocateVertices(int numverts) => vertices3Component.EnsureCapacity(numverts * 3);

    /// <summary>btTriangleMesh::preallocateIndices</summary>
    public override void PreallocateIndices(int numindices) => indices32Bit.EnsureCapacity(numindices);

    /// <summary>btTriangleIndexVertexArray::getIndexedMeshArray (single-element cache; array copies are lazily rebuilt after data changes)</summary>
    public List<IndexedMesh> GetIndexedMeshArray()
    {
        if (meshArraysDirty)
        {
            cachedVertices = vertices3Component.ToArray();
            cachedIndices = indices32Bit.ToArray();
            indexedMeshes[0].vertexBase = cachedVertices;
            indexedMeshes[0].triangleIndexBase = cachedIndices;
            meshArraysDirty = false;
        }
        return indexedMeshes;
    }

    /// <summary>btStridingMeshInterface::InternalProcessAllTriangles (iterates the index array; vertices have Scaling applied)</summary>
    public override void InternalProcessAllTriangles(InternalTriangleIndexCallback callback, Vector3 aabbMin, Vector3 aabbMax)
    {
        Vector3 meshScaling = Scaling;
        Vector3[] triangle = new Vector3[3];

        int numTriangles = NumTriangles;
        for (int gfxIndex = 0; gfxIndex < numTriangles; gfxIndex++)
        {
            for (int j = 0; j < 3; j++)
            {
                int graphicsIndex = indices32Bit[gfxIndex * 3 + j];
                int offset = graphicsIndex * 3;
                triangle[j] = new Vector3(
                    vertices3Component[offset] * meshScaling.X,
                    vertices3Component[offset + 1] * meshScaling.Y,
                    vertices3Component[offset + 2] * meshScaling.Z);
            }
            callback.InternalProcessTriangleIndex(triangle, 0, gfxIndex);
        }
    }

    /// <summary>Returns the triangle at (subPart, triangleIndex) with Scaling applied to the vertices</summary>
    internal override void GetTriangle(int subPart, int triangleIndex, Vector3[] triangle)
    {
        Vector3 meshScaling = Scaling;
        for (int j = 2; j >= 0; j--)
        {
            int graphicsIndex = indices32Bit[triangleIndex * 3 + j];
            int offset = graphicsIndex * 3;
            triangle[j] = new Vector3(
                vertices3Component[offset] * meshScaling.X,
                vertices3Component[offset + 1] * meshScaling.Y,
                vertices3Component[offset + 2] * meshScaling.Z);
        }
    }
}
