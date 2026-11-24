namespace BulletSharp;

/// <summary>
/// Ported from btIndexedMesh: a single mesh sub-part (a contiguous range of vertex and index data).
/// Note: in the C# port the data is managed arrays; VertexStride is the number of components per
/// vertex (default 3, i.e. xyz), corresponding to m_vertexStride = 3 * sizeof(btScalar) in C++.
/// </summary>
public class IndexedMesh
{
    internal int numTriangles;
    internal int numVertices;
    internal int[] triangleIndexBase;
    internal btScalar[] vertexBase;
    internal int vertexStride = 3;

    /// <summary>btIndexedMesh::m_numTriangles</summary>
    public int NumTriangles
    {
        get => numTriangles;
        set => numTriangles = value;
    }

    /// <summary>btIndexedMesh::m_numVertices</summary>
    public int NumVertices
    {
        get => numVertices;
        set => numVertices = value;
    }

    /// <summary>btIndexedMesh::m_triangleIndexBase (int index array, 3 indices per triangle)</summary>
    public int[] TriangleIndexBase
    {
        get => triangleIndexBase;
        set => triangleIndexBase = value;
    }

    /// <summary>btIndexedMesh::m_vertexBase (flat vertex array, VertexStride components per vertex)</summary>
    public btScalar[] VertexBase
    {
        get => vertexBase;
        set => vertexBase = value;
    }

    /// <summary>btIndexedMesh::m_vertexStride (components per vertex, default 3)</summary>
    public int VertexStride
    {
        get => vertexStride;
        set => vertexStride = value;
    }
}
