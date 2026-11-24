using System.Diagnostics;
using BulletSharp.Math;

namespace BulletSharp.SoftBody;

/// <summary>
/// Ported from LinearMath/btConvexHull.h (HullFlag).
/// </summary>
[Flags]
internal enum HullFlag : uint
{
    Triangles = 1 << 0,
    ReverseOrder = 1 << 1,
    Default = Triangles,
}

/// <summary>
/// Ported from LinearMath/btConvexHull.h (HullError).
/// </summary>
internal enum HullError
{
    Ok,
    Fail,
}

/// <summary>
/// Ported from LinearMath/btConvexHull.h (HullResult).
/// </summary>
internal sealed class HullResult
{
    public bool Polygons = true;
    public uint NumOutputVertices;
    public readonly List<Vector3> OutputVertices = new();
    public uint NumFaces;
    public uint NumIndices;
    public readonly List<uint> Indices = new();
}

/// <summary>
/// Ported from LinearMath/btConvexHull.h (HullDesc).
/// </summary>
internal sealed class HullDesc
{
    public HullFlag Flags = HullFlag.Default;
    public uint Vcount;
    public IReadOnlyList<Vector3> Vertices;
    public btScalar NormalEpsilon = (btScalar)0.001;
    public uint MaxVertices = 4096;
    public uint MaxFaces = 4096;

    public HullDesc()
    {
    }

    public HullDesc(HullFlag flag, int vcount, IReadOnlyList<Vector3> vertices)
    {
        Flags = flag;
        Vcount = (uint)vcount;
        Vertices = vertices;
        NormalEpsilon = (btScalar)0.001;
        MaxVertices = 4096;
    }

    public bool HasHullFlag(HullFlag flag) => (Flags & flag) != 0;

    public void SetHullFlag(HullFlag flag) => Flags |= flag;

    public void ClearHullFlag(HullFlag flag) => Flags &= ~flag;
}

/// <summary>
/// Ported from LinearMath/btConvexHull.h and btConvexHull.cpp (Stan Melax / John Ratcliff HullLibrary subset).
/// </summary>
internal sealed class HullLibrary
{
    private readonly List<HullTriangle> tris = new();
    internal readonly List<int> vertexIndexMapping = new();

    private struct Int3
    {
        public int X;
        public int Y;
        public int Z;

        public Int3(int x, int y, int z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public readonly int this[int i] => i switch
        {
            0 => X,
            1 => Y,
            2 => Z,
            _ => throw new ArgumentOutOfRangeException(nameof(i)),
        };
    }

    private struct Int4
    {
        public int X;
        public int Y;
        public int Z;
        public int W;

        public Int4(int x, int y, int z, int w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }

        public readonly int this[int i] => i switch
        {
            0 => X,
            1 => Y,
            2 => Z,
            3 => W,
            _ => throw new ArgumentOutOfRangeException(nameof(i)),
        };
    }

    private sealed class PHullResult
    {
        public uint Vcount;
        public uint IndexCount;
        public uint FaceCount;
        public Vector3[] Vertices;
        public readonly List<uint> Indices = new();
    }

    private sealed class HullTriangle
    {
        public Int3 v;
        public Int3 n;
        public int id;
        public int vmax;
        public btScalar rise;
        private int dummyNeib = -1;

        public HullTriangle(int a, int b, int c)
        {
            v = new Int3(a, b, c);
            n = new Int3(-1, -1, -1);
            vmax = -1;
            rise = (btScalar)0.0;
        }

        public int this[int i] => v[i];

        public ref int Neib(int a, int b)
        {
            for (int i = 0; i < 3; i++)
            {
                int i1 = (i + 1) % 3;
                int i2 = (i + 2) % 3;
                if ((v[i] == a && v[i1] == b) || (v[i] == b && v[i1] == a))
                {
                    switch (i2)
                    {
                        case 0: return ref n.X;
                        case 1: return ref n.Y;
                        default: return ref n.Z;
                    }
                }
            }
            Debug.Assert(false);
            dummyNeib = -1;
            return ref dummyNeib;
        }
    }

    public HullError CreateConvexHull(HullDesc desc, HullResult result)
    {
        HullError ret = HullError.Fail;
        var hr = new PHullResult();

        uint vcount = desc.Vcount;
        if (vcount < 8)
        {
            vcount = 8;
        }

        var vertexSource = new Vector3[vcount];
        bool ok = CleanupVertices(
            desc.Vcount,
            desc.Vertices,
            out uint ovcount,
            vertexSource,
            desc.NormalEpsilon,
            out Vector3 scale);

        if (ok)
        {
            for (uint i = 0; i < ovcount; i++)
            {
                Vector3 v = vertexSource[i];
                vertexSource[i] = new Vector3(v.X * scale.X, v.Y * scale.Y, v.Z * scale.Z);
            }

            ok = ComputeHull(ovcount, vertexSource, hr, desc.MaxVertices);

            if (ok)
            {
                var vertexScratch = new Vector3[hr.Vcount];
                BringOutYourDead(hr.Vertices, hr.Vcount, vertexScratch, out ovcount, hr.Indices, hr.IndexCount);

                ret = HullError.Ok;

                if (desc.HasHullFlag(HullFlag.Triangles))
                {
                    result.Polygons = false;
                    result.NumOutputVertices = ovcount;
                    result.OutputVertices.Clear();
                    result.OutputVertices.Capacity = BtMath.Max(result.OutputVertices.Capacity, (int)ovcount);
                    for (uint i = 0; i < ovcount; i++)
                    {
                        result.OutputVertices.Add(vertexScratch[i]);
                    }

                    result.NumFaces = hr.FaceCount;
                    result.NumIndices = hr.IndexCount;
                    result.Indices.Clear();
                    result.Indices.Capacity = BtMath.Max(result.Indices.Capacity, (int)hr.IndexCount);

                    if (desc.HasHullFlag(HullFlag.ReverseOrder))
                    {
                        for (int i = 0; i < (int)hr.FaceCount; i++)
                        {
                            int src = i * 3;
                            result.Indices.Add(hr.Indices[src + 2]);
                            result.Indices.Add(hr.Indices[src + 1]);
                            result.Indices.Add(hr.Indices[src + 0]);
                        }
                    }
                    else
                    {
                        for (int i = 0; i < (int)hr.IndexCount; i++)
                        {
                            result.Indices.Add(hr.Indices[i]);
                        }
                    }
                }
                else
                {
                    result.Polygons = true;
                    result.NumOutputVertices = ovcount;
                    result.OutputVertices.Clear();
                    result.OutputVertices.Capacity = BtMath.Max(result.OutputVertices.Capacity, (int)ovcount);
                    for (uint i = 0; i < ovcount; i++)
                    {
                        result.OutputVertices.Add(vertexScratch[i]);
                    }

                    result.NumFaces = hr.FaceCount;
                    result.NumIndices = hr.IndexCount + hr.FaceCount;
                    result.Indices.Clear();
                    result.Indices.Capacity = BtMath.Max(result.Indices.Capacity, (int)result.NumIndices);

                    for (int i = 0; i < (int)hr.FaceCount; i++)
                    {
                        int src = i * 3;
                        result.Indices.Add(3u);
                        if (desc.HasHullFlag(HullFlag.ReverseOrder))
                        {
                            result.Indices.Add(hr.Indices[src + 2]);
                            result.Indices.Add(hr.Indices[src + 1]);
                            result.Indices.Add(hr.Indices[src + 0]);
                        }
                        else
                        {
                            result.Indices.Add(hr.Indices[src + 0]);
                            result.Indices.Add(hr.Indices[src + 1]);
                            result.Indices.Add(hr.Indices[src + 2]);
                        }
                    }
                }

                ReleaseHull(hr);
            }
        }

        return ret;
    }

    public HullError ReleaseResult(HullResult result)
    {
        if (result.OutputVertices.Count > 0)
        {
            result.NumOutputVertices = 0;
            result.OutputVertices.Clear();
        }
        if (result.Indices.Count > 0)
        {
            result.NumIndices = 0;
            result.Indices.Clear();
        }
        return HullError.Ok;
    }

    private static void ReleaseHull(PHullResult result)
    {
        if (result.Indices.Count > 0)
        {
            result.Indices.Clear();
        }
        result.Vcount = 0;
        result.IndexCount = 0;
        result.Vertices = null;
    }

    private static Vector3 TriNormal(in Vector3 v0, in Vector3 v1, in Vector3 v2)
    {
        Vector3 cp = Vector3.Cross(v1 - v0, v2 - v1);
        btScalar m = cp.Length();
        if (m == 0)
        {
            return new Vector3(1, 0, 0);
        }
        return cp * ((btScalar)1.0 / m);
    }

    private static Vector3 Orth(in Vector3 v)
    {
        Vector3 a = Vector3.Cross(v, new Vector3(0, 0, 1));
        Vector3 b = Vector3.Cross(v, new Vector3(0, 1, 0));
        if (a.Length() > b.Length())
        {
            return a.Normalized();
        }
        return b.Normalized();
    }

    private static int MaxDirFiltered(Vector3[] p, int count, in Vector3 dir, List<int> allow)
    {
        Debug.Assert(count > 0);
        int m = -1;
        for (int i = 0; i < count; i++)
        {
            if (allow[i] != 0)
            {
                if (m == -1 || Vector3.Dot(p[i], dir) > Vector3.Dot(p[m], dir))
                {
                    m = i;
                }
            }
        }
        Debug.Assert(m != -1);
        return m;
    }

    private static int MaxDirSterid(Vector3[] p, int count, in Vector3 dir, List<int> allow)
    {
        int m = -1;
        while (m == -1)
        {
            m = MaxDirFiltered(p, count, in dir, allow);
            if (m == -1)
            {
                return 0;
            }
            if (allow[m] == 3)
            {
                return m;
            }
            Vector3 u = Orth(in dir);
            Vector3 v = Vector3.Cross(u, dir);
            int ma = -1;
            for (btScalar x = (btScalar)0.0; x <= (btScalar)360.0; x += (btScalar)45.0)
            {
                btScalar s = BtMath.Sin(BtMath.SimdRadsPerDeg * x);
                btScalar c = BtMath.Cos(BtMath.SimdRadsPerDeg * x);
                Vector3 d1 = dir + (u * s + v * c) * (btScalar)0.025;
                int mb = MaxDirFiltered(p, count, in d1, allow);
                if (ma == m && mb == m)
                {
                    allow[m] = 3;
                    return m;
                }
                if (ma != -1 && ma != mb)
                {
                    int mc = ma;
                    for (btScalar xx = x - (btScalar)40.0; xx <= x; xx += (btScalar)5.0)
                    {
                        btScalar s2 = BtMath.Sin(BtMath.SimdRadsPerDeg * xx);
                        btScalar c2 = BtMath.Cos(BtMath.SimdRadsPerDeg * xx);
                        Vector3 d2 = dir + (u * s2 + v * c2) * (btScalar)0.025;
                        int md = MaxDirFiltered(p, count, in d2, allow);
                        if (mc == m && md == m)
                        {
                            allow[m] = 3;
                            return m;
                        }
                        mc = md;
                    }
                }
                ma = mb;
            }
            allow[m] = 0;
            m = -1;
        }
        Debug.Assert(false);
        return m;
    }

    private static bool Above(Vector3[] vertices, in Int3 t, in Vector3 p, btScalar epsilon)
    {
        Vector3 n = TriNormal(in vertices[t.X], in vertices[t.Y], in vertices[t.Z]);
        return Vector3.Dot(n, p - vertices[t.X]) > epsilon;
    }

    private static bool HasVert(in Int3 t, int v) => t.X == v || t.Y == v || t.Z == v;

    private HullTriangle AllocateTriangle(int a, int b, int c)
    {
        var tr = new HullTriangle(a, b, c)
        {
            id = tris.Count,
        };
        tris.Add(tr);
        return tr;
    }

    private void DeAllocateTriangle(HullTriangle tri)
    {
        Debug.Assert(tris[tri.id] == tri);
        tris[tri.id] = null;
    }

    private void B2bFix(HullTriangle s, HullTriangle t)
    {
        for (int i = 0; i < 3; i++)
        {
            int i1 = (i + 1) % 3;
            int i2 = (i + 2) % 3;
            int a = s[i1];
            int b = s[i2];
            Debug.Assert(tris[s.Neib(a, b)].Neib(b, a) == s.id);
            Debug.Assert(tris[t.Neib(a, b)].Neib(b, a) == t.id);
            tris[s.Neib(a, b)].Neib(b, a) = t.Neib(b, a);
            tris[t.Neib(b, a)].Neib(a, b) = s.Neib(a, b);
        }
    }

    private void RemoveB2b(HullTriangle s, HullTriangle t)
    {
        B2bFix(s, t);
        DeAllocateTriangle(s);
        DeAllocateTriangle(t);
    }

    [Conditional("DEBUG")]
    private void CheckIt(HullTriangle t)
    {
        Debug.Assert(tris[t.id] == t);
        for (int i = 0; i < 3; i++)
        {
            int i1 = (i + 1) % 3;
            int i2 = (i + 2) % 3;
            int a = t[i1];
            int b = t[i2];
            Debug.Assert(a != b);
            Debug.Assert(tris[t.n[i]].Neib(b, a) == t.id);
        }
    }

    private void Extrude(HullTriangle t0, int v)
    {
        Int3 t = t0.v;
        int n = tris.Count;
        HullTriangle ta = AllocateTriangle(v, t.Y, t.Z);
        ta.n = new Int3(t0.n.X, n + 1, n + 2);
        tris[t0.n.X].Neib(t.Y, t.Z) = n + 0;
        HullTriangle tb = AllocateTriangle(v, t.Z, t.X);
        tb.n = new Int3(t0.n.Y, n + 2, n + 0);
        tris[t0.n.Y].Neib(t.Z, t.X) = n + 1;
        HullTriangle tc = AllocateTriangle(v, t.X, t.Y);
        tc.n = new Int3(t0.n.Z, n + 0, n + 1);
        tris[t0.n.Z].Neib(t.X, t.Y) = n + 2;
        CheckIt(ta);
        CheckIt(tb);
        CheckIt(tc);
        if (HasVert(in tris[ta.n.X].v, v))
        {
            RemoveB2b(ta, tris[ta.n.X]);
        }
        if (HasVert(in tris[tb.n.X].v, v))
        {
            RemoveB2b(tb, tris[tb.n.X]);
        }
        if (HasVert(in tris[tc.n.X].v, v))
        {
            RemoveB2b(tc, tris[tc.n.X]);
        }
        DeAllocateTriangle(t0);
    }

    private HullTriangle Extrudable(btScalar epsilon)
    {
        HullTriangle t = null;
        for (int i = 0; i < tris.Count; i++)
        {
            if (t == null || (tris[i] != null && t.rise < tris[i].rise))
            {
                t = tris[i];
            }
        }
        return (t != null && t.rise > epsilon) ? t : null;
    }

    private static Int4 FindSimplex(Vector3[] verts, int vertsCount, List<int> allow)
    {
        Vector3 basis0 = new((btScalar)0.01, (btScalar)0.02, (btScalar)1.0);
        int p0 = MaxDirSterid(verts, vertsCount, in basis0, allow);
        Vector3 negBasis0 = -basis0;
        int p1 = MaxDirSterid(verts, vertsCount, in negBasis0, allow);
        basis0 = verts[p0] - verts[p1];
        if (p0 == p1 || basis0 == Vector3.Zero)
        {
            return new Int4(-1, -1, -1, -1);
        }
        Vector3 basis1 = Vector3.Cross(new Vector3((btScalar)1, (btScalar)0.02, (btScalar)0), basis0);
        Vector3 basis2 = Vector3.Cross(new Vector3((btScalar)(-0.02), (btScalar)1, (btScalar)0), basis0);
        if (basis1.Length() > basis2.Length())
        {
            basis1.Normalize();
        }
        else
        {
            basis1 = basis2;
            basis1.Normalize();
        }
        int p2 = MaxDirSterid(verts, vertsCount, in basis1, allow);
        if (p2 == p0 || p2 == p1)
        {
            Vector3 negBasis1 = -basis1;
            p2 = MaxDirSterid(verts, vertsCount, in negBasis1, allow);
        }
        if (p2 == p0 || p2 == p1)
        {
            return new Int4(-1, -1, -1, -1);
        }
        basis1 = verts[p2] - verts[p0];
        basis2 = Vector3.Cross(basis1, basis0).Normalized();
        int p3 = MaxDirSterid(verts, vertsCount, in basis2, allow);
        if (p3 == p0 || p3 == p1 || p3 == p2)
        {
            Vector3 negBasis2 = -basis2;
            p3 = MaxDirSterid(verts, vertsCount, in negBasis2, allow);
        }
        if (p3 == p0 || p3 == p1 || p3 == p2)
        {
            return new Int4(-1, -1, -1, -1);
        }
        Debug.Assert(!(p0 == p1 || p0 == p2 || p0 == p3 || p1 == p2 || p1 == p3 || p2 == p3));
        if (Vector3.Dot(verts[p3] - verts[p0], Vector3.Cross(verts[p1] - verts[p0], verts[p2] - verts[p0])) < 0)
        {
            (p2, p3) = (p3, p2);
        }
        return new Int4(p0, p1, p2, p3);
    }

    private int Calchullgen(Vector3[] verts, int vertsCount, int vlimit)
    {
        if (vertsCount < 4)
        {
            return 0;
        }
        if (vlimit == 0)
        {
            vlimit = 1000000000;
        }
        Vector3 bmin = verts[0];
        Vector3 bmax = verts[0];
        var isextreme = new List<int>(vertsCount);
        var allow = new List<int>(vertsCount);

        for (int j = 0; j < vertsCount; j++)
        {
            allow.Add(1);
            isextreme.Add(0);
            bmin.SetMin(verts[j]);
            bmax.SetMax(verts[j]);
        }
        btScalar epsilon = (bmax - bmin).Length() * (btScalar)0.001;
        Debug.Assert(epsilon != (btScalar)0.0);

        Int4 p = FindSimplex(verts, vertsCount, allow);
        if (p.X == -1)
        {
            return 0;
        }

        Vector3 center = (verts[p[0]] + verts[p[1]] + verts[p[2]] + verts[p[3]]) / (btScalar)4.0;
        HullTriangle t0 = AllocateTriangle(p[2], p[3], p[1]);
        t0.n = new Int3(2, 3, 1);
        HullTriangle t1 = AllocateTriangle(p[3], p[2], p[0]);
        t1.n = new Int3(3, 2, 0);
        HullTriangle t2 = AllocateTriangle(p[0], p[1], p[3]);
        t2.n = new Int3(0, 1, 3);
        HullTriangle t3 = AllocateTriangle(p[1], p[0], p[2]);
        t3.n = new Int3(1, 0, 2);
        isextreme[p[0]] = isextreme[p[1]] = isextreme[p[2]] = isextreme[p[3]] = 1;
        CheckIt(t0);
        CheckIt(t1);
        CheckIt(t2);
        CheckIt(t3);

        for (int j = 0; j < tris.Count; j++)
        {
            HullTriangle t = tris[j];
            Debug.Assert(t != null);
            Debug.Assert(t.vmax < 0);
            Vector3 n = TriNormal(in verts[t[0]], in verts[t[1]], in verts[t[2]]);
            t.vmax = MaxDirSterid(verts, vertsCount, in n, allow);
            t.rise = Vector3.Dot(n, verts[t.vmax] - verts[t[0]]);
        }

        HullTriangle te;
        vlimit -= 4;
        while (vlimit > 0 && (te = Extrudable(epsilon)) != null)
        {
            int v = te.vmax;
            Debug.Assert(v != -1);
            Debug.Assert(isextreme[v] == 0);
            isextreme[v] = 1;

            int j = tris.Count;
            while (j-- > 0)
            {
                if (tris[j] == null)
                {
                    continue;
                }
                Int3 t = tris[j].v;
                if (Above(verts, in t, in verts[v], (btScalar)0.01 * epsilon))
                {
                    Extrude(tris[j], v);
                }
            }

            // Check for degenerate cases where we have a flipped or very skinny triangle
            j = tris.Count;
            while (j-- > 0)
            {
                if (tris[j] == null)
                {
                    continue;
                }
                if (!HasVert(in tris[j].v, v))
                {
                    break;
                }
                Int3 nt = tris[j].v;
                if (Above(verts, in nt, in center, (btScalar)0.01 * epsilon) ||
                    Vector3.Cross(verts[nt.Y] - verts[nt.X], verts[nt.Z] - verts[nt.Y]).Length() < epsilon * epsilon * (btScalar)0.1)
                {
                    HullTriangle nb = tris[tris[j].n.X];
                    Debug.Assert(nb != null);
                    Debug.Assert(!HasVert(in nb.v, v));
                    Debug.Assert(nb.id < j);
                    Extrude(nb, v);
                    j = tris.Count;
                }
            }

            j = tris.Count;
            while (j-- > 0)
            {
                HullTriangle t = tris[j];
                if (t == null)
                {
                    continue;
                }
                if (t.vmax >= 0)
                {
                    break;
                }
                Vector3 n = TriNormal(in verts[t[0]], in verts[t[1]], in verts[t[2]]);
                t.vmax = MaxDirSterid(verts, vertsCount, in n, allow);
                if (isextreme[t.vmax] != 0)
                {
                    t.vmax = -1;
                }
                else
                {
                    t.rise = Vector3.Dot(n, verts[t.vmax] - verts[t[0]]);
                }
            }
            vlimit--;
        }
        return 1;
    }

    private int Calchull(Vector3[] verts, int vertsCount, List<uint> trisOut, out int trisCount, int vlimit)
    {
        trisCount = 0;
        int rc = Calchullgen(verts, vertsCount, vlimit);
        if (rc == 0)
        {
            return 0;
        }

        var ts = new List<int>();
        for (int i = 0; i < tris.Count; i++)
        {
            HullTriangle tri = tris[i];
            if (tri != null)
            {
                ts.Add(tri.v.X);
                ts.Add(tri.v.Y);
                ts.Add(tri.v.Z);
                DeAllocateTriangle(tri);
            }
        }
        trisCount = ts.Count / 3;
        trisOut.Clear();
        trisOut.Capacity = BtMath.Max(trisOut.Capacity, ts.Count);
        for (int i = 0; i < ts.Count; i++)
        {
            trisOut.Add((uint)ts[i]);
        }
        tris.Clear();
        return 1;
    }

    private bool ComputeHull(uint vcount, Vector3[] vertices, PHullResult result, uint vlimit)
    {
        int ret = Calchull(vertices, (int)vcount, result.Indices, out int trisCount, (int)vlimit);
        if (ret == 0)
        {
            return false;
        }
        result.IndexCount = (uint)(trisCount * 3);
        result.FaceCount = (uint)trisCount;
        result.Vertices = vertices;
        result.Vcount = vcount;
        return true;
    }

    private static void AddPoint(ref uint vcount, Vector3[] p, btScalar x, btScalar y, btScalar z)
    {
        p[vcount] = new Vector3(x, y, z);
        vcount++;
    }

    private static btScalar GetDist(btScalar px, btScalar py, btScalar pz, in Vector3 p2)
    {
        btScalar dx = px - p2.X;
        btScalar dy = py - p2.Y;
        btScalar dz = pz - p2.Z;
        return dx * dx + dy * dy + dz * dz;
    }

    private bool CleanupVertices(
        uint svcount,
        IReadOnlyList<Vector3> svertices,
        out uint vcount,
        Vector3[] vertices,
        btScalar normalepsilon,
        out Vector3 scale)
    {
        vcount = 0;
        scale = Vector3.One;
        if (svcount == 0 || svertices == null)
        {
            return false;
        }

        vertexIndexMapping.Clear();

        const btScalar Epsilon = (btScalar)0.000001;
        const btScalar FltMax = (btScalar)float.MaxValue;

        btScalar recipX;
        btScalar recipY;
        btScalar recipZ;

        btScalar bminX = FltMax, bminY = FltMax, bminZ = FltMax;
        btScalar bmaxX = -FltMax, bmaxY = -FltMax, bmaxZ = -FltMax;

        for (int i = 0; i < (int)svcount; i++)
        {
            Vector3 p = svertices[i];
            if (p.X < bminX) bminX = p.X;
            if (p.X > bmaxX) bmaxX = p.X;
            if (p.Y < bminY) bminY = p.Y;
            if (p.Y > bmaxY) bmaxY = p.Y;
            if (p.Z < bminZ) bminZ = p.Z;
            if (p.Z > bmaxZ) bmaxZ = p.Z;
        }

        btScalar dx = bmaxX - bminX;
        btScalar dy = bmaxY - bminY;
        btScalar dz = bmaxZ - bminZ;

        Vector3 center = new(
            dx * (btScalar)0.5 + bminX,
            dy * (btScalar)0.5 + bminY,
            dz * (btScalar)0.5 + bminZ);

        if (dx < Epsilon || dy < Epsilon || dz < Epsilon || svcount < 3)
        {
            btScalar len = FltMax;
            if (dx > Epsilon && dx < len) len = dx;
            if (dy > Epsilon && dy < len) len = dy;
            if (dz > Epsilon && dz < len) len = dz;

            if (len == FltMax)
            {
                dx = dy = dz = (btScalar)0.01;
            }
            else
            {
                if (dx < Epsilon) dx = len * (btScalar)0.05;
                if (dy < Epsilon) dy = len * (btScalar)0.05;
                if (dz < Epsilon) dz = len * (btScalar)0.05;
            }

            btScalar x1 = center.X - dx;
            btScalar x2 = center.X + dx;
            btScalar y1 = center.Y - dy;
            btScalar y2 = center.Y + dy;
            btScalar z1 = center.Z - dz;
            btScalar z2 = center.Z + dz;

            AddPoint(ref vcount, vertices, x1, y1, z1);
            AddPoint(ref vcount, vertices, x2, y1, z1);
            AddPoint(ref vcount, vertices, x2, y2, z1);
            AddPoint(ref vcount, vertices, x1, y2, z1);
            AddPoint(ref vcount, vertices, x1, y1, z2);
            AddPoint(ref vcount, vertices, x2, y1, z2);
            AddPoint(ref vcount, vertices, x2, y2, z2);
            AddPoint(ref vcount, vertices, x1, y2, z2);

            return true;
        }

        scale = new Vector3(dx, dy, dz);
        recipX = 1 / dx;
        recipY = 1 / dy;
        recipZ = 1 / dz;
        center = new Vector3(center.X * recipX, center.Y * recipY, center.Z * recipZ);

        for (int i = 0; i < (int)svcount; i++)
        {
            Vector3 p = svertices[i];
            btScalar px = p.X * recipX;
            btScalar py = p.Y * recipY;
            btScalar pz = p.Z * recipZ;

            uint j;
            for (j = 0; j < vcount; j++)
            {
                Vector3 v = vertices[j];
                btScalar adx = BtMath.Abs(v.X - px);
                btScalar ady = BtMath.Abs(v.Y - py);
                btScalar adz = BtMath.Abs(v.Z - pz);

                if (adx < normalepsilon && ady < normalepsilon && adz < normalepsilon)
                {
                    btScalar dist1 = GetDist(px, py, pz, in center);
                    btScalar dist2 = GetDist(v.X, v.Y, v.Z, in center);
                    if (dist1 > dist2)
                    {
                        vertices[j] = new Vector3(px, py, pz);
                    }
                    break;
                }
            }

            if (j == vcount)
            {
                vertices[vcount] = new Vector3(px, py, pz);
                vcount++;
            }
            vertexIndexMapping.Add((int)j);
        }

        bminX = bminY = bminZ = FltMax;
        bmaxX = bmaxY = bmaxZ = -FltMax;

        for (uint i = 0; i < vcount; i++)
        {
            Vector3 p = vertices[i];
            if (p.X < bminX) bminX = p.X;
            if (p.X > bmaxX) bmaxX = p.X;
            if (p.Y < bminY) bminY = p.Y;
            if (p.Y > bmaxY) bmaxY = p.Y;
            if (p.Z < bminZ) bminZ = p.Z;
            if (p.Z > bmaxZ) bmaxZ = p.Z;
        }

        dx = bmaxX - bminX;
        dy = bmaxY - bminY;
        dz = bmaxZ - bminZ;

        if (dx < Epsilon || dy < Epsilon || dz < Epsilon || vcount < 3)
        {
            btScalar cx = dx * (btScalar)0.5 + bminX;
            btScalar cy = dy * (btScalar)0.5 + bminY;
            btScalar cz = dz * (btScalar)0.5 + bminZ;

            btScalar len = FltMax;
            if (dx >= Epsilon && dx < len) len = dx;
            if (dy >= Epsilon && dy < len) len = dy;
            if (dz >= Epsilon && dz < len) len = dz;

            if (len == FltMax)
            {
                dx = dy = dz = (btScalar)0.01;
            }
            else
            {
                if (dx < Epsilon) dx = len * (btScalar)0.05;
                if (dy < Epsilon) dy = len * (btScalar)0.05;
                if (dz < Epsilon) dz = len * (btScalar)0.05;
            }

            btScalar x1 = cx - dx;
            btScalar x2 = cx + dx;
            btScalar y1 = cy - dy;
            btScalar y2 = cy + dy;
            btScalar z1 = cz - dz;
            btScalar z2 = cz + dz;

            vcount = 0;
            AddPoint(ref vcount, vertices, x1, y1, z1);
            AddPoint(ref vcount, vertices, x2, y1, z1);
            AddPoint(ref vcount, vertices, x2, y2, z1);
            AddPoint(ref vcount, vertices, x1, y2, z1);
            AddPoint(ref vcount, vertices, x1, y1, z2);
            AddPoint(ref vcount, vertices, x2, y1, z2);
            AddPoint(ref vcount, vertices, x2, y2, z2);
            AddPoint(ref vcount, vertices, x1, y2, z2);

            return true;
        }

        return true;
    }

    private void BringOutYourDead(
        Vector3[] verts,
        uint vcount,
        Vector3[] overts,
        out uint ocount,
        List<uint> indices,
        uint indexcount)
    {
        var tmpIndices = new int[vertexIndexMapping.Count];
        for (int i = 0; i < vertexIndexMapping.Count; i++)
        {
            tmpIndices[i] = vertexIndexMapping[i];
        }

        var usedIndices = new uint[vcount];
        ocount = 0;

        for (int i = 0; i < (int)indexcount; i++)
        {
            uint v = indices[i];
            Debug.Assert(v < vcount);

            if (usedIndices[v] != 0)
            {
                indices[i] = usedIndices[v] - 1;
            }
            else
            {
                indices[i] = ocount;
                overts[ocount] = new Vector3(verts[v].X, verts[v].Y, verts[v].Z);

                for (int k = 0; k < vertexIndexMapping.Count; k++)
                {
                    if (tmpIndices[k] == (int)v)
                    {
                        vertexIndexMapping[k] = (int)ocount;
                    }
                }

                ocount++;
                Debug.Assert(ocount <= vcount);
                usedIndices[v] = ocount;
            }
        }
    }
}
