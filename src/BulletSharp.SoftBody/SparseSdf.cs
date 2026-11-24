using System.Runtime.CompilerServices;
using BulletSharp.Math;

namespace BulletSharp.SoftBody;

using GjkEpa2 = global::BulletSharp.GjkEpaSolver2;

/// <summary>
/// Ported from BulletSoftBody/btSparseSDF.h (btSparseSdf&lt;3&gt;).
/// Hashed sparse voxel signed-distance field cache with 4x4x4 samples per cell.
/// </summary>
public class SparseSdf
{
    public const int CellSize = 3;
    private const int Side = CellSize + 1;

    public struct IntFrac
    {
        public int b;
        public int i;
        public btScalar f;
    }

    public sealed class Cell
    {
        internal readonly btScalar[] d = new btScalar[Side * Side * Side];
        internal int c0;
        internal int c1;
        internal int c2;
        internal int puid;
        internal uint hash;
        internal CollisionShape pclient;
        internal Cell next;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public btScalar GetD(int i, int j, int k) => d[(i << 4) | (j << 2) | k];

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetD(int i, int j, int k, btScalar value) => d[(i << 4) | (j << 2) | k] = value;
    }

    internal Cell[] cells = Array.Empty<Cell>();
    private readonly Stack<Cell> cellPool = new();
    internal btScalar voxelsz;
    internal btScalar defaultVoxelsz;
    internal int puid;
    internal int ncells;
    internal int clampCells;
    internal int nprobes;
    internal int nqueries;

    public btScalar VoxelSize
    {
        get => voxelsz;
        set => voxelsz = value;
    }

    public btScalar DefaultVoxelSize
    {
        get => defaultVoxelsz;
        set => SetDefaultVoxelsz(value);
    }

    public int Puid
    {
        get => puid;
        set => puid = value;
    }

    public int NumCells => ncells;

    public int ClampCells
    {
        get => clampCells;
        set => clampCells = value;
    }

    public int NumProbes => nprobes;

    public int NumQueries => nqueries;

    public void Initialize(int hashsize = 2383, int clampCells = 256 * 1024)
    {
        this.clampCells = clampCells;
        if (cells.Length != hashsize)
        {
            cells = new Cell[hashsize];
        }
        defaultVoxelsz = (btScalar)0.25;
        Reset();
    }

    public void SetDefaultVoxelsz(btScalar sz)
    {
        defaultVoxelsz = sz;
    }

    public void Reset()
    {
        for (int i = 0, ni = cells.Length; i < ni; ++i)
        {
            cells[i] = null;
        }
        voxelsz = defaultVoxelsz;
        puid = 0;
        ncells = 0;
        nprobes = 1;
        nqueries = 1;
    }

    public void GarbageCollect(int lifetime = 256)
    {
        int life = puid - lifetime;
        for (int i = 0; i < cells.Length; ++i)
        {
            ref Cell root = ref cells[i];
            Cell pp = null;
            Cell pc = root;
            while (pc != null)
            {
                Cell pn = pc.next;
                if (pc.puid < life)
                {
                    if (pp != null)
                        pp.next = pn;
                    else
                        root = pn;
                    cellPool.Push(pc);
                    pc = pp;
                    --ncells;
                }
                pp = pc;
                pc = pn;
            }
        }
        nqueries = 1;
        nprobes = 1;
        ++puid;
    }

    public int RemoveReferences(CollisionShape pcs)
    {
        int refcount = 0;
        for (int i = 0; i < cells.Length; ++i)
        {
            ref Cell root = ref cells[i];
            Cell pp = null;
            Cell pc = root;
            while (pc != null)
            {
                Cell pn = pc.next;
                if (ReferenceEquals(pc.pclient, pcs))
                {
                    if (pp != null)
                        pp.next = pn;
                    else
                        root = pn;
                    pc = pp;
                    --ncells;
                    ++refcount;
                }
                pp = pc;
                pc = pn;
            }
        }
        return refcount;
    }

    public btScalar Evaluate(in Vector3 x, CollisionShape shape, out Vector3 normal, btScalar margin)
    {
        Vector3 scx = x / voxelsz;
        IntFrac ix = Decompose(scx.X);
        IntFrac iy = Decompose(scx.Y);
        IntFrac iz = Decompose(scx.Z);
        uint h = Hash(ix.b, iy.b, iz.b, shape);
        ref Cell root = ref cells[(int)(h % (uint)cells.Length)];
        Cell c = root;
        ++nqueries;
        while (c != null)
        {
            ++nprobes;
            if (c.hash == h &&
                c.c0 == ix.b &&
                c.c1 == iy.b &&
                c.c2 == iz.b &&
                ReferenceEquals(c.pclient, shape))
            {
                break;
            }
            c = c.next;
        }
        if (c == null)
        {
            ++nprobes;
            ++ncells;
            if (ncells > clampCells)
            {
                Reset();
            }

            c = cellPool.TryPop(out Cell pooled) ? pooled : new Cell();
            c.next = root;
            c.pclient = shape;
            c.hash = h;
            c.c0 = ix.b;
            c.c1 = iy.b;
            c.c2 = iz.b;
            root = c;
            BuildCell(c);
        }
        c.puid = puid;

        int o0 = ix.i;
        int o1 = iy.i;
        int o2 = iz.i;
        btScalar d0 = c.GetD(o0 + 0, o1 + 0, o2 + 0);
        btScalar d1 = c.GetD(o0 + 1, o1 + 0, o2 + 0);
        btScalar d2 = c.GetD(o0 + 1, o1 + 1, o2 + 0);
        btScalar d3 = c.GetD(o0 + 0, o1 + 1, o2 + 0);
        btScalar d4 = c.GetD(o0 + 0, o1 + 0, o2 + 1);
        btScalar d5 = c.GetD(o0 + 1, o1 + 0, o2 + 1);
        btScalar d6 = c.GetD(o0 + 1, o1 + 1, o2 + 1);
        btScalar d7 = c.GetD(o0 + 0, o1 + 1, o2 + 1);

        btScalar gx0 = d1 - d0, gx1 = d2 - d3, gx2 = d5 - d4, gx3 = d6 - d7;
        btScalar gy0 = d3 - d0, gy1 = d2 - d1, gy2 = d7 - d4, gy3 = d6 - d5;
        btScalar gz0 = d4 - d0, gz1 = d5 - d1, gz2 = d7 - d3, gz3 = d6 - d2;

        normal = new Vector3(
            Lerp(Lerp(gx0, gx1, iy.f), Lerp(gx2, gx3, iy.f), iz.f),
            Lerp(Lerp(gy0, gy1, ix.f), Lerp(gy2, gy3, ix.f), iz.f),
            Lerp(Lerp(gz0, gz1, ix.f), Lerp(gz2, gz3, ix.f), iy.f));

        // btVector3::safeNormalize()
        btScalar l2 = normal.X * normal.X + normal.Y * normal.Y + normal.Z * normal.Z;
        if (l2 >= BtMath.SimdEpsilon * BtMath.SimdEpsilon)
        {
            normal /= BtMath.Sqrt(l2);
        }
        else
        {
            normal = new Vector3(1, 0, 0);
        }

        btScalar dist0 = Lerp(Lerp(d0, d1, ix.f), Lerp(d3, d2, ix.f), iy.f);
        btScalar dist1 = Lerp(Lerp(d4, d5, ix.f), Lerp(d7, d6, ix.f), iy.f);
        return Lerp(dist0, dist1, iz.f) - margin;
    }

    public void BuildCell(Cell c)
    {
        Vector3 org = new Vector3((btScalar)c.c0, (btScalar)c.c1, (btScalar)c.c2) * CellSize * voxelsz;
        for (int k = 0; k <= CellSize; ++k)
        {
            btScalar z = voxelsz * k + org.Z;
            for (int j = 0; j <= CellSize; ++j)
            {
                btScalar y = voxelsz * j + org.Y;
                for (int i = 0; i <= CellSize; ++i)
                {
                    btScalar x = voxelsz * i + org.X;
                    c.SetD(i, j, k, DistanceToShape(new Vector3(x, y, z), c.pclient));
                }
            }
        }
    }

    public static btScalar DistanceToShape(in Vector3 x, CollisionShape shape)
    {
        Transform unit = Transform.Identity;
        if (shape != null && shape.IsConvex)
        {
            GjkEpa2.Results res = default;
            return GjkEpa2.SignedDistance(x, 0, (ConvexShape)shape, in unit, ref res);
        }
        return 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static IntFrac Decompose(btScalar x)
    {
        IntFrac r;
        x /= CellSize;
        int o = x < 0 ? (int)(-x + 1) : 0;
        x += o;
        r.b = (int)x;
        btScalar k = (x - r.b) * CellSize;
        r.i = (int)k;
        r.f = k - r.i;
        r.b -= o;
        return r;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Lerp(btScalar a, btScalar b, btScalar t) => a + (b - a) * t;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static uint HsiehStep(uint hash, uint word)
    {
        unchecked
        {
            hash += word & 0xFFFFu;
            uint tmp = ((word >> 16) << 11) ^ hash;
            hash = (hash << 16) ^ tmp;
            hash += hash >> 11;
            return hash;
        }
    }

    /// <summary>Paul Hsieh SuperFastHash over (x, y, z, 0, shapeIdentity, 0) (24 bytes).</summary>
    public static uint Hash(int x, int y, int z, CollisionShape shape)
    {
        unchecked
        {
            uint shapeId = shape != null ? (uint)RuntimeHelpers.GetHashCode(shape) : 0u;
            uint hash = 24u;
            hash = HsiehStep(hash, (uint)x);
            hash = HsiehStep(hash, (uint)y);
            hash = HsiehStep(hash, (uint)z);
            hash = HsiehStep(hash, 0u);
            hash = HsiehStep(hash, shapeId);
            hash = HsiehStep(hash, 0u);

            hash ^= hash << 3;
            hash += hash >> 5;
            hash ^= hash << 4;
            hash += hash >> 17;
            hash ^= hash << 25;
            hash += hash >> 6;
            return hash;
        }
    }
}
