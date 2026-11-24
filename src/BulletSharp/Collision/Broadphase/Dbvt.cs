using System.Runtime.CompilerServices;
using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btDbvtAabbMm (typedef btDbvtVolume). AABB bounding box (mi/mx).
/// </summary>
public struct DbvtVolume
{
    internal Vector3 mi;
    internal Vector3 mx;

    // btDbvtAabbMm::Mins/Maxs
    public readonly Vector3 Mins => mi;
    public readonly Vector3 Maxs => mx;

    // btDbvtAabbMm::Center
    public readonly Vector3 Center() => (mi + mx) / 2;

    // btDbvtAabbMm::Lengths
    public readonly Vector3 Lengths() => mx - mi;

    // btDbvtAabbMm::Extents
    public readonly Vector3 Extents() => (mx - mi) / 2;

    // btDbvtAabbMm::FromCE
    public static DbvtVolume FromCE(Vector3 c, Vector3 e)
    {
        DbvtVolume box;
        box.mi = c - e;
        box.mx = c + e;
        return box;
    }

    // btDbvtAabbMm::FromCR
    public static DbvtVolume FromCR(Vector3 c, btScalar r) => FromCE(c, new Vector3(r));

    // btDbvtAabbMm::FromMM
    public static DbvtVolume FromMM(Vector3 mi, Vector3 mx)
    {
        DbvtVolume box;
        box.mi = mi;
        box.mx = mx;
        return box;
    }

    // btDbvtAabbMm::FromPoints
    public static DbvtVolume FromPoints(Vector3[] pts, int n)
    {
        DbvtVolume box;
        box.mi = box.mx = pts[0];
        for (int i = 1; i < n; ++i)
        {
            box.mi.SetMin(pts[i]);
            box.mx.SetMax(pts[i]);
        }
        return box;
    }

    // btDbvtAabbMm::Expand
    public void Expand(Vector3 e)
    {
        mi -= e;
        mx += e;
    }

    // btDbvtAabbMm::SignedExpand
    public void SignedExpand(Vector3 e)
    {
        if (e.X > 0) mx.X += e.X; else mi.X += e.X;
        if (e.Y > 0) mx.Y += e.Y; else mi.Y += e.Y;
        if (e.Z > 0) mx.Z += e.Z; else mi.Z += e.Z;
    }

    // btDbvtAabbMm::Contain
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool Contains(DbvtVolume a) => Contains(in a);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal readonly bool Contains(in DbvtVolume a) =>
        mi.X <= a.mi.X && mi.Y <= a.mi.Y && mi.Z <= a.mi.Z &&
        mx.X >= a.mx.X && mx.Y >= a.mx.Y && mx.Z >= a.mx.Z;

    // btDbvtAabbMm::Classify
    public readonly int Classify(Vector3 n, btScalar o, int s)
    {
        Vector3 pi, px;
        switch (s)
        {
            case 0 + 0 + 0:
                px = new Vector3(mi.X, mi.Y, mi.Z);
                pi = new Vector3(mx.X, mx.Y, mx.Z);
                break;
            case 1 + 0 + 0:
                px = new Vector3(mx.X, mi.Y, mi.Z);
                pi = new Vector3(mi.X, mx.Y, mx.Z);
                break;
            case 0 + 2 + 0:
                px = new Vector3(mi.X, mx.Y, mi.Z);
                pi = new Vector3(mx.X, mi.Y, mx.Z);
                break;
            case 1 + 2 + 0:
                px = new Vector3(mx.X, mx.Y, mi.Z);
                pi = new Vector3(mi.X, mi.Y, mx.Z);
                break;
            case 0 + 0 + 4:
                px = new Vector3(mi.X, mi.Y, mx.Z);
                pi = new Vector3(mx.X, mx.Y, mi.Z);
                break;
            case 1 + 0 + 4:
                px = new Vector3(mx.X, mi.Y, mx.Z);
                pi = new Vector3(mi.X, mx.Y, mi.Z);
                break;
            case 0 + 2 + 4:
                px = new Vector3(mi.X, mx.Y, mx.Z);
                pi = new Vector3(mx.X, mi.Y, mi.Z);
                break;
            default: // case 1 + 2 + 4
                px = new Vector3(mx.X, mx.Y, mx.Z);
                pi = new Vector3(mi.X, mi.Y, mi.Z);
                break;
        }
        if (n.Dot(px) + o < 0) return -1;
        if (n.Dot(pi) + o >= 0) return +1;
        return 0;
    }

    // btDbvtAabbMm::ProjectMinimum
    public readonly btScalar ProjectMinimum(Vector3 v, int signs)
    {
        Vector3 p = new(
            ((signs >> 0) & 1) != 0 ? mi.X : mx.X,
            ((signs >> 1) & 1) != 0 ? mi.Y : mx.Y,
            ((signs >> 2) & 1) != 0 ? mi.Z : mx.Z);
        return p.Dot(v);
    }

    // btDbvtAabbMm::AddSpan
    internal readonly void AddSpan(Vector3 d, ref btScalar smi, ref btScalar smx)
    {
        for (int i = 0; i < 3; ++i)
        {
            if (d[i] < 0)
            {
                smi += mx[i] * d[i];
                smx += mi[i] * d[i];
            }
            else
            {
                smi += mi[i] * d[i];
                smx += mx[i] * d[i];
            }
        }
    }

    // Intersect(a, b) (generic DBVT_INT0_IMPL implementation)
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Intersect(DbvtVolume a, DbvtVolume b) => Intersect(in a, in b);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool Intersect(in DbvtVolume a, in DbvtVolume b) =>
        a.mi.X <= b.mx.X && a.mx.X >= b.mi.X &&
        a.mi.Y <= b.mx.Y && a.mx.Y >= b.mi.Y &&
        a.mi.Z <= b.mx.Z && a.mx.Z >= b.mi.Z;

    // Intersect(a, point)
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Intersect(DbvtVolume a, Vector3 b) =>
        b.X >= a.mi.X && b.Y >= a.mi.Y && b.Z >= a.mi.Z &&
        b.X <= a.mx.X && b.Y <= a.mx.Y && b.Z <= a.mx.Z;

    // Proximity
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static btScalar Proximity(DbvtVolume a, DbvtVolume b) => Proximity(in a, in b);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static btScalar Proximity(in DbvtVolume a, in DbvtVolume b)
    {
        Vector3 d = (a.mi + a.mx) - (b.mi + b.mx);
        return BtMath.Abs(d.X) + BtMath.Abs(d.Y) + BtMath.Abs(d.Z);
    }

    // Select (generic DBVT_SELECT_IMPL implementation)
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Select(DbvtVolume o, DbvtVolume a, DbvtVolume b) => Select(in o, in a, in b);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static int Select(in DbvtVolume o, in DbvtVolume a, in DbvtVolume b) =>
        Proximity(in o, in a) < Proximity(in o, in b) ? 0 : 1;

    // Merge (generic DBVT_MERGE_IMPL implementation)
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Merge(DbvtVolume a, DbvtVolume b, out DbvtVolume r) => Merge(in a, in b, out r);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Merge(in DbvtVolume a, in DbvtVolume b, out DbvtVolume r)
    {
        r = default;
        r.mi.X = a.mi.X < b.mi.X ? a.mi.X : b.mi.X;
        r.mi.Y = a.mi.Y < b.mi.Y ? a.mi.Y : b.mi.Y;
        r.mi.Z = a.mi.Z < b.mi.Z ? a.mi.Z : b.mi.Z;
        r.mx.X = a.mx.X > b.mx.X ? a.mx.X : b.mx.X;
        r.mx.Y = a.mx.Y > b.mx.Y ? a.mx.Y : b.mx.Y;
        r.mx.Z = a.mx.Z > b.mx.Z ? a.mx.Z : b.mx.Z;
    }

    // NotEqual
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool NotEqual(DbvtVolume a, DbvtVolume b) => NotEqual(in a, in b);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool NotEqual(in DbvtVolume a, in DbvtVolume b) =>
        a.mi.X != b.mi.X || a.mi.Y != b.mi.Y || a.mi.Z != b.mi.Z ||
        a.mx.X != b.mx.X || a.mx.Y != b.mx.Y || a.mx.Z != b.mx.Z;
}

/// <summary>
/// Ported from btDbvtNode. A leaf node has childs[1] == null; user data is stored in data / dataAsInt.
/// </summary>
public class DbvtNode
{
    internal DbvtVolume volume;
    internal DbvtNode parent;
    internal readonly DbvtNode[] childs = new DbvtNode[2];
    internal object data;
    internal int dataAsInt;
    // Replaces the pointer address comparison (p > n in sort()) used by optimizeIncremental in C++
    internal long allocId;

    public bool IsLeaf => childs[1] == null;
    public bool IsInternal => childs[1] != null;

    public DbvtVolume Volume => volume;
    public DbvtNode Parent => parent;
    public DbvtNode Child0 => childs[0];
    public DbvtNode Child1 => childs[1];

    public object Data
    {
        get => data;
        set => data = value;
    }

    public int DataAsInt
    {
        get => dataAsInt;
        set => dataAsInt = value;
    }
}

/// <summary>
/// Ported from btDbvt: dynamic AABB bounding volume tree.
/// The DBVT_USE_TEMPLATE functions are written as plain methods (policy is ICollide).
/// </summary>
public class Dbvt
{
    /// <summary>btDbvt::ICollide</summary>
    public abstract class ICollide
    {
        public virtual void Process(DbvtNode n0, DbvtNode n1) { }
        public virtual void Process(DbvtNode n) { }
        public virtual void Process(DbvtNode n, btScalar f) => Process(n);
        public virtual bool Descent(DbvtNode n) => true;
        public virtual bool AllLeaves(DbvtNode n) => true;
    }

    // btDbvt constants
    internal const int SimpleStackSize = 64;
    internal const int DoubleStackSize = SimpleStackSize * 2;

    // btDbvt::sStkNN
    private struct StkNn
    {
        public DbvtNode a;
        public DbvtNode b;

        public StkNn(DbvtNode na, DbvtNode nb)
        {
            a = na;
            b = nb;
        }
    }

    // Fields (btDbvt::m_*)
    internal DbvtNode root;
    internal DbvtNode free;
    internal int lkhd = -1;
    internal int leaves;
    internal uint opath;

    // btDbvt fields (m_stkStack is a btAlignedObjectArray in C++; here plain arrays with
    // manual doubling growth — List<T> adds an un-eliminable bounds check + store helper
    // call per push in the hot traversal loop)
    private StkNn[] stkStack = new StkNn[DoubleStackSize];
    private DbvtNode[] leafStack = new DbvtNode[SimpleStackSize];
    private DbvtNode[] rayStack = new DbvtNode[DoubleStackSize];

    private static long nextAllocId;

    public DbvtNode Root => root;
    public int Leaves => leaves;

    // ---- Internal helpers (btDbvt.cpp static functions) ----

    // indexof
    private static int IndexOf(DbvtNode node) => node.parent.childs[1] == node ? 1 : 0;

    // merge + size
    private static DbvtVolume MergeVolumes(DbvtVolume a, DbvtVolume b)
    {
        DbvtVolume.Merge(a, b, out DbvtVolume res);
        return res;
    }

    private static btScalar Size(DbvtVolume a)
    {
        Vector3 edges = a.Lengths();
        return edges.X * edges.Y * edges.Z + edges.X + edges.Y + edges.Z;
    }

    // createnode
    private DbvtNode CreateNode(DbvtNode parent, object data)
    {
        DbvtNode node;
        if (free != null)
        {
            node = free;
            free = null;
        }
        else
        {
            node = new DbvtNode();
            node.allocId = ++nextAllocId;
        }
        node.parent = parent;
        node.data = data;
        node.childs[0] = null;
        node.childs[1] = null;
        return node;
    }

    private DbvtNode CreateNode(DbvtNode parent, DbvtVolume volume, object data)
    {
        DbvtNode node = CreateNode(parent, data);
        node.volume = volume;
        return node;
    }

    private DbvtNode CreateNode(DbvtNode parent, in DbvtVolume volume0, in DbvtVolume volume1, object data)
    {
        DbvtNode node = CreateNode(parent, data);
        DbvtVolume.Merge(in volume0, in volume1, out node.volume);
        return node;
    }

    // deletenode (m_free single-slot reuse)
    private void DeleteNode(DbvtNode node)
    {
        node.parent = null;
        node.childs[0] = null;
        node.childs[1] = null;
        node.data = null;
        free = node;
    }

    // recursedeletenode
    private void RecursiveDeleteNode(DbvtNode node)
    {
        if (node == null) return;
        if (!node.IsLeaf)
        {
            RecursiveDeleteNode(node.childs[0]);
            RecursiveDeleteNode(node.childs[1]);
        }
        if (node == root) root = null;
        DeleteNode(node);
    }

    // insertleaf
    private void InsertLeaf(DbvtNode root, DbvtNode leaf)
    {
        if (this.root == null)
        {
            this.root = leaf;
            leaf.parent = null;
        }
        else
        {
            if (!root.IsLeaf)
            {
                do
                {
                    DbvtNode[] c = root.childs;
                    root = c[DbvtVolume.Select(in leaf.volume, in c[0].volume, in c[1].volume)];
                } while (!root.IsLeaf);
            }
            DbvtNode prev = root.parent;
            DbvtNode node = CreateNode(prev, in leaf.volume, in root.volume, null);
            if (prev != null)
            {
                prev.childs[IndexOf(root)] = node;
                node.childs[0] = root;
                root.parent = node;
                node.childs[1] = leaf;
                leaf.parent = node;
                do
                {
                    if (!prev.volume.Contains(in node.volume))
                    {
                        DbvtNode[] pc = prev.childs;
                        DbvtVolume.Merge(in pc[0].volume, in pc[1].volume, out prev.volume);
                    }
                    else
                    {
                        break;
                    }
                    node = prev;
                } while ((prev = node.parent) != null);
            }
            else
            {
                node.childs[0] = root;
                root.parent = node;
                node.childs[1] = leaf;
                leaf.parent = node;
                this.root = node;
            }
        }
    }

    // removeleaf
    private DbvtNode RemoveLeaf(DbvtNode leaf)
    {
        if (leaf == root)
        {
            root = null;
            return null;
        }
        DbvtNode parent = leaf.parent;
        DbvtNode prev = parent.parent;
        DbvtNode sibling = parent.childs[1 - IndexOf(leaf)];
        if (prev != null)
        {
            prev.childs[IndexOf(parent)] = sibling;
            sibling.parent = prev;
            DeleteNode(parent);
            while (prev != null)
            {
                DbvtVolume pb = prev.volume;
                DbvtNode[] pc = prev.childs;
                DbvtVolume.Merge(in pc[0].volume, in pc[1].volume, out prev.volume);
                if (DbvtVolume.NotEqual(in pb, in prev.volume))
                {
                    prev = prev.parent;
                }
                else
                {
                    break;
                }
            }
            return prev ?? root;
        }
        else
        {
            root = sibling;
            sibling.parent = null;
            DeleteNode(parent);
            return root;
        }
    }

    // fetchleaves
    private void FetchLeaves(DbvtNode root, List<DbvtNode> leaves, int depth = -1)
    {
        if (root.IsInternal && depth != 0)
        {
            FetchLeaves(root.childs[0], leaves, depth - 1);
            FetchLeaves(root.childs[1], leaves, depth - 1);
            DeleteNode(root);
        }
        else
        {
            leaves.Add(root);
        }
    }

    // leftOfAxis
    private static bool LeftOfAxis(DbvtNode node, Vector3 org, Vector3 axis) =>
        axis.Dot(node.volume.Center() - org) <= 0;

    // split (partitions leaves[start, start+count) in place)
    private static int Split(List<DbvtNode> leaves, int start, int count, Vector3 org, Vector3 axis)
    {
        int begin = 0;
        int end = count;
        for (;;)
        {
            while (begin != end && LeftOfAxis(leaves[start + begin], org, axis))
            {
                ++begin;
            }
            if (begin == end) break;
            while (begin != end && !LeftOfAxis(leaves[start + end - 1], org, axis))
            {
                --end;
            }
            if (begin == end) break;
            --end;
            (leaves[start + begin], leaves[start + end]) = (leaves[start + end], leaves[start + begin]);
            ++begin;
        }
        return begin;
    }

    // bounds
    private static DbvtVolume Bounds(List<DbvtNode> leaves, int start, int count)
    {
        DbvtVolume volume = leaves[start].volume;
        for (int i = 1; i < count; ++i)
        {
            DbvtVolume.Merge(volume, leaves[start + i].volume, out volume);
        }
        return volume;
    }

    // bottomup
    private void BottomUp(List<DbvtNode> leaves, int start, int count)
    {
        while (count > 1)
        {
            btScalar minsize = BtMath.SimdInfinity;
            int minidx0 = -1, minidx1 = -1;
            for (int i = 0; i < count; ++i)
            {
                for (int j = i + 1; j < count; ++j)
                {
                    btScalar sz = Size(MergeVolumes(leaves[start + i].volume, leaves[start + j].volume));
                    if (sz < minsize)
                    {
                        minsize = sz;
                        minidx0 = i;
                        minidx1 = j;
                    }
                }
            }
            DbvtNode n0 = leaves[start + minidx0];
            DbvtNode n1 = leaves[start + minidx1];
            DbvtNode p = CreateNode(null, in n0.volume, in n1.volume, null);
            p.childs[0] = n0;
            p.childs[1] = n1;
            n0.parent = p;
            n1.parent = p;
            leaves[start + minidx0] = p;
            leaves[start + minidx1] = leaves[start + count - 1];
            --count;
        }
    }

    private static readonly Vector3[] s_axes = { Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ };

    // topdown
    private DbvtNode TopDown(List<DbvtNode> leaves, int start, int count, int buTreshold)
    {
        if (count > 1)
        {
            if (count > buTreshold)
            {
                DbvtVolume vol = Bounds(leaves, start, count);
                Vector3 org = vol.Center();
                int bestaxis = -1;
                int bestmidp = count;
                int[,] splitcount = new int[3, 2];
                for (int i = 0; i < count; ++i)
                {
                    Vector3 x = leaves[start + i].volume.Center() - org;
                    for (int j = 0; j < 3; ++j)
                    {
                        ++splitcount[j, x.Dot(s_axes[j]) > 0 ? 1 : 0];
                    }
                }
                for (int i = 0; i < 3; ++i)
                {
                    if (splitcount[i, 0] > 0 && splitcount[i, 1] > 0)
                    {
                        int midp = (int)BtMath.Abs((btScalar)(splitcount[i, 0] - splitcount[i, 1]));
                        if (midp < bestmidp)
                        {
                            bestaxis = i;
                            bestmidp = midp;
                        }
                    }
                }
                int partition = bestaxis >= 0
                    ? Split(leaves, start, count, org, s_axes[bestaxis])
                    : count / 2 + 1;
                DbvtNode node = CreateNode(null, vol, null);
                node.childs[0] = TopDown(leaves, start, partition, buTreshold);
                node.childs[1] = TopDown(leaves, start + partition, count - partition, buTreshold);
                node.childs[0].parent = node;
                node.childs[1].parent = node;
                return node;
            }
            else
            {
                BottomUp(leaves, start, count);
                return leaves[start];
            }
        }
        return leaves[start];
    }

    // sort (static sort in btDbvt.cpp; C++ compares pointer addresses p > n, here an allocation sequence number is used instead)
    private DbvtNode SortNode(DbvtNode n)
    {
        DbvtNode p = n.parent;
        if (p != null && p.allocId > n.allocId)
        {
            int i = IndexOf(n);
            int j = 1 - i;
            DbvtNode s = p.childs[j];
            DbvtNode q = p.parent;
            if (q != null)
            {
                q.childs[IndexOf(p)] = n;
            }
            else
            {
                root = n;
            }
            s.parent = n;
            p.parent = n;
            n.parent = q;
            p.childs[0] = n.childs[0];
            p.childs[1] = n.childs[1];
            n.childs[0].parent = p;
            n.childs[1].parent = p;
            n.childs[i] = p;
            n.childs[j] = s;
            (p.volume, n.volume) = (n.volume, p.volume);
            return p;
        }
        return n;
    }

    // ---- API (btDbvt member functions) ----

    // btDbvt::clear
    public void Clear()
    {
        if (root != null) RecursiveDeleteNode(root);
        free = null;
        lkhd = -1;
        Array.Clear(stkStack, 0, stkStack.Length);
        Array.Clear(leafStack, 0, leafStack.Length);
        opath = 0;
    }

    // btDbvt::empty
    public bool Empty() => root == null;

    // btDbvt::optimizeBottomUp
    public void OptimizeBottomUp()
    {
        if (root != null)
        {
            var leaves = new List<DbvtNode>(this.leaves);
            FetchLeaves(root, leaves);
            BottomUp(leaves, 0, leaves.Count);
            root = leaves[0];
        }
    }

    // btDbvt::optimizeTopDown
    public void OptimizeTopDown(int buTreshold = 128)
    {
        if (root != null)
        {
            var leaves = new List<DbvtNode>(this.leaves);
            FetchLeaves(root, leaves);
            root = TopDown(leaves, 0, leaves.Count, buTreshold);
        }
    }

    // btDbvt::optimizeIncremental
    public void OptimizeIncremental(int passes)
    {
        if (passes < 0) passes = leaves;
        if (root != null && passes > 0)
        {
            do
            {
                DbvtNode node = root;
                uint bit = 0;
                while (node.IsInternal)
                {
                    node = SortNode(node).childs[(opath >> (int)bit) & 1];
                    bit = (bit + 1) & (sizeof(uint) * 8 - 1);
                }
                Update(node);
                ++opath;
            } while (--passes != 0);
        }
    }

    // btDbvt::insert
    public DbvtNode Insert(DbvtVolume volume, object data)
    {
        DbvtNode leaf = CreateNode(null, volume, data);
        InsertLeaf(root, leaf);
        ++leaves;
        return leaf;
    }

    public DbvtNode Insert(DbvtVolume volume, int data)
    {
        DbvtNode leaf = CreateNode(null, volume, null);
        leaf.dataAsInt = data;
        InsertLeaf(root, leaf);
        ++leaves;
        return leaf;
    }

    // btDbvt::update(leaf, lookahead)
    public void Update(DbvtNode leaf, int lookahead = -1)
    {
        DbvtNode root = RemoveLeaf(leaf);
        if (root != null)
        {
            if (lookahead >= 0)
            {
                for (int i = 0; i < lookahead && root.parent != null; ++i)
                {
                    root = root.parent;
                }
            }
            else
            {
                root = this.root;
            }
        }
        InsertLeaf(root, leaf);
    }

    // btDbvt::update(leaf, volume)
    public void Update(DbvtNode leaf, DbvtVolume volume)
    {
        DbvtNode root = RemoveLeaf(leaf);
        if (root != null)
        {
            if (lkhd >= 0)
            {
                for (int i = 0; i < lkhd && root.parent != null; ++i)
                {
                    root = root.parent;
                }
            }
            else
            {
                root = this.root;
            }
        }
        leaf.volume = volume;
        InsertLeaf(root, leaf);
    }

    // btDbvt::update(leaf, volume, velocity, margin)
    public bool Update(DbvtNode leaf, DbvtVolume volume, Vector3 velocity, btScalar margin)
    {
        if (leaf.volume.Contains(in volume)) return false;
        volume.Expand(new Vector3(margin));
        volume.SignedExpand(velocity);
        Update(leaf, volume);
        return true;
    }

    // btDbvt::update(leaf, volume, velocity)
    public bool Update(DbvtNode leaf, DbvtVolume volume, Vector3 velocity)
    {
        if (leaf.volume.Contains(in volume)) return false;
        volume.SignedExpand(velocity);
        Update(leaf, volume);
        return true;
    }

    // btDbvt::update(leaf, volume, margin)
    public bool Update(DbvtNode leaf, DbvtVolume volume, btScalar margin)
    {
        if (leaf.volume.Contains(in volume)) return false;
        volume.Expand(new Vector3(margin));
        Update(leaf, volume);
        return true;
    }

    // btDbvt::remove
    public void Remove(DbvtNode leaf)
    {
        RemoveLeaf(leaf);
        DeleteNode(leaf);
        --leaves;
    }

    // btDbvt::maxdepth
    public static int MaxDepth(DbvtNode node)
    {
        int depth = 0;
        if (node != null) GetMaxDepth(node, 1, ref depth);
        return depth;
    }

    private static void GetMaxDepth(DbvtNode node, int depth, ref int maxdepth)
    {
        if (node.IsInternal)
        {
            GetMaxDepth(node.childs[0], depth + 1, ref maxdepth);
            GetMaxDepth(node.childs[1], depth + 1, ref maxdepth);
        }
        else
        {
            maxdepth = BtMath.Max(maxdepth, depth);
        }
    }

    // btDbvt::countLeaves
    public static int CountLeaves(DbvtNode node) =>
        node.IsInternal ? CountLeaves(node.childs[0]) + CountLeaves(node.childs[1]) : 1;

    // btDbvt::extractLeaves
    public static void ExtractLeaves(DbvtNode node, List<DbvtNode> leaves)
    {
        if (node.IsInternal)
        {
            ExtractLeaves(node.childs[0], leaves);
            ExtractLeaves(node.childs[1], leaves);
        }
        else
        {
            leaves.Add(node);
        }
    }

    // btDbvt::collideTT (local stack)
    public void CollideTT(DbvtNode root0, DbvtNode root1, ICollide policy)
    {
        if (root0 != null && root1 != null)
        {
            int depth = 1;
            int treshold = DoubleStackSize - 4;
            var stkStack = new List<StkNn>(DoubleStackSize);
            for (int i = 0; i < DoubleStackSize; ++i) stkStack.Add(default);
            stkStack[0] = new StkNn(root0, root1);
            do
            {
                StkNn p = stkStack[--depth];
                if (depth > treshold)
                {
                    for (int i = stkStack.Count; i < stkStack.Count * 2; ++i) stkStack.Add(default);
                    treshold = stkStack.Count - 4;
                }
                if (p.a == p.b)
                {
                    if (p.a.IsInternal)
                    {
                        stkStack[depth++] = new StkNn(p.a.childs[0], p.a.childs[0]);
                        stkStack[depth++] = new StkNn(p.a.childs[1], p.a.childs[1]);
                        stkStack[depth++] = new StkNn(p.a.childs[0], p.a.childs[1]);
                    }
                }
                else if (DbvtVolume.Intersect(p.a.volume, p.b.volume))
                {
                    if (p.a.IsInternal)
                    {
                        if (p.b.IsInternal)
                        {
                            stkStack[depth++] = new StkNn(p.a.childs[0], p.b.childs[0]);
                            stkStack[depth++] = new StkNn(p.a.childs[1], p.b.childs[0]);
                            stkStack[depth++] = new StkNn(p.a.childs[0], p.b.childs[1]);
                            stkStack[depth++] = new StkNn(p.a.childs[1], p.b.childs[1]);
                        }
                        else
                        {
                            stkStack[depth++] = new StkNn(p.a.childs[0], p.b);
                            stkStack[depth++] = new StkNn(p.a.childs[1], p.b);
                        }
                    }
                    else
                    {
                        if (p.b.IsInternal)
                        {
                            stkStack[depth++] = new StkNn(p.a, p.b.childs[0]);
                            stkStack[depth++] = new StkNn(p.a, p.b.childs[1]);
                        }
                        else
                        {
                            policy.Process(p.a, p.b);
                        }
                    }
                }
            } while (depth != 0);
        }
    }

    // btDbvt::selfCollideTT
    public void SelfCollideTT(DbvtNode root, ICollide policy) => CollideTT(root, root, policy);

    // btDbvt::collideTTpersistentStack (reuses m_stkStack)
    public void CollideTTpersistentStack(DbvtNode root0, DbvtNode root1, ICollide policy)
    {
        if (root0 != null && root1 != null)
        {
            int depth = 1;
            int treshold = DoubleStackSize - 4;
            StkNn[] stkStack = this.stkStack;
            if (stkStack.Length < DoubleStackSize)
            {
                stkStack = new StkNn[DoubleStackSize];
                this.stkStack = stkStack;
            }
            stkStack[0] = new StkNn(root0, root1);
            do
            {
                StkNn p = stkStack[--depth];
                if (depth > treshold)
                {
                    int newSize = stkStack.Length * 2;
                    Array.Resize(ref stkStack, newSize);
                    this.stkStack = stkStack;
                    treshold = newSize - 4;
                }
                if (p.a == p.b)
                {
                    if (p.a.IsInternal)
                    {
                        stkStack[depth++] = new StkNn(p.a.childs[0], p.a.childs[0]);
                        stkStack[depth++] = new StkNn(p.a.childs[1], p.a.childs[1]);
                        stkStack[depth++] = new StkNn(p.a.childs[0], p.a.childs[1]);
                    }
                }
                else if (DbvtVolume.Intersect(in p.a.volume, in p.b.volume))
                {
                    if (p.a.IsInternal)
                    {
                        if (p.b.IsInternal)
                        {
                            stkStack[depth++] = new StkNn(p.a.childs[0], p.b.childs[0]);
                            stkStack[depth++] = new StkNn(p.a.childs[1], p.b.childs[0]);
                            stkStack[depth++] = new StkNn(p.a.childs[0], p.b.childs[1]);
                            stkStack[depth++] = new StkNn(p.a.childs[1], p.b.childs[1]);
                        }
                        else
                        {
                            stkStack[depth++] = new StkNn(p.a.childs[0], p.b);
                            stkStack[depth++] = new StkNn(p.a.childs[1], p.b);
                        }
                    }
                    else
                    {
                        if (p.b.IsInternal)
                        {
                            stkStack[depth++] = new StkNn(p.a, p.b.childs[0]);
                            stkStack[depth++] = new StkNn(p.a, p.b.childs[1]);
                        }
                        else
                        {
                            policy.Process(p.a, p.b);
                        }
                    }
                }
            } while (depth != 0);
        }
    }

    // Specialized collideTTpersistentStack traversal when the second node is known to be a
    // leaf (DbvtBroadphase.SetAabb / setAabbForceUpdate): same LIFO push/pop order as the
    // generic version, but with a plain node array and no (a, b) node-pair bookkeeping.
    public void CollideLeaf(DbvtNode root, DbvtNode leaf, ICollide policy)
    {
        if (root != null && leaf != null)
        {
            int depth = 1;
            int treshold = SimpleStackSize - 2;
            DbvtNode[] stack = leafStack;
            stack[0] = root;
            do
            {
                DbvtNode n = stack[--depth];
                if (n != leaf && DbvtVolume.Intersect(in n.volume, in leaf.volume))
                {
                    if (n.IsInternal)
                    {
                        if (depth > treshold)
                        {
                            int newSize = stack.Length * 2;
                            Array.Resize(ref stack, newSize);
                            leafStack = stack;
                            treshold = newSize - 2;
                        }
                        stack[depth++] = n.childs[0];
                        stack[depth++] = n.childs[1];
                    }
                    else
                    {
                        policy.Process(n, leaf);
                    }
                }
            } while (depth != 0);
        }
    }

    // btDbvt::collideTV
    public void CollideTV(DbvtNode root, DbvtVolume volume, ICollide policy)
    {
        if (root != null)
        {
            var stack = new List<DbvtNode>(SimpleStackSize) { root };
            do
            {
                DbvtNode n = stack[stack.Count - 1];
                stack.RemoveAt(stack.Count - 1);
                if (DbvtVolume.Intersect(n.volume, volume))
                {
                    if (n.IsInternal)
                    {
                        stack.Add(n.childs[0]);
                        stack.Add(n.childs[1]);
                    }
                    else
                    {
                        policy.Process(n);
                    }
                }
            } while (stack.Count > 0);
        }
    }

    // btDbvt::collideTVNoStackAlloc (external stack)
    public void CollideTVNoStackAlloc(DbvtNode root, DbvtVolume volume, List<DbvtNode> stack, ICollide policy)
    {
        if (root != null)
        {
            stack.Clear();
            stack.Add(root);
            do
            {
                DbvtNode n = stack[stack.Count - 1];
                stack.RemoveAt(stack.Count - 1);
                if (DbvtVolume.Intersect(n.volume, volume))
                {
                    if (n.IsInternal)
                    {
                        stack.Add(n.childs[0]);
                        stack.Add(n.childs[1]);
                    }
                    else
                    {
                        policy.Process(n);
                    }
                }
            } while (stack.Count > 0);
        }
    }

    // btDbvt::rayTestInternal (reuses the persistent stack; signs is int[3])
    public void RayTestInternal(DbvtNode root, Vector3 rayFrom, Vector3 rayTo,
        Vector3 rayDirectionInverse, int[] signs, btScalar lambdaMax,
        Vector3 aabbMin, Vector3 aabbMax, ICollide policy)
    {
        if (root != null)
        {
            int depth = 1;
            int treshold = DoubleStackSize - 2;
            DbvtNode[] stack = rayStack;
            int signX = signs[0], signY = signs[1], signZ = signs[2];
            stack[0] = root;
            do
            {
                DbvtNode node = stack[--depth];
                Vector3 bounds0 = node.volume.mi - aabbMax;
                Vector3 bounds1 = node.volume.mx - aabbMin;
                btScalar tmin = 1;
                btScalar lambdaMin = 0;
                bool result1 = AabbUtil.RayAabb2(rayFrom, rayDirectionInverse,
                    signX, signY, signZ, bounds0, bounds1,
                    ref tmin, lambdaMin, lambdaMax);
                if (result1)
                {
                    if (node.IsInternal)
                    {
                        if (depth > treshold)
                        {
                            int newSize = stack.Length * 2;
                            Array.Resize(ref stack, newSize);
                            rayStack = stack;
                            treshold = newSize - 2;
                        }
                        stack[depth++] = node.childs[0];
                        stack[depth++] = node.childs[1];
                    }
                    else
                    {
                        policy.Process(node);
                    }
                }
            } while (depth != 0);
        }
    }

    // btDbvt::rayTest (static, reentrant; stack and signs are locals)
    public static void RayTest(DbvtNode root, Vector3 rayFrom, Vector3 rayTo, ICollide policy)
    {
        if (root != null)
        {
            Vector3 rayDir = rayTo - rayFrom;
            rayDir.Normalize();

            // Use BT_LARGE_FLOAT as the direction inverse on division by zero
            Vector3 rayDirectionInverse = new(
                rayDir.X == 0 ? BtMath.BtLargeFloat : 1 / rayDir.X,
                rayDir.Y == 0 ? BtMath.BtLargeFloat : 1 / rayDir.Y,
                rayDir.Z == 0 ? BtMath.BtLargeFloat : 1 / rayDir.Z);
            int signX = rayDirectionInverse.X < 0 ? 1 : 0;
            int signY = rayDirectionInverse.Y < 0 ? 1 : 0;
            int signZ = rayDirectionInverse.Z < 0 ? 1 : 0;

            btScalar lambdaMax = rayDir.Dot(rayTo - rayFrom);

            int depth = 1;
            int treshold = DoubleStackSize - 2;
            DbvtNode[] stack = new DbvtNode[DoubleStackSize];
            stack[0] = root;
            do
            {
                DbvtNode node = stack[--depth];
                Vector3 bounds0 = node.volume.mi;
                Vector3 bounds1 = node.volume.mx;
                btScalar tmin = 1;
                btScalar lambdaMin = 0;
                bool result1 = AabbUtil.RayAabb2(rayFrom, rayDirectionInverse,
                    signX, signY, signZ, bounds0, bounds1,
                    ref tmin, lambdaMin, lambdaMax);
                if (result1)
                {
                    if (node.IsInternal)
                    {
                        if (depth > treshold)
                        {
                            int newSize = stack.Length * 2;
                            Array.Resize(ref stack, newSize);
                            treshold = newSize - 2;
                        }
                        stack[depth++] = node.childs[0];
                        stack[depth++] = node.childs[1];
                    }
                    else
                    {
                        policy.Process(node);
                    }
                }
            } while (depth != 0);
        }
    }
}
