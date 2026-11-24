using BulletSharp.Math;

namespace BulletSharp.SoftBody;

/// <summary>
/// Ported from btSoftBody::RayFromToCaster (ray-triangle intersection policy using rayFrom and rayTo).
/// </summary>
public class RayFromToCaster : Dbvt.ICollide
{
    internal Vector3 rayFrom;
    internal Vector3 rayTo;
    internal Vector3 rayNormalizedDirection;
    internal btScalar mint;
    internal Face face;
    internal int tests;

    public RayFromToCaster(in Vector3 rayFrom, in Vector3 rayTo, btScalar mxt)
    {
        this.rayFrom = rayFrom;
        rayNormalizedDirection = rayTo - rayFrom;
        this.rayTo = rayTo;
        mint = mxt;
        face = null;
        tests = 0;
    }

    public Vector3 RayFrom
    {
        get => rayFrom;
        set => rayFrom = value;
    }

    public Vector3 RayTo
    {
        get => rayTo;
        set => rayTo = value;
    }

    public Vector3 RayNormalizedDirection
    {
        get => rayNormalizedDirection;
        set => rayNormalizedDirection = value;
    }

    public btScalar Mint
    {
        get => mint;
        set => mint = value;
    }

    public Face Face
    {
        get => face;
        set => face = value;
    }

    public int Tests
    {
        get => tests;
        set => tests = value;
    }

    public override void Process(DbvtNode leaf)
    {
        Face f = (Face)leaf.data;
        btScalar t = RayFromToTriangle(
            in rayFrom,
            in rayTo,
            in rayNormalizedDirection,
            in f.n[0].x,
            in f.n[1].x,
            in f.n[2].x,
            mint);
        if (t > 0 && t < mint)
        {
            mint = t;
            face = f;
        }
        ++tests;
    }

    public static btScalar RayFromToTriangle(
        in Vector3 rayFrom,
        in Vector3 rayTo,
        in Vector3 rayNormalizedDirection,
        in Vector3 a,
        in Vector3 b,
        in Vector3 c) =>
        RayFromToTriangle(in rayFrom, in rayTo, in rayNormalizedDirection, in a, in b, in c, BtMath.SimdInfinity);

    public static btScalar RayFromToTriangle(
        in Vector3 rayFrom,
        in Vector3 rayTo,
        in Vector3 rayNormalizedDirection,
        in Vector3 a,
        in Vector3 b,
        in Vector3 c,
        btScalar maxt)
    {
        btScalar ceps = -BtMath.SimdEpsilon * 10;
        btScalar teps = BtMath.SimdEpsilon * 10;

        Vector3 n = Vector3.Cross(b - a, c - a);
        btScalar d = Vector3.Dot(a, n);
        btScalar den = Vector3.Dot(rayNormalizedDirection, n);
        if (BtMath.Abs(den) > BtMath.SimdEpsilon)
        {
            btScalar num = Vector3.Dot(rayFrom, n) - d;
            btScalar t = -num / den;
            if (t > teps && t < maxt)
            {
                Vector3 hit = rayFrom + rayNormalizedDirection * t;
                if (Vector3.Dot(n, Vector3.Cross(a - hit, b - hit)) > ceps &&
                    Vector3.Dot(n, Vector3.Cross(b - hit, c - hit)) > ceps &&
                    Vector3.Dot(n, Vector3.Cross(c - hit, a - hit)) > ceps)
                {
                    return t;
                }
            }
        }
        return -1;
    }
}

public partial class SoftBody
{
    private static readonly btScalar SafeEpsilon = BtMath.SimdEpsilon * (btScalar)100;

    private static readonly int[][] s_tetFaces =
    {
        new[] { 0, 1, 2 },
        new[] { 0, 1, 3 },
        new[] { 1, 2, 3 },
        new[] { 0, 2, 3 },
    };

    [ThreadStatic]
    private static Vector3[] t_treePoints;

    private static DbvtNode BuildTreeBottomUp(List<DbvtNode> leafNodes, List<List<int>> adj)
    {
        int nCount = leafNodes.Count;
        if (nCount == 0)
        {
            return null;
        }

        while (nCount > 1)
        {
            bool[] marked = new bool[nCount];
            var newLeafNodes = new List<DbvtNode>();
            var childIds = new List<(int First, int Second)>();

            // Pair adjacent nodes into new parent node
            for (int i = 0; i < nCount; ++i)
            {
                if (marked[i])
                {
                    continue;
                }
                marked[i] = true;

                bool merged = false;
                List<int> adjI = adj[i];
                for (int j = 0; j < adjI.Count; ++j)
                {
                    int n = adjI[j];
                    if (!marked[n])
                    {
                        var node = new DbvtNode
                        {
                            parent = null,
                        };
                        node.childs[0] = leafNodes[i];
                        node.childs[1] = leafNodes[n];
                        DbvtVolume.Merge(leafNodes[i].volume, leafNodes[n].volume, out node.volume);
                        leafNodes[i].parent = node;
                        leafNodes[n].parent = node;
                        newLeafNodes.Add(node);
                        childIds.Add((i, n));
                        merged = true;
                        marked[n] = true;
                        break;
                    }
                }
                if (!merged)
                {
                    newLeafNodes.Add(leafNodes[i]);
                    childIds.Add((i, -1));
                }
            }

            // Fallback when remaining subtrees belong to disconnected components (no remaining edges)
            if (newLeafNodes.Count == nCount)
            {
                while (leafNodes.Count > 1)
                {
                    var paired = new List<DbvtNode>((leafNodes.Count + 1) / 2);
                    for (int i = 0; i < leafNodes.Count; i += 2)
                    {
                        if (i + 1 < leafNodes.Count)
                        {
                            DbvtNode left = leafNodes[i];
                            DbvtNode right = leafNodes[i + 1];
                            var parent = new DbvtNode
                            {
                                parent = null,
                            };
                            parent.childs[0] = left;
                            parent.childs[1] = right;
                            DbvtVolume.Merge(left.volume, right.volume, out parent.volume);
                            left.parent = parent;
                            right.parent = parent;
                            paired.Add(parent);
                        }
                        else
                        {
                            paired.Add(leafNodes[i]);
                        }
                    }
                    leafNodes = paired;
                }
                return leafNodes[0];
            }

            // Update adjacency matrix
            var newAdj = new List<List<int>>(newLeafNodes.Count);
            for (int i = 0; i < newLeafNodes.Count; ++i)
            {
                newAdj.Add(new List<int>());
            }
            for (int i = 0; i < newLeafNodes.Count; ++i)
            {
                for (int j = i + 1; j < newLeafNodes.Count; ++j)
                {
                    bool neighbor = false;
                    List<int> leftChildNeighbors = adj[childIds[i].First];
                    for (int k = 0; k < leftChildNeighbors.Count; ++k)
                    {
                        if (leftChildNeighbors[k] == childIds[j].First ||
                            leftChildNeighbors[k] == childIds[j].Second)
                        {
                            neighbor = true;
                            break;
                        }
                    }
                    if (!neighbor && childIds[i].Second != -1)
                    {
                        List<int> rightChildNeighbors = adj[childIds[i].Second];
                        for (int k = 0; k < rightChildNeighbors.Count; ++k)
                        {
                            if (rightChildNeighbors[k] == childIds[j].First ||
                                rightChildNeighbors[k] == childIds[j].Second)
                            {
                                neighbor = true;
                                break;
                            }
                        }
                    }
                    if (neighbor)
                    {
                        newAdj[i].Add(j);
                        newAdj[j].Add(i);
                    }
                }
            }

            leafNodes = newLeafNodes;
            adj = newAdj;
            nCount = leafNodes.Count;
        }

        return leafNodes[0];
    }

    public void InitializeFaceTree()
    {
        fdbvt.Clear();
        int faceCount = faces.Count;
        var leafNodes = new List<DbvtNode>(faceCount);
        for (int i = 0; i < faceCount; ++i)
        {
            Face f = faces[i];
            DbvtVolume vol = SoftBodyInternals.VolumeOf(f, 0);
            var node = new DbvtNode
            {
                parent = null,
                data = f,
                volume = vol,
            };
            node.childs[1] = null;
            leafNodes.Add(node);
            f.leaf = node;
        }

        var adj = new List<List<int>>(faceCount);
        for (int i = 0; i < faceCount; ++i)
        {
            adj.Add(new List<int>());
        }

        // Construct the adjacency list for triangles sharing an edge (two vertices)
        for (int i = 0; i < faceCount; ++i)
        {
            for (int j = i + 1; j < faceCount; ++j)
            {
                int dup = 0;
                for (int k = 0; k < 3; ++k)
                {
                    for (int l = 0; l < 3; ++l)
                    {
                        if (ReferenceEquals(faces[i].n[k], faces[j].n[l]))
                        {
                            ++dup;
                            break;
                        }
                    }
                    if (dup == 2)
                    {
                        adj[i].Add(j);
                        adj[j].Add(i);
                    }
                }
            }
        }

        fdbvt.root = BuildTreeBottomUp(leafNodes, adj);
        fdbvt.leaves = faceCount;
        UpdateFaceTree(false, false);
        RebuildNodeTree();
    }

    private void RebuildNodeTree()
    {
        ndbvt.Clear();
        int nodeCount = nodes.Count;
        var leafNodes = new List<DbvtNode>(nodeCount);
        for (int i = 0; i < nodeCount; ++i)
        {
            Node n = nodes[i];
            DbvtVolume vol = DbvtVolume.FromCR(n.x, 0);
            var node = new DbvtNode
            {
                parent = null,
                data = n,
                volume = vol,
            };
            node.childs[1] = null;
            leafNodes.Add(node);
            n.leaf = node;
        }

        var adj = new List<List<int>>(nodeCount);
        int[] oldId = new int[nodeCount];
        for (int i = 0; i < nodeCount; ++i)
        {
            adj.Add(new List<int>());
            oldId[i] = nodes[i].index;
            nodes[i].index = i;
        }

        for (int i = 0; i < links.Count; ++i)
        {
            Link l = links[i];
            adj[l.n[0].index].Add(l.n[1].index);
            adj[l.n[1].index].Add(l.n[0].index);
        }

        ndbvt.root = BuildTreeBottomUp(leafNodes, adj);
        ndbvt.leaves = nodeCount;

        for (int i = 0; i < nodeCount; ++i)
        {
            nodes[i].index = oldId[i];
        }
    }

    private void UpdateNode(DbvtNode node, bool useVelocity, bool margin, Vector3[] pts)
    {
        if (node.IsLeaf)
        {
            Node n = (Node)node.data;
            btScalar pad = margin ? sst.radmrg : SafeEpsilon;
            DbvtVolume vol;
            if (useVelocity)
            {
                pts[0] = n.x;
                pts[1] = n.x + sst.sdt * n.v;
                vol = DbvtVolume.FromPoints(pts, 2);
                vol.Expand(new Vector3(pad, pad, pad));
            }
            else
            {
                vol = DbvtVolume.FromCR(n.x, pad);
            }
            node.volume = vol;
        }
        else
        {
            UpdateNode(node.childs[0], useVelocity, margin, pts);
            UpdateNode(node.childs[1], useVelocity, margin, pts);
            DbvtVolume.Merge(node.childs[0].volume, node.childs[1].volume, out node.volume);
        }
    }

    public void UpdateNodeTree(bool useVelocity = true, bool margin = true)
    {
        if (ndbvt.root != null)
        {
            Vector3[] pts = t_treePoints ??= new Vector3[6];
            UpdateNode(ndbvt.root, useVelocity, margin, pts);
        }
    }

    private void UpdateFace(DbvtNode node, bool useVelocity, bool margin, Vector3[] pts)
    {
        if (node.IsLeaf)
        {
            Face f = (Face)node.data;
            btScalar pad = margin ? sst.radmrg : SafeEpsilon;
            DbvtVolume vol;
            if (useVelocity)
            {
                pts[0] = f.n[0].x;
                pts[1] = f.n[0].x + sst.sdt * f.n[0].v;
                pts[2] = f.n[1].x;
                pts[3] = f.n[1].x + sst.sdt * f.n[1].v;
                pts[4] = f.n[2].x;
                pts[5] = f.n[2].x + sst.sdt * f.n[2].v;
                vol = DbvtVolume.FromPoints(pts, 6);
            }
            else
            {
                pts[0] = f.n[0].x;
                pts[1] = f.n[1].x;
                pts[2] = f.n[2].x;
                vol = DbvtVolume.FromPoints(pts, 3);
            }
            vol.Expand(new Vector3(pad, pad, pad));
            node.volume = vol;
        }
        else
        {
            UpdateFace(node.childs[0], useVelocity, margin, pts);
            UpdateFace(node.childs[1], useVelocity, margin, pts);
            DbvtVolume.Merge(node.childs[0].volume, node.childs[1].volume, out node.volume);
        }
    }

    public void UpdateFaceTree(bool useVelocity = true, bool margin = true)
    {
        if (fdbvt.root != null)
        {
            Vector3[] pts = t_treePoints ??= new Vector3[6];
            UpdateFace(fdbvt.root, useVelocity, margin, pts);
        }
    }

    public void UpdateClusterTree()
    {
        for (int i = 0, ni = clusters.Count; i < ni; ++i)
        {
            Cluster c = clusters[i];
            int n = c.nodes.Count;
            if (n > 0 && c.collide)
            {
                Vector3 mi = c.nodes[0].x;
                Vector3 mx = mi;
                for (int j = 1; j < n; ++j)
                {
                    mi.SetMin(c.nodes[j].x);
                    mx.SetMax(c.nodes[j].x);
                }
                DbvtVolume clusterBounds = DbvtVolume.FromMM(mi, mx);
                if (c.leaf != null)
                {
                    cdbvt.Update(c.leaf, clusterBounds, c.lv * sst.sdt * 3, sst.radmrg);
                }
                else
                {
                    c.leaf = cdbvt.Insert(clusterBounds, c);
                }
            }
        }
    }

    private static Vector3 SafeNormalize(Vector3 v)
    {
        btScalar l2 = v.LengthSquared();
        if (l2 >= BtMath.SimdEpsilon * BtMath.SimdEpsilon)
        {
            return v / BtMath.Sqrt(l2);
        }
        return new Vector3(1, 0, 0);
    }

    public void UpdateNormals()
    {
        Vector3 zv = Vector3.Zero;
        for (int i = 0, ni = nodes.Count; i < ni; ++i)
        {
            nodes[i].n = zv;
        }
        for (int i = 0, ni = faces.Count; i < ni; ++i)
        {
            Face f = faces[i];
            Vector3 n = Vector3.Cross(f.n[1].x - f.n[0].x, f.n[2].x - f.n[0].x);
            f.normal = SafeNormalize(n);
            f.n[0].n += n;
            f.n[1].n += n;
            f.n[2].n += n;
        }
        for (int i = 0, ni = nodes.Count; i < ni; ++i)
        {
            btScalar len = nodes[i].n.Length();
            if (len > BtMath.SimdEpsilon)
            {
                nodes[i].n /= len;
            }
        }
    }

    public void UpdateBounds()
    {
        if (nodes.Count > 0)
        {
            Vector3 mins = nodes[0].x;
            Vector3 maxs = nodes[0].x;
            for (int i = 1, ni = nodes.Count; i < ni; ++i)
            {
                mins.SetMin(nodes[i].x);
                maxs.SetMax(nodes[i].x);
            }
            btScalar csm = collisionShape.Margin;
            Vector3 mrg = new(csm, csm, csm);
            bounds[0] = mins - mrg;
            bounds[1] = maxs + mrg;
            if (broadphaseHandle != null && worldInfo?.broadphase != null)
            {
                worldInfo.broadphase.SetAabb(
                    broadphaseHandle,
                    bounds[0],
                    bounds[1],
                    worldInfo.dispatcher);
            }
        }
        else
        {
            bounds[0] = Vector3.Zero;
            bounds[1] = Vector3.Zero;
        }
    }

    private void UpdatePose()
    {
        if (pose.bframe)
        {
            Vector3 com = EvaluateCom();
            pose.com = com;

            btScalar eps = BtMath.SimdEpsilon;
            var apq = new Matrix3x3(
                new Vector3(eps, 0, 0),
                new Vector3(0, eps * 2, 0),
                new Vector3(0, 0, eps * 3));
            for (int i = 0, ni = nodes.Count; i < ni; ++i)
            {
                Vector3 a = pose.wgh[i] * (nodes[i].x - com);
                Vector3 b = pose.pos[i];
                apq.Row0 += a.X * b;
                apq.Row1 += a.Y * b;
                apq.Row2 += a.Z * b;
            }
            SoftBodyInternals.PolarDecompose(in apq, out Matrix3x3 r, out Matrix3x3 _);
            pose.rot = r;
            pose.scl = pose.aqq * r.Transpose() * apq;
            if (cfg.maxvolume > 1)
            {
                btScalar idet = SoftBodyInternals.Clamp(
                    1 / pose.scl.Determinant(),
                    (btScalar)1,
                    cfg.maxvolume);
                pose.scl = SoftBodyInternals.Mul(in pose.scl, idet);
            }
        }
    }

    public void UpdateArea(bool averageArea = true)
    {
        // Face area
        for (int i = 0, ni = faces.Count; i < ni; ++i)
        {
            Face f = faces[i];
            f.ra = SoftBodyInternals.AreaOf(in f.n[0].x, in f.n[1].x, in f.n[2].x);
        }

        // Node area
        if (averageArea)
        {
            int[] counts = new int[nodes.Count];
            for (int i = 0, ni = nodes.Count; i < ni; ++i)
            {
                nodes[i].area = 0;
            }
            for (int i = 0, ni = faces.Count; i < ni; ++i)
            {
                Face f = faces[i];
                for (int j = 0; j < 3; ++j)
                {
                    int index = f.n[j].index;
                    counts[index]++;
                    f.n[j].area += BtMath.Abs(f.ra);
                }
            }
            for (int i = 0, ni = nodes.Count; i < ni; ++i)
            {
                if (counts[i] > 0)
                {
                    nodes[i].area /= (btScalar)counts[i];
                }
                else
                {
                    nodes[i].area = 0;
                }
            }
        }
        else
        {
            for (int i = 0, ni = nodes.Count; i < ni; ++i)
            {
                nodes[i].area = 0;
            }
            for (int i = 0, ni = faces.Count; i < ni; ++i)
            {
                Face f = faces[i];
                for (int j = 0; j < 3; ++j)
                {
                    f.n[j].area += f.ra;
                }
            }
            for (int i = 0, ni = nodes.Count; i < ni; ++i)
            {
                nodes[i].area *= (btScalar)0.3333333f;
            }
        }
    }

    public void UpdateLinkConstants()
    {
        for (int i = 0, ni = links.Count; i < ni; ++i)
        {
            Link l = links[i];
            Material m = l.material;
            l.c0 = (l.n[0].im + l.n[1].im) / m.kLST;
        }
    }

    public void UpdateConstants()
    {
        ResetLinkRestLengths();
        UpdateLinkConstants();
        UpdateArea();
    }

    public void InitializeDmInverse()
    {
        btScalar unitSimplexMeasure = (btScalar)(1.0 / 6.0);

        for (int i = 0, ni = tetras.Count; i < ni; ++i)
        {
            Tetra t = tetras[i];
            Vector3 c1 = t.n[1].x - t.n[0].x;
            Vector3 c2 = t.n[2].x - t.n[0].x;
            Vector3 c3 = t.n[3].x - t.n[0].x;
            var dm = new Matrix3x3(
                new Vector3(c1.X, c2.X, c3.X),
                new Vector3(c1.Y, c2.Y, c3.Y),
                new Vector3(c1.Z, c2.Z, c3.Z));
            t.elementMeasure = dm.Determinant() * unitSimplexMeasure;
            t.dmInverse = dm.Inverse();

            // Calculate the first three columns of P^{-1}
            Vector3 a = t.n[0].x;
            Vector3 b = t.n[1].x;
            Vector3 c = t.n[2].x;
            Vector3 d = t.n[3].x;

            btScalar det = 1 / (
                a.X * b.Y * c.Z - a.X * b.Y * d.Z - a.X * b.Z * c.Y + a.X * b.Z * d.Y + a.X * c.Y * d.Z - a.X * c.Z * d.Y +
                a.Y * (-b.X * c.Z + b.X * d.Z + b.Z * c.X - b.Z * d.X - c.X * d.Z + c.Z * d.X) +
                a.Z * (b.X * c.Y - b.X * d.Y + b.Y * (d.X - c.X) + c.X * d.Y - c.Y * d.X) -
                b.X * c.Y * d.Z + b.X * c.Z * d.Y + b.Y * c.X * d.Z - b.Y * c.Z * d.X - b.Z * c.X * d.Y + b.Z * c.Y * d.X);

            btScalar p11 = -b.Z * c.Y + d.Z * c.Y + b.Y * c.Z + b.Z * d.Y - c.Z * d.Y - b.Y * d.Z;
            btScalar p12 = b.Z * c.X - d.Z * c.X - b.X * c.Z - b.Z * d.X + c.Z * d.X + b.X * d.Z;
            btScalar p13 = -b.Y * c.X + d.Y * c.X + b.X * c.Y + b.Y * d.X - c.Y * d.X - b.X * d.Y;
            btScalar p21 = a.Z * c.Y - d.Z * c.Y - a.Y * c.Z - a.Z * d.Y + c.Z * d.Y + a.Y * d.Z;
            btScalar p22 = -a.Z * c.X + d.Z * c.X + a.X * c.Z + a.Z * d.X - c.Z * d.X - a.X * d.Z;
            btScalar p23 = a.Y * c.X - d.Y * c.X - a.X * c.Y - a.Y * d.X + c.Y * d.X + a.X * d.Y;
            btScalar p31 = -a.Z * b.Y + d.Z * b.Y + a.Y * b.Z + a.Z * d.Y - b.Z * d.Y - a.Y * d.Z;
            btScalar p32 = a.Z * b.X - d.Z * b.X - a.X * b.Z - a.Z * d.X + b.Z * d.X + a.X * d.Z;
            btScalar p33 = -a.Y * b.X + d.Y * b.X + a.X * b.Y + a.Y * d.X - b.Y * d.X - a.X * d.Y;
            btScalar p41 = a.Z * b.Y - c.Z * b.Y - a.Y * b.Z - a.Z * c.Y + b.Z * c.Y + a.Y * c.Z;
            btScalar p42 = -a.Z * b.X + c.Z * b.X + a.X * b.Z + a.Z * c.X - b.Z * c.X - a.X * c.Z;
            btScalar p43 = a.Y * b.X - c.Y * b.X - a.X * b.Y - a.Y * c.X + b.Y * c.X + a.X * c.Y;

            t.pInv[0][0] = p11 * det;
            t.pInv[0][1] = p21 * det;
            t.pInv[0][2] = p31 * det;
            t.pInv[0][3] = p41 * det;

            t.pInv[1][0] = p12 * det;
            t.pInv[1][1] = p22 * det;
            t.pInv[1][2] = p32 * det;
            t.pInv[1][3] = p42 * det;

            t.pInv[2][0] = p13 * det;
            t.pInv[2][1] = p23 * det;
            t.pInv[2][2] = p33 * det;
            t.pInv[2][3] = p43 * det;
        }
    }

    private void ApplyForces()
    {
        btScalar kLF = cfg.kLF;
        btScalar kDG = cfg.kDG;
        btScalar kPR = cfg.kPR;
        btScalar kVC = cfg.kVC;
        bool asLift = kLF > 0;
        bool asDrag = kDG > 0;
        bool asPressure = kPR != 0;
        bool asVolume = kVC > 0;
        bool asAero = asLift || asDrag;
        bool useMedium = asAero;
        bool useVolume = asPressure || asVolume;

        btScalar volume;
        btScalar ivolumetp = 0;
        btScalar dvolumetv = 0;
        if (useVolume)
        {
            volume = GetVolume();
            ivolumetp = 1 / BtMath.Abs(volume) * kPR;
            dvolumetv = (pose.volume - volume) * kVC;
        }

        // Per-vertex forces
        for (int i = 0, ni = nodes.Count; i < ni; ++i)
        {
            Node n = nodes[i];
            if (n.im > 0)
            {
                if (useMedium)
                {
                    AddAeroForceToNode(in windVelocity, i);
                }
                if (asPressure)
                {
                    n.f += n.n * (n.area * ivolumetp);
                }
                if (asVolume)
                {
                    n.f += n.n * (n.area * dvolumetv);
                }
            }
        }

        // Per-face forces
        for (int i = 0, ni = faces.Count; i < ni; ++i)
        {
            AddAeroForceToFace(in windVelocity, i);
        }
    }

    public void PredictMotion(btScalar dt)
    {
        // Update
        if (bUpdateRtCst)
        {
            bUpdateRtCst = false;
            UpdateConstants();
            fdbvt.Clear();
            if ((cfg.collisions & (int)Collisions.VF_SS) != 0)
            {
                InitializeFaceTree();
            }
        }

        // Prepare
        sst.sdt = dt * cfg.timescale;
        sst.isdt = 1 / sst.sdt;
        sst.velmrg = sst.sdt * 3;
        sst.radmrg = collisionShape.Margin;
        sst.updmrg = sst.radmrg * (btScalar)0.25;

        // Forces
        AddVelocity(worldInfo.gravity * sst.sdt);
        ApplyForces();

        // Integrate
        btScalar maxDisplacement = worldInfo.maxDisplacement;
        btScalar clampDeltaV = maxDisplacement / sst.sdt;
        for (int i = 0, ni = nodes.Count; i < ni; ++i)
        {
            Node n = nodes[i];
            n.q = n.x;
            Vector3 deltaV = n.f * n.im * sst.sdt;
            if (deltaV.X > clampDeltaV) deltaV.X = clampDeltaV;
            if (deltaV.X < -clampDeltaV) deltaV.X = -clampDeltaV;
            if (deltaV.Y > clampDeltaV) deltaV.Y = clampDeltaV;
            if (deltaV.Y < -clampDeltaV) deltaV.Y = -clampDeltaV;
            if (deltaV.Z > clampDeltaV) deltaV.Z = clampDeltaV;
            if (deltaV.Z < -clampDeltaV) deltaV.Z = -clampDeltaV;

            n.v += deltaV;
            n.x += n.v * sst.sdt;
            n.f = Vector3.Zero;
        }

        // Clusters
        UpdateClusters();

        // Bounds
        UpdateBounds();

        // Nodes
        for (int i = 0, ni = nodes.Count; i < ni; ++i)
        {
            Node n = nodes[i];
            DbvtVolume vol = DbvtVolume.FromCR(n.x, sst.radmrg);
            ndbvt.Update(n.leaf, vol, n.v * sst.velmrg, sst.updmrg);
        }

        // Faces
        if (!fdbvt.Empty())
        {
            for (int i = 0, ni = faces.Count; i < ni; ++i)
            {
                Face f = faces[i];
                Vector3 v = (f.n[0].v + f.n[1].v + f.n[2].v) / 3;
                DbvtVolume vol = SoftBodyInternals.VolumeOf(f, sst.radmrg);
                fdbvt.Update(f.leaf, vol, v * sst.velmrg, sst.updmrg);
            }
        }

        // Pose
        UpdatePose();

        // Match
        if (pose.bframe && cfg.kMT > 0)
        {
            Matrix3x3 posetrs = pose.rot;
            for (int i = 0, ni = nodes.Count; i < ni; ++i)
            {
                Node n = nodes[i];
                if (n.im > 0)
                {
                    Vector3 targetX = posetrs * pose.pos[i] + pose.com;
                    n.x = SoftBodyInternals.Lerp(in n.x, in targetX, cfg.kMT);
                }
            }
        }

        // Clear contacts
        rcontacts.Clear();
        scontacts.Clear();

        // Optimize DBVTs
        ndbvt.OptimizeIncremental(1);
        fdbvt.OptimizeIncremental(1);
        cdbvt.OptimizeIncremental(1);
    }

    public void IntegrateMotion()
    {
        UpdateNormals();
    }

    public bool RayTest(in Vector3 rayFrom, in Vector3 rayTo, ref sRayCast results)
    {
        if (faces.Count > 0 && fdbvt.Empty())
        {
            InitializeFaceTree();
        }

        results.body = this;
        results.fraction = 1;
        results.feature = FeatureType.None;
        results.index = -1;

        return RayTest(in rayFrom, in rayTo, ref results.fraction, ref results.feature, ref results.index, false) != 0;
    }

    public int RayTest(
        in Vector3 rayFrom,
        in Vector3 rayTo,
        ref btScalar mint,
        ref FeatureType feature,
        ref int index,
        bool bcountonly)
    {
        int cnt = 0;
        Vector3 dir = rayTo - rayFrom;

        if (bcountonly || fdbvt.Empty())
        {
            // Full search
            for (int i = 0, ni = faces.Count; i < ni; ++i)
            {
                Face f = faces[i];
                btScalar t = RayFromToCaster.RayFromToTriangle(
                    in rayFrom,
                    in rayTo,
                    in dir,
                    in f.n[0].x,
                    in f.n[1].x,
                    in f.n[2].x,
                    mint);
                if (t > 0)
                {
                    ++cnt;
                    if (!bcountonly)
                    {
                        feature = FeatureType.Face;
                        index = i;
                        mint = t;
                    }
                }
            }
        }
        else
        {
            // Use DBVT
            var collider = new RayFromToCaster(in rayFrom, in rayTo, mint);
            Dbvt.RayTest(fdbvt.root, rayFrom, rayTo, collider);
            if (collider.face != null)
            {
                mint = collider.mint;
                feature = FeatureType.Face;
                int faceIndex = collider.face.index;
                index = (uint)faceIndex < (uint)faces.Count && ReferenceEquals(faces[faceIndex], collider.face)
                    ? faceIndex
                    : faces.IndexOf(collider.face);
                cnt = 1;
            }
        }

        for (int i = 0, ni = tetras.Count; i < ni; ++i)
        {
            Tetra tet = tetras[i];
            for (int f = 0; f < 4; ++f)
            {
                int[] tf = s_tetFaces[f];
                btScalar t = RayFromToCaster.RayFromToTriangle(
                    in rayFrom,
                    in rayTo,
                    in dir,
                    in tet.n[tf[0]].x,
                    in tet.n[tf[1]].x,
                    in tet.n[tf[2]].x,
                    mint);
                if (t > 0)
                {
                    ++cnt;
                    if (!bcountonly)
                    {
                        feature = FeatureType.Tetra;
                        index = i;
                        mint = t;
                    }
                }
            }
        }

        return cnt;
    }

    public bool RayFaceTest(in Vector3 rayFrom, in Vector3 rayTo, ref sRayCast results)
    {
        if (faces.Count == 0)
        {
            return false;
        }
        if (fdbvt.Empty())
        {
            InitializeFaceTree();
        }

        results.body = this;
        results.fraction = 1;
        results.index = -1;

        return RayFaceTest(in rayFrom, in rayTo, ref results.fraction, ref results.index) != 0;
    }

    public int RayFaceTest(in Vector3 rayFrom, in Vector3 rayTo, ref btScalar mint, ref int index)
    {
        int cnt = 0;
        var collider = new RayFromToCaster(in rayFrom, in rayTo, mint);
        Dbvt.RayTest(fdbvt.root, rayFrom, rayTo, collider);
        if (collider.face != null)
        {
            mint = collider.mint;
            int faceIndex = collider.face.index;
            index = (uint)faceIndex < (uint)faces.Count && ReferenceEquals(faces[faceIndex], collider.face)
                ? faceIndex
                : faces.IndexOf(collider.face);
            cnt = 1;
        }
        return cnt;
    }
}
