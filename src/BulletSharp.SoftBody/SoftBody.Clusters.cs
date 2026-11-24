using BulletSharp.Math;

namespace BulletSharp.SoftBody;

/// <summary>
/// Ported from BulletSoftBody/btSoftBody.h and btSoftBody.cpp (cluster dynamics and soft-body joints).
/// </summary>
public partial class SoftBody
{
    private Vector3[] clusterDeltas = Array.Empty<Vector3>();
    private btScalar[] clusterWeights = Array.Empty<btScalar>();

    public int ClusterCount => clusters.Count;

    public List<bool> ClusterConnectivity => clusterConnectivity;

    private int GetNodeIndex(Node node)
    {
        int idx = node.index;
        if ((uint)idx < (uint)nodes.Count && ReferenceEquals(nodes[idx], node))
        {
            return idx;
        }
        return nodes.IndexOf(node);
    }

    public static Vector3 ClusterCom(Cluster cluster)
    {
        Vector3 com = Vector3.Zero;
        for (int i = 0, ni = cluster.nodes.Count; i < ni; ++i)
        {
            com += cluster.nodes[i].x * cluster.masses[i];
        }
        return com * cluster.imass;
    }

    public Vector3 ClusterCom(int cluster) => ClusterCom(clusters[cluster]);

    public static Vector3 ClusterVelocity(Cluster cluster, in Vector3 rpos) =>
        cluster.lv + Vector3.Cross(cluster.av, rpos);

    public static void ClusterVImpulse(Cluster cluster, in Vector3 rpos, in Vector3 impulse) =>
        cluster.ApplyVImpulse(in rpos, in impulse);

    public static void ClusterDImpulse(Cluster cluster, in Vector3 rpos, in Vector3 impulse) =>
        cluster.ApplyDImpulse(in rpos, in impulse);

    public static void ClusterImpulse(Cluster cluster, in Vector3 rpos, in Impulse impulse)
    {
        if (impulse.asVelocity != 0)
        {
            ClusterVImpulse(cluster, in rpos, in impulse.velocity);
        }
        if (impulse.asDrift != 0)
        {
            ClusterDImpulse(cluster, in rpos, in impulse.drift);
        }
    }

    public static void ClusterVAImpulse(Cluster cluster, in Vector3 impulse) =>
        cluster.ApplyVAImpulse(in impulse);

    public static void ClusterDAImpulse(Cluster cluster, in Vector3 impulse) =>
        cluster.ApplyDAImpulse(in impulse);

    public static void ClusterAImpulse(Cluster cluster, in Impulse impulse)
    {
        if (impulse.asVelocity != 0)
        {
            ClusterVAImpulse(cluster, in impulse.velocity);
        }
        if (impulse.asDrift != 0)
        {
            ClusterDAImpulse(cluster, in impulse.drift);
        }
    }

    public static void ClusterDCImpulse(Cluster cluster, in Vector3 impulse) =>
        cluster.ApplyDCImpulse(in impulse);

    public void ReleaseCluster(int index)
    {
        Cluster c = clusters[index];
        if (c.leaf != null)
        {
            cdbvt.Remove(c.leaf);
            c.leaf = null;
        }
        int last = clusters.Count - 1;
        clusters[index] = clusters[last];
        clusters.RemoveAt(last);
    }

    public void ReleaseClusters()
    {
        while (clusters.Count > 0)
        {
            ReleaseCluster(0);
        }
    }

    public int GenerateClusters(int k, int maxiterations = 8192)
    {
        ReleaseClusters();
        int initialCount = System.Math.Max(0, System.Math.Min(k, nodes.Count));
        if (clusters.Capacity < initialCount)
        {
            clusters.Capacity = initialCount;
        }
        for (int i = 0; i < initialCount; ++i)
        {
            clusters.Add(new Cluster { collide = true });
        }

        k = clusters.Count;
        if (k > 0)
        {
            // Initialize
            var centers = new Vector3[k];
            Vector3 cog = Vector3.Zero;
            for (int i = 0; i < nodes.Count; ++i)
            {
                cog += nodes[i].x;
                clusters[(i * 29873) % clusters.Count].nodes.Add(nodes[i]);
            }
            cog /= (btScalar)nodes.Count;
            Array.Fill(centers, cog);

            // Iterate
            const btScalar slope = 16;
            bool changed;
            int iterations = 0;
            do
            {
                btScalar w = (btScalar)2 - BtMath.Min((btScalar)1, (btScalar)iterations / slope);
                changed = false;
                iterations++;

                for (int i = 0; i < k; ++i)
                {
                    Vector3 c = Vector3.Zero;
                    List<Node> cnodes = clusters[i].nodes;
                    for (int j = 0; j < cnodes.Count; ++j)
                    {
                        c += cnodes[j].x;
                    }
                    if (cnodes.Count > 0)
                    {
                        c /= (btScalar)cnodes.Count;
                        c = centers[i] + (c - centers[i]) * w;
                        changed |= (c - centers[i]).LengthSquared() > BtMath.SimdEpsilon;
                        centers[i] = c;
                        cnodes.Clear();
                    }
                }

                for (int i = 0; i < nodes.Count; ++i)
                {
                    Vector3 nx = nodes[i].x;
                    int kbest = 0;
                    btScalar kdist = SoftBodyInternals.ClusterMetric(in centers[0], in nx);
                    for (int j = 1; j < k; ++j)
                    {
                        btScalar d = SoftBodyInternals.ClusterMetric(in centers[j], in nx);
                        if (d < kdist)
                        {
                            kbest = j;
                            kdist = d;
                        }
                    }
                    clusters[kbest].nodes.Add(nodes[i]);
                }
            }
            while (changed && iterations < maxiterations);

            // Merge
            int[] cids = new int[nodes.Count];
            Array.Fill(cids, -1);
            for (int i = 0; i < clusters.Count; ++i)
            {
                List<Node> cnodes = clusters[i].nodes;
                for (int j = 0; j < cnodes.Count; ++j)
                {
                    cids[GetNodeIndex(cnodes[j])] = i;
                }
            }

            int[] idx = new int[3];
            for (int i = 0; i < faces.Count; ++i)
            {
                Face f = faces[i];
                idx[0] = GetNodeIndex(f.n[0]);
                idx[1] = GetNodeIndex(f.n[1]);
                idx[2] = GetNodeIndex(f.n[2]);
                for (int j = 0; j < 3; ++j)
                {
                    int cid = cids[idx[j]];
                    for (int q = 1; q < 3; ++q)
                    {
                        int kid = idx[(j + q) % 3];
                        if (cids[kid] != cid)
                        {
                            Node knode = nodes[kid];
                            if (!clusters[cid].nodes.Contains(knode))
                            {
                                clusters[cid].nodes.Add(knode);
                            }
                        }
                    }
                }
            }

            // Master
            if (clusters.Count > 1)
            {
                var pmaster = new Cluster { collide = false };
                pmaster.nodes.Capacity = nodes.Count;
                for (int i = 0; i < nodes.Count; ++i)
                {
                    pmaster.nodes.Add(nodes[i]);
                }
                clusters.Add(pmaster);
                (clusters[0], clusters[clusters.Count - 1]) = (clusters[clusters.Count - 1], clusters[0]);
            }

            // Terminate
            for (int i = 0; i < clusters.Count; ++i)
            {
                if (clusters[i].nodes.Count == 0)
                {
                    ReleaseCluster(i--);
                }
            }
        }
        else
        {
            // Create a cluster for each tetrahedron (if tetrahedra exist) or each face
            if (tetras.Count > 0)
            {
                clusters.Capacity = System.Math.Max(clusters.Capacity, tetras.Count);
                for (int i = 0; i < tetras.Count; ++i)
                {
                    clusters.Add(new Cluster { collide = true });
                }
                for (int i = 0; i < tetras.Count; ++i)
                {
                    for (int j = 0; j < 4; ++j)
                    {
                        clusters[i].nodes.Add(tetras[i].n[j]);
                    }
                }
            }
            else
            {
                clusters.Capacity = System.Math.Max(clusters.Capacity, faces.Count);
                for (int i = 0; i < faces.Count; ++i)
                {
                    clusters.Add(new Cluster { collide = true });
                }
                for (int i = 0; i < faces.Count; ++i)
                {
                    for (int j = 0; j < 3; ++j)
                    {
                        clusters[i].nodes.Add(faces[i].n[j]);
                    }
                }
            }
        }

        if (clusters.Count > 0)
        {
            InitializeClusters();
            UpdateClusters();

            // For self-collision
            int count = clusters.Count;
            ResizeList(clusterConnectivity, count * count);
            for (int c0 = 0; c0 < count; ++c0)
            {
                clusters[c0].clusterIndex = c0;
                for (int c1 = 0; c1 < count; ++c1)
                {
                    bool connected = false;
                    Cluster cla = clusters[c0];
                    Cluster clb = clusters[c1];
                    for (int i = 0; !connected && i < cla.nodes.Count; ++i)
                    {
                        for (int j = 0; j < clb.nodes.Count; ++j)
                        {
                            if (ReferenceEquals(cla.nodes[i], clb.nodes[j]))
                            {
                                connected = true;
                                break;
                            }
                        }
                    }
                    clusterConnectivity[c0 + c1 * count] = connected;
                }
            }
        }

        return clusters.Count;
    }

    public void InitializeClusters()
    {
        for (int i = 0; i < clusters.Count; ++i)
        {
            Cluster c = clusters[i];
            int nodeCount = c.nodes.Count;
            c.imass = 0;
            ResizeList(c.masses, nodeCount);
            for (int j = 0; j < nodeCount; ++j)
            {
                if (c.nodes[j].im == 0)
                {
                    c.containsAnchor = true;
                    c.masses[j] = BtMath.BtLargeFloat;
                }
                else
                {
                    c.masses[j] = (btScalar)1 / c.nodes[j].im;
                }
                c.imass += c.masses[j];
            }
            c.imass = (btScalar)1 / c.imass;
            c.com = ClusterCom(c);
            c.lv = Vector3.Zero;
            c.av = Vector3.Zero;
            c.leaf = null;

            // Inertia
            btScalar ii00 = 0;
            btScalar ii11 = 0;
            btScalar ii22 = 0;
            btScalar ii01 = 0;
            btScalar ii02 = 0;
            btScalar ii12 = 0;
            for (int j = 0; j < nodeCount; ++j)
            {
                Vector3 k = c.nodes[j].x - c.com;
                Vector3 q = new(k.X * k.X, k.Y * k.Y, k.Z * k.Z);
                btScalar m = c.masses[j];
                ii00 += m * (q.Y + q.Z);
                ii11 += m * (q.X + q.Z);
                ii22 += m * (q.X + q.Y);
                ii01 -= m * k.X * k.Y;
                ii02 -= m * k.X * k.Z;
                ii12 -= m * k.Y * k.Z;
            }
            var ii = new Matrix3x3(
                new Vector3(ii00, ii01, ii02),
                new Vector3(ii01, ii11, ii12),
                new Vector3(ii02, ii12, ii22));
            c.locii = ii.Inverse();

            // Frame
            c.framexform = BulletSharp.Math.Transform.Identity;
            c.framexform.Origin = c.com;
            ResizeList(c.framerefs, nodeCount);
            for (int j = 0; j < nodeCount; ++j)
            {
                c.framerefs[j] = c.nodes[j].x - c.com;
            }
        }
    }

    public void UpdateClusters()
    {
        for (int i = 0; i < clusters.Count; ++i)
        {
            Cluster c = clusters[i];
            int n = c.nodes.Count;
            if (n > 0)
            {
                // Frame
                btScalar eps = (btScalar)0.0001;
                var m = new Matrix3x3(
                    new Vector3(eps * 1, 0, 0),
                    new Vector3(0, eps * 2, 0),
                    new Vector3(0, 0, eps * 3));
                c.com = ClusterCom(c);
                for (int j = 0; j < n; ++j)
                {
                    Vector3 a = c.nodes[j].x - c.com;
                    Vector3 b = c.framerefs[j];
                    m.Row0 += a.X * b;
                    m.Row1 += a.Y * b;
                    m.Row2 += a.Z * b;
                }
                SoftBodyInternals.PolarDecompose(in m, out Matrix3x3 r, out Matrix3x3 _);
                c.framexform.Origin = c.com;
                c.framexform.Basis = r;

                // Inertia (constant local inertia rotated into world)
                c.invwi = c.framexform.Basis * c.locii * c.framexform.Basis.Transpose();

                // Velocities
                c.lv = Vector3.Zero;
                c.av = Vector3.Zero;
                for (int j = 0; j < n; ++j)
                {
                    Vector3 v = c.nodes[j].v * c.masses[j];
                    c.lv += v;
                    c.av += Vector3.Cross(c.nodes[j].x - c.com, v);
                }
                c.lv = c.imass * c.lv * (1 - c.ldamping);
                c.av = c.invwi * c.av * (1 - c.adamping);
                c.vimpulses[0] = Vector3.Zero;
                c.vimpulses[1] = Vector3.Zero;
                c.dimpulses[0] = Vector3.Zero;
                c.dimpulses[1] = Vector3.Zero;
                c.nvimpulses = 0;
                c.ndimpulses = 0;

                // Matching
                if (c.matching > 0)
                {
                    for (int j = 0; j < n; ++j)
                    {
                        Node node = c.nodes[j];
                        Vector3 x = c.framexform * c.framerefs[j];
                        node.x = SoftBodyInternals.Lerp(in node.x, in x, c.matching);
                    }
                }

                // Dbvt
                if (c.collide)
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
    }

    public void CleanupClusters()
    {
        for (int i = 0; i < joints.Count; ++i)
        {
            joints[i].Terminate(sst.sdt);
            if (joints[i].delete)
            {
                int last = joints.Count - 1;
                joints[i] = joints[last];
                joints.RemoveAt(last);
                --i;
            }
        }
    }

    public void PrepareClusters(int iterations)
    {
        for (int i = 0; i < joints.Count; ++i)
        {
            joints[i].Prepare(sst.sdt, iterations);
        }
    }

    public void SolveClusters(btScalar sor)
    {
        for (int i = 0, ni = joints.Count; i < ni; ++i)
        {
            joints[i].Solve(sst.sdt, sor);
        }
    }

    public static void SolveClusters(IReadOnlyList<SoftBody> bodies)
    {
        int nb = bodies.Count;
        int iterations = 0;

        for (int i = 0; i < nb; ++i)
        {
            iterations = System.Math.Max(iterations, bodies[i].cfg.citerations);
        }
        for (int i = 0; i < nb; ++i)
        {
            bodies[i].PrepareClusters(iterations);
        }
        for (int i = 0; i < iterations; ++i)
        {
            const btScalar sor = 1;
            for (int j = 0; j < nb; ++j)
            {
                bodies[j].SolveClusters(sor);
            }
        }
        for (int i = 0; i < nb; ++i)
        {
            bodies[i].CleanupClusters();
        }
    }

    public void ApplyClusters(bool drift)
    {
        if (clusters.Count == 0)
        {
            return;
        }

        int nodeCount = nodes.Count;
        if (clusterDeltas.Length < nodeCount)
        {
            clusterDeltas = new Vector3[nodeCount];
            clusterWeights = new btScalar[nodeCount];
        }
        else
        {
            Array.Clear(clusterDeltas, 0, nodeCount);
            Array.Clear(clusterWeights, 0, nodeCount);
        }

        if (drift)
        {
            for (int i = 0; i < clusters.Count; ++i)
            {
                Cluster c = clusters[i];
                if (c.ndimpulses != 0)
                {
                    c.dimpulses[0] /= (btScalar)c.ndimpulses;
                    c.dimpulses[1] /= (btScalar)c.ndimpulses;
                }
            }
        }

        for (int i = 0; i < clusters.Count; ++i)
        {
            Cluster c = clusters[i];
            if (0 < (drift ? c.ndimpulses : c.nvimpulses))
            {
                Vector3 v = (drift ? c.dimpulses[0] : c.vimpulses[0]) * sst.sdt;
                Vector3 w = (drift ? c.dimpulses[1] : c.vimpulses[1]) * sst.sdt;
                for (int j = 0; j < c.nodes.Count; ++j)
                {
                    int idx = GetNodeIndex(c.nodes[j]);
                    Vector3 x = c.nodes[j].x;
                    btScalar q = c.masses[j];
                    clusterDeltas[idx] += (v + Vector3.Cross(w, x - c.com)) * q;
                    clusterWeights[idx] += q;
                }
            }
        }

        for (int i = 0; i < nodeCount; ++i)
        {
            if (clusterWeights[i] > 0)
            {
                nodes[i].x += clusterDeltas[i] / clusterWeights[i];
            }
        }
    }

    public void DampClusters()
    {
        for (int i = 0; i < clusters.Count; ++i)
        {
            Cluster c = clusters[i];
            if (c.ndamping > 0)
            {
                for (int j = 0; j < c.nodes.Count; ++j)
                {
                    Node n = c.nodes[j];
                    if (n.im > 0)
                    {
                        Vector3 vx = c.lv + Vector3.Cross(c.av, n.q - c.com);
                        if (vx.LengthSquared() <= n.v.LengthSquared())
                        {
                            n.v += c.ndamping * (vx - n.v);
                        }
                    }
                }
            }
        }
    }
}

public abstract partial class Joint
{
    public virtual void Prepare(btScalar dt, int iterations)
    {
        bodies[0].Activate();
        bodies[1].Activate();
    }

    public abstract void Solve(btScalar dt, btScalar sor);

    public abstract void Terminate(btScalar dt);
}

public partial class LJoint
{
    public override void Prepare(btScalar dt, int iterations)
    {
        const btScalar maxdrift = 4;
        base.Prepare(dt, iterations);
        Transform xform0 = bodies[0].Xform();
        Transform xform1 = bodies[1].Xform();
        rpos[0] = xform0 * refs[0];
        rpos[1] = xform1 * refs[1];
        drift = SoftBodyInternals.Clamp(rpos[0] - rpos[1], maxdrift) * erp / dt;
        rpos[0] -= xform0.Origin;
        rpos[1] -= xform1.Origin;
        massmatrix = SoftBodyInternals.ImpulseMatrix(
            bodies[0].InvMass(),
            bodies[0].InvWorldInertia(),
            in rpos[0],
            bodies[1].InvMass(),
            bodies[1].InvWorldInertia(),
            in rpos[1]);
        if (split > 0)
        {
            sdrift = massmatrix * (drift * split);
            drift *= 1 - split;
        }
        drift /= (btScalar)iterations;
    }

    public override void Solve(btScalar dt, btScalar sor)
    {
        Vector3 va = bodies[0].Velocity(in rpos[0]);
        Vector3 vb = bodies[1].Velocity(in rpos[1]);
        Vector3 vr = va - vb;
        Impulse impulse = default;
        impulse.asVelocity = 1;
        impulse.velocity = massmatrix * (drift + vr * cfm) * sor;
        bodies[0].ApplyImpulse(-impulse, in rpos[0]);
        bodies[1].ApplyImpulse(in impulse, in rpos[1]);
    }

    public override void Terminate(btScalar dt)
    {
        if (split > 0)
        {
            bodies[0].ApplyDImpulse(-sdrift, in rpos[0]);
            bodies[1].ApplyDImpulse(in sdrift, in rpos[1]);
        }
    }
}

public partial class AJoint
{
    public override void Prepare(btScalar dt, int iterations)
    {
        const btScalar maxdrift = BtMath.Pi / 16;
        (icontrol ?? IControl.Default()).Prepare(this);
        base.Prepare(dt, iterations);
        axis[0] = bodies[0].Xform().Basis * refs[0];
        axis[1] = bodies[1].Xform().Basis * refs[1];
        drift = SoftBodyInternals.NormalizeAny(Vector3.Cross(axis[1], axis[0]));
        drift *= BtMath.Min(
            maxdrift,
            BtMath.Acos(SoftBodyInternals.Clamp(Vector3.Dot(axis[0], axis[1]), (btScalar)(-1), (btScalar)1)));
        drift *= erp / dt;
        massmatrix = SoftBodyInternals.AngularImpulseMatrix(
            bodies[0].InvWorldInertia(),
            bodies[1].InvWorldInertia());
        if (split > 0)
        {
            sdrift = massmatrix * (drift * split);
            drift *= 1 - split;
        }
        drift /= (btScalar)iterations;
    }

    public override void Solve(btScalar dt, btScalar sor)
    {
        Vector3 va = bodies[0].AngularVelocity();
        Vector3 vb = bodies[1].AngularVelocity();
        Vector3 vr = va - vb;
        btScalar sp = Vector3.Dot(vr, axis[0]);
        Vector3 vc = vr - axis[0] * (icontrol ?? IControl.Default()).Speed(this, sp);
        Impulse impulse = default;
        impulse.asVelocity = 1;
        impulse.velocity = massmatrix * (drift + vc * cfm) * sor;
        bodies[0].ApplyAImpulse(-impulse);
        bodies[1].ApplyAImpulse(in impulse);
    }

    public override void Terminate(btScalar dt)
    {
        if (split > 0)
        {
            bodies[0].ApplyDAImpulse(-sdrift);
            bodies[1].ApplyDAImpulse(in sdrift);
        }
    }
}

public partial class CJoint
{
    public override void Prepare(btScalar dt, int iterations)
    {
        base.Prepare(dt, iterations);
        bool dodrift = life == 0;
        delete = (++life) > maxlife;
        if (dodrift)
        {
            drift = drift * erp / dt;
            if (split > 0)
            {
                sdrift = massmatrix * (drift * split);
                drift *= 1 - split;
            }
            drift /= (btScalar)iterations;
        }
        else
        {
            drift = sdrift = Vector3.Zero;
        }
    }

    public override void Solve(btScalar dt, btScalar sor)
    {
        Vector3 va = bodies[0].Velocity(in rpos[0]);
        Vector3 vb = bodies[1].Velocity(in rpos[1]);
        Vector3 vrel = va - vb;
        btScalar rvac = Vector3.Dot(vrel, normal);
        Impulse impulse = default;
        impulse.asVelocity = 1;
        impulse.velocity = drift;
        if (rvac < 0)
        {
            Vector3 iv = normal * rvac;
            Vector3 fv = vrel - iv;
            impulse.velocity += iv + fv * friction;
        }
        impulse.velocity = massmatrix * impulse.velocity * sor;

        if (bodies[0].soft != null && ReferenceEquals(bodies[0].soft, bodies[1].soft))
        {
            // ReSharper disable EqualExpressionComparison
            if ((impulse.velocity.X == impulse.velocity.X) &&
                (impulse.velocity.Y == impulse.velocity.Y) &&
                (impulse.velocity.Z == impulse.velocity.Z))
            {
                if (impulse.asVelocity != 0)
                {
                    if (impulse.velocity.Length() < bodies[0].soft.maxSelfCollisionImpulse)
                    {
                    }
                    else
                    {
                        bodies[0].ApplyImpulse(-impulse * bodies[0].soft.selfCollisionImpulseFactor, in rpos[0]);
                        bodies[1].ApplyImpulse(impulse * bodies[0].soft.selfCollisionImpulseFactor, in rpos[1]);
                    }
                }
            }
            // ReSharper restore EqualExpressionComparison
        }
        else
        {
            bodies[0].ApplyImpulse(-impulse, in rpos[0]);
            bodies[1].ApplyImpulse(in impulse, in rpos[1]);
        }
    }

    public override void Terminate(btScalar dt)
    {
        if (split > 0)
        {
            bodies[0].ApplyDImpulse(-sdrift, in rpos[0]);
            bodies[1].ApplyDImpulse(in sdrift, in rpos[1]);
        }
    }
}
