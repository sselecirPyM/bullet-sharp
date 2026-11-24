using System.Globalization;
using BulletSharp.Math;

namespace BulletSharp.SoftBody;

/// <summary>
/// Ported from BulletSoftBody/btSoftBodyHelpers.h and btSoftBodyHelpers.cpp (debug drawing, geometry factory, mesh optimization, and barycentric weight helpers).
/// </summary>
public static partial class SoftBodyHelpers
{
    /// <summary>
    /// Ported from btSoftBodyHelpers::Draw.
    /// Note: Cluster convex hulls use <see cref="HullLibrary"/> (the <c>#else</c> path in Bullet C++)
    /// since <c>btConvexHullComputer</c> is not ported, and cluster colors use a local deterministic
    /// LCG seeded with <c>1806u</c> instead of global <c>srand</c>/<c>rand</c>.
    /// </summary>
    public static void Draw(
        SoftBody psb,
        IDebugDraw idraw,
        DrawFlags drawflags = DrawFlags.Std)
    {
        if (psb == null || idraw == null)
        {
            return;
        }

        const btScalar scl = (btScalar)0.1;
        const btScalar nscl = scl * 5;
        Vector3 lcolor = new(0, 0, 0);
        Vector3 ncolor = new(1, 1, 1);
        Vector3 ccolor = new(1, 0, 0);

        // Clusters
        if ((drawflags & DrawFlags.Clusters) != 0)
        {
            uint seed = 1806u;
            HullLibrary hlib = null;
            List<Vector3> vertices = null;
            for (int i = 0; i < psb.clusters.Count; ++i)
            {
                Cluster cluster = psb.clusters[i];
                if (cluster.collide)
                {
                    Vector3 color = new(NextRand01(ref seed), NextRand01(ref seed), NextRand01(ref seed));
                    color = color.Normalized() * (btScalar)0.75;

                    int nj = cluster.nodes.Count;
                    vertices ??= new List<Vector3>(nj);
                    vertices.Clear();
                    if (vertices.Capacity < nj)
                    {
                        vertices.Capacity = nj;
                    }
                    for (int j = 0; j < nj; ++j)
                    {
                        vertices.Add(cluster.nodes[j].x);
                    }

                    var hdsc = new HullDesc(HullFlag.Triangles, vertices.Count, vertices)
                    {
                        MaxVertices = (uint)vertices.Count,
                    };
                    var hres = new HullResult();
                    hlib ??= new HullLibrary();
                    if (hlib.CreateConvexHull(hdsc, hres) == HullError.Ok && hres.OutputVertices.Count > 0)
                    {
                        Vector3 center = Average(hres.OutputVertices);
                        Vector3 negCenter = -center;
                        Add(hres.OutputVertices, in negCenter);
                        Mul(hres.OutputVertices, (btScalar)1);
                        Add(hres.OutputVertices, in center);
                        for (int j = 0; j < (int)hres.NumFaces; ++j)
                        {
                            Vector3 v0 = hres.OutputVertices[(int)hres.Indices[j * 3 + 0]];
                            Vector3 v1 = hres.OutputVertices[(int)hres.Indices[j * 3 + 1]];
                            Vector3 v2 = hres.OutputVertices[(int)hres.Indices[j * 3 + 2]];
                            idraw.DrawTriangle(ref v0, ref v1, ref v2, ref color, (btScalar)1);
                        }
                    }
                    else if (vertices.Count == 3)
                    {
                        Vector3 v0 = vertices[0];
                        Vector3 v1 = vertices[1];
                        Vector3 v2 = vertices[2];
                        idraw.DrawTriangle(ref v0, ref v1, ref v2, ref color, (btScalar)1);
                    }
                    else if (vertices.Count == 2)
                    {
                        Vector3 v0 = vertices[0];
                        Vector3 v1 = vertices[1];
                        idraw.DrawLine(ref v0, ref v1, ref color);
                    }
                    else if (vertices.Count == 1)
                    {
                        Vector3 v0 = vertices[0];
                        DrawVertex(idraw, in v0, scl, in color);
                    }
                    hlib.ReleaseResult(hres);
                }
            }
        }
        else
        {
            // Nodes
            if ((drawflags & DrawFlags.Nodes) != 0)
            {
                Vector3 red = new(1, 0, 0);
                Vector3 green = new(0, 1, 0);
                Vector3 blue = new(0, 0, 1);
                Vector3 dx = new(scl, 0, 0);
                Vector3 dy = new(0, scl, 0);
                Vector3 dz = new(0, 0, scl);
                for (int i = 0; i < psb.nodes.Count; ++i)
                {
                    Node n = psb.nodes[i];
                    if (n.material != null && (n.material.Flags & MaterialFlags.DebugDraw) == 0) continue;
                    Vector3 a = n.x - dx;
                    Vector3 b = n.x + dx;
                    idraw.DrawLine(ref a, ref b, ref red);
                    a = n.x - dy;
                    b = n.x + dy;
                    idraw.DrawLine(ref a, ref b, ref green);
                    a = n.x - dz;
                    b = n.x + dz;
                    idraw.DrawLine(ref a, ref b, ref blue);
                }
            }

            // Links
            if ((drawflags & DrawFlags.Links) != 0)
            {
                for (int i = 0; i < psb.links.Count; ++i)
                {
                    Link l = psb.links[i];
                    if (l.material != null && (l.material.Flags & MaterialFlags.DebugDraw) == 0) continue;
                    Vector3 a = l.n[0].x;
                    Vector3 b = l.n[1].x;
                    idraw.DrawLine(ref a, ref b, ref lcolor);
                }
            }

            // Normals
            if ((drawflags & DrawFlags.Normals) != 0)
            {
                Vector3 halfNColor = ncolor * (btScalar)0.5;
                for (int i = 0; i < psb.nodes.Count; ++i)
                {
                    Node n = psb.nodes[i];
                    if (n.material != null && (n.material.Flags & MaterialFlags.DebugDraw) == 0) continue;
                    Vector3 d = n.n * nscl;
                    Vector3 from = n.x;
                    Vector3 toPos = n.x + d;
                    Vector3 toNeg = n.x - d;
                    idraw.DrawLine(ref from, ref toPos, ref ncolor);
                    idraw.DrawLine(ref from, ref toNeg, ref halfNColor);
                }
            }

            // Contacts
            if ((drawflags & DrawFlags.Contacts) != 0)
            {
                Vector3 yellow = new(1, 1, 0);
                btScalar epsSq = BtMath.SimdEpsilon * BtMath.SimdEpsilon;
                for (int i = 0; i < psb.rcontacts.Count; ++i)
                {
                    RContact c = psb.rcontacts[i];
                    Vector3 o = c.node.x - c.cti.normal * (Vector3.Dot(c.node.x, c.cti.normal) + c.cti.offset);
                    Vector3 rawX = Vector3.Cross(c.cti.normal, s_contactAxes[c.cti.normal.MinAxis()]);
                    if (rawX.LengthSquared() <= epsSq)
                    {
                        rawX = Vector3.Cross(c.cti.normal, s_contactAxes[c.cti.normal.FurthestAxis()]);
                    }
                    Vector3 x = rawX.LengthSquared() > epsSq ? rawX.Normalized() : Vector3.Zero;
                    Vector3 rawY = Vector3.Cross(x, c.cti.normal);
                    Vector3 y = rawY.LengthSquared() > epsSq ? rawY.Normalized() : Vector3.Zero;
                    Vector3 x0 = o - x * nscl;
                    Vector3 x1 = o + x * nscl;
                    Vector3 y0 = o - y * nscl;
                    Vector3 y1 = o + y * nscl;
                    Vector3 n1 = o + c.cti.normal * nscl * 3;
                    idraw.DrawLine(ref x0, ref x1, ref ccolor);
                    idraw.DrawLine(ref y0, ref y1, ref ccolor);
                    idraw.DrawLine(ref o, ref n1, ref yellow);
                }
            }

            // Faces
            if ((drawflags & DrawFlags.Faces) != 0)
            {
                const btScalar faceScl = (btScalar)0.8;
                const btScalar alp = (btScalar)1;
                Vector3 col = new(0, (btScalar)0.7, 0);
                for (int i = 0; i < psb.faces.Count; ++i)
                {
                    Face f = psb.faces[i];
                    if (f.material != null && (f.material.Flags & MaterialFlags.DebugDraw) == 0) continue;
                    Vector3 x0 = f.n[0].x;
                    Vector3 x1 = f.n[1].x;
                    Vector3 x2 = f.n[2].x;
                    Vector3 c = (x0 + x1 + x2) / 3;
                    Vector3 v0 = (x0 - c) * faceScl + c;
                    Vector3 v1 = (x1 - c) * faceScl + c;
                    Vector3 v2 = (x2 - c) * faceScl + c;
                    idraw.DrawTriangle(ref v0, ref v1, ref v2, ref col, alp);
                }
            }

            // Tetras
            if ((drawflags & DrawFlags.Tetras) != 0)
            {
                const btScalar tetraScl = (btScalar)0.8;
                const btScalar alp = (btScalar)1;
                Vector3 col = new((btScalar)0.3, (btScalar)0.3, (btScalar)0.7);
                for (int i = 0; i < psb.tetras.Count; ++i)
                {
                    Tetra t = psb.tetras[i];
                    if (t.material != null && (t.material.Flags & MaterialFlags.DebugDraw) == 0) continue;
                    Vector3 x0 = t.n[0].x;
                    Vector3 x1 = t.n[1].x;
                    Vector3 x2 = t.n[2].x;
                    Vector3 x3 = t.n[3].x;
                    Vector3 c = (x0 + x1 + x2 + x3) / 4;
                    Vector3 v0 = (x0 - c) * tetraScl + c;
                    Vector3 v1 = (x1 - c) * tetraScl + c;
                    Vector3 v2 = (x2 - c) * tetraScl + c;
                    Vector3 v3 = (x3 - c) * tetraScl + c;
                    idraw.DrawTriangle(ref v0, ref v1, ref v2, ref col, alp);
                    idraw.DrawTriangle(ref v0, ref v1, ref v3, ref col, alp);
                    idraw.DrawTriangle(ref v1, ref v2, ref v3, ref col, alp);
                    idraw.DrawTriangle(ref v2, ref v0, ref v3, ref col, alp);
                }
            }
        }

        // Anchors
        if ((drawflags & DrawFlags.Anchors) != 0)
        {
            Vector3 red = new(1, 0, 0);
            Vector3 green = new(0, 1, 0);
            Vector3 white = new(1, 1, 1);
            for (int i = 0; i < psb.anchors.Count; ++i)
            {
                Anchor a = psb.anchors[i];
                Vector3 p = a.node.x;
                Vector3 q = a.body.worldTransform * a.local;
                DrawVertex(idraw, in p, (btScalar)0.25, in red);
                DrawVertex(idraw, in q, (btScalar)0.25, in green);
                idraw.DrawLine(ref p, ref q, ref white);
            }
            for (int i = 0; i < psb.nodes.Count; ++i)
            {
                Node n = psb.nodes[i];
                if (n.material != null && (n.material.Flags & MaterialFlags.DebugDraw) == 0) continue;
                if (n.im <= 0)
                {
                    DrawVertex(idraw, in n.x, (btScalar)0.25, in red);
                }
            }
        }

        // Notes
        if ((drawflags & DrawFlags.Notes) != 0)
        {
            for (int i = 0; i < psb.notes.Count; ++i)
            {
                Note n = psb.notes[i];
                Vector3 p = n.offset;
                for (int j = 0; j < n.rank; ++j)
                {
                    p += n.nodes[j].x * n.coords[j];
                }
                idraw.Draw3DText(ref p, n.text);
            }
        }

        // Node tree
        if ((drawflags & DrawFlags.NodeTree) != 0) DrawNodeTree(psb, idraw);
        // Face tree
        if ((drawflags & DrawFlags.FaceTree) != 0) DrawFaceTree(psb, idraw);
        // Cluster tree
        if ((drawflags & DrawFlags.ClusterTree) != 0) DrawClusterTree(psb, idraw);

        // Joints
        if ((drawflags & DrawFlags.Joints) != 0)
        {
            Vector3 yellow = new(1, 1, 0);
            Vector3 cyan = new(0, 1, 1);
            for (int i = 0; i < psb.joints.Count; ++i)
            {
                Joint pj = psb.joints[i];
                switch (pj.Type)
                {
                    case JointType.Linear:
                    {
                        Transform xf0 = pj.bodies[0].Xform();
                        Transform xf1 = pj.bodies[1].Xform();
                        Vector3 a0 = xf0 * pj.refs[0];
                        Vector3 a1 = xf1 * pj.refs[1];
                        Vector3 o0 = xf0.Origin;
                        Vector3 o1 = xf1.Origin;
                        idraw.DrawLine(ref o0, ref a0, ref yellow);
                        idraw.DrawLine(ref o1, ref a1, ref cyan);
                        DrawVertex(idraw, in a0, (btScalar)0.25, in yellow);
                        DrawVertex(idraw, in a1, (btScalar)0.25, in cyan);
                        break;
                    }
                    case JointType.Angular:
                    {
                        Transform xf0 = pj.bodies[0].Xform();
                        Transform xf1 = pj.bodies[1].Xform();
                        Vector3 o0 = xf0.Origin;
                        Vector3 o1 = xf1.Origin;
                        Vector3 a0 = xf0.Basis * pj.refs[0];
                        Vector3 a1 = xf1.Basis * pj.refs[1];
                        Vector3 o0a0 = o0 + a0 * 10;
                        Vector3 o0a1 = o0 + a1 * 10;
                        Vector3 o1a0 = o1 + a0 * 10;
                        Vector3 o1a1 = o1 + a1 * 10;
                        idraw.DrawLine(ref o0, ref o0a0, ref yellow);
                        idraw.DrawLine(ref o0, ref o0a1, ref yellow);
                        idraw.DrawLine(ref o1, ref o1a0, ref cyan);
                        idraw.DrawLine(ref o1, ref o1a1, ref cyan);
                        break;
                    }
                }
            }
        }
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::Draw.
    /// </summary>
    public static void Draw(SoftBody psb, IDebugDraw idraw, int drawflags) =>
        Draw(psb, idraw, (DrawFlags)drawflags);

    /// <summary>
    /// Ported from btSoftBodyHelpers::DrawInfos.
    /// </summary>
    public static void DrawInfos(
        SoftBody psb,
        IDebugDraw idraw,
        bool masses,
        bool areas,
        bool stress)
    {
        _ = stress;
        if (psb == null || idraw == null || (!masses && !areas))
        {
            return;
        }

        for (int i = 0; i < psb.nodes.Count; ++i)
        {
            Node n = psb.nodes[i];
            string text;
            if (masses && areas)
            {
                btScalar mass = (btScalar)1 / n.im;
                text = " M(" + mass.ToString("F2", CultureInfo.InvariantCulture) +
                       ") A(" + n.area.ToString("F2", CultureInfo.InvariantCulture) + ")";
            }
            else if (masses)
            {
                btScalar mass = (btScalar)1 / n.im;
                text = " M(" + mass.ToString("F2", CultureInfo.InvariantCulture) + ")";
            }
            else
            {
                text = " A(" + n.area.ToString("F2", CultureInfo.InvariantCulture) + ")";
            }

            Vector3 pos = n.x;
            idraw.Draw3DText(ref pos, text);
        }
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::DrawNodeTree.
    /// </summary>
    public static void DrawNodeTree(
        SoftBody psb,
        IDebugDraw idraw,
        int mindepth = 0,
        int maxdepth = -1)
    {
        if (psb == null || idraw == null)
        {
            return;
        }

        Vector3 ncolor = new(1, 0, 1);
        Vector3 lcolor = new(1, 1, 1);
        DrawTree(idraw, psb.ndbvt.Root, 0, in ncolor, in lcolor, mindepth, maxdepth);
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::DrawFaceTree.
    /// </summary>
    public static void DrawFaceTree(
        SoftBody psb,
        IDebugDraw idraw,
        int mindepth = 0,
        int maxdepth = -1)
    {
        if (psb == null || idraw == null)
        {
            return;
        }

        Vector3 ncolor = new(0, 1, 0);
        Vector3 lcolor = new(1, 0, 0);
        DrawTree(idraw, psb.fdbvt.Root, 0, in ncolor, in lcolor, mindepth, maxdepth);
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::DrawClusterTree.
    /// </summary>
    public static void DrawClusterTree(
        SoftBody psb,
        IDebugDraw idraw,
        int mindepth = 0,
        int maxdepth = -1)
    {
        if (psb == null || idraw == null)
        {
            return;
        }

        Vector3 ncolor = new(0, 1, 1);
        Vector3 lcolor = new(1, 0, 0);
        DrawTree(idraw, psb.cdbvt.Root, 0, in ncolor, in lcolor, mindepth, maxdepth);
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::DrawFrame.
    /// </summary>
    public static void DrawFrame(SoftBody psb, IDebugDraw idraw)
    {
        if (psb != null && idraw != null && psb.pose.bframe)
        {
            const btScalar ascl = 10;
            const btScalar nscl = (btScalar)0.1;
            Vector3 com = psb.pose.com;
            Matrix3x3 trs = psb.pose.rot * psb.pose.scl;
            Vector3 xAxis = (trs * new Vector3(1, 0, 0)).Normalized();
            Vector3 yAxis = (trs * new Vector3(0, 1, 0)).Normalized();
            Vector3 zAxis = (trs * new Vector3(0, 0, 1)).Normalized();
            Vector3 red = new(1, 0, 0);
            Vector3 green = new(0, 1, 0);
            Vector3 blue = new(0, 0, 1);
            Vector3 magenta = new(1, 0, 1);
            Vector3 toX = com + xAxis * ascl;
            Vector3 toY = com + yAxis * ascl;
            Vector3 toZ = com + zAxis * ascl;
            idraw.DrawLine(ref com, ref toX, ref red);
            idraw.DrawLine(ref com, ref toY, ref green);
            idraw.DrawLine(ref com, ref toZ, ref blue);
            for (int i = 0; i < psb.pose.pos.Count; ++i)
            {
                Vector3 x = com + trs * psb.pose.pos[i];
                DrawVertex(idraw, in x, nscl, in magenta);
            }
        }
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::CreateRope.
    /// </summary>
    public static SoftBody CreateRope(
        SoftBodyWorldInfo worldInfo,
        in Vector3 from,
        in Vector3 to,
        int res,
        int fixeds)
    {
        int r = res + 2;
        var x = new Vector3[r];
        var m = new btScalar[r];

        for (int i = 0; i < r; ++i)
        {
            btScalar t = i / (btScalar)(r - 1);
            x[i] = SoftBodyInternals.Lerp(in from, in to, t);
            m[i] = 1;
        }

        var psb = new SoftBody(worldInfo, r, x, m);
        if ((fixeds & 1) != 0) psb.SetMass(0, 0);
        if ((fixeds & 2) != 0) psb.SetMass(r - 1, 0);

        for (int i = 1; i < r; ++i)
        {
            psb.AppendLink(i - 1, i);
        }

        return psb;
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::CreatePatch.
    /// Note: When <paramref name="perturbation"/> is non-zero, uses a local deterministic LCG
    /// instead of C++ global <c>rand()</c> to avoid shared mutable state.
    /// </summary>
    public static SoftBody CreatePatch(
        SoftBodyWorldInfo worldInfo,
        in Vector3 corner00,
        in Vector3 corner10,
        in Vector3 corner01,
        in Vector3 corner11,
        int resx,
        int resy,
        int fixeds,
        bool gendiags,
        btScalar perturbation = 0)
    {
        if (resx < 2 || resy < 2)
        {
            return null;
        }

        int rx = resx;
        int ry = resy;
        int tot = rx * ry;
        var x = new Vector3[tot];
        var m = new btScalar[tot];
        uint seed = 243703u;

        for (int iy = 0; iy < ry; ++iy)
        {
            btScalar ty = iy / (btScalar)(ry - 1);
            Vector3 py0 = SoftBodyInternals.Lerp(in corner00, in corner01, ty);
            Vector3 py1 = SoftBodyInternals.Lerp(in corner10, in corner11, ty);
            for (int ix = 0; ix < rx; ++ix)
            {
                btScalar tx = ix / (btScalar)(rx - 1);
                btScalar pert = perturbation * NextRand01(ref seed);
                Vector3 temp1 = py1;
                temp1.Y = py1.Y + pert;
                Vector3 temp = py0;
                pert = perturbation * NextRand01(ref seed);
                temp.Y = py0.Y + pert;
                int idx = iy * rx + ix;
                x[idx] = SoftBodyInternals.Lerp(in temp, in temp1, tx);
                m[idx] = 1;
            }
        }

        var psb = new SoftBody(worldInfo, tot, x, m);
        if ((fixeds & 1) != 0) psb.SetMass(0 * rx + 0, 0);
        if ((fixeds & 2) != 0) psb.SetMass(0 * rx + (rx - 1), 0);
        if ((fixeds & 4) != 0) psb.SetMass((ry - 1) * rx + 0, 0);
        if ((fixeds & 8) != 0) psb.SetMass((ry - 1) * rx + (rx - 1), 0);

        for (int iy = 0; iy < ry; ++iy)
        {
            for (int ix = 0; ix < rx; ++ix)
            {
                int idx = iy * rx + ix;
                bool mdx = (ix + 1) < rx;
                bool mdy = (iy + 1) < ry;
                if (mdx) psb.AppendLink(idx, iy * rx + (ix + 1));
                if (mdy) psb.AppendLink(idx, (iy + 1) * rx + ix);
                if (mdx && mdy)
                {
                    if (((ix + iy) & 1) != 0)
                    {
                        psb.AppendFace(iy * rx + ix, iy * rx + (ix + 1), (iy + 1) * rx + (ix + 1));
                        psb.AppendFace(iy * rx + ix, (iy + 1) * rx + (ix + 1), (iy + 1) * rx + ix);
                        if (gendiags)
                        {
                            psb.AppendLink(iy * rx + ix, (iy + 1) * rx + (ix + 1));
                        }
                    }
                    else
                    {
                        psb.AppendFace((iy + 1) * rx + ix, iy * rx + ix, iy * rx + (ix + 1));
                        psb.AppendFace((iy + 1) * rx + ix, iy * rx + (ix + 1), (iy + 1) * rx + (ix + 1));
                        if (gendiags)
                        {
                            psb.AppendLink(iy * rx + (ix + 1), (iy + 1) * rx + ix);
                        }
                    }
                }
            }
        }

        return psb;
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::CreatePatchUV.
    /// </summary>
    public static SoftBody CreatePatchUV(
        SoftBodyWorldInfo worldInfo,
        in Vector3 corner00,
        in Vector3 corner10,
        in Vector3 corner01,
        in Vector3 corner11,
        int resx,
        int resy,
        int fixeds,
        bool gendiags,
        float[] texCoords = null)
    {
        if (resx < 2 || resy < 2)
        {
            return null;
        }

        int rx = resx;
        int ry = resy;
        int tot = rx * ry;
        var x = new Vector3[tot];
        var m = new btScalar[tot];

        for (int iy = 0; iy < ry; ++iy)
        {
            btScalar ty = iy / (btScalar)(ry - 1);
            Vector3 py0 = SoftBodyInternals.Lerp(in corner00, in corner01, ty);
            Vector3 py1 = SoftBodyInternals.Lerp(in corner10, in corner11, ty);
            for (int ix = 0; ix < rx; ++ix)
            {
                btScalar tx = ix / (btScalar)(rx - 1);
                int idx = iy * rx + ix;
                x[idx] = SoftBodyInternals.Lerp(in py0, in py1, tx);
                m[idx] = 1;
            }
        }

        var psb = new SoftBody(worldInfo, tot, x, m);
        if ((fixeds & 1) != 0) psb.SetMass(0 * rx + 0, 0);
        if ((fixeds & 2) != 0) psb.SetMass(0 * rx + (rx - 1), 0);
        if ((fixeds & 4) != 0) psb.SetMass((ry - 1) * rx + 0, 0);
        if ((fixeds & 8) != 0) psb.SetMass((ry - 1) * rx + (rx - 1), 0);
        if ((fixeds & 16) != 0) psb.SetMass(0 * rx + ((rx - 1) / 2), 0);
        if ((fixeds & 32) != 0) psb.SetMass(((ry - 1) / 2) * rx + 0, 0);
        if ((fixeds & 64) != 0) psb.SetMass(((ry - 1) / 2) * rx + (rx - 1), 0);
        if ((fixeds & 128) != 0) psb.SetMass((ry - 1) * rx + ((rx - 1) / 2), 0);
        if ((fixeds & 256) != 0) psb.SetMass(((ry - 1) / 2) * rx + ((rx - 1) / 2), 0);

        int z = 0;
        for (int iy = 0; iy < ry; ++iy)
        {
            for (int ix = 0; ix < rx; ++ix)
            {
                bool mdx = (ix + 1) < rx;
                bool mdy = (iy + 1) < ry;

                int node00 = iy * rx + ix;
                int node01 = iy * rx + (ix + 1);
                int node10 = (iy + 1) * rx + ix;
                int node11 = (iy + 1) * rx + (ix + 1);

                if (mdx) psb.AppendLink(node00, node01);
                if (mdy) psb.AppendLink(node00, node10);
                if (mdx && mdy)
                {
                    psb.AppendFace(node00, node10, node11);
                    if (texCoords != null)
                    {
                        texCoords[z + 0] = CalculateUV(resx, resy, ix, iy, 0);
                        texCoords[z + 1] = CalculateUV(resx, resy, ix, iy, 1);
                        texCoords[z + 2] = CalculateUV(resx, resy, ix, iy, 0);
                        texCoords[z + 3] = CalculateUV(resx, resy, ix, iy, 2);
                        texCoords[z + 4] = CalculateUV(resx, resy, ix, iy, 3);
                        texCoords[z + 5] = CalculateUV(resx, resy, ix, iy, 2);
                    }
                    psb.AppendFace(node11, node01, node00);
                    if (texCoords != null)
                    {
                        texCoords[z + 6] = CalculateUV(resx, resy, ix, iy, 3);
                        texCoords[z + 7] = CalculateUV(resx, resy, ix, iy, 2);
                        texCoords[z + 8] = CalculateUV(resx, resy, ix, iy, 3);
                        texCoords[z + 9] = CalculateUV(resx, resy, ix, iy, 1);
                        texCoords[z + 10] = CalculateUV(resx, resy, ix, iy, 0);
                        texCoords[z + 11] = CalculateUV(resx, resy, ix, iy, 1);
                    }
                    if (gendiags) psb.AppendLink(node00, node11);
                    z += 12;
                }
            }
        }

        return psb;
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::CalculateUV.
    /// </summary>
    public static float CalculateUV(int resx, int resy, int ix, int iy, int id)
    {
        float tc = 0.0f;
        if (id == 0)
        {
            tc = (1.0f / (resx - 1)) * ix;
        }
        else if (id == 1)
        {
            tc = (1.0f / (resy - 1)) * (resy - 1 - iy);
        }
        else if (id == 2)
        {
            tc = (1.0f / (resy - 1)) * (resy - 1 - iy - 1);
        }
        else if (id == 3)
        {
            tc = (1.0f / (resx - 1)) * (ix + 1);
        }
        return tc;
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::CreateEllipsoid.
    /// </summary>
    public static SoftBody CreateEllipsoid(
        SoftBodyWorldInfo worldInfo,
        in Vector3 center,
        in Vector3 radius,
        int res)
    {
        int n = 3 + res;
        var vtx = new Vector3[n];
        GenerateHammersley(vtx, n);
        for (int i = 0; i < n; ++i)
        {
            vtx[i] = vtx[i] * radius + center;
        }
        return CreateFromConvexHull(worldInfo, vtx, n);
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::CreateFromTriMesh.
    /// </summary>
    public static SoftBody CreateFromTriMesh(
        SoftBodyWorldInfo worldInfo,
        IReadOnlyList<btScalar> vertices,
        IReadOnlyList<int> triangles,
        bool randomizeConstraints = true) =>
        CreateFromTriMesh(worldInfo, vertices, triangles, triangles.Count / 3, randomizeConstraints);

    /// <summary>
    /// Ported from btSoftBodyHelpers::CreateFromTriMesh.
    /// </summary>
    public static SoftBody CreateFromTriMesh(
        SoftBodyWorldInfo worldInfo,
        IReadOnlyList<btScalar> vertices,
        IReadOnlyList<int> triangles,
        int ntriangles,
        bool randomizeConstraints = true)
    {
        if (ntriangles <= 0)
        {
            return new SoftBody(worldInfo, 0, null, null);
        }

        int maxidx = 0;
        int ni = ntriangles * 3;
        for (int i = 0; i < ni; ++i)
        {
            maxidx = BtMath.Max(triangles[i], maxidx);
        }
        ++maxidx;

        var vtx = new Vector3[maxidx];
        for (int i = 0, j = 0, nv = maxidx * 3; i < nv; ++j, i += 3)
        {
            vtx[j] = new Vector3(vertices[i], vertices[i + 1], vertices[i + 2]);
        }

        return CreateFromTriMeshCore(worldInfo, vtx, maxidx, triangles, ntriangles, randomizeConstraints);
    }

    /// <summary>
    /// Convenience overload of <see cref="CreateFromTriMesh(SoftBodyWorldInfo, IReadOnlyList{btScalar}, IReadOnlyList{int}, int, bool)"/>
    /// accepting <see cref="Vector3"/> vertices.
    /// </summary>
    public static SoftBody CreateFromTriMesh(
        SoftBodyWorldInfo worldInfo,
        IReadOnlyList<Vector3> vertices,
        IReadOnlyList<int> triangles,
        bool randomizeConstraints = true) =>
        CreateFromTriMesh(worldInfo, vertices, triangles, triangles.Count / 3, randomizeConstraints);

    /// <summary>
    /// Convenience overload of <see cref="CreateFromTriMesh(SoftBodyWorldInfo, IReadOnlyList{btScalar}, IReadOnlyList{int}, int, bool)"/>
    /// accepting <see cref="Vector3"/> vertices.
    /// </summary>
    public static SoftBody CreateFromTriMesh(
        SoftBodyWorldInfo worldInfo,
        IReadOnlyList<Vector3> vertices,
        IReadOnlyList<int> triangles,
        int ntriangles,
        bool randomizeConstraints = true)
    {
        if (ntriangles <= 0)
        {
            return new SoftBody(worldInfo, 0, null, null);
        }

        int maxidx = 0;
        int ni = ntriangles * 3;
        for (int i = 0; i < ni; ++i)
        {
            maxidx = BtMath.Max(triangles[i], maxidx);
        }
        ++maxidx;

        return CreateFromTriMeshCore(worldInfo, vertices, maxidx, triangles, ntriangles, randomizeConstraints);
    }

    private static SoftBody CreateFromTriMeshCore(
        SoftBodyWorldInfo worldInfo,
        IReadOnlyList<Vector3> vtx,
        int maxidx,
        IReadOnlyList<int> triangles,
        int ntriangles,
        bool randomizeConstraints)
    {
        var chks = new bool[maxidx * maxidx];
        var psb = new SoftBody(worldInfo, maxidx, vtx, null);
        int ni = ntriangles * 3;
        for (int i = 0; i < ni; i += 3)
        {
            int idx0 = triangles[i];
            int idx1 = triangles[i + 1];
            int idx2 = triangles[i + 2];
            for (int j = 2, k = 0; k < 3; j = k++)
            {
                int ij = j == 0 ? idx0 : (j == 1 ? idx1 : idx2);
                int ik = k == 0 ? idx0 : (k == 1 ? idx1 : idx2);
                int chk0 = ik * maxidx + ij;
                if (!chks[chk0])
                {
                    chks[chk0] = true;
                    chks[ij * maxidx + ik] = true;
                    psb.AppendLink(ij, ik);
                }
            }
            psb.AppendFace(idx0, idx1, idx2);
        }

        if (randomizeConstraints)
        {
            psb.RandomizeConstraints();
        }

        return psb;
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::CreateFromConvexHull.
    /// </summary>
    public static SoftBody CreateFromConvexHull(
        SoftBodyWorldInfo worldInfo,
        IReadOnlyList<Vector3> vertices,
        bool randomizeConstraints = true) =>
        CreateFromConvexHull(worldInfo, vertices, vertices.Count, randomizeConstraints);

    /// <summary>
    /// Ported from btSoftBodyHelpers::CreateFromConvexHull.
    /// </summary>
    public static SoftBody CreateFromConvexHull(
        SoftBodyWorldInfo worldInfo,
        IReadOnlyList<Vector3> vertices,
        int nvertices,
        bool randomizeConstraints = true)
    {
        var hdsc = new HullDesc(HullFlag.Triangles, nvertices, vertices)
        {
            MaxVertices = (uint)nvertices,
        };
        var hres = new HullResult();
        var hlib = new HullLibrary();
        hlib.CreateConvexHull(hdsc, hres);

        var psb = new SoftBody(worldInfo, (int)hres.NumOutputVertices, hres.OutputVertices, null);
        for (int i = 0; i < (int)hres.NumFaces; ++i)
        {
            int idx0 = (int)hres.Indices[i * 3 + 0];
            int idx1 = (int)hres.Indices[i * 3 + 1];
            int idx2 = (int)hres.Indices[i * 3 + 2];
            if (idx0 < idx1) psb.AppendLink(idx0, idx1);
            if (idx1 < idx2) psb.AppendLink(idx1, idx2);
            if (idx2 < idx0) psb.AppendLink(idx2, idx0);
            psb.AppendFace(idx0, idx1, idx2);
        }
        hlib.ReleaseResult(hres);

        if (randomizeConstraints)
        {
            psb.RandomizeConstraints();
        }
        return psb;
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::CreateFromTetGenData.
    /// Note: As in Bullet C++, the <paramref name="face"/>, <paramref name="bfacelinks"/>, and
    /// <paramref name="bfacesfromtetras"/> arguments are inactive (<c>#if 0</c> in reference).
    /// </summary>
    public static SoftBody CreateFromTetGenData(
        SoftBodyWorldInfo worldInfo,
        string ele,
        string face,
        string node,
        bool bfacelinks,
        bool btetralinks,
        bool bfacesfromtetras)
    {
        _ = face;
        _ = bfacelinks;
        _ = bfacesfromtetras;

        if (string.IsNullOrEmpty(node))
        {
            return null;
        }

        ReadOnlySpan<char> nodeSpan = node.AsSpan();
        int nodeLineStart = 0;
        int posCursor = nodeLineStart;
        int nnode = TryReadInt(nodeSpan, ref posCursor);
        nodeLineStart = NextLine(nodeSpan, nodeLineStart);

        var pos = new Vector3[nnode];
        for (int i = 0; i < nnode; ++i)
        {
            posCursor = nodeLineStart;
            int index = TryReadInt(nodeSpan, ref posCursor);
            float x = TryReadFloat(nodeSpan, ref posCursor);
            float y = TryReadFloat(nodeSpan, ref posCursor);
            float z = TryReadFloat(nodeSpan, ref posCursor);
            nodeLineStart = NextLine(nodeSpan, nodeLineStart);
            pos[index] = new Vector3((btScalar)x, (btScalar)y, (btScalar)z);
        }

        var psb = new SoftBody(worldInfo, nnode, pos, null);

        if (!string.IsNullOrEmpty(ele))
        {
            ReadOnlySpan<char> eleSpan = ele.AsSpan();
            int eleLineStart = 0;
            posCursor = eleLineStart;
            int ntetra = TryReadInt(eleSpan, ref posCursor);
            eleLineStart = NextLine(eleSpan, eleLineStart);

            for (int i = 0; i < ntetra; ++i)
            {
                posCursor = eleLineStart;
                _ = TryReadInt(eleSpan, ref posCursor); // index
                int ni0 = TryReadInt(eleSpan, ref posCursor);
                int ni1 = TryReadInt(eleSpan, ref posCursor);
                int ni2 = TryReadInt(eleSpan, ref posCursor);
                int ni3 = TryReadInt(eleSpan, ref posCursor);
                eleLineStart = NextLine(eleSpan, eleLineStart);

                psb.AppendTetra(ni0, ni1, ni2, ni3);
                if (btetralinks)
                {
                    psb.AppendLink(ni0, ni1, null, true);
                    psb.AppendLink(ni1, ni2, null, true);
                    psb.AppendLink(ni2, ni0, null, true);
                    psb.AppendLink(ni0, ni3, null, true);
                    psb.AppendLink(ni1, ni3, null, true);
                    psb.AppendLink(ni2, ni3, null, true);
                }
            }
        }

        psb.InitializeDmInverse();
        ResizeScratchList(psb.tetraScratches, psb.Tetras.Count);
        ResizeScratchList(psb.tetraScratchesTn, psb.Tetras.Count);
        return psb;
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::CreateFromVtkFile.
    /// </summary>
    public static SoftBody CreateFromVtkFile(SoftBodyWorldInfo worldInfo, string vtkFile)
    {
        using var reader = new StreamReader(vtkFile);

        Vector3[] points = Array.Empty<Vector3>();
        (int I0, int I1, int I2, int I3)[] indices = Array.Empty<(int, int, int, int)>();
        bool readingPoints = false;
        bool readingTets = false;
        int nPoints = 0;
        int nTets = 0;
        int xCount = 0;
        int indicesCount = 0;

        string line;
        while ((line = reader.ReadLine()) != null)
        {
            if (line.Length == 0 || string.IsNullOrWhiteSpace(line))
            {
                continue;
            }

            if (line.StartsWith("POINTS", StringComparison.Ordinal))
            {
                readingPoints = true;
                readingTets = false;
                ReadOnlySpan<char> span = line.AsSpan(6);
                int cursor = 0;
                nPoints = TryReadInt(span, ref cursor);
                points = new Vector3[nPoints];
            }
            else if (line.StartsWith("CELLS", StringComparison.Ordinal))
            {
                readingPoints = false;
                readingTets = true;
                ReadOnlySpan<char> span = line.AsSpan(5);
                int cursor = 0;
                nTets = TryReadInt(span, ref cursor);
                indices = new (int, int, int, int)[nTets];
            }
            else if (line.StartsWith("CELL_TYPES", StringComparison.Ordinal))
            {
                readingPoints = false;
                readingTets = false;
            }
            else if (readingPoints)
            {
                ReadOnlySpan<char> span = line.AsSpan();
                int cursor = 0;
                btScalar px = TryReadScalar(span, ref cursor);
                btScalar py = TryReadScalar(span, ref cursor);
                btScalar pz = TryReadScalar(span, ref cursor);
                points[xCount++] = new Vector3(px, py, pz);
            }
            else if (readingTets)
            {
                ReadOnlySpan<char> span = line.AsSpan();
                int cursor = 0;
                int d = TryReadInt(span, ref cursor);
                if (d != 4)
                {
                    return null;
                }
                int t0 = TryReadInt(span, ref cursor);
                int t1 = TryReadInt(span, ref cursor);
                int t2 = TryReadInt(span, ref cursor);
                int t3 = TryReadInt(span, ref cursor);
                indices[indicesCount++] = (t0, t1, t2, t3);
            }
        }

        var psb = new SoftBody(worldInfo, nPoints, points, null);
        for (int i = 0; i < nTets; ++i)
        {
            var (ni0, ni1, ni2, ni3) = indices[i];
            psb.AppendTetra(ni0, ni1, ni2, ni3);
            psb.AppendLink(ni0, ni1, null, true);
            psb.AppendLink(ni1, ni2, null, true);
            psb.AppendLink(ni2, ni0, null, true);
            psb.AppendLink(ni0, ni3, null, true);
            psb.AppendLink(ni1, ni3, null, true);
            psb.AppendLink(ni2, ni3, null, true);
        }

        GenerateBoundaryFaces(psb);
        psb.InitializeDmInverse();
        ResizeScratchList(psb.tetraScratches, psb.Tetras.Count);
        ResizeScratchList(psb.tetraScratchesTn, psb.Tetras.Count);
        return psb;
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::ReoptimizeLinkOrder.
    /// </summary>
    public static void ReoptimizeLinkOrder(SoftBody psb)
    {
        int nLinks = psb.links.Count;
        int nNodes = psb.nodes.Count;
        if (nLinks <= 1 || nNodes == 0)
        {
            return;
        }

        // Allocate temporary buffers
        var nodeWrittenAt = new int[nNodes + 1];
        var linkDepA = new int[nLinks];
        var linkDepB = new int[nLinks];
        var readyList = new int[nLinks];
        var linkDepFreeList = new LinkDep[2 * nLinks];
        var linkDepListStarts = new int[nLinks];

        // Copy the original, unsorted links to a side buffer
        var linkBuffer = new Link[nLinks];
        psb.links.CopyTo(linkBuffer);

        // Clear out the node setup and ready list
        for (int i = 0; i < nNodes; i++)
        {
            psb.nodes[i].index = i;
            nodeWrittenAt[i] = ReopNotDependent;
        }
        nodeWrittenAt[nNodes] = ReopNotDependent;
        for (int i = 0; i < nLinks; i++)
        {
            linkDepListStarts[i] = -1;
        }
        int readyListHead = 0;
        int readyListTail = 0;
        int linkDepFrees = 0;

        // Initial link analysis to set up data structures
        for (int i = 0; i < nLinks; i++)
        {
            Link lr = psb.links[i];
            int ar = lr.n[0].index;
            int br = lr.n[1].index;
            if (nodeWrittenAt[ar] > ReopNotDependent)
            {
                linkDepA[i] = nodeWrittenAt[ar];
                int depIdx = linkDepFrees++;
                linkDepFreeList[depIdx].Value = i;
                linkDepFreeList[depIdx].Next = linkDepListStarts[nodeWrittenAt[ar]];
                linkDepListStarts[nodeWrittenAt[ar]] = depIdx;
            }
            else
            {
                linkDepA[i] = ReopNotDependent;
            }

            if (nodeWrittenAt[br] > ReopNotDependent)
            {
                linkDepB[i] = nodeWrittenAt[br];
                int depIdx = linkDepFrees++;
                linkDepFreeList[depIdx].Value = -(i + 1);
                linkDepFreeList[depIdx].Next = linkDepListStarts[nodeWrittenAt[br]];
                linkDepListStarts[nodeWrittenAt[br]] = depIdx;
            }
            else
            {
                linkDepB[i] = ReopNotDependent;
            }

            // Add this link to the initial ready list, if it is not dependent on any other links
            if (linkDepA[i] == ReopNotDependent && linkDepB[i] == ReopNotDependent)
            {
                readyList[readyListTail++] = i;
                linkDepA[i] = linkDepB[i] = ReopNodeComplete;
            }

            // Update the nodes to mark which ones are calculated by this link
            nodeWrittenAt[ar] = nodeWrittenAt[br] = i;
        }

        // Process the ready list and create the sorted list of links
        int outIdx = 0;
        while (readyListHead != readyListTail)
        {
            int linkNum = readyList[readyListHead++];
            psb.links[outIdx++] = linkBuffer[linkNum];

            int linkDep = linkDepListStarts[linkNum];
            while (linkDep >= 0)
            {
                int depLink = linkDepFreeList[linkDep].Value;
                if (depLink >= 0)
                {
                    linkDepA[depLink] = ReopNotDependent;
                }
                else
                {
                    depLink = -depLink - 1;
                    linkDepB[depLink] = ReopNotDependent;
                }

                if (linkDepA[depLink] == ReopNotDependent && linkDepB[depLink] == ReopNotDependent)
                {
                    readyList[readyListTail++] = depLink;
                    linkDepA[depLink] = linkDepB[depLink] = ReopNodeComplete;
                }
                linkDep = linkDepFreeList[linkDep].Next;
            }
        }
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::getBarycentricWeights (tetrahedron overload).
    /// </summary>
    public static void GetBarycentricWeights(
        in Vector3 a,
        in Vector3 b,
        in Vector3 c,
        in Vector3 d,
        in Vector3 p,
        btScalar[] bary)
    {
        Vector3 vap = p - a;
        Vector3 vbp = p - b;

        Vector3 vab = b - a;
        Vector3 vac = c - a;
        Vector3 vad = d - a;

        Vector3 vbc = c - b;
        Vector3 vbd = d - b;
        btScalar va6 = vbp.Cross(vbd).Dot(vbc);
        btScalar vb6 = vap.Cross(vac).Dot(vad);
        btScalar vc6 = vap.Cross(vad).Dot(vab);
        btScalar vd6 = vap.Cross(vab).Dot(vac);
        btScalar v6 = (btScalar)1 / vab.Cross(vac).Dot(vad);
        bary[0] = va6 * v6;
        bary[1] = vb6 * v6;
        bary[2] = vc6 * v6;
        bary[3] = vd6 * v6;
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::getBarycentricWeights (triangle overload).
    /// </summary>
    public static void GetBarycentricWeights(
        in Vector3 a,
        in Vector3 b,
        in Vector3 c,
        in Vector3 p,
        btScalar[] bary)
    {
        Vector3 v0 = b - a;
        Vector3 v1 = c - a;
        Vector3 v2 = p - a;
        btScalar d00 = Vector3.Dot(v0, v0);
        btScalar d01 = Vector3.Dot(v0, v1);
        btScalar d11 = Vector3.Dot(v1, v1);
        btScalar d20 = Vector3.Dot(v2, v0);
        btScalar d21 = Vector3.Dot(v2, v1);
        btScalar invDenom = (btScalar)1 / (d00 * d11 - d01 * d01);
        bary[1] = (d11 * d20 - d01 * d21) * invDenom;
        bary[2] = (d00 * d21 - d01 * d20) * invDenom;
        bary[0] = (btScalar)1 - bary[1] - bary[2];
        bary[3] = 0;
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::interpolateBarycentricWeights.
    /// </summary>
    public static void InterpolateBarycentricWeights(SoftBody psb)
    {
        int renderNodeCount = psb.renderNodes.Count;
        psb.z.Clear();
        ResizeList(psb.renderNodesInterpolationWeights, renderNodeCount);
        ResizeList(psb.renderNodesParents, renderNodeCount);

        var bary = new btScalar[4];
        for (int i = 0; i < renderNodeCount; ++i)
        {
            Vector3 p = psb.renderNodes[i].x;
            var optimalBary = new btScalar[4];
            btScalar minBaryWeight = (btScalar)(-1e3);
            var optimalParents = new List<Node>(4);

            for (int j = 0; j < psb.tetras.Count; ++j)
            {
                Tetra t = psb.tetras[j];
                GetBarycentricWeights(in t.n[0].x, in t.n[1].x, in t.n[2].x, in t.n[3].x, in p, bary);
                btScalar newMinBaryWeight = bary[0];
                for (int k = 1; k < 4; ++k)
                {
                    newMinBaryWeight = BtMath.Min(newMinBaryWeight, bary[k]);
                }
                if (newMinBaryWeight > minBaryWeight)
                {
                    optimalParents.Clear();
                    optimalParents.Add(t.n[0]);
                    optimalParents.Add(t.n[1]);
                    optimalParents.Add(t.n[2]);
                    optimalParents.Add(t.n[3]);
                    optimalBary[0] = bary[0];
                    optimalBary[1] = bary[1];
                    optimalBary[2] = bary[2];
                    optimalBary[3] = bary[3];
                    minBaryWeight = newMinBaryWeight;

                    // Stop searching if p is inside the tetrahedron at hand
                    if (bary[0] >= 0 && bary[1] >= 0 && bary[2] >= 0 && bary[3] >= 0)
                    {
                        break;
                    }
                }
            }

            psb.renderNodesInterpolationWeights[i] = optimalBary;
            psb.renderNodesParents[i] = optimalParents;
        }
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::extrapolateBarycentricWeights.
    /// </summary>
    public static void ExtrapolateBarycentricWeights(SoftBody psb)
    {
        int renderNodeCount = psb.renderNodes.Count;
        ResizeList(psb.renderNodesInterpolationWeights, renderNodeCount);
        ResizeList(psb.renderNodesParents, renderNodeCount);
        ResizeList(psb.z, renderNodeCount);

        var bary = new btScalar[4];
        for (int i = 0; i < renderNodeCount; ++i)
        {
            Vector3 p = psb.renderNodes[i].x;
            var optimalBary = new btScalar[4];
            btScalar minBaryWeight = -BtMath.SimdInfinity;
            var optimalParents = new List<Node>(3);
            btScalar dist = 0;
            btScalar optimalDist = 0;

            for (int j = 0; j < psb.faces.Count; ++j)
            {
                Face f = psb.faces[j];
                Vector3 n = Vector3.Cross(f.n[1].x - f.n[0].x, f.n[2].x - f.n[0].x);
                Vector3 unitN = n.Normalized();
                dist = (p - f.n[0].x).Dot(unitN);
                Vector3 projP = p - dist * unitN;
                GetBarycentricWeights(in f.n[0].x, in f.n[1].x, in f.n[2].x, in projP, bary);
                btScalar newMinBaryWeight = bary[0];
                for (int k = 1; k < 3; ++k)
                {
                    newMinBaryWeight = BtMath.Min(newMinBaryWeight, bary[k]);
                }

                // p is out of the current best triangle, we found a triangle that's better
                bool betterThanClosestOutside = newMinBaryWeight > minBaryWeight && minBaryWeight < 0;
                // p is inside of the current best triangle, we found a triangle that's better
                bool betterThanBestInside = newMinBaryWeight >= 0 && minBaryWeight >= 0 && BtMath.Abs(dist) < BtMath.Abs(optimalDist);

                if (betterThanClosestOutside || betterThanBestInside)
                {
                    optimalParents.Clear();
                    optimalParents.Add(f.n[0]);
                    optimalParents.Add(f.n[1]);
                    optimalParents.Add(f.n[2]);
                    optimalBary[0] = bary[0];
                    optimalBary[1] = bary[1];
                    optimalBary[2] = bary[2];
                    optimalBary[3] = bary[3];
                    optimalDist = dist;
                    minBaryWeight = newMinBaryWeight;
                }
            }

            psb.renderNodesInterpolationWeights[i] = optimalBary;
            psb.renderNodesParents[i] = optimalParents;
            psb.z[i] = optimalDist;
        }
    }

    /// <summary>
    /// Ported from btSoftBodyHelpers::generateBoundaryFaces.
    /// </summary>
    public static void GenerateBoundaryFaces(SoftBody psb)
    {
        for (int i = 0; i < psb.Nodes.Count; ++i)
        {
            psb.Nodes[i].index = i;
        }

        var dict = new Dictionary<SortedTriKey, (int F0, int F1, int F2)>();
        for (int i = 0; i < psb.Tetras.Count; ++i)
        {
            Tetra t = psb.Tetras[i];
            int idx0 = t.n[0].index;
            int idx1 = t.n[1].index;
            int idx2 = t.n[2].index;
            int idx3 = t.n[3].index;

            ToggleBoundaryFace(dict, idx1, idx0, idx2);
            ToggleBoundaryFace(dict, idx3, idx0, idx1);
            ToggleBoundaryFace(dict, idx3, idx1, idx2);
            ToggleBoundaryFace(dict, idx2, idx0, idx3);
        }

        if (dict.Count == 0)
        {
            return;
        }

        var remaining = new List<KeyValuePair<SortedTriKey, (int F0, int F1, int F2)>>(dict.Count);
        foreach (KeyValuePair<SortedTriKey, (int F0, int F1, int F2)> kvp in dict)
        {
            remaining.Add(kvp);
        }

        // Preserve C++ std::map<std::vector<int>, std::vector<int>> lexicographical order.
        BtQuickSort.QuickSort(remaining, static (a, b) => a.Key.CompareTo(b.Key));

        for (int i = 0; i < remaining.Count; ++i)
        {
            var (f0, f1, f2) = remaining[i].Value;
            psb.AppendFace(f0, f1, f2);
        }
    }

    private readonly struct SortedTriKey : IEquatable<SortedTriKey>, IComparable<SortedTriKey>
    {
        public readonly int A;
        public readonly int B;
        public readonly int C;

        public SortedTriKey(int f0, int f1, int f2)
        {
            int a = f0;
            int b = f1;
            int c = f2;
            if (a > b) (a, b) = (b, a);
            if (b > c) (b, c) = (c, b);
            if (a > b) (a, b) = (b, a);
            A = a;
            B = b;
            C = c;
        }

        public bool Equals(SortedTriKey other) => A == other.A && B == other.B && C == other.C;

        public override bool Equals(object obj) => obj is SortedTriKey other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(A, B, C);

        public int CompareTo(SortedTriKey other)
        {
            int cmp = A.CompareTo(other.A);
            if (cmp != 0) return cmp;
            cmp = B.CompareTo(other.B);
            return cmp != 0 ? cmp : C.CompareTo(other.C);
        }
    }

    private static void ToggleBoundaryFace(
        Dictionary<SortedTriKey, (int F0, int F1, int F2)> dict,
        int f0,
        int f1,
        int f2)
    {
        var key = new SortedTriKey(f0, f1, f2);
        if (!dict.Remove(key))
        {
            dict.Add(key, (f0, f1, f2));
        }
    }

    private static void GenerateHammersley(Vector3[] x, int n)
    {
        for (int i = 0; i < n; i++)
        {
            btScalar p = (btScalar)0.5;
            btScalar t = 0;
            for (int j = i; j != 0; p *= (btScalar)0.5, j >>= 1)
            {
                if ((j & 1) != 0)
                {
                    t += p;
                }
            }
            btScalar w = 2 * t - 1;
            btScalar a = (BtMath.Pi + 2 * i * BtMath.Pi) / n;
            btScalar s = BtMath.Sqrt(1 - w * w);
            x[i] = new Vector3(s * BtMath.Cos(a), s * BtMath.Sin(a), w);
        }
    }

    private static btScalar NextRand01(ref uint seed)
    {
        unchecked
        {
            seed = 1664525u * seed + 1013904223u;
        }
        return (btScalar)((seed >> 16) & 0x7fffu) / (btScalar)0x7fff;
    }

    private static int NextLine(ReadOnlySpan<char> buffer, int offset)
    {
        while (offset < buffer.Length && buffer[offset] != '\n')
        {
            offset++;
        }
        if (offset < buffer.Length && buffer[offset] == '\n')
        {
            offset++;
        }
        return offset;
    }

    private static ReadOnlySpan<char> NextToken(ReadOnlySpan<char> span, ref int cursor)
    {
        while (cursor < span.Length && char.IsWhiteSpace(span[cursor]))
        {
            cursor++;
        }
        int start = cursor;
        while (cursor < span.Length && !char.IsWhiteSpace(span[cursor]))
        {
            cursor++;
        }
        return span.Slice(start, cursor - start);
    }

    private static int TryReadInt(ReadOnlySpan<char> span, ref int cursor)
    {
        ReadOnlySpan<char> token = NextToken(span, ref cursor);
        return token.Length > 0 && int.TryParse(token, NumberStyles.Integer, CultureInfo.InvariantCulture, out int v)
            ? v
            : 0;
    }

    private static float TryReadFloat(ReadOnlySpan<char> span, ref int cursor)
    {
        ReadOnlySpan<char> token = NextToken(span, ref cursor);
        return token.Length > 0 && float.TryParse(token, NumberStyles.Float | NumberStyles.AllowThousands, CultureInfo.InvariantCulture, out float v)
            ? v
            : 0f;
    }

    private static btScalar TryReadScalar(ReadOnlySpan<char> span, ref int cursor)
    {
        ReadOnlySpan<char> token = NextToken(span, ref cursor);
        if (token.Length == 0)
        {
            return 0;
        }
#if BT_USE_DOUBLE_PRECISION
        return double.TryParse(token, NumberStyles.Float | NumberStyles.AllowThousands, CultureInfo.InvariantCulture, out double v)
            ? v
            : 0.0;
#else
        return float.TryParse(token, NumberStyles.Float | NumberStyles.AllowThousands, CultureInfo.InvariantCulture, out float v)
            ? v
            : 0f;
#endif
    }

    private const int ReopNotDependent = -1;
    private const int ReopNodeComplete = -2;

    private struct LinkDep
    {
        public int Value;
        public int Next;
    }

    private static readonly Vector3[] s_contactAxes =
    {
        new(1, 0, 0),
        new(0, 1, 0),
        new(0, 0, 1),
    };

    private static void DrawVertex(IDebugDraw idraw, in Vector3 x, btScalar s, in Vector3 c)
    {
        Vector3 color = c;
        Vector3 dx = new(s, 0, 0);
        Vector3 dy = new(0, s, 0);
        Vector3 dz = new(0, 0, s);
        Vector3 a = x - dx;
        Vector3 b = x + dx;
        idraw.DrawLine(ref a, ref b, ref color);
        a = x - dy;
        b = x + dy;
        idraw.DrawLine(ref a, ref b, ref color);
        a = x - dz;
        b = x + dz;
        idraw.DrawLine(ref a, ref b, ref color);
    }

    private static void DrawBox(IDebugDraw idraw, in Vector3 mins, in Vector3 maxs, in Vector3 color)
    {
        Vector3 col = color;
        Vector3 c0 = new(mins.X, mins.Y, mins.Z);
        Vector3 c1 = new(maxs.X, mins.Y, mins.Z);
        Vector3 c2 = new(maxs.X, maxs.Y, mins.Z);
        Vector3 c3 = new(mins.X, maxs.Y, mins.Z);
        Vector3 c4 = new(mins.X, mins.Y, maxs.Z);
        Vector3 c5 = new(maxs.X, mins.Y, maxs.Z);
        Vector3 c6 = new(maxs.X, maxs.Y, maxs.Z);
        Vector3 c7 = new(mins.X, maxs.Y, maxs.Z);
        idraw.DrawLine(ref c0, ref c1, ref col);
        idraw.DrawLine(ref c1, ref c2, ref col);
        idraw.DrawLine(ref c2, ref c3, ref col);
        idraw.DrawLine(ref c3, ref c0, ref col);
        idraw.DrawLine(ref c4, ref c5, ref col);
        idraw.DrawLine(ref c5, ref c6, ref col);
        idraw.DrawLine(ref c6, ref c7, ref col);
        idraw.DrawLine(ref c7, ref c4, ref col);
        idraw.DrawLine(ref c0, ref c4, ref col);
        idraw.DrawLine(ref c1, ref c5, ref col);
        idraw.DrawLine(ref c2, ref c6, ref col);
        idraw.DrawLine(ref c3, ref c7, ref col);
    }

    private static void DrawTree(
        IDebugDraw idraw,
        DbvtNode node,
        int depth,
        in Vector3 ncolor,
        in Vector3 lcolor,
        int mindepth,
        int maxdepth)
    {
        if (node != null)
        {
            if (node.IsInternal && ((depth < maxdepth) || (maxdepth < 0)))
            {
                DrawTree(idraw, node.Child0, depth + 1, in ncolor, in lcolor, mindepth, maxdepth);
                DrawTree(idraw, node.Child1, depth + 1, in ncolor, in lcolor, mindepth, maxdepth);
            }
            if (depth >= mindepth)
            {
                btScalar scl = (btScalar)(node.IsInternal ? 1 : 1);
                Vector3 center = node.Volume.Center();
                Vector3 extents = node.Volume.Extents() * scl;
                Vector3 mi = center - extents;
                Vector3 mx = center + extents;
                Vector3 col = node.IsLeaf ? lcolor : ncolor;
                DrawBox(idraw, in mi, in mx, in col);
            }
        }
    }

    private static Vector3 Sum(List<Vector3> items)
    {
        Vector3 v = Vector3.Zero;
        if (items.Count > 0)
        {
            v = items[0];
            for (int i = 1, ni = items.Count; i < ni; ++i)
            {
                v += items[i];
            }
        }
        return v;
    }

    private static void Add(List<Vector3> items, in Vector3 value)
    {
        for (int i = 0, ni = items.Count; i < ni; ++i)
        {
            items[i] += value;
        }
    }

    private static void Mul(List<Vector3> items, btScalar value)
    {
        for (int i = 0, ni = items.Count; i < ni; ++i)
        {
            items[i] *= value;
        }
    }

    private static Vector3 Average(List<Vector3> items)
    {
        btScalar n = (btScalar)(items.Count > 0 ? items.Count : 1);
        return Sum(items) / n;
    }

    private static void ResizeScratchList(List<TetraScratch> list, int count) =>
        ResizeList(list, count);

    private static void ResizeList<T>(List<T> list, int count)
    {
        if (list.Count > count)
        {
            list.RemoveRange(count, list.Count - count);
        }
        else if (list.Count < count)
        {
            if (list.Capacity < count)
            {
                list.Capacity = count;
            }
            while (list.Count < count)
            {
                list.Add(default);
            }
        }
    }
}
