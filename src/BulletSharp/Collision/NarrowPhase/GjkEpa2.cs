using BulletSharp.Math;
using GJK = BulletSharp.GjkEpa2Impl.GJK;
using EPA = BulletSharp.GjkEpa2Impl.EPA;

namespace BulletSharp;

/// <summary>
/// Ported from btGjkEpa2.cpp (gjkepa2_impl namespace): GJK/EPA solver by Nathanael Presson.
/// </summary>
internal static class GjkEpa2Impl
{
    // ---------------- Config ----------------
    public const int GjkMaxIterations = 128;

#if BT_USE_DOUBLE_PRECISION
    public const btScalar GjkAccuracy = (btScalar)1e-12;
    public const btScalar GjkMinDistance = (btScalar)1e-12;
    public const btScalar GjkDuplicatedEps = (btScalar)1e-12;
#else
    public const btScalar GjkAccuracy = (btScalar)0.0001;
    public const btScalar GjkMinDistance = (btScalar)0.0001;
    public const btScalar GjkDuplicatedEps = (btScalar)0.0001;
#endif

    public const btScalar GjkSimplex2Eps = 0;
    public const btScalar GjkSimplex3Eps = 0;
    public const btScalar GjkSimplex4Eps = 0;

    public const int EpaMaxVertices = 128;
    public const int EpaMaxIterations = 255;

#if BT_USE_DOUBLE_PRECISION
    public const btScalar EpaAccuracy = (btScalar)1e-12;
    public const btScalar EpaPlaneEps = (btScalar)1e-14;
    public const btScalar EpaInsideEps = (btScalar)1e-9;
#else
    public const btScalar EpaAccuracy = (btScalar)0.0001;
    public const btScalar EpaPlaneEps = (btScalar)0.00001;
    public const btScalar EpaInsideEps = (btScalar)0.01;
#endif

    public const int EpaMaxFaces = EpaMaxVertices * 2;

    // ---------------- MinkowskiDiff ----------------
    public struct MinkowskiDiff
    {
        public ConvexShape shape0;
        public ConvexShape shape1;
        public Matrix3x3 toshape1;
        public Transform toshape0;
        public bool enableMargin;

        public MinkowskiDiff()
        {
            enableMargin = true;
        }

        public void EnableMargin(bool enable) => enableMargin = enable;

        public readonly Vector3 Support0(Vector3 d) =>
            enableMargin ? shape0.LocalGetSupportingVertex(d) : shape0.LocalGetSupportingVertexWithoutMargin(d);

        public readonly Vector3 Support1(Vector3 d) =>
            toshape0 * (enableMargin
                ? shape1.LocalGetSupportingVertex(toshape1 * d)
                : shape1.LocalGetSupportingVertexWithoutMargin(toshape1 * d));

        public readonly Vector3 Support(Vector3 d) => Support0(d) - Support1(-d);

        public readonly Vector3 Support(Vector3 d, int index) => index != 0 ? Support1(d) : Support0(d);
    }

    // ---------------- GJK ----------------
    public sealed class GJK
    {
        public enum Status
        {
            Valid,
            Inside,
            Failed,
        }

        public sealed class SV
        {
            public Vector3 d, w;
        }

        public sealed class Simplex
        {
            public readonly SV[] c = new SV[4];
            public readonly btScalar[] p = new btScalar[4];
            public int rank;
        }

        /* Fields */
        public MinkowskiDiff shape;
        public Vector3 ray;
        public btScalar distance;
        public readonly Simplex[] simplices = { new Simplex(), new Simplex() };
        public readonly SV[] store = { new SV(), new SV(), new SV(), new SV() };
        public readonly SV[] free = new SV[4];
        public int nfree;
        public int current;
        public Simplex simplex;
        public Status status;

        public GJK()
        {
            Initialize();
        }

        public void Initialize()
        {
            ray = Vector3.Zero;
            nfree = 0;
            status = Status.Failed;
            current = 0;
            distance = 0;
        }

        public Status Evaluate(MinkowskiDiff shapearg, Vector3 guess)
        {
            int iterations = 0;
            btScalar sqdist;
            btScalar alpha = 0;
            Span<Vector3> lastw = stackalloc Vector3[4];
            Span<btScalar> weights = stackalloc btScalar[4];
            int clastw = 0;

            // Initialize the solver
            free[0] = store[0];
            free[1] = store[1];
            free[2] = store[2];
            free[3] = store[3];
            nfree = 4;
            current = 0;
            status = Status.Valid;
            shape = shapearg;
            distance = 0;

            // Initialize the simplex
            simplices[0].rank = 0;
            ray = guess;
            btScalar sqrl = ray.LengthSquared();
            AppendVertice(simplices[0], sqrl > 0 ? -ray : new Vector3(1, 0, 0));
            simplices[0].p[0] = 1;
            ray = simplices[0].c[0].w;
            sqdist = sqrl;
            lastw[0] = lastw[1] = lastw[2] = lastw[3] = ray;

            // Main loop
            do
            {
                int next = 1 - current;
                Simplex cs = simplices[current];
                Simplex ns = simplices[next];

                // Check for a zero vector
                btScalar rl = ray.Length();
                if (rl < GjkMinDistance)
                {
                    // Touching or contained
                    status = Status.Inside;
                    break;
                }

                // Append a new vertex along -v
                AppendVertice(cs, -ray);
                Vector3 w = cs.c[cs.rank - 1].w;
                bool found = false;
                for (int i = 0; i < 4; ++i)
                {
                    if ((w - lastw[i]).LengthSquared() < GjkDuplicatedEps)
                    {
                        found = true;
                        break;
                    }
                }
                if (found)
                {
                    // Return the old simplex
                    RemoveVertice(simplices[current]);
                    break;
                }
                else
                {
                    // Update lastw
                    lastw[clastw = (clastw + 1) & 3] = w;
                }

                // Check the termination condition
                btScalar omega = ray.Dot(w) / rl;
                alpha = BtMath.Max(omega, alpha);
                if (rl - alpha - GjkAccuracy * rl <= 0)
                {
                    // Return the old simplex
                    RemoveVertice(simplices[current]);
                    break;
                }

                // Reduce the simplex
                int mask;
                switch (cs.rank)
                {
                    case 2:
                        sqdist = ProjectOrigin(cs.c[0].w, cs.c[1].w, weights, out mask);
                        break;
                    case 3:
                        sqdist = ProjectOrigin(cs.c[0].w, cs.c[1].w, cs.c[2].w, weights, out mask);
                        break;
                    case 4:
                        sqdist = ProjectOrigin(cs.c[0].w, cs.c[1].w, cs.c[2].w, cs.c[3].w, weights, out mask);
                        break;
                    default:
                        sqdist = -1;
                        mask = 0;
                        break;
                }
                if (sqdist >= 0)
                {
                    // Valid
                    ns.rank = 0;
                    ray = Vector3.Zero;
                    current = next;
                    for (int i = 0, ni = cs.rank; i < ni; ++i)
                    {
                        if ((mask & (1 << i)) != 0)
                        {
                            ns.c[ns.rank] = cs.c[i];
                            ns.p[ns.rank++] = weights[i];
                            ray += cs.c[i].w * weights[i];
                        }
                        else
                        {
                            free[nfree++] = cs.c[i];
                        }
                    }
                    if (mask == 15) status = Status.Inside;
                }
                else
                {
                    // Return the old simplex
                    RemoveVertice(simplices[current]);
                    break;
                }
                status = ++iterations < GjkMaxIterations ? status : Status.Failed;
            } while (status == Status.Valid);

            simplex = simplices[current];
            switch (status)
            {
                case Status.Valid:
                    distance = ray.Length();
                    break;
                case Status.Inside:
                    distance = 0;
                    break;
            }
            return status;
        }

        public bool EncloseOrigin()
        {
            switch (simplex.rank)
            {
                case 1:
                {
                    for (int i = 0; i < 3; ++i)
                    {
                        Vector3 axis = Vector3.Zero;
                        axis[i] = 1;
                        AppendVertice(simplex, axis);
                        if (EncloseOrigin()) return true;
                        RemoveVertice(simplex);
                        AppendVertice(simplex, -axis);
                        if (EncloseOrigin()) return true;
                        RemoveVertice(simplex);
                    }
                    break;
                }
                case 2:
                {
                    Vector3 d = simplex.c[1].w - simplex.c[0].w;
                    for (int i = 0; i < 3; ++i)
                    {
                        Vector3 axis = Vector3.Zero;
                        axis[i] = 1;
                        Vector3 p = d.Cross(axis);
                        if (p.LengthSquared() > 0)
                        {
                            AppendVertice(simplex, p);
                            if (EncloseOrigin()) return true;
                            RemoveVertice(simplex);
                            AppendVertice(simplex, -p);
                            if (EncloseOrigin()) return true;
                            RemoveVertice(simplex);
                        }
                    }
                    break;
                }
                case 3:
                {
                    Vector3 n = (simplex.c[1].w - simplex.c[0].w).Cross(simplex.c[2].w - simplex.c[0].w);
                    if (n.LengthSquared() > 0)
                    {
                        AppendVertice(simplex, n);
                        if (EncloseOrigin()) return true;
                        RemoveVertice(simplex);
                        AppendVertice(simplex, -n);
                        if (EncloseOrigin()) return true;
                        RemoveVertice(simplex);
                    }
                    break;
                }
                case 4:
                {
                    if (BtMath.Abs(Det(simplex.c[0].w - simplex.c[3].w,
                            simplex.c[1].w - simplex.c[3].w,
                            simplex.c[2].w - simplex.c[3].w)) > 0)
                        return true;
                    break;
                }
            }
            return false;
        }

        /* Internals */
        public void GetSupport(Vector3 d, SV sv)
        {
            sv.d = d / d.Length();
            sv.w = shape.Support(sv.d);
        }

        public void RemoveVertice(Simplex simplex)
        {
            free[nfree++] = simplex.c[--simplex.rank];
        }

        public void AppendVertice(Simplex simplex, Vector3 v)
        {
            simplex.p[simplex.rank] = 0;
            simplex.c[simplex.rank] = free[--nfree];
            GetSupport(v, simplex.c[simplex.rank++]);
        }

        public static btScalar Det(in Vector3 a, in Vector3 b, in Vector3 c) =>
            a.Y * b.Z * c.X + a.Z * b.X * c.Y -
            a.X * b.Z * c.Y - a.Y * b.X * c.Z +
            a.X * b.Y * c.Z - a.Z * b.Y * c.X;

        public static btScalar ProjectOrigin(in Vector3 a, in Vector3 b, Span<btScalar> w, out int m)
        {
            Vector3 d = b - a;
            btScalar l = d.LengthSquared();
            if (l > GjkSimplex2Eps)
            {
                btScalar t = l > 0 ? -a.Dot(d) / l : 0;
                if (t >= 1)
                {
                    w[0] = 0;
                    w[1] = 1;
                    m = 2;
                    return b.LengthSquared();
                }
                else if (t <= 0)
                {
                    w[0] = 1;
                    w[1] = 0;
                    m = 1;
                    return a.LengthSquared();
                }
                else
                {
                    w[0] = 1 - (w[1] = t);
                    m = 3;
                    return (a + d * t).LengthSquared();
                }
            }
            m = 0;
            return -1;
        }

        private static readonly int[] imd3 = { 1, 2, 0 };

        public static btScalar ProjectOrigin(in Vector3 a, in Vector3 b, in Vector3 c, Span<btScalar> w, out int m)
        {
            m = 0;
            Span<Vector3> vt = stackalloc Vector3[3] { a, b, c };
            Span<Vector3> dl = stackalloc Vector3[3] { a - b, b - c, c - a };
            Vector3 n = dl[0].Cross(dl[1]);
            btScalar l = n.LengthSquared();
            if (l > GjkSimplex3Eps)
            {
                btScalar mindist = -1;
                Span<btScalar> subw = stackalloc btScalar[2];
                for (int i = 0; i < 3; ++i)
                {
                    if (vt[i].Dot(dl[i].Cross(n)) > 0)
                    {
                        int j = imd3[i];
                        btScalar subd = ProjectOrigin(vt[i], vt[j], subw, out int subm);
                        if (mindist < 0 || subd < mindist)
                        {
                            mindist = subd;
                            m = ((subm & 1) != 0 ? 1 << i : 0) + ((subm & 2) != 0 ? 1 << j : 0);
                            w[i] = subw[0];
                            w[j] = subw[1];
                            w[imd3[j]] = 0;
                        }
                    }
                }
                if (mindist < 0)
                {
                    btScalar d = a.Dot(n);
                    btScalar s = BtMath.Sqrt(l);
                    Vector3 p = n * (d / l);
                    mindist = p.LengthSquared();
                    m = 7;
                    w[0] = dl[1].Cross(b - p).Length() / s;
                    w[1] = dl[2].Cross(c - p).Length() / s;
                    w[2] = 1 - (w[0] + w[1]);
                }
                return mindist;
            }
            return -1;
        }

        public static btScalar ProjectOrigin(in Vector3 a, in Vector3 b, in Vector3 c, in Vector3 d,
            Span<btScalar> w, out int m)
        {
            m = 0;
            Span<Vector3> vt = stackalloc Vector3[4] { a, b, c, d };
            Span<Vector3> dl = stackalloc Vector3[3] { a - d, b - d, c - d };
            btScalar vl = Det(dl[0], dl[1], dl[2]);
            bool ng = vl * a.Dot((b - c).Cross(a - b)) <= 0;
            if (ng && BtMath.Abs(vl) > GjkSimplex4Eps)
            {
                btScalar mindist = -1;
                Span<btScalar> subw = stackalloc btScalar[3];
                for (int i = 0; i < 3; ++i)
                {
                    int j = imd3[i];
                    btScalar s = vl * d.Dot(dl[i].Cross(dl[j]));
                    if (s > 0)
                    {
                        btScalar subd = ProjectOrigin(vt[i], vt[j], d, subw, out int subm);
                        if (mindist < 0 || subd < mindist)
                        {
                            mindist = subd;
                            m = ((subm & 1) != 0 ? 1 << i : 0) +
                                ((subm & 2) != 0 ? 1 << j : 0) +
                                ((subm & 4) != 0 ? 8 : 0);
                            w[i] = subw[0];
                            w[j] = subw[1];
                            w[imd3[j]] = 0;
                            w[3] = subw[2];
                        }
                    }
                }
                if (mindist < 0)
                {
                    mindist = 0;
                    m = 15;
                    w[0] = Det(c, b, d) / vl;
                    w[1] = Det(a, c, d) / vl;
                    w[2] = Det(b, a, d) / vl;
                    w[3] = 1 - (w[0] + w[1] + w[2]);
                }
                return mindist;
            }
            return -1;
        }
    }

    // ---------------- EPA ----------------
    public sealed class EPA
    {
        public enum Status
        {
            Valid,
            Touching,
            Degenerated,
            NonConvex,
            InvalidHull,
            OutOfFaces,
            OutOfVertices,
            AccuraryReached,
            FallBack,
            Failed,
        }

        public sealed class Face
        {
            public Vector3 n;
            public btScalar d;
            public readonly GJK.SV[] c = new GJK.SV[3];
            public readonly Face[] f = new Face[3];
            public readonly Face[] l = new Face[2];
            public readonly int[] e = new int[3];
            public int pass;

            /// <summary>Copy (corresponds to the C++ sFace value copy outer = *best)</summary>
            public void CopyFrom(Face other)
            {
                n = other.n;
                d = other.d;
                c[0] = other.c[0];
                c[1] = other.c[1];
                c[2] = other.c[2];
                f[0] = other.f[0];
                f[1] = other.f[1];
                f[2] = other.f[2];
                e[0] = other.e[0];
                e[1] = other.e[1];
                e[2] = other.e[2];
                pass = other.pass;
            }
        }

        public sealed class FaceList
        {
            public Face root;
            public int count;
        }

        public struct Horizon
        {
            public Face cf;
            public Face ff;
            public int nf;
        }

        /* Fields */
        public Status status;
        public readonly GJK.Simplex result = new GJK.Simplex();
        public Vector3 normal;
        public btScalar depth;
        private readonly GJK.SV[] svStore = new GJK.SV[EpaMaxVertices];
        private readonly Face[] fcStore = new Face[EpaMaxFaces];
        private int nextsv;
        private readonly FaceList hull = new FaceList();
        private readonly FaceList stock = new FaceList();
        // Reused copy of best (stack object outer in C++)
        private readonly Face outer = new Face();

        public EPA()
        {
            for (int i = 0; i < EpaMaxVertices; i++) svStore[i] = new GJK.SV();
            for (int i = 0; i < EpaMaxFaces; i++) fcStore[i] = new Face();
            Initialize();
        }

        private static void Bind(Face fa, int ea, Face fb, int eb)
        {
            fa.e[ea] = eb;
            fa.f[ea] = fb;
            fb.e[eb] = ea;
            fb.f[eb] = fa;
        }

        private static void Append(FaceList list, Face face)
        {
            face.l[0] = null;
            face.l[1] = list.root;
            if (list.root != null) list.root.l[0] = face;
            list.root = face;
            ++list.count;
        }

        private static void Remove(FaceList list, Face face)
        {
            if (face.l[1] != null) face.l[1].l[0] = face.l[0];
            if (face.l[0] != null) face.l[0].l[1] = face.l[1];
            if (face == list.root) list.root = face.l[1];
            --list.count;
        }

        public void Initialize()
        {
            status = Status.Failed;
            normal = Vector3.Zero;
            depth = 0;
            nextsv = 0;
            for (int i = 0; i < EpaMaxFaces; ++i)
            {
                Append(stock, fcStore[EpaMaxFaces - i - 1]);
            }
        }

        public Status Evaluate(GJK gjk, Vector3 guess)
        {
            GJK.Simplex simplex = gjk.simplex;
            if (simplex.rank > 1 && gjk.EncloseOrigin())
            {
                // Cleanup: return all hull faces, then rebuild stock in the canonical construction order.
                // (Keeps reused instances bit-identical to constructing a fresh EPA each time, avoiding tie-breaking differences.)
                while (hull.root != null)
                {
                    Remove(hull, hull.root);
                }
                stock.root = null;
                stock.count = 0;
                for (int i = 0; i < EpaMaxFaces; ++i)
                {
                    Append(stock, fcStore[EpaMaxFaces - i - 1]);
                }
                status = Status.Valid;
                nextsv = 0;

                // Fix simplex orientation
                if (GJK.Det(simplex.c[0].w - simplex.c[3].w,
                        simplex.c[1].w - simplex.c[3].w,
                        simplex.c[2].w - simplex.c[3].w) < 0)
                {
                    (simplex.c[0], simplex.c[1]) = (simplex.c[1], simplex.c[0]);
                    (simplex.p[0], simplex.p[1]) = (simplex.p[1], simplex.p[0]);
                }

                // Build the initial hull
                Face[] tetra =
                {
                    NewFace(simplex.c[0], simplex.c[1], simplex.c[2], true),
                    NewFace(simplex.c[1], simplex.c[0], simplex.c[3], true),
                    NewFace(simplex.c[2], simplex.c[1], simplex.c[3], true),
                    NewFace(simplex.c[0], simplex.c[2], simplex.c[3], true),
                };
                if (hull.count == 4)
                {
                    Face best = FindBest();
                    outer.CopyFrom(best);
                    int pass = 0;
                    int iterations = 0;
                    Bind(tetra[0], 0, tetra[1], 0);
                    Bind(tetra[0], 1, tetra[2], 0);
                    Bind(tetra[0], 2, tetra[3], 0);
                    Bind(tetra[1], 1, tetra[3], 2);
                    Bind(tetra[1], 2, tetra[2], 1);
                    Bind(tetra[2], 2, tetra[3], 1);
                    status = Status.Valid;
                    for (; iterations < EpaMaxIterations; ++iterations)
                    {
                        if (nextsv < EpaMaxVertices)
                        {
                            Horizon horizon = default;
                            GJK.SV w = svStore[nextsv++];
                            bool valid = true;
                            best.pass = ++pass;
                            gjk.GetSupport(best.n, w);
                            btScalar wdist = best.n.Dot(w.w) - best.d;
                            if (wdist > EpaAccuracy)
                            {
                                for (int j = 0; j < 3 && valid; ++j)
                                {
                                    valid &= Expand(pass, w, best.f[j], best.e[j], ref horizon);
                                }
                                if (valid && horizon.nf >= 3)
                                {
                                    Bind(horizon.cf, 1, horizon.ff, 2);
                                    Remove(hull, best);
                                    Append(stock, best);
                                    best = FindBest();
                                    outer.CopyFrom(best);
                                }
                                else
                                {
                                    status = Status.InvalidHull;
                                    break;
                                }
                            }
                            else
                            {
                                status = Status.AccuraryReached;
                                break;
                            }
                        }
                        else
                        {
                            status = Status.OutOfVertices;
                            break;
                        }
                    }
                    Vector3 projection = outer.n * outer.d;
                    normal = outer.n;
                    depth = outer.d;
                    result.rank = 3;
                    result.c[0] = outer.c[0];
                    result.c[1] = outer.c[1];
                    result.c[2] = outer.c[2];
                    result.p[0] = (outer.c[1].w - projection).Cross(outer.c[2].w - projection).Length();
                    result.p[1] = (outer.c[2].w - projection).Cross(outer.c[0].w - projection).Length();
                    result.p[2] = (outer.c[0].w - projection).Cross(outer.c[1].w - projection).Length();
                    btScalar sum = result.p[0] + result.p[1] + result.p[2];
                    result.p[0] /= sum;
                    result.p[1] /= sum;
                    result.p[2] /= sum;
                    return status;
                }
            }

            // Fallback
            status = Status.FallBack;
            normal = -guess;
            btScalar nl = normal.Length();
            if (nl > 0)
                normal /= nl;
            else
                normal = new Vector3(1, 0, 0);
            depth = 0;
            result.rank = 1;
            result.c[0] = simplex.c[0];
            result.p[0] = 1;
            return status;
        }

        private bool GetEdgeDist(Face face, GJK.SV a, GJK.SV b, out btScalar dist)
        {
            Vector3 ba = b.w - a.w;
            Vector3 nAb = ba.Cross(face.n);  // outward edge normal direction in the triangle's plane
            btScalar aDotNab = a.w.Dot(nAb); // only the sign matters; no need to normalize

            if (aDotNab < 0)
            {
                // Outside edge a->b
                btScalar baL2 = ba.LengthSquared();
                btScalar aDotBa = a.w.Dot(ba);
                btScalar bDotBa = b.w.Dot(ba);

                if (aDotBa > 0)
                {
                    // Distance to vertex a
                    dist = a.w.Length();
                }
                else if (bDotBa < 0)
                {
                    // Distance to vertex b
                    dist = b.w.Length();
                }
                else
                {
                    // Distance to edge a->b
                    btScalar aDotB = a.w.Dot(b.w);
                    dist = BtMath.Sqrt(BtMath.Max((a.w.LengthSquared() * b.w.LengthSquared() - aDotB * aDotB) / baL2, 0));
                }

                return true;
            }

            dist = 0;
            return false;
        }

        private Face NewFace(GJK.SV a, GJK.SV b, GJK.SV c, bool forced)
        {
            if (stock.root != null)
            {
                Face face = stock.root;
                Remove(stock, face);
                Append(hull, face);
                face.pass = 0;
                face.c[0] = a;
                face.c[1] = b;
                face.c[2] = c;
                face.n = (b.w - a.w).Cross(c.w - a.w);
                btScalar l = face.n.Length();
                bool v = l > EpaAccuracy;

                if (v)
                {
                    if (!(GetEdgeDist(face, a, b, out face.d) ||
                          GetEdgeDist(face, b, c, out face.d) ||
                          GetEdgeDist(face, c, a, out face.d)))
                    {
                        // Origin projects inside the triangle; use the distance to the triangle's plane
                        face.d = a.w.Dot(face.n) / l;
                    }

                    face.n /= l;
                    if (forced || face.d >= -EpaPlaneEps)
                    {
                        return face;
                    }
                    else
                    {
                        status = Status.NonConvex;
                    }
                }
                else
                {
                    status = Status.Degenerated;
                }

                Remove(hull, face);
                Append(stock, face);
                return null;
            }
            status = stock.root != null ? Status.OutOfVertices : Status.OutOfFaces;
            return null;
        }

        private Face FindBest()
        {
            Face minf = hull.root;
            btScalar mind = minf.d * minf.d;
            for (Face f = minf.l[1]; f != null; f = f.l[1])
            {
                btScalar sqd = f.d * f.d;
                if (sqd < mind)
                {
                    minf = f;
                    mind = sqd;
                }
            }
            return minf;
        }

        private static readonly int[] i1m3 = { 1, 2, 0 };
        private static readonly int[] i2m3 = { 2, 0, 1 };

        private bool Expand(int pass, GJK.SV w, Face f, int e, ref Horizon horizon)
        {
            if (f.pass != pass)
            {
                int e1 = i1m3[e];
                if (f.n.Dot(w.w) - f.d < -EpaPlaneEps)
                {
                    Face nf = NewFace(f.c[e1], f.c[e], w, false);
                    if (nf != null)
                    {
                        Bind(nf, 0, f, e);
                        if (horizon.cf != null)
                            Bind(horizon.cf, 1, nf, 2);
                        else
                            horizon.ff = nf;
                        horizon.cf = nf;
                        ++horizon.nf;
                        return true;
                    }
                }
                else
                {
                    int e2 = i2m3[e];
                    f.pass = pass;
                    if (Expand(pass, w, f.f[e1], f.e[e1], ref horizon) &&
                        Expand(pass, w, f.f[e2], f.e[e2], ref horizon))
                    {
                        Remove(hull, f);
                        Append(stock, f);
                        return true;
                    }
                }
            }
            return false;
        }
    }

    /// <summary>gjkepa2_impl::Initialize</summary>
    public static void Initialize(ConvexShape shape0, in Transform wtrs0,
        ConvexShape shape1, in Transform wtrs1,
        ref GjkEpaSolver2.Results results,
        ref MinkowskiDiff shape,
        bool withmargins)
    {
        // Results
        results.witness0 = Vector3.Zero;
        results.witness1 = Vector3.Zero;
        results.status = GjkEpaSolver2.ResultsStatus.Separated;
        // Shape
        shape.shape0 = shape0;
        shape.shape1 = shape1;
        shape.toshape1 = wtrs1.Basis.TransposeTimes(wtrs0.Basis);
        shape.toshape0 = wtrs0.InverseTimes(wtrs1);
        shape.EnableMargin(withmargins);
    }
}

/// <summary>
/// Ported from btGjkEpaSolver2 (btGjkEpa2.h). Static API of the GJK/EPA collision solver.
/// </summary>
public static class GjkEpaSolver2
{
    /// <summary>sResults::eStatus</summary>
    public enum ResultsStatus
    {
        Separated,   // shapes are not penetrating
        Penetrating, // shapes are penetrating
        GjkFailed,   // GJK phase failed (usually just "touching")
        EpaFailed,   // EPA phase failed
    }

    /// <summary>btGjkEpaSolver2::sResults (value type, equivalent to the caller-side sResults on the stack in C++)</summary>
    public struct Results
    {
        internal ResultsStatus status;
        internal Vector3 witness0;
        internal Vector3 witness1;
        internal Vector3 normal;
        internal btScalar distance;

        public ResultsStatus Status
        {
            readonly get => status;
            set => status = value;
        }

        public Vector3 Witness0
        {
            readonly get => witness0;
            set => witness0 = value;
        }

        public Vector3 Witness1
        {
            readonly get => witness1;
            set => witness1 = value;
        }

        public Vector3 Normal
        {
            readonly get => normal;
            set => normal = value;
        }

        public btScalar Distance
        {
            readonly get => distance;
            set => distance = value;
        }
    }

    public static int StackSizeRequirement()
    {
        // The managed implementation doesn't need to reserve native stack space
        return 0;
    }

    // Reused solver scratch (stack objects in C++; ThreadStatic keeps it safe across threads/worlds).
    // GJK.Evaluate / EPA.Evaluate reset their own state on each call, so reuse is safe.
    [ThreadStatic]
    private static GJK s_gjk;
    [ThreadStatic]
    private static EPA s_epa;
    [ThreadStatic]
    private static SphereShape s_pointSphere;

    /// <summary>btGjkEpaSolver2::Distance</summary>
    public static bool Distance(ConvexShape shape0, in Transform wtrs0,
        ConvexShape shape1, in Transform wtrs1,
        Vector3 guess, ref Results results)
    {
        GjkEpa2Impl.MinkowskiDiff shape = default;
        GjkEpa2Impl.Initialize(shape0, in wtrs0, shape1, in wtrs1, ref results, ref shape, false);
        GJK gjk = s_gjk ??= new GJK();
        GJK.Status gjkStatus = gjk.Evaluate(shape, guess);
        if (gjkStatus == GJK.Status.Valid)
        {
            Vector3 w0 = Vector3.Zero;
            Vector3 w1 = Vector3.Zero;
            for (int i = 0; i < gjk.simplex.rank; ++i)
            {
                btScalar p = gjk.simplex.p[i];
                w0 += shape.Support(gjk.simplex.c[i].d, 0) * p;
                w1 += shape.Support(-gjk.simplex.c[i].d, 1) * p;
            }
            results.witness0 = wtrs0 * w0;
            results.witness1 = wtrs0 * w1;
            results.normal = w0 - w1;
            results.distance = results.normal.Length();
            results.normal /= results.distance > GjkEpa2Impl.GjkMinDistance ? results.distance : 1;
            return true;
        }
        else
        {
            results.Status = gjkStatus == GJK.Status.Inside ? ResultsStatus.Penetrating : ResultsStatus.GjkFailed;
            return false;
        }
    }

    /// <summary>btGjkEpaSolver2::Penetration</summary>
    public static bool Penetration(ConvexShape shape0, in Transform wtrs0,
        ConvexShape shape1, in Transform wtrs1,
        Vector3 guess, ref Results results, bool usemargins = true)
    {
        GjkEpa2Impl.MinkowskiDiff shape = default;
        GjkEpa2Impl.Initialize(shape0, in wtrs0, shape1, in wtrs1, ref results, ref shape, usemargins);
        GJK gjk = s_gjk ??= new GJK();
        GJK.Status gjkStatus = gjk.Evaluate(shape, -guess);
        switch (gjkStatus)
        {
            case GJK.Status.Inside:
            {
                EPA epa = s_epa ??= new EPA();
                EPA.Status epaStatus = epa.Evaluate(gjk, -guess);
                if (epaStatus != EPA.Status.Failed)
                {
                    Vector3 w0 = Vector3.Zero;
                    for (int i = 0; i < epa.result.rank; ++i)
                    {
                        w0 += shape.Support(epa.result.c[i].d, 0) * epa.result.p[i];
                    }
                    results.Status = ResultsStatus.Penetrating;
                    results.witness0 = wtrs0 * w0;
                    results.witness1 = wtrs0 * (w0 - epa.normal * epa.depth);
                    results.normal = -epa.normal;
                    results.distance = -epa.depth;
                    return true;
                }
                else
                {
                    results.Status = ResultsStatus.EpaFailed;
                }
                break;
            }
            case GJK.Status.Failed:
                results.Status = ResultsStatus.GjkFailed;
                break;
        }
        return false;
    }

    /// <summary>btGjkEpaSolver2::SignedDistance (point vs convex)</summary>
    public static btScalar SignedDistance(Vector3 position, btScalar margin,
        ConvexShape shape0, in Transform wtrs0, ref Results results)
    {
        GjkEpa2Impl.MinkowskiDiff shape = default;
        SphereShape shape1 = s_pointSphere ??= new SphereShape(0);
        shape1.Margin = margin;
        var wtrs1 = new Transform(Quaternion.Identity, position);
        GjkEpa2Impl.Initialize(shape0, in wtrs0, shape1, in wtrs1, ref results, ref shape, false);
        GJK gjk = s_gjk ??= new GJK();
        GJK.Status gjkStatus = gjk.Evaluate(shape, new Vector3(1, 1, 1));
        if (gjkStatus == GJK.Status.Valid)
        {
            Vector3 w0 = Vector3.Zero;
            Vector3 w1 = Vector3.Zero;
            for (int i = 0; i < gjk.simplex.rank; ++i)
            {
                btScalar p = gjk.simplex.p[i];
                w0 += shape.Support(gjk.simplex.c[i].d, 0) * p;
                w1 += shape.Support(-gjk.simplex.c[i].d, 1) * p;
            }
            results.witness0 = wtrs0 * w0;
            results.witness1 = wtrs0 * w1;
            Vector3 delta = results.witness1 - results.witness0;
            btScalar margins = shape0.Margin + shape1.Margin;
            btScalar length = delta.Length();
            results.normal = delta / length;
            results.witness0 += results.normal * margins;
            results.distance = length - margins;
            return results.distance;
        }
        else
        {
            if (gjkStatus == GJK.Status.Inside)
            {
                if (Penetration(shape0, in wtrs0, shape1, in wtrs1, gjk.ray, ref results))
                {
                    Vector3 delta = results.witness0 - results.witness1;
                    btScalar length = delta.Length();
                    if (length >= BtMath.SimdEpsilon)
                        results.normal = delta / length;
                    return -length;
                }
            }
        }
        return BtMath.SimdInfinity;
    }

    /// <summary>btGjkEpaSolver2::SignedDistance (convex vs convex)</summary>
    public static bool SignedDistance(ConvexShape shape0, in Transform wtrs0,
        ConvexShape shape1, in Transform wtrs1,
        Vector3 guess, ref Results results)
    {
        if (!Distance(shape0, in wtrs0, shape1, in wtrs1, guess, ref results))
            return Penetration(shape0, in wtrs0, shape1, in wtrs1, guess, ref results, false);
        return true;
    }
}
