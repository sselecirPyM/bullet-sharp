using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btBoxBoxDetector: Bullet's adaptation of the ODE box-box collision detection.
/// dMatrix3 is expressed as a Span (stackalloc, row-major, row stride 4).
/// </summary>
public class BoxBoxDetector : DiscreteCollisionDetectorInterface
{
    private const btScalar FudgeFactor = (btScalar)1.05;
#if BT_USE_DOUBLE_PRECISION
    private const btScalar Fudge2 = (btScalar)1.0e-5;
    private const btScalar Infinity = btScalar.MaxValue;
    private const btScalar MPi = (btScalar)3.14159265358979323846;
#else
    private const btScalar Fudge2 = (btScalar)1.0e-5;
    private const btScalar Infinity = float.MaxValue;
    private const btScalar MPi = (btScalar)3.14159265f;
#endif

    internal readonly BoxShape box1;
    internal readonly BoxShape box2;

    /// <summary>
    /// 差分调试开关：输出 dBoxBox2 中间数据（与 native-ref BoxBoxDump.cpp 对应，DBOX_DUMP 环境变量）
    /// </summary>
    public static bool Dump;

    public BoxBoxDetector(BoxShape box1, BoxShape box2)
    {
        this.box1 = box1;
        this.box2 = box2;
    }

    // ---------------- ODE matrix/vector helper functions ----------------
    // dDOTpq(a,b,p,q) = a[0]*b[0] + a[p]*b[q] + a[2p]*b[2q]

    private static btScalar dDot(in Vector3 a, ReadOnlySpan<btScalar> b, int bi) =>
        a.X * b[bi] + a.Y * b[bi + 1] + a.Z * b[bi + 2];

    /// <summary>dDOT41: b has stride 4, c has stride 1 (matrix column vector dot product)</summary>
    private static btScalar dDot41(ReadOnlySpan<btScalar> b, int bi, in Vector3 c) =>
        b[bi] * c.X + b[bi + 4] * c.Y + b[bi + 8] * c.Z;

    /// <summary>dDOT41 (array-array version)</summary>
    private static btScalar dDot41(ReadOnlySpan<btScalar> b, int bi, ReadOnlySpan<btScalar> c, int ci) =>
        b[bi] * c[ci] + b[bi + 4] * c[ci + 1] + b[bi + 8] * c[ci + 2];

    /// <summary>dDOT44: both operands have stride 4</summary>
    private static btScalar dDot44(ReadOnlySpan<btScalar> a, int ai, ReadOnlySpan<btScalar> b, int bi) =>
        a[ai] * b[bi] + a[ai + 4] * b[bi + 4] + a[ai + 8] * b[bi + 8];

    /// <summary>dDOT14: a has stride 1, b has stride 4 (vector dot matrix column)</summary>
    private static btScalar dDot14(in Vector3 a, ReadOnlySpan<btScalar> b, int bi) =>
        a.X * b[bi] + a.Y * b[bi + 4] + a.Z * b[bi + 8];

    /// <summary>dMULTIPLY1_331: A = B^T * C (B has row stride 4)</summary>
    private static Vector3 Multiply1_331(ReadOnlySpan<btScalar> b, in Vector3 c) =>
        new(dDot41(b, 0, in c), dDot41(b, 1, in c), dDot41(b, 2, in c));

    /// <summary>dMULTIPLY0_331: A = B * C (B has row stride 4)</summary>
    private static Vector3 Multiply0_331(ReadOnlySpan<btScalar> b, in Vector3 c) =>
        new(dDot(in c, b, 0), dDot(in c, b, 4), dDot(in c, b, 8));

    /// <summary>dLineClosestApproach</summary>
    private static void LineClosestApproach(in Vector3 pa, in Vector3 ua, in Vector3 pb, in Vector3 ub,
        out btScalar alpha, out btScalar beta)
    {
        Vector3 p = pb - pa;
        btScalar uaub = ua.Dot(ub);
        btScalar q1 = ua.Dot(p);
        btScalar q2 = -ub.Dot(p);
        btScalar d = 1 - uaub * uaub;
        if (d <= (btScalar)0.0001)
        {
            // @@@ this needs to be made more robust
            alpha = 0;
            beta = 0;
        }
        else
        {
            d = 1 / d;
            alpha = (q1 + uaub * q2) * d;
            beta = (uaub * q1 + q2) * d;
        }
    }

    /// <summary>
    /// intersectRectQuad2: computes the intersection of a 2D rectangle (±h[0], ±h[1])
    /// with a 2D quadrilateral (8 values in p). Intersection points are returned as
    /// x,y pairs in ret; returns the number of intersection points (0..8).
    /// Raw pointers keep the ping-pong buffers entirely on the stack (a stackalloc
    /// Span cannot be reassigned to an outer Span across blocks).
    /// </summary>
    private static unsafe int IntersectRectQuad2(btScalar* h, btScalar* p, btScalar* ret)
    {
        // q (and r) hold the nq (and nr) coordinates of the current (and clipped) polygon
        int nq = 4, nr = 0;
        btScalar* buffer = stackalloc btScalar[16];
        btScalar* q = p;
        btScalar* r = ret;
        bool doneAll = false;
        for (int dir = 0; dir <= 1 && !doneAll; dir++)
        {
            // direction notation: xy[0] = x axis, xy[1] = y axis
            for (int sign = -1; sign <= 1; sign += 2)
            {
                // clip q against the line xy[dir] = sign*h[dir]
                int pqIdx = 0;
                int prIdx = 0;
                nr = 0;
                for (int i = nq; i > 0; i--)
                {
                    // iterate over all points of q and the line segments between adjacent points
                    if (sign * q[pqIdx + dir] < h[dir])
                    {
                        // this point is inside the clipping line
                        r[prIdx] = q[pqIdx];
                        r[prIdx + 1] = q[pqIdx + 1];
                        prIdx += 2;
                        nr++;
                        if ((nr & 8) != 0)
                        {
                            q = r;
                            doneAll = true;
                            break;
                        }
                    }
                    // nextq is the next point (wraps back to the start of q when i==1)
                    int nextqIdx = i > 1 ? pqIdx + 2 : 0;
                    bool insideCur = sign * q[pqIdx + dir] < h[dir];
                    bool insideNext = sign * q[nextqIdx + dir] < h[dir];
                    if (insideCur != insideNext)
                    {
                        // this line segment crosses the clipping line
                        r[prIdx + 1 - dir] = q[pqIdx + 1 - dir] + (q[nextqIdx + 1 - dir] - q[pqIdx + 1 - dir]) /
                            (q[nextqIdx + dir] - q[pqIdx + dir]) * (sign * h[dir] - q[pqIdx + dir]);
                        r[prIdx + dir] = sign * h[dir];
                        prIdx += 2;
                        nr++;
                        if ((nr & 8) != 0)
                        {
                            q = r;
                            doneAll = true;
                            break;
                        }
                    }
                    pqIdx += 2;
                }
                if (doneAll)
                    break;
                // swap q/r (r alternates between ret and buffer)
                q = r;
                r = q == ret ? buffer : ret;
                nq = nr;
            }
            if (nq == 0)
                break;
        }

        if (q != ret)
        {
            for (int i = 0; i < nr * 2; i++)
                ret[i] = q[i];
        }
        return nr;
    }

    /// <summary>
    /// cullPoints2: from n planar points (p, size 2n) select the m points that best
    /// represent the whole set. Indices of the selected points are returned in iret
    /// (size m); i0 is always the first element.
    /// </summary>
    private static void CullPoints2(int n, ReadOnlySpan<btScalar> p, int m, int i0, Span<int> iret)
    {
        // compute the polygon centroid cx,cy
        btScalar a, cx, cy, q;
        if (n == 1)
        {
            cx = p[0];
            cy = p[1];
        }
        else if (n == 2)
        {
            cx = (btScalar)0.5 * (p[0] + p[2]);
            cy = (btScalar)0.5 * (p[1] + p[3]);
        }
        else
        {
            a = 0;
            cx = 0;
            cy = 0;
            for (int i = 0; i < n - 1; i++)
            {
                q = p[i * 2] * p[i * 2 + 3] - p[i * 2 + 2] * p[i * 2 + 1];
                a += q;
                cx += q * (p[i * 2] + p[i * 2 + 2]);
                cy += q * (p[i * 2 + 1] + p[i * 2 + 3]);
            }
            q = p[n * 2 - 2] * p[1] - p[0] * p[n * 2 - 1];
            if (BtMath.Abs(a + q) > BtMath.SimdEpsilon)
            {
                a = 1 / (3 * (a + q));
            }
            else
            {
                a = BtMath.BtLargeFloat;
            }
            cx = a * (cx + q * (p[n * 2 - 2] + p[0]));
            cy = a * (cy + q * (p[n * 2 - 1] + p[1]));
        }

        // compute the angle of each point relative to the centroid
        Span<btScalar> angles = stackalloc btScalar[8];
        for (int i = 0; i < n; i++) angles[i] = BtMath.Atan2(p[i * 2 + 1] - cy, p[i * 2] - cx);

        // search for the points whose angle is closest to A[i0] + i*(2*pi/m)
        Span<int> avail = stackalloc int[8];
        for (int i = 0; i < n; i++) avail[i] = 1;
        avail[i0] = 0;
        iret[0] = i0;
        int iretIdx = 1;
        for (int j = 1; j < m; j++)
        {
            a = j * (2 * MPi / m) + angles[i0];
            if (a > MPi) a -= 2 * MPi;
            btScalar maxdiff = (btScalar)1e9;

            iret[iretIdx] = i0; // iret should not keep this value, but occasionally does when diff=#QNAN0

            for (int i = 0; i < n; i++)
            {
                if (avail[i] != 0)
                {
                    btScalar diff = BtMath.Abs(angles[i] - a);
                    if (diff > MPi) diff = 2 * MPi - diff;
                    if (diff < maxdiff)
                    {
                        maxdiff = diff;
                        iret[iretIdx] = i;
                    }
                }
            }
            avail[iret[iretIdx]] = 0;
            iretIdx++;
        }
    }

    /// <summary>
    /// dBoxBox2: collides two boxes (p1,R1,side1) and (p2,R2,side2) and generates
    /// contact points. Returns 0 if there is no contact, otherwise the number of
    /// contact points generated.
    /// returnCode: 1,2,3 = face of box2 intersects box1; 4,5,6 = face of box1
    /// intersects box2; 7..15 = edge-edge contact.
    /// </summary>
    private static unsafe int DBoxBox2(in Vector3 p1, ReadOnlySpan<btScalar> r1, in Vector3 side1,
        in Vector3 p2, ReadOnlySpan<btScalar> r2, in Vector3 side2,
        out Vector3 normal, out btScalar depth, out int returnCode,
        int maxc, Result output)
    {
        normal = Vector3.Zero;
        depth = 0;
        returnCode = 0;

        Vector3 normalC = Vector3.Zero;
        ReadOnlySpan<btScalar> normalR = default; // points to a column of R1/R2
        int normalROffset = 0;
        btScalar s, s2;

        if (Dump)
            Console.WriteLine($"DB IN p1=({p1.X:E8} {p1.Y:E8} {p1.Z:E8}) p2=({p2.X:E8} {p2.Y:E8} {p2.Z:E8}) side1=({side1.X:E8} {side1.Y:E8} {side1.Z:E8}) side2=({side2.X:E8} {side2.Y:E8} {side2.Z:E8})");

        // vector from box1 center to box2 center, in the box1 frame
        Vector3 p = p2 - p1;
        Vector3 pp = Multiply1_331(r1, in p); // pp = p relative to body1

        // half side lengths
        Span<btScalar> a = stackalloc btScalar[3] { side1.X * (btScalar)0.5, side1.Y * (btScalar)0.5, side1.Z * (btScalar)0.5 };
        Span<btScalar> b = stackalloc btScalar[3] { side2.X * (btScalar)0.5, side2.Y * (btScalar)0.5, side2.Z * (btScalar)0.5 };

        // Rij is R1'*R2, the relative rotation between R1 and R2
        btScalar r11 = dDot44(r1, 0, r2, 0);
        btScalar r12 = dDot44(r1, 0, r2, 1);
        btScalar r13 = dDot44(r1, 0, r2, 2);
        btScalar r21 = dDot44(r1, 1, r2, 0);
        btScalar r22 = dDot44(r1, 1, r2, 1);
        btScalar r23 = dDot44(r1, 1, r2, 2);
        btScalar r31 = dDot44(r1, 2, r2, 0);
        btScalar r32 = dDot44(r1, 2, r2, 1);
        btScalar r33 = dDot44(r1, 2, r2, 2);

        btScalar q11 = BtMath.Abs(r11);
        btScalar q12 = BtMath.Abs(r12);
        btScalar q13 = BtMath.Abs(r13);
        btScalar q21 = BtMath.Abs(r21);
        btScalar q22 = BtMath.Abs(r22);
        btScalar q23 = BtMath.Abs(r23);
        btScalar q31 = BtMath.Abs(r31);
        btScalar q32 = BtMath.Abs(r32);
        btScalar q33 = BtMath.Abs(r33);

        // iterate over the 15 possible separating axes:
        //   if an axis separates the two boxes, return 0; otherwise compute the
        //   penetration depth s2 along that axis and record it if it is larger than
        //   the current best depth. The final normal is the axis with minimum depth.
        // normalR points to a column of R1/R2; otherwise normalR is empty and
        // normalC holds the normal relative to body1.
        // invertNormal indicates whether the normal sign should be flipped.

        if (Dump)
            Console.WriteLine($"DB ROT pp=({pp.X:E8} {pp.Y:E8} {pp.Z:E8}) R1=({r1[0]:E8} {r1[1]:E8} {r1[2]:E8} {r1[4]:E8} {r1[5]:E8} {r1[6]:E8} {r1[8]:E8} {r1[9]:E8} {r1[10]:E8}) R2=({r2[0]:E8} {r2[1]:E8} {r2[2]:E8} {r2[4]:E8} {r2[5]:E8} {r2[6]:E8} {r2[8]:E8} {r2[9]:E8} {r2[10]:E8})");

        s = -Infinity;
        int invertNormal = 0;
        int code = 0;

        // separating axis = u1,u2,u3 (TST macro expanded)
        {
            btScalar expr1 = pp.X;
            s2 = BtMath.Abs(expr1) - (a[0] + b[0] * q11 + b[1] * q12 + b[2] * q13);
            if (s2 > 0) return 0;
            if (Dump) Console.WriteLine($"DB FACE 1 s2={s2:E8}");
            if (s2 > s) { s = s2; normalR = r1; normalROffset = 0; invertNormal = expr1 < 0 ? 1 : 0; code = 1; }
        }
        {
            btScalar expr1 = pp.Y;
            s2 = BtMath.Abs(expr1) - (a[1] + b[0] * q21 + b[1] * q22 + b[2] * q23);
            if (s2 > 0) return 0;
            if (Dump) Console.WriteLine($"DB FACE 2 s2={s2:E8}");
            if (s2 > s) { s = s2; normalR = r1; normalROffset = 1; invertNormal = expr1 < 0 ? 1 : 0; code = 2; }
        }
        {
            btScalar expr1 = pp.Z;
            s2 = BtMath.Abs(expr1) - (a[2] + b[0] * q31 + b[1] * q32 + b[2] * q33);
            if (s2 > 0) return 0;
            if (Dump) Console.WriteLine($"DB FACE 3 s2={s2:E8}");
            if (s2 > s) { s = s2; normalR = r1; normalROffset = 2; invertNormal = expr1 < 0 ? 1 : 0; code = 3; }
        }

        // separating axis = v1,v2,v3
        {
            btScalar expr1 = dDot41(r2, 0, in p);
            s2 = BtMath.Abs(expr1) - (a[0] * q11 + a[1] * q21 + a[2] * q31 + b[0]);
            if (s2 > 0) return 0;
            if (Dump) Console.WriteLine($"DB FACE 4 s2={s2:E8}");
            if (s2 > s) { s = s2; normalR = r2; normalROffset = 0; invertNormal = expr1 < 0 ? 1 : 0; code = 4; }
        }
        {
            btScalar expr1 = dDot41(r2, 1, in p);
            s2 = BtMath.Abs(expr1) - (a[0] * q12 + a[1] * q22 + a[2] * q32 + b[1]);
            if (s2 > 0) return 0;
            if (Dump) Console.WriteLine($"DB FACE 5 s2={s2:E8}");
            if (s2 > s) { s = s2; normalR = r2; normalROffset = 1; invertNormal = expr1 < 0 ? 1 : 0; code = 5; }
        }
        {
            btScalar expr1 = dDot41(r2, 2, in p);
            s2 = BtMath.Abs(expr1) - (a[0] * q13 + a[1] * q23 + a[2] * q33 + b[2]);
            if (s2 > 0) return 0;
            if (Dump) Console.WriteLine($"DB FACE 6 s2={s2:E8}");
            if (s2 > s) { s = s2; normalR = r2; normalROffset = 2; invertNormal = expr1 < 0 ? 1 : 0; code = 6; }
        }

        // note: cross-product axes need scaling when computing s. The normal
        // (n1,n2,n3) is relative to box1.
        q11 += Fudge2;
        q12 += Fudge2;
        q13 += Fudge2;
        q21 += Fudge2;
        q22 += Fudge2;
        q23 += Fudge2;
        q31 += Fudge2;
        q32 += Fudge2;
        q33 += Fudge2;

        // inlined implementation of the cross-product-axis TST
        static bool EdgeTst(btScalar expr1, btScalar expr2, btScalar n1, btScalar n2, btScalar n3, int cc,
            ref btScalar sRef, ref Vector3 normalCRef, ref ReadOnlySpan<btScalar> normalRRef, ref int invertNormalRef, ref int codeRef, out bool separated)
        {
            separated = false;
            btScalar s2 = BtMath.Abs(expr1) - expr2;
            if (s2 > BtMath.SimdEpsilon) { separated = true; return false; }
            btScalar l = BtMath.Sqrt(n1 * n1 + n2 * n2 + n3 * n3);
            if (l > BtMath.SimdEpsilon)
            {
                s2 /= l;
                if (Dump) Console.WriteLine($"DB EDGE {cc} s2={s2:E8} l={l:E8}");
                if (s2 * FudgeFactor > sRef)
                {
                    sRef = s2;
                    normalRRef = default; // C++ TST: normalR = 0
                    normalCRef = new Vector3(n1 / l, n2 / l, n3 / l);
                    invertNormalRef = expr1 < 0 ? 1 : 0;
                    codeRef = cc;
                }
            }
            return true;
        }

        // separating axis = u1 x (v1,v2,v3)
        if (!EdgeTst(pp.Z * r21 - pp.Y * r31, a[1] * q31 + a[2] * q21 + b[1] * q13 + b[2] * q12, 0, -r31, r21, 7,
                ref s, ref normalC, ref normalR, ref invertNormal, ref code, out bool sep)) return 0;
        if (sep) return 0;
        if (!EdgeTst(pp.Z * r22 - pp.Y * r32, a[1] * q32 + a[2] * q22 + b[0] * q13 + b[2] * q11, 0, -r32, r22, 8,
                ref s, ref normalC, ref normalR, ref invertNormal, ref code, out sep)) return 0;
        if (sep) return 0;
        if (!EdgeTst(pp.Z * r23 - pp.Y * r33, a[1] * q33 + a[2] * q23 + b[0] * q12 + b[1] * q11, 0, -r33, r23, 9,
                ref s, ref normalC, ref normalR, ref invertNormal, ref code, out sep)) return 0;
        if (sep) return 0;

        // separating axis = u2 x (v1,v2,v3)
        if (!EdgeTst(pp.X * r31 - pp.Z * r11, a[0] * q31 + a[2] * q11 + b[1] * q23 + b[2] * q22, r31, 0, -r11, 10,
                ref s, ref normalC, ref normalR, ref invertNormal, ref code, out sep)) return 0;
        if (sep) return 0;
        if (!EdgeTst(pp.X * r32 - pp.Z * r12, a[0] * q32 + a[2] * q12 + b[0] * q23 + b[2] * q21, r32, 0, -r12, 11,
                ref s, ref normalC, ref normalR, ref invertNormal, ref code, out sep)) return 0;
        if (sep) return 0;
        if (!EdgeTst(pp.X * r33 - pp.Z * r13, a[0] * q33 + a[2] * q13 + b[0] * q22 + b[1] * q21, r33, 0, -r13, 12,
                ref s, ref normalC, ref normalR, ref invertNormal, ref code, out sep)) return 0;
        if (sep) return 0;

        // separating axis = u3 x (v1,v2,v3)
        if (!EdgeTst(pp.Y * r11 - pp.X * r21, a[0] * q21 + a[1] * q11 + b[1] * q33 + b[2] * q32, -r21, r11, 0, 13,
                ref s, ref normalC, ref normalR, ref invertNormal, ref code, out sep)) return 0;
        if (sep) return 0;
        if (!EdgeTst(pp.Y * r12 - pp.X * r22, a[0] * q22 + a[1] * q12 + b[0] * q33 + b[2] * q31, -r22, r12, 0, 14,
                ref s, ref normalC, ref normalR, ref invertNormal, ref code, out sep)) return 0;
        if (sep) return 0;
        if (!EdgeTst(pp.Y * r13 - pp.X * r23, a[0] * q23 + a[1] * q13 + b[0] * q32 + b[1] * q31, -r23, r13, 0, 15,
                ref s, ref normalC, ref normalR, ref invertNormal, ref code, out sep)) return 0;
        if (sep) return 0;

        if (code == 0) return 0;

        // at this point the boxes interpenetrate; compute the normal in global coordinates
        if (!normalR.IsEmpty)
        {
            normal = new Vector3(normalR[normalROffset], normalR[normalROffset + 4], normalR[normalROffset + 8]);
        }
        else
        {
            normal = Multiply0_331(r1, normalC);
        }
        if (invertNormal != 0)
        {
            normal = -normal;
        }
        depth = -s;

        if (Dump)
            Console.WriteLine($"DB code={code} depth={depth:E8} s={s:E8} inv={invertNormal} normal=({normal.X:E8} {normal.Y:E8} {normal.Z:E8})");

        // compute contact points
        if (code > 6)
        {
            // an edge of box1 is in contact with an edge of box2.
            // find point pa on the intersecting edge of box1
            Vector3 pa = p1;
            for (int j = 0; j < 3; j++)
            {
                btScalar sign = dDot14(in normal, r1, j) > 0 ? 1 : -1;
                for (int i = 0; i < 3; i++) pa[i] += sign * a[j] * r1[i * 4 + j];
            }

            // find point pb on the intersecting edge of box2
            Vector3 pb = p2;
            for (int j = 0; j < 3; j++)
            {
                btScalar sign = dDot14(in normal, r2, j) > 0 ? -1 : 1;
                for (int i = 0; i < 3; i++) pb[i] += sign * b[j] * r2[i * 4 + j];
            }

            Vector3 ua = new(r1[(code - 7) / 3], r1[(code - 7) / 3 + 4], r1[(code - 7) / 3 + 8]);
            Vector3 ub = new(r2[(code - 7) % 3], r2[(code - 7) % 3 + 4], r2[(code - 7) % 3 + 8]);

            LineClosestApproach(in pa, in ua, in pb, in ub, out btScalar alpha, out btScalar beta);
            pa += ua * alpha;
            pb += ub * beta;

            output.AddContactPoint(-normal, pb, -depth);
            returnCode = code;
            return 1;
        }

        // a face intersects something (the separating axis is perpendicular to a
        // face). Define the reference face 'a' (the face whose normal is the axis)
        // and the incident face 'b' (the closest face of the other box).
        // sa/sb are copies of the half side lengths (a stackalloc Span cannot be
        // reassigned to an outer variable across blocks; the 3-element copy cost is negligible)
        ReadOnlySpan<btScalar> ra, rb;
        Vector3 paC, pbC;
        Span<btScalar> sa = stackalloc btScalar[3];
        Span<btScalar> sb = stackalloc btScalar[3];
        if (code <= 3)
        {
            ra = r1;
            rb = r2;
            paC = p1;
            pbC = p2;
            a.CopyTo(sa);
            b.CopyTo(sb);
        }
        else
        {
            ra = r2;
            rb = r1;
            paC = p2;
            pbC = p1;
            b.CopyTo(sa);
            a.CopyTo(sb);
        }

        // nr = dot product of the reference-face normal with each axis of the incident box; anr = its absolute value
        Vector3 normal2 = code <= 3 ? normal : -normal;
        Vector3 nr = Multiply1_331(rb, normal2);
        Vector3 anr = nr.Absolute();

        // find the largest component of anr: the incident-face normal. The other two
        // incident-face axis indices go into a1,a2
        int lanr, a1, a2;
        if (anr.Y > anr.X)
        {
            if (anr.Y > anr.Z)
            {
                a1 = 0;
                lanr = 1;
                a2 = 2;
            }
            else
            {
                a1 = 0;
                a2 = 1;
                lanr = 2;
            }
        }
        else
        {
            if (anr.X > anr.Z)
            {
                lanr = 0;
                a1 = 1;
                a2 = 2;
            }
            else
            {
                a1 = 0;
                a2 = 1;
                lanr = 2;
            }
        }

        // compute the center point of the incident face, in reference-face coordinates
        Vector3 center;
        if (nr[lanr] < 0)
        {
            center = new Vector3(
                pbC.X - paC.X + sb[lanr] * rb[0 * 4 + lanr],
                pbC.Y - paC.Y + sb[lanr] * rb[1 * 4 + lanr],
                pbC.Z - paC.Z + sb[lanr] * rb[2 * 4 + lanr]);
        }
        else
        {
            center = new Vector3(
                pbC.X - paC.X - sb[lanr] * rb[0 * 4 + lanr],
                pbC.Y - paC.Y - sb[lanr] * rb[1 * 4 + lanr],
                pbC.Z - paC.Z - sb[lanr] * rb[2 * 4 + lanr]);
        }

        // normal and non-normal axis indices of the reference box
        int codeN, code1, code2;
        if (code <= 3)
            codeN = code - 1;
        else
            codeN = code - 4;
        if (codeN == 0)
        {
            code1 = 1;
            code2 = 2;
        }
        else if (codeN == 1)
        {
            code1 = 0;
            code2 = 2;
        }
        else
        {
            code1 = 0;
            code2 = 1;
        }

        // find the four corners of the incident face, in reference-face 2D coordinates
        btScalar* quad = stackalloc btScalar[8];
        btScalar c1 = dDot14(in center, ra, code1);
        btScalar c2 = dDot14(in center, ra, code2);
        btScalar m11 = dDot44(ra, code1, rb, a1);
        btScalar m12 = dDot44(ra, code1, rb, a2);
        btScalar m21 = dDot44(ra, code2, rb, a1);
        btScalar m22 = dDot44(ra, code2, rb, a2);
        {
            btScalar k1 = m11 * sb[a1];
            btScalar k2 = m21 * sb[a1];
            btScalar k3 = m12 * sb[a2];
            btScalar k4 = m22 * sb[a2];
            quad[0] = c1 - k1 - k3;
            quad[1] = c2 - k2 - k4;
            quad[2] = c1 - k1 + k3;
            quad[3] = c2 - k2 + k4;
            quad[4] = c1 + k1 + k3;
            quad[5] = c2 + k2 + k4;
            quad[6] = c1 + k1 - k3;
            quad[7] = c2 + k2 - k4;
        }

        // reference face size
        btScalar* rect = stackalloc btScalar[2];
        rect[0] = sa[code1];
        rect[1] = sa[code2];

        // find the intersection of the incident face and the reference face
        btScalar* ret = stackalloc btScalar[16];
        int n = IntersectRectQuad2(rect, quad, ret);
        if (n < 1) return 0; // should never happen

        if (Dump)
        {
            Console.WriteLine($"DB c1={c1:E8} c2={c2:E8} m11={m11:E8} m12={m12:E8} m21={m21:E8} m22={m22:E8}");
            Console.WriteLine($"DB quad={quad[0]:E8} {quad[1]:E8} {quad[2]:E8} {quad[3]:E8} {quad[4]:E8} {quad[5]:E8} {quad[6]:E8} {quad[7]:E8} rect={rect[0]:E8} {rect[1]:E8} n={n}");
            for (int q = 0; q < n; q++) Console.WriteLine($"DB ret {q} {ret[q * 2]:E8} {ret[q * 2 + 1]:E8}");
        }

        // convert the intersection points to reference-face coordinates and compute
        // the contact position and depth of each point. Keep only points with
        // positive penetration depth, compacting ret so that point and ret correspond.
        Span<btScalar> point = stackalloc btScalar[3 * 8]; // penetrating contact points
        Span<btScalar> dep = stackalloc btScalar[8];       // depths for those points
        btScalar det1 = 1 / (m11 * m22 - m12 * m21);
        m11 *= det1;
        m12 *= det1;
        m21 *= det1;
        m22 *= det1;
        int cnum = 0; // number of penetrating contact points found
        for (int j = 0; j < n; j++)
        {
            btScalar k1 = m22 * (ret[j * 2] - c1) - m12 * (ret[j * 2 + 1] - c2);
            btScalar k2 = -m21 * (ret[j * 2] - c1) + m11 * (ret[j * 2 + 1] - c2);
            for (int i = 0; i < 3; i++)
                point[cnum * 3 + i] = center[i] + k1 * rb[i * 4 + a1] + k2 * rb[i * 4 + a2];
            dep[cnum] = sa[codeN] - dDot(normal2, point, cnum * 3);
            if (dep[cnum] >= 0)
            {
                ret[cnum * 2] = ret[j * 2];
                ret[cnum * 2 + 1] = ret[j * 2 + 1];
                cnum++;
            }
        }
        if (cnum < 1) return 0; // should never happen

        if (Dump)
        {
            for (int q = 0; q < cnum; q++)
                Console.WriteLine($"DB dep {q} {dep[q]:E8} point=({point[q * 3]:E8} {point[q * 3 + 1]:E8} {point[q * 3 + 2]:E8})");
            Console.WriteLine($"DB cnum={cnum} maxc={maxc}");
        }

        // we can't generate more contacts than are actually available
        if (maxc > cnum) maxc = cnum;
        if (maxc < 1) maxc = 1;

        if (cnum <= maxc)
        {
            if (code < 4)
            {
                // fewer contacts than requested, use them all
                for (int j = 0; j < cnum; j++)
                {
                    Vector3 pointInWorld = new(
                        point[j * 3] + paC.X,
                        point[j * 3 + 1] + paC.Y,
                        point[j * 3 + 2] + paC.Z);
                    output.AddContactPoint(-normal, pointInWorld, -dep[j]);
                }
            }
            else
            {
                for (int j = 0; j < cnum; j++)
                {
                    Vector3 pointInWorld = new(
                        point[j * 3] + paC.X - normal.X * dep[j],
                        point[j * 3 + 1] + paC.Y - normal.Y * dep[j],
                        point[j * 3 + 2] + paC.Z - normal.Z * dep[j]);
                    output.AddContactPoint(-normal, pointInWorld, -dep[j]);
                }
            }
        }
        else
        {
            // we have more contacts than are wanted, some of them must be culled.
            // the deepest point is always the first contact point
            int i1 = 0;
            btScalar maxdepth = dep[0];
            for (int i = 1; i < cnum; i++)
            {
                if (dep[i] > maxdepth)
                {
                    maxdepth = dep[i];
                    i1 = i;
                }
            }

            Span<int> iret = stackalloc int[8];
            CullPoints2(cnum, new ReadOnlySpan<btScalar>(ret, 16), maxc, i1, iret);

            if (Dump)
            {
                Console.WriteLine($"DB i1={i1} iret={iret[0]} {iret[1]} {iret[2]} {iret[3]}");
                for (int q = 0; q < cnum; q++) Console.WriteLine($"DB ret2 {q} {ret[q * 2]:E8} {ret[q * 2 + 1]:E8}");
            }

            for (int j = 0; j < maxc; j++)
            {
                Vector3 posInWorld = new(
                    point[iret[j] * 3] + paC.X,
                    point[iret[j] * 3 + 1] + paC.Y,
                    point[iret[j] * 3 + 2] + paC.Z);
                if (code < 4)
                {
                    if (Dump)
                        Console.WriteLine($"DB ADD pos=({posInWorld.X:E8} {posInWorld.Y:E8} {posInWorld.Z:E8}) dep={-dep[iret[j]]:E8}");
                    output.AddContactPoint(-normal, posInWorld, -dep[iret[j]]);
                }
                else
                {
                    if (Dump)
                    {
                        Vector3 pos2 = posInWorld - normal * dep[iret[j]];
                        Console.WriteLine($"DB ADD pos=({pos2.X:E8} {pos2.Y:E8} {pos2.Z:E8}) dep={-dep[iret[j]]:E8}");
                    }
                    output.AddContactPoint(-normal, posInWorld - normal * dep[iret[j]], -dep[iret[j]]);
                }
            }
            cnum = maxc;
        }

        returnCode = code;
        return cnum;
    }

    /// <summary>btBoxBoxDetector::getClosestPoints</summary>
    public override void GetClosestPoints(ClosestPointInput input, Result output, IDebugDraw debugDraw, bool swapResults = false)
    {
        Transform transformA = input.transformA;
        Transform transformB = input.transformB;

        Span<btScalar> r1 = stackalloc btScalar[12];
        Span<btScalar> r2 = stackalloc btScalar[12];

        for (int j = 0; j < 3; j++)
        {
            Vector3 rowA = transformA.Basis.Row(j);
            Vector3 rowB = transformB.Basis.Row(j);
            r1[0 + 4 * j] = rowA.X;
            r2[0 + 4 * j] = rowB.X;

            r1[1 + 4 * j] = rowA.Y;
            r2[1 + 4 * j] = rowB.Y;

            r1[2 + 4 * j] = rowA.Z;
            r2[2 + 4 * j] = rowB.Z;
        }

        int maxc = 4;

        DBoxBox2(transformA.Origin,
            r1,
            2 * box1.HalfExtentsWithMargin,
            transformB.Origin,
            r2,
            2 * box2.HalfExtentsWithMargin,
            out _, out _, out _,
            maxc,
            output);
    }
}
