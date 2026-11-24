using BulletSharp.Math;

namespace BulletSharp;

/// <summary>Internal interface form of btConvexPenetrationDepthSolver (penetration depth solver)</summary>
internal interface IConvexPenetrationDepthSolver
{
    bool CalcPenDepth(VoronoiSimplexSolver simplexSolver,
        ConvexShape pConvexA, ConvexShape pConvexB,
        in Transform transformA, in Transform transformB,
        ref Vector3 v, out Vector3 wWitnessOnA, out Vector3 wWitnessOnB);
}

/// <summary>
/// Ported from btGjkPairDetector: GJK-based implementation of btDiscreteCollisionDetectorInterface.
/// Starts with a libccd-style intersection pre-check ("Real-time Collision Detection with Implicit Objects",
/// Leif Olvang) to fix issue #1703.
/// </summary>
public class GjkPairDetector : DiscreteCollisionDetectorInterface
{
#if BT_USE_DOUBLE_PRECISION
    private const btScalar RelError2 = (btScalar)1.0e-12;
    /// <summary>gGjkEpaPenetrationTolerance</summary>
    public static btScalar GjkEpaPenetrationTolerance = (btScalar)1.0e-12;
#else
    private const btScalar RelError2 = (btScalar)1.0e-6;
    /// <summary>gGjkEpaPenetrationTolerance</summary>
    public static btScalar GjkEpaPenetrationTolerance = (btScalar)0.001;
#endif

    internal Vector3 cachedSeparatingAxis = new(0, 1, 0);
    internal ConvexCast penetrationDepthSolver;
    internal VoronoiSimplexSolver simplexSolver;
    internal ConvexShape minkowskiA;
    internal ConvexShape minkowskiB;
    internal int shapeTypeA;
    internal int shapeTypeB;
    internal btScalar marginA;
    internal btScalar marginB;

    internal bool ignoreMargin;
    internal btScalar cachedSeparatingDistance;

    // For debugging degenerate cases
    internal int lastUsedMethod = -1;
    internal int curIter;
    internal int degenerateSimplex;
    internal int catchDegeneracies = 1;
    internal int fixContactNormalDirection = 1;

    public GjkPairDetector(ConvexShape objectA, ConvexShape objectB,
        VoronoiSimplexSolver simplexSolver, ConvexCast penetrationDepthSolver)
    {
        this.penetrationDepthSolver = penetrationDepthSolver;
        this.simplexSolver = simplexSolver;
        minkowskiA = objectA;
        minkowskiB = objectB;
        shapeTypeA = (int)objectA.ShapeType;
        shapeTypeB = (int)objectB.ShapeType;
        marginA = objectA.Margin;
        marginB = objectB.Margin;
    }

    public GjkPairDetector(ConvexShape objectA, ConvexShape objectB,
        int shapeTypeA, int shapeTypeB, btScalar marginA, btScalar marginB,
        VoronoiSimplexSolver simplexSolver, ConvexCast penetrationDepthSolver)
    {
        this.penetrationDepthSolver = penetrationDepthSolver;
        this.simplexSolver = simplexSolver;
        minkowskiA = objectA;
        minkowskiB = objectB;
        this.shapeTypeA = shapeTypeA;
        this.shapeTypeB = shapeTypeB;
        this.marginA = marginA;
        this.marginB = marginB;
    }

    public ConvexShape MinkowskiA
    {
        get => minkowskiA;
        set => minkowskiA = value;
    }

    public ConvexShape MinkowskiB
    {
        get => minkowskiB;
        set => minkowskiB = value;
    }

    public Vector3 CachedSeparatingAxis
    {
        get => cachedSeparatingAxis;
        set => cachedSeparatingAxis = value;
    }

    public btScalar CachedSeparatingDistance => cachedSeparatingDistance;

    public ConvexCast PenetrationDepthSolver
    {
        get => penetrationDepthSolver;
        set => penetrationDepthSolver = value;
    }

    /// <summary>don't use SetIgnoreMargin, it's for Bullet's internal use</summary>
    public void SetIgnoreMargin(bool ignoreMargin) => this.ignoreMargin = ignoreMargin;

    public int LastUsedMethod => lastUsedMethod;
    public int CurrentIteration => curIter;
    public int DegenerateSimplex => degenerateSimplex;

    public int CatchDegeneracies
    {
        get => catchDegeneracies;
        set => catchDegeneracies = value;
    }

    public int FixContactNormalDirection
    {
        get => fixContactNormalDirection;
        set => fixContactNormalDirection = value;
    }

    /// <summary>btGjkPairDetector::getClosestPoints</summary>
    public override void GetClosestPoints(ClosestPointInput input, Result output, IDebugDraw debugDraw, bool swapResults = false)
    {
        GetClosestPointsNonVirtual(input, output, debugDraw);
    }

    // ---------------- Helper structures for the libccd-style intersection test ----------------

    private struct SupportVector
    {
        public Vector3 v;  // support point in the Minkowski difference
        public Vector3 v1; // support point in obj1
        public Vector3 v2; // support point in obj2
    }

    private sealed class CcdSimplex
    {
        public readonly SupportVector[] ps = new SupportVector[4];
        public int last = -1; // index of the last added point

        public int Size => last + 1;
    }

    // Reused libccd simplex (a stack object in C++; avoids an allocation per GJK call)
    private readonly CcdSimplex ccdSimplex = new();

    private static void ComputeSupport(ConvexShape convexA, in Transform localTransA,
        ConvexShape convexB, in Transform localTransB, Vector3 dir, bool check2d,
        out Vector3 supAworld, out Vector3 supBworld, out Vector3 aMinb)
    {
        Vector3 separatingAxisInA = dir * localTransA.Basis;
        Vector3 separatingAxisInB = -dir * localTransB.Basis;

        Vector3 pInA = convexA.LocalGetSupportingVertexWithoutMargin(separatingAxisInA);
        Vector3 qInB = convexB.LocalGetSupportingVertexWithoutMargin(separatingAxisInB);

        supAworld = localTransA * pInA;
        supBworld = localTransB * qInB;

        if (check2d)
        {
            supAworld.Z = 0;
            supBworld.Z = 0;
        }

        aMinb = supAworld - supBworld;
    }

    private static bool CcdEq(btScalar a, btScalar b)
    {
        btScalar ab = BtMath.Abs(a - b);
        if (ab < BtMath.SimdEpsilon)
            return true;

        btScalar absA = BtMath.Abs(a);
        btScalar absB = BtMath.Abs(b);
        if (absB > absA)
            return ab < BtMath.SimdEpsilon * absB;
        return ab < BtMath.SimdEpsilon * absA;
    }

    private static bool Vec3Eq(in Vector3 a, in Vector3 b) =>
        CcdEq(a.X, b.X) && CcdEq(a.Y, b.Y) && CcdEq(a.Z, b.Z);

    private static bool FuzzyZero(btScalar v) => BtMath.Abs(v) < BtMath.SimdEpsilon;

    private static int CcdSign(btScalar val)
    {
        if (FuzzyZero(val))
            return 0;
        if (val < 0)
            return -1;
        return 1;
    }

    /// <summary>btVec3PointSegmentDist2: squared distance from point P to segment x0-b (optionally outputs a witness)</summary>
    private static btScalar Vec3PointSegmentDist2(in Vector3 p, in Vector3 x0, in Vector3 b, ref Vector3 witness, bool computeWitness)
    {
        Vector3 d = b - x0;
        Vector3 a = x0 - p;

        btScalar t = -1 * a.Dot(d);
        t /= d.Dot(d);

        btScalar dist;
        if (t < 0 || FuzzyZero(t))
        {
            dist = (x0 - p).LengthSquared();
            if (computeWitness)
                witness = x0;
        }
        else if (t > 1 || CcdEq(t, 1))
        {
            dist = (b - p).LengthSquared();
            if (computeWitness)
                witness = b;
        }
        else
        {
            if (computeWitness)
            {
                witness = d * t + x0;
                dist = (witness - p).LengthSquared();
            }
            else
            {
                Vector3 dt = d * t + a;
                dist = dt.Dot(dt);
            }
        }

        return dist;
    }

    /// <summary>btVec3PointTriDist2: squared distance from point P to triangle (x0,B,C)</summary>
    private static btScalar Vec3PointTriDist2(in Vector3 p, in Vector3 x0, in Vector3 b, in Vector3 c,
        ref Vector3 witness, bool computeWitness)
    {
        Vector3 d1 = b - x0;
        Vector3 d2 = c - x0;
        Vector3 a = x0 - p;

        double u = a.Dot(a);
        double v = d1.Dot(d1);
        double w = d2.Dot(d2);
        double pp = a.Dot(d1);
        double q = a.Dot(d2);
        double r = d1.Dot(d2);

        double s = (q * r - w * pp) / (w * v - r * r);
        double t = (-s * r - q) / w;

        btScalar dist;
        if ((FuzzyZero((btScalar)s) || s > 0) && (CcdEq((btScalar)s, 1) || s < 1) &&
            (FuzzyZero((btScalar)t) || t > 0) && (CcdEq((btScalar)t, 1) || t < 1) &&
            (CcdEq((btScalar)(t + s), 1) || t + s < 1))
        {
            if (computeWitness)
            {
                witness = x0 + d1 * (btScalar)s + d2 * (btScalar)t;
                dist = (witness - p).LengthSquared();
            }
            else
            {
                double distD = s * s * v;
                distD += t * t * w;
                distD += 2 * s * t * r;
                distD += 2 * s * pp;
                distD += 2 * t * q;
                distD += u;
                dist = (btScalar)distD;
            }
        }
        else
        {
            Vector3 witness2 = Vector3.Zero;
            dist = Vec3PointSegmentDist2(in p, in x0, in b, ref witness, computeWitness);

            btScalar dist2 = Vec3PointSegmentDist2(in p, in x0, in c, ref witness2, true);
            if (dist2 < dist)
            {
                dist = dist2;
                if (computeWitness)
                    witness = witness2;
            }

            dist2 = Vec3PointSegmentDist2(in p, in b, in c, ref witness2, true);
            if (dist2 < dist)
            {
                dist = dist2;
                if (computeWitness)
                    witness = witness2;
            }
        }

        return dist;
    }

    /// <summary>btDoSimplex2 (simplex is a segment)</summary>
    private static int DoSimplex2(CcdSimplex simplex, ref Vector3 dir)
    {
        SupportVector a = simplex.ps[simplex.last];
        SupportVector b = simplex.ps[0];
        Vector3 ab = b.v - a.v;
        Vector3 ao = -a.v;

        btScalar dot = ab.Dot(ao);

        // Check whether the origin lies on segment AB (zero cross product and positive projection means it does)
        Vector3 tmp = ab.Cross(ao);
        if (FuzzyZero(tmp.Dot(tmp)) && dot > 0)
        {
            return 1;
        }

        if (FuzzyZero(dot) || dot < 0)
        {
            // Origin is in the outer region of A
            simplex.ps[0] = a;
            simplex.last = 0;
            dir = ao;
        }
        else
        {
            // Origin is in the region of segment AB; keep the simplex and set direction to AB x AO x AB
            dir = ab.Cross(ao).Cross(ab);
        }

        return 0;
    }

    /// <summary>btDoSimplex3 (simplex is a triangle)</summary>
    private static int DoSimplex3(CcdSimplex simplex, ref Vector3 dir)
    {
        SupportVector a = simplex.ps[simplex.last];
        SupportVector b = simplex.ps[1];
        SupportVector c = simplex.ps[0];

        // Check for contact (origin at zero distance from the triangle)
        Vector3 dummy = Vector3.Zero;
        btScalar dist = Vec3PointTriDist2(Vector3.Zero, a.v, b.v, c.v, ref dummy, false);
        if (FuzzyZero(dist))
        {
            return 1;
        }

        // Is the triangle degenerate (zero area)?
        if (Vec3Eq(a.v, b.v) || Vec3Eq(a.v, c.v))
        {
            return -1;
        }

        Vector3 ao = -a.v;

        Vector3 ab = b.v - a.v;
        Vector3 ac = c.v - a.v;
        Vector3 abc = ab.Cross(ac);

        Vector3 tmp = abc.Cross(ac);
        btScalar dot = tmp.Dot(ao);
        if (FuzzyZero(dot) || dot > 0)
        {
            dot = ac.Dot(ao);
            if (FuzzyZero(dot) || dot > 0)
            {
                // C is already in place
                simplex.ps[1] = a;
                simplex.last = 1;
                dir = ac.Cross(ao).Cross(ac);
            }
            else
            {
                dot = ab.Dot(ao);
                if (FuzzyZero(dot) || dot > 0)
                {
                    simplex.ps[0] = b;
                    simplex.ps[1] = a;
                    simplex.last = 1;
                    dir = ab.Cross(ao).Cross(ab);
                }
                else
                {
                    simplex.ps[0] = a;
                    simplex.last = 0;
                    dir = ao;
                }
            }
        }
        else
        {
            tmp = ab.Cross(abc);
            dot = tmp.Dot(ao);
            if (FuzzyZero(dot) || dot > 0)
            {
                dot = ab.Dot(ao);
                if (FuzzyZero(dot) || dot > 0)
                {
                    simplex.ps[0] = b;
                    simplex.ps[1] = a;
                    simplex.last = 1;
                    dir = ab.Cross(ao).Cross(ab);
                }
                else
                {
                    simplex.ps[0] = a;
                    simplex.last = 0;
                    dir = ao;
                }
            }
            else
            {
                dot = abc.Dot(ao);
                if (FuzzyZero(dot) || dot > 0)
                {
                    dir = abc;
                }
                else
                {
                    SupportVector tmpSv = c;
                    simplex.ps[0] = b;
                    simplex.ps[1] = tmpSv;

                    dir = -abc;
                }
            }
        }

        return 0;
    }

    /// <summary>btDoSimplex4 (simplex is a tetrahedron)</summary>
    private static int DoSimplex4(CcdSimplex simplex, ref Vector3 dir)
    {
        SupportVector a = simplex.ps[simplex.last];
        SupportVector b = simplex.ps[2];
        SupportVector c = simplex.ps[1];
        SupportVector d = simplex.ps[0];

        Vector3 dummy = Vector3.Zero;

        // Is the tetrahedron degenerate (zero volume)?
        btScalar dist = Vec3PointTriDist2(a.v, b.v, c.v, d.v, ref dummy, false);
        if (FuzzyZero(dist))
        {
            return -1;
        }

        // If the origin lies on any face of the tetrahedron, the two objects intersect
        dist = Vec3PointTriDist2(Vector3.Zero, a.v, b.v, c.v, ref dummy, false);
        if (FuzzyZero(dist))
            return 1;
        dist = Vec3PointTriDist2(Vector3.Zero, a.v, c.v, d.v, ref dummy, false);
        if (FuzzyZero(dist))
            return 1;
        dist = Vec3PointTriDist2(Vector3.Zero, a.v, b.v, d.v, ref dummy, false);
        if (FuzzyZero(dist))
            return 1;
        dist = Vec3PointTriDist2(Vector3.Zero, b.v, c.v, d.v, ref dummy, false);
        if (FuzzyZero(dist))
            return 1;

        Vector3 ao = -a.v;
        Vector3 ab = b.v - a.v;
        Vector3 ac = c.v - a.v;
        Vector3 ad = d.v - a.v;
        Vector3 abc = ab.Cross(ac);
        Vector3 acd = ac.Cross(ad);
        Vector3 adb = ad.Cross(ab);

        int bOnAcd = CcdSign(acd.Dot(ab));
        int cOnAdb = CcdSign(adb.Dot(ac));
        int dOnAbc = CcdSign(abc.Dot(ad));

        bool abO = CcdSign(acd.Dot(ao)) == bOnAcd;
        bool acO = CcdSign(adb.Dot(ao)) == cOnAdb;
        bool adO = CcdSign(abc.Dot(ao)) == dOnAbc;

        if (abO && acO && adO)
        {
            // Origin is inside the tetrahedron
            return 1;
        }
        else if (!abO)
        {
            // B is farthest from the origin; remove B and fall back to a triangle
            simplex.ps[2] = a;
            simplex.last = 2;
        }
        else if (!acO)
        {
            // C is farthest
            simplex.ps[1] = d;
            simplex.ps[0] = b;
            simplex.ps[2] = a;
            simplex.last = 2;
        }
        else
        {
            // (!adO)
            simplex.ps[0] = c;
            simplex.ps[1] = b;
            simplex.ps[2] = a;
            simplex.last = 2;
        }

        return DoSimplex3(simplex, ref dir);
    }

    /// <summary>btDoSimplex</summary>
    private static int DoSimplex(CcdSimplex simplex, ref Vector3 dir)
    {
        if (simplex.Size == 2)
        {
            return DoSimplex2(simplex, ref dir);
        }
        else if (simplex.Size == 3)
        {
            return DoSimplex3(simplex, ref dir);
        }
        else
        {
            // Tetrahedron
            return DoSimplex4(simplex, ref dir);
        }
    }

    /// <summary>btGjkPairDetector::getClosestPointsNonVirtual</summary>
    public void GetClosestPointsNonVirtual(ClosestPointInput input, Result output, IDebugDraw debugDraw)
    {
        cachedSeparatingDistance = 0;

        btScalar distance = 0;
        Vector3 normalInB = Vector3.Zero;

        Vector3 pointOnA = Vector3.Zero, pointOnB = Vector3.Zero;
        Transform localTransA = input.transformA;
        Transform localTransB = input.transformB;
        Vector3 positionOffset = (localTransA.Origin + localTransB.Origin) * (btScalar)0.5;
        localTransA.Origin -= positionOffset;
        localTransB.Origin -= positionOffset;

        bool check2d = BroadphaseProxy.IsConvex2D(minkowskiA.ShapeType) && BroadphaseProxy.IsConvex2D(minkowskiB.ShapeType);

        btScalar marginA = this.marginA;
        btScalar marginB = this.marginB;

        // Margins are not used in CCD
        if (ignoreMargin)
        {
            marginA = 0;
            marginB = 0;
        }

        curIter = 0;
        int gGjkMaxIter = 1000; // catches invalid input (e.g. NaN)
        cachedSeparatingAxis = new Vector3(0, 1, 0);

        bool isValid = false;
        bool checkSimplex = false;
        bool checkPenetration = true;
        degenerateSimplex = 0;

        lastUsedMethod = -1;
        int status = -2;
        Vector3 orgNormalInB = Vector3.Zero;
        btScalar margin = marginA + marginB;

        // First check convex intersection with the standalone libccd-style implementation (see file header comment)
        {
            btScalar squaredDistance = BtMath.BtLargeFloat;
            btScalar delta = 0;

            var simplex = ccdSimplex;
            simplex.last = -1;

            Vector3 dir = new(1, 0, 0);

            {
                ComputeSupport(minkowskiA, in localTransA, minkowskiB, in localTransB, dir, check2d,
                    out Vector3 supAworld, out Vector3 supBworld, out Vector3 lastSupV);

                var last = new SupportVector { v = lastSupV, v1 = supAworld, v2 = supBworld };

                simplex.ps[++simplex.last] = last;

                dir = -lastSupV;

                // Start iterating
                for (int iterations = 0; iterations < gGjkMaxIter; iterations++)
                {
                    // Compute the support point
                    ComputeSupport(minkowskiA, in localTransA, minkowskiB, in localTransB, dir, check2d,
                        out supAworld, out supBworld, out lastSupV);

                    // If the farthest point of the Minkowski difference along dir is still "before" the origin (negative dot product), the two objects don't intersect
                    btScalar delta1 = lastSupV.Dot(dir);
                    if (delta1 < 0)
                    {
                        // No intersection apart from margins
                        status = -1;
                        break;
                    }

                    // Add the new support point to the simplex
                    last.v = lastSupV;
                    last.v1 = supAworld;
                    last.v2 = supBworld;

                    simplex.ps[++simplex.last] = last;

                    // DoSimplex returns 1 for intersection, -1 for no intersection, 0 to keep iterating
                    int doSimplexRes = DoSimplex(simplex, ref dir);

                    if (doSimplexRes == 1)
                    {
                        status = 0; // intersection found
                        break;
                    }
                    else if (doSimplexRes == -1)
                    {
                        status = -1;
                        break;
                    }

                    if (FuzzyZero(dir.Dot(dir)))
                    {
                        status = -1;
                    }

                    if (dir.LengthSquared() < BtMath.SimdEpsilon)
                    {
                        status = -1;
                        break;
                    }

                    if (dir.FuzzyZero())
                    {
                        status = -1;
                        break;
                    }
                }
            }

            simplexSolver.Reset();

            {
                for (;;)
                {
                    Vector3 separatingAxisInA = -cachedSeparatingAxis * localTransA.Basis;
                    Vector3 separatingAxisInB = cachedSeparatingAxis * localTransB.Basis;

                    Vector3 pInA = minkowskiA.LocalGetSupportingVertexWithoutMargin(separatingAxisInA);
                    Vector3 qInB = minkowskiB.LocalGetSupportingVertexWithoutMargin(separatingAxisInB);

                    Vector3 pWorld = localTransA * pInA;
                    Vector3 qWorld = localTransB * qInB;

                    if (check2d)
                    {
                        pWorld.Z = 0;
                        qWorld.Z = 0;
                    }

                    Vector3 w = pWorld - qWorld;
                    delta = cachedSeparatingAxis.Dot(w);

                    // Potential exit condition: the two don't overlap
                    if (delta > 0 && delta * delta > squaredDistance * input.maximumDistanceSquared)
                    {
                        degenerateSimplex = 10;
                        checkSimplex = true;
                        break;
                    }

                    // Exit 0: the new point is already in the simplex, or we're not getting closer
                    if (simplexSolver.InSimplex(w))
                    {
                        degenerateSimplex = 1;
                        checkSimplex = true;
                        break;
                    }
                    // Are we getting closer?
                    btScalar f0 = squaredDistance - delta;
                    btScalar f1 = squaredDistance * RelError2;

                    if (f0 <= f1)
                    {
                        if (f0 <= 0)
                        {
                            degenerateSimplex = 2;
                        }
                        else
                        {
                            degenerateSimplex = 11;
                        }
                        checkSimplex = true;
                        break;
                    }

                    // Add the current vertex to the simplex
                    simplexSolver.AddVertex(w, pWorld, qWorld);
                    Vector3 newCachedSeparatingAxis = Vector3.Zero;

                    // Compute the point closest to the origin (update vector v)
                    if (!simplexSolver.ClosestPoint(ref newCachedSeparatingAxis))
                    {
                        degenerateSimplex = 3;
                        checkSimplex = true;
                        break;
                    }

                    if (newCachedSeparatingAxis.LengthSquared() < RelError2)
                    {
                        cachedSeparatingAxis = newCachedSeparatingAxis;
                        degenerateSimplex = 6;
                        checkSimplex = true;
                        break;
                    }

                    btScalar previousSquaredDistance = squaredDistance;
                    squaredDistance = newCachedSeparatingAxis.LengthSquared();

                    // Are we getting closer?
                    if (previousSquaredDistance - squaredDistance <= BtMath.SimdEpsilon * previousSquaredDistance)
                    {
                        checkSimplex = true;
                        degenerateSimplex = 12;
                        break;
                    }

                    cachedSeparatingAxis = newCachedSeparatingAxis;

                    // Degenerate case, usually caused by an invalid/uninitialized world transform on a btCollisionObject
                    if (curIter++ > gGjkMaxIter)
                    {
                        break;
                    }

                    bool check = !simplexSolver.FullSimplex;

                    if (!check)
                    {
                        degenerateSimplex = 13;
                        break;
                    }
                }

                if (checkSimplex)
                {
                    simplexSolver.ComputePoints(out pointOnA, out pointOnB);
                    normalInB = cachedSeparatingAxis;

                    btScalar lenSqr = cachedSeparatingAxis.LengthSquared();

                    // Valid normal
                    if (lenSqr < RelError2)
                    {
                        degenerateSimplex = 5;
                    }
                    if (lenSqr > BtMath.SimdEpsilon * BtMath.SimdEpsilon)
                    {
                        btScalar rlen = 1 / BtMath.Sqrt(lenSqr);
                        normalInB *= rlen; // normalize

                        btScalar s = BtMath.Sqrt(squaredDistance);

                        pointOnA -= cachedSeparatingAxis * (marginA / s);
                        pointOnB += cachedSeparatingAxis * (marginB / s);
                        distance = 1 / rlen - margin;
                        isValid = true;
                        orgNormalInB = normalInB;

                        lastUsedMethod = 1;
                    }
                    else
                    {
                        lastUsedMethod = 2;
                    }
                }
            }

            bool catchDegeneratePenetrationCase =
                catchDegeneracies != 0 && penetrationDepthSolver != null && degenerateSimplex != 0 &&
                distance + margin < GjkEpaPenetrationTolerance;

            if ((checkPenetration && (!isValid || catchDegeneratePenetrationCase)) || status == 0)
            {
                // Penetration case

                // Give up if we can't handle penetration
                if (penetrationDepthSolver is IConvexPenetrationDepthSolver penSolver)
                {
                    cachedSeparatingAxis = Vector3.Zero;

                    bool isValid2 = penSolver.CalcPenDepth(
                        simplexSolver,
                        minkowskiA, minkowskiB,
                        in localTransA, in localTransB,
                        ref cachedSeparatingAxis, out Vector3 tmpPointOnA, out Vector3 tmpPointOnB);

                    if (cachedSeparatingAxis.LengthSquared() != 0)
                    {
                        if (isValid2)
                        {
                            Vector3 tmpNormalInB = tmpPointOnB - tmpPointOnA;
                            btScalar lenSqr = tmpNormalInB.LengthSquared();
                            if (lenSqr <= BtMath.SimdEpsilon * BtMath.SimdEpsilon)
                            {
                                tmpNormalInB = cachedSeparatingAxis;
                                lenSqr = cachedSeparatingAxis.LengthSquared();
                            }

                            if (lenSqr > BtMath.SimdEpsilon * BtMath.SimdEpsilon)
                            {
                                tmpNormalInB /= BtMath.Sqrt(lenSqr);
                                btScalar distance2 = -(tmpPointOnA - tmpPointOnB).Length();
                                lastUsedMethod = 3;
                                // Only replace a valid penetration with a deeper result
                                if (!isValid || distance2 < distance)
                                {
                                    distance = distance2;
                                    pointOnA = tmpPointOnA;
                                    pointOnB = tmpPointOnB;
                                    normalInB = tmpNormalInB;
                                    isValid = true;
                                }
                                else
                                {
                                    lastUsedMethod = 8;
                                }
                            }
                            else
                            {
                                lastUsedMethod = 9;
                            }
                        }
                        else
                        {
                            // Another degenerate case: the initial GJK reports degeneracy, EPA reports
                            // no penetration, and the second GJK (support vectors without margin) reports
                            // a valid positive distance. Use the second GJK result instead of failing.
                            // Thanks to Jacob.Langford for the repro case (issue #250)
                            if (cachedSeparatingAxis.LengthSquared() > 0)
                            {
                                btScalar distance2 = (tmpPointOnA - tmpPointOnB).Length() - margin;
                                // Only replace a valid distance with a smaller one
                                if (!isValid || distance2 < distance)
                                {
                                    distance = distance2;
                                    pointOnA = tmpPointOnA;
                                    pointOnB = tmpPointOnB;
                                    pointOnA -= cachedSeparatingAxis * marginA;
                                    pointOnB += cachedSeparatingAxis * marginB;
                                    normalInB = cachedSeparatingAxis;
                                    normalInB.Normalize();

                                    isValid = true;
                                    lastUsedMethod = 6;
                                }
                                else
                                {
                                    lastUsedMethod = 5;
                                }
                            }
                        }
                    }
                }
            }
        }

        if (isValid && (distance < 0 || distance * distance < input.maximumDistanceSquared))
        {
            cachedSeparatingAxis = normalInB;
            cachedSeparatingDistance = distance;
            {
                // todo: track down a degeneracy in the EPA penetration solver:
                // the solver reports penetration but the normal connecting the contact points
                // points the opposite way; detect this and flip the normal for now
                btScalar d2 = 0;
                {
                    Vector3 separatingAxisInA = -orgNormalInB * localTransA.Basis;
                    Vector3 separatingAxisInB = orgNormalInB * localTransB.Basis;

                    Vector3 pInA = minkowskiA.LocalGetSupportingVertexWithoutMargin(separatingAxisInA);
                    Vector3 qInB = minkowskiB.LocalGetSupportingVertexWithoutMargin(separatingAxisInB);

                    Vector3 pWorld = localTransA * pInA;
                    Vector3 qWorld = localTransB * qInB;
                    Vector3 w = pWorld - qWorld;
                    d2 = orgNormalInB.Dot(w) - margin;
                }

                btScalar d1 = 0;
                {
                    Vector3 separatingAxisInA = normalInB * localTransA.Basis;
                    Vector3 separatingAxisInB = -normalInB * localTransB.Basis;

                    Vector3 pInA = minkowskiA.LocalGetSupportingVertexWithoutMargin(separatingAxisInA);
                    Vector3 qInB = minkowskiB.LocalGetSupportingVertexWithoutMargin(separatingAxisInB);

                    Vector3 pWorld = localTransA * pInA;
                    Vector3 qWorld = localTransB * qInB;
                    Vector3 w = pWorld - qWorld;
                    d1 = -normalInB.Dot(w) - margin;
                }
                btScalar d0 = 0;
                {
                    Vector3 separatingAxisInA = -normalInB * input.transformA.Basis;
                    Vector3 separatingAxisInB = normalInB * input.transformB.Basis;

                    Vector3 pInA = minkowskiA.LocalGetSupportingVertexWithoutMargin(separatingAxisInA);
                    Vector3 qInB = minkowskiB.LocalGetSupportingVertexWithoutMargin(separatingAxisInB);

                    Vector3 pWorld = localTransA * pInA;
                    Vector3 qWorld = localTransB * qInB;
                    Vector3 w = pWorld - qWorld;
                    d0 = normalInB.Dot(w) - margin;
                }

                if (d1 > d0)
                {
                    lastUsedMethod = 10;
                    normalInB *= -1;
                }

                if (orgNormalInB.LengthSquared() != 0)
                {
                    if (d2 > d0 && d2 > d1 && d2 > distance)
                    {
                        normalInB = orgNormalInB;
                        distance = d2;
                    }
                }
            }

            output.AddContactPoint(
                normalInB,
                pointOnB + positionOffset,
                distance);
        }
    }
}
