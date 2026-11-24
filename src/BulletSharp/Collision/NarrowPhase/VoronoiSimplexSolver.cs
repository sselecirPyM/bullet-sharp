using BulletSharp.Math;

namespace BulletSharp;

/// <summary>btUsageBitfield (value type: the 4 flags are inlined, no heap indirection)</summary>
public struct UsageBitfield
{
    internal bool usedVertexA;
    internal bool usedVertexB;
    internal bool usedVertexC;
    internal bool usedVertexD;

    public UsageBitfield()
    {
    }

    internal void Reset()
    {
        usedVertexA = false;
        usedVertexB = false;
        usedVertexC = false;
        usedVertexD = false;
    }
}

/// <summary>btSubSimplexClosestResult (value type: barycentric coordinates are inlined scalar fields, equivalent to a C++ stack object)</summary>
public struct SubSimplexClosestResult
{
    internal Vector3 closestPointOnSimplex;
    internal UsageBitfield usedVertices;
    internal btScalar bc0;
    internal btScalar bc1;
    internal btScalar bc2;
    internal btScalar bc3;
    internal bool degenerate;

    internal void Reset()
    {
        degenerate = false;
        SetBarycentricCoordinates();
        usedVertices.Reset();
    }

    internal readonly bool IsValid() =>
        bc0 >= 0 &&
        bc1 >= 0 &&
        bc2 >= 0 &&
        bc3 >= 0;

    internal void SetBarycentricCoordinates(btScalar a = 0, btScalar b = 0, btScalar c = 0, btScalar d = 0)
    {
        bc0 = a;
        bc1 = b;
        bc2 = c;
        bc3 = d;
    }
}

/// <summary>
/// Ported from btVoronoiSimplexSolver: computes the closest point to the origin of a simplex
/// of 1 to 4 vertices; an alternative to Johnson's distance algorithm for GJK.
/// </summary>
public class VoronoiSimplexSolver
{
    internal const int VoronoiSimplexMaxVerts = 5;

    internal int numVertices;

    internal readonly Vector3[] simplexVectorW = new Vector3[VoronoiSimplexMaxVerts];
    internal readonly Vector3[] simplexPointsP = new Vector3[VoronoiSimplexMaxVerts];
    internal readonly Vector3[] simplexPointsQ = new Vector3[VoronoiSimplexMaxVerts];

    internal Vector3 cachedP1;
    internal Vector3 cachedP2;
    internal Vector3 cachedV;
    internal Vector3 lastW;

    internal btScalar equalVertexThreshold;
    internal bool cachedValidClosest;

    internal SubSimplexClosestResult cachedBC;

    internal bool needsUpdate;

#if BT_USE_DOUBLE_PRECISION
    internal const btScalar VoronoiDefaultEqualVertexThreshold = (btScalar)1e-12;
#else
    internal const btScalar VoronoiDefaultEqualVertexThreshold = (btScalar)0.0001;
#endif

    public VoronoiSimplexSolver()
    {
        equalVertexThreshold = VoronoiDefaultEqualVertexThreshold;
        Reset();
    }

    public int NumVertices => numVertices;

    public btScalar EqualVertexThreshold
    {
        get => equalVertexThreshold;
        set => equalVertexThreshold = value;
    }

    /// <summary>btVoronoiSimplexSolver::removeVertex</summary>
    internal void RemoveVertex(int index)
    {
        numVertices--;
        simplexVectorW[index] = simplexVectorW[numVertices];
        simplexPointsP[index] = simplexPointsP[numVertices];
        simplexPointsQ[index] = simplexPointsQ[numVertices];
    }

    /// <summary>btVoronoiSimplexSolver::reduceVertices</summary>
    internal void ReduceVertices(in UsageBitfield usedVerts)
    {
        if (NumVertices >= 4 && !usedVerts.usedVertexD)
            RemoveVertex(3);
        if (NumVertices >= 3 && !usedVerts.usedVertexC)
            RemoveVertex(2);
        if (NumVertices >= 2 && !usedVerts.usedVertexB)
            RemoveVertex(1);
        if (NumVertices >= 1 && !usedVerts.usedVertexA)
            RemoveVertex(0);
    }

    /// <summary>Clears the simplex (btVoronoiSimplexSolver::reset)</summary>
    public void Reset()
    {
        cachedValidClosest = false;
        numVertices = 0;
        needsUpdate = true;
        lastW = new Vector3(BtMath.BtLargeFloat, BtMath.BtLargeFloat, BtMath.BtLargeFloat);
        cachedBC.Reset();
    }

    /// <summary>Adds a vertex (btVoronoiSimplexSolver::addVertex)</summary>
    public void AddVertex(Vector3 w, Vector3 p, Vector3 q)
    {
        lastW = w;
        needsUpdate = true;

        simplexVectorW[numVertices] = w;
        simplexPointsP[numVertices] = p;
        simplexPointsQ[numVertices] = q;

        numVertices++;
    }

    /// <summary>btVoronoiSimplexSolver::updateClosestVectorAndPoints</summary>
    internal bool UpdateClosestVectorAndPoints()
    {
        if (needsUpdate)
        {
            cachedBC.Reset();
            needsUpdate = false;

            switch (NumVertices)
            {
                case 0:
                    cachedValidClosest = false;
                    break;
                case 1:
                {
                    cachedP1 = simplexPointsP[0];
                    cachedP2 = simplexPointsQ[0];
                    cachedV = cachedP1 - cachedP2; // == simplexVectorW[0]
                    cachedBC.Reset();
                    cachedBC.SetBarycentricCoordinates(1, 0, 0, 0);
                    cachedValidClosest = cachedBC.IsValid();
                    break;
                }
                case 2:
                {
                    // Closest point from the origin to a segment
                    Vector3 from = simplexVectorW[0];
                    Vector3 to = simplexVectorW[1];

                    Vector3 p = Vector3.Zero;
                    Vector3 diff = p - from;
                    Vector3 v = to - from;
                    btScalar t = v.Dot(diff);

                    if (t > 0)
                    {
                        btScalar dotVV = v.Dot(v);
                        if (t < dotVV)
                        {
                            t /= dotVV;
                            diff -= t * v;
                            cachedBC.usedVertices.usedVertexA = true;
                            cachedBC.usedVertices.usedVertexB = true;
                        }
                        else
                        {
                            t = 1;
                            diff -= v;
                            // Degenerates to a single point
                            cachedBC.usedVertices.usedVertexB = true;
                        }
                    }
                    else
                    {
                        t = 0;
                        // Degenerates to a single point
                        cachedBC.usedVertices.usedVertexA = true;
                    }
                    cachedBC.SetBarycentricCoordinates(1 - t, t);
                    Vector3 nearest = from + t * v;

                    cachedP1 = simplexPointsP[0] + t * (simplexPointsP[1] - simplexPointsP[0]);
                    cachedP2 = simplexPointsQ[0] + t * (simplexPointsQ[1] - simplexPointsQ[0]);
                    cachedV = cachedP1 - cachedP2;

                    ReduceVertices(cachedBC.usedVertices);

                    cachedValidClosest = cachedBC.IsValid();
                    break;
                }
                case 3:
                {
                    // Closest point from the origin to a triangle
                    Vector3 p = Vector3.Zero;
                    Vector3 a = simplexVectorW[0];
                    Vector3 b = simplexVectorW[1];
                    Vector3 c = simplexVectorW[2];

                    ClosestPtPointTriangle(p, a, b, c, ref cachedBC);
                    cachedP1 = simplexPointsP[0] * cachedBC.bc0 +
                               simplexPointsP[1] * cachedBC.bc1 +
                               simplexPointsP[2] * cachedBC.bc2;
                    cachedP2 = simplexPointsQ[0] * cachedBC.bc0 +
                               simplexPointsQ[1] * cachedBC.bc1 +
                               simplexPointsQ[2] * cachedBC.bc2;
                    cachedV = cachedP1 - cachedP2;

                    ReduceVertices(cachedBC.usedVertices);

                    cachedValidClosest = cachedBC.IsValid();
                    break;
                }
                case 4:
                {
                    // Closest point from the origin to a tetrahedron
                    Vector3 p = Vector3.Zero;
                    Vector3 a = simplexVectorW[0];
                    Vector3 b = simplexVectorW[1];
                    Vector3 c = simplexVectorW[2];
                    Vector3 d = simplexVectorW[3];

                    bool hasSeparation = ClosestPtPointTetrahedron(p, a, b, c, d, ref cachedBC);

                    if (hasSeparation)
                    {
                        cachedP1 = simplexPointsP[0] * cachedBC.bc0 +
                                   simplexPointsP[1] * cachedBC.bc1 +
                                   simplexPointsP[2] * cachedBC.bc2 +
                                   simplexPointsP[3] * cachedBC.bc3;
                        cachedP2 = simplexPointsQ[0] * cachedBC.bc0 +
                                   simplexPointsQ[1] * cachedBC.bc1 +
                                   simplexPointsQ[2] * cachedBC.bc2 +
                                   simplexPointsQ[3] * cachedBC.bc3;
                        cachedV = cachedP1 - cachedP2;
                        ReduceVertices(cachedBC.usedVertices);
                    }
                    else
                    {
                        if (cachedBC.degenerate)
                        {
                            cachedValidClosest = false;
                        }
                        else
                        {
                            cachedValidClosest = true;
                            // Penetration: cachedV is the zero vector
                            cachedV = Vector3.Zero;
                        }
                        break;
                    }

                    cachedValidClosest = cachedBC.IsValid();
                    break;
                }
                default:
                    cachedValidClosest = false;
                    break;
            }
        }

        return cachedValidClosest;
    }

    /// <summary>Returns/computes the closest vector (btVoronoiSimplexSolver::closest)</summary>
    public bool ClosestPoint(ref Vector3 v)
    {
        bool success = UpdateClosestVectorAndPoints();
        v = cachedV;
        return success;
    }

    /// <summary>btVoronoiSimplexSolver::maxVertex</summary>
    public btScalar MaxVertex()
    {
        btScalar maxV = 0;
        for (int i = 0; i < NumVertices; i++)
        {
            btScalar curLen2 = simplexVectorW[i].LengthSquared();
            if (maxV < curLen2)
                maxV = curLen2;
        }
        return maxV;
    }

    public bool FullSimplex => numVertices == 4;

    /// <summary>Returns the current simplex (btVoronoiSimplexSolver::getSimplex)</summary>
    public int GetSimplex(Vector3[] pBuf, Vector3[] qBuf, Vector3[] yBuf)
    {
        for (int i = 0; i < NumVertices; i++)
        {
            yBuf[i] = simplexVectorW[i];
            pBuf[i] = simplexPointsP[i];
            qBuf[i] = simplexPointsQ[i];
        }
        return NumVertices;
    }

    /// <summary>btVoronoiSimplexSolver::inSimplex</summary>
    public bool InSimplex(Vector3 w)
    {
        bool found = false;
        // Is w already in the current (reduced) simplex?
        for (int i = 0; i < NumVertices; i++)
        {
            if (simplexVectorW[i].DistanceSquared(w) <= equalVertexThreshold)
            {
                found = true;
                break;
            }
        }

        // lastW may already have been removed
        if (w == lastW)
            return true;

        return found;
    }

    /// <summary>btVoronoiSimplexSolver::backup_closest</summary>
    public void BackupClosest(ref Vector3 v)
    {
        v = cachedV;
    }

    public bool EmptySimplex => NumVertices == 0;

    /// <summary>btVoronoiSimplexSolver::compute_points</summary>
    public void ComputePoints(out Vector3 p1, out Vector3 p2)
    {
        UpdateClosestVectorAndPoints();
        p1 = cachedP1;
        p2 = cachedP2;
    }

    /// <summary>btVoronoiSimplexSolver::closestPtPointTriangle</summary>
    internal static bool ClosestPtPointTriangle(in Vector3 p, in Vector3 a, in Vector3 b, in Vector3 c, ref SubSimplexClosestResult result)
    {
        result.usedVertices.Reset();

        // Is P outside the vertex region of A?
        Vector3 ab = b - a;
        Vector3 ac = c - a;
        Vector3 ap = p - a;
        btScalar d1 = ab.Dot(ap);
        btScalar d2 = ac.Dot(ap);
        if (d1 <= 0 && d2 <= 0)
        {
            result.closestPointOnSimplex = a;
            result.usedVertices.usedVertexA = true;
            result.SetBarycentricCoordinates(1, 0, 0);
            return true;
        }

        // Is P outside the vertex region of B?
        Vector3 bp = p - b;
        btScalar d3 = ab.Dot(bp);
        btScalar d4 = ac.Dot(bp);
        if (d3 >= 0 && d4 <= d3)
        {
            result.closestPointOnSimplex = b;
            result.usedVertices.usedVertexB = true;
            result.SetBarycentricCoordinates(0, 1, 0);
            return true;
        }

        // Is P in the edge region of AB?
        btScalar vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0)
        {
            btScalar v = d1 / (d1 - d3);
            result.closestPointOnSimplex = a + v * ab;
            result.usedVertices.usedVertexA = true;
            result.usedVertices.usedVertexB = true;
            result.SetBarycentricCoordinates(1 - v, v, 0);
            return true;
        }

        // Is P outside the vertex region of C?
        Vector3 cp = p - c;
        btScalar d5 = ab.Dot(cp);
        btScalar d6 = ac.Dot(cp);
        if (d6 >= 0 && d5 <= d6)
        {
            result.closestPointOnSimplex = c;
            result.usedVertices.usedVertexC = true;
            result.SetBarycentricCoordinates(0, 0, 1);
            return true;
        }

        // Is P in the edge region of AC?
        btScalar vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0)
        {
            btScalar w = d2 / (d2 - d6);
            result.closestPointOnSimplex = a + w * ac;
            result.usedVertices.usedVertexA = true;
            result.usedVertices.usedVertexC = true;
            result.SetBarycentricCoordinates(1 - w, 0, w);
            return true;
        }

        // Is P in the edge region of BC?
        btScalar va = d3 * d6 - d5 * d4;
        if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
        {
            btScalar w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            result.closestPointOnSimplex = b + w * (c - b);
            result.usedVertices.usedVertexB = true;
            result.usedVertices.usedVertexC = true;
            result.SetBarycentricCoordinates(0, 1 - w, w);
            return true;
        }

        // P is inside the face region; compute using barycentric coordinates
        btScalar denom = 1 / (va + vb + vc);
        btScalar v2 = vb * denom;
        btScalar w2 = vc * denom;

        result.closestPointOnSimplex = a + ab * v2 + ac * w2;
        result.usedVertices.usedVertexA = true;
        result.usedVertices.usedVertexB = true;
        result.usedVertices.usedVertexC = true;
        result.SetBarycentricCoordinates(1 - v2 - w2, v2, w2);

        return true;
    }

    /// <summary>btVoronoiSimplexSolver::pointOutsideOfPlane: are p and d on opposite sides of plane abc? (returns -1 when degenerate)</summary>
    internal static int PointOutsideOfPlane(in Vector3 p, in Vector3 a, in Vector3 b, in Vector3 c, in Vector3 d)
    {
        Vector3 normal = (b - a).Cross(c - a);

        btScalar signp = (p - a).Dot(normal);
        btScalar signd = (d - a).Dot(normal);

        // CATCH_DEGENERATE_TETRAHEDRON
#if BT_USE_DOUBLE_PRECISION
        if (signd * signd < (btScalar)1e-8 * (btScalar)1e-8)
        {
            return -1;
        }
#else
        if (signd * signd < (btScalar)1e-4 * (btScalar)1e-4)
        {
            return -1;
        }
#endif

        return signp * signd < 0 ? 1 : 0;
    }

    /// <summary>btVoronoiSimplexSolver::closestPtPointTetrahedron</summary>
    internal static bool ClosestPtPointTetrahedron(in Vector3 p, in Vector3 a, in Vector3 b, in Vector3 c, in Vector3 d, ref SubSimplexClosestResult finalResult)
    {
        // A stack object in C++ (value semantics); a local struct here means zero allocation
        SubSimplexClosestResult tempResult = default;
        tempResult.Reset();

        // Start by assuming the point is inside all half-spaces, so the closest point is the point itself
        finalResult.closestPointOnSimplex = p;
        finalResult.usedVertices.Reset();
        finalResult.usedVertices.usedVertexA = true;
        finalResult.usedVertices.usedVertexB = true;
        finalResult.usedVertices.usedVertexC = true;
        finalResult.usedVertices.usedVertexD = true;

        int pointOutsideABC = PointOutsideOfPlane(p, a, b, c, d);
        int pointOutsideACD = PointOutsideOfPlane(p, a, c, d, b);
        int pointOutsideADB = PointOutsideOfPlane(p, a, d, b, c);
        int pointOutsideBDC = PointOutsideOfPlane(p, b, d, c, a);

        if (pointOutsideABC < 0 || pointOutsideACD < 0 || pointOutsideADB < 0 || pointOutsideBDC < 0)
        {
            finalResult.degenerate = true;
            return false;
        }

        if (pointOutsideABC == 0 && pointOutsideACD == 0 && pointOutsideADB == 0 && pointOutsideBDC == 0)
        {
            return false;
        }

        btScalar bestSqDist = BtMath.SimdInfinity;
        // The point is outside face abc → compute the closest point on abc
        if (pointOutsideABC != 0)
        {
            ClosestPtPointTriangle(p, a, b, c, ref tempResult);
            Vector3 q = tempResult.closestPointOnSimplex;
            btScalar sqDist = (q - p).Dot(q - p);
            if (sqDist < bestSqDist)
            {
                bestSqDist = sqDist;
                finalResult.closestPointOnSimplex = q;
                finalResult.usedVertices.Reset();
                finalResult.usedVertices.usedVertexA = tempResult.usedVertices.usedVertexA;
                finalResult.usedVertices.usedVertexB = tempResult.usedVertices.usedVertexB;
                finalResult.usedVertices.usedVertexC = tempResult.usedVertices.usedVertexC;
                finalResult.SetBarycentricCoordinates(
                    tempResult.bc0,
                    tempResult.bc1,
                    tempResult.bc2,
                    0);
            }
        }

        // Face acd
        if (pointOutsideACD != 0)
        {
            ClosestPtPointTriangle(p, a, c, d, ref tempResult);
            Vector3 q = tempResult.closestPointOnSimplex;
            btScalar sqDist = (q - p).Dot(q - p);
            if (sqDist < bestSqDist)
            {
                bestSqDist = sqDist;
                finalResult.closestPointOnSimplex = q;
                finalResult.usedVertices.Reset();
                finalResult.usedVertices.usedVertexA = tempResult.usedVertices.usedVertexA;
                finalResult.usedVertices.usedVertexC = tempResult.usedVertices.usedVertexB;
                finalResult.usedVertices.usedVertexD = tempResult.usedVertices.usedVertexC;
                finalResult.SetBarycentricCoordinates(
                    tempResult.bc0,
                    0,
                    tempResult.bc1,
                    tempResult.bc2);
            }
        }

        // Face adb
        if (pointOutsideADB != 0)
        {
            ClosestPtPointTriangle(p, a, d, b, ref tempResult);
            Vector3 q = tempResult.closestPointOnSimplex;
            btScalar sqDist = (q - p).Dot(q - p);
            if (sqDist < bestSqDist)
            {
                bestSqDist = sqDist;
                finalResult.closestPointOnSimplex = q;
                finalResult.usedVertices.Reset();
                finalResult.usedVertices.usedVertexA = tempResult.usedVertices.usedVertexA;
                finalResult.usedVertices.usedVertexB = tempResult.usedVertices.usedVertexC;
                finalResult.usedVertices.usedVertexD = tempResult.usedVertices.usedVertexB;
                finalResult.SetBarycentricCoordinates(
                    tempResult.bc0,
                    tempResult.bc2,
                    0,
                    tempResult.bc1);
            }
        }

        // Face bdc
        if (pointOutsideBDC != 0)
        {
            ClosestPtPointTriangle(p, b, d, c, ref tempResult);
            Vector3 q = tempResult.closestPointOnSimplex;
            btScalar sqDist = (q - p).Dot(q - p);
            if (sqDist < bestSqDist)
            {
                bestSqDist = sqDist;
                finalResult.closestPointOnSimplex = q;
                finalResult.usedVertices.Reset();
                finalResult.usedVertices.usedVertexB = tempResult.usedVertices.usedVertexA;
                finalResult.usedVertices.usedVertexC = tempResult.usedVertices.usedVertexC;
                finalResult.usedVertices.usedVertexD = tempResult.usedVertices.usedVertexB;
                finalResult.SetBarycentricCoordinates(
                    0,
                    tempResult.bc0,
                    tempResult.bc2,
                    tempResult.bc1);
            }
        }

        return true;
    }
}
