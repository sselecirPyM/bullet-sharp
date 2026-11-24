using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btPersistentManifold (4-point contact cache).
/// </summary>
public class PersistentManifold
{
    public const int ManifoldCacheSize = 4;

    /// <summary>Global default for gContactBreakingThreshold</summary>
    public static btScalar GlobalContactBreakingThreshold = BtMath.ContactBreakingThreshold;

    /// <summary>gContactCalcArea3Points</summary>
    public static bool GlobalContactCalcArea3Points = true;

    internal readonly ManifoldPoint[] pointCache = new ManifoldPoint[ManifoldCacheSize]
    {
        new ManifoldPoint(), new ManifoldPoint(), new ManifoldPoint(), new ManifoldPoint(),
    };

    internal CollisionObject body0;
    internal CollisionObject body1;
    internal int cachedPoints;
    internal btScalar contactBreakingThreshold;
    internal btScalar contactProcessingThreshold;
    internal int companionIdA;
    internal int companionIdB;
    internal int index1a;

    public PersistentManifold()
    {
    }

    public PersistentManifold(CollisionObject body0, CollisionObject body1, int unused, btScalar contactBreakingThreshold, btScalar contactProcessingThreshold)
    {
        this.body0 = body0;
        this.body1 = body1;
        this.contactBreakingThreshold = contactBreakingThreshold;
        this.contactProcessingThreshold = contactProcessingThreshold;
    }

    public CollisionObject Body0 => body0;
    public CollisionObject Body1 => body1;

    internal void SetBodies(CollisionObject body0, CollisionObject body1)
    {
        this.body0 = body0;
        this.body1 = body1;
    }

    internal void ClearUserCache(ManifoldPoint pt)
    {
        pt.userPersistentData = null;
    }

    public int NumContacts => cachedPoints;

    internal void SetNumContacts(int n) => cachedPoints = n;

    public ManifoldPoint GetContactPoint(int index) => pointCache[index];

    public btScalar ContactBreakingThreshold
    {
        get => contactBreakingThreshold;
        set => contactBreakingThreshold = value;
    }

    public btScalar ContactProcessingThreshold
    {
        get => contactProcessingThreshold;
        set => contactProcessingThreshold = value;
    }

    public int CompanionIdA
    {
        get => companionIdA;
        set => companionIdA = value;
    }

    public int CompanionIdB
    {
        get => companionIdB;
        set => companionIdB = value;
    }

    private static btScalar CalcArea4Points(in Vector3 p0, in Vector3 p1, in Vector3 p2, in Vector3 p3)
    {
        Vector3 a0 = p0 - p1, a1 = p0 - p2, a2 = p0 - p3;
        Vector3 b0 = p2 - p3, b1 = p1 - p3, b2 = p1 - p2;
        Vector3 tmp0 = a0.Cross(b0);
        Vector3 tmp1 = a1.Cross(b1);
        Vector3 tmp2 = a2.Cross(b2);
        return BtMath.Max(BtMath.Max(tmp0.LengthSquared(), tmp1.LengthSquared()), tmp2.LengthSquared());
    }

    internal int SortCachedPoints(ManifoldPoint pt)
    {
        // Compute the area for all 4 cases, keeping the deepest point
        int maxPenetrationIndex = -1;
        btScalar maxPenetration = pt.distance1;
        for (int i = 0; i < 4; i++)
        {
            if (pointCache[i].distance1 < maxPenetration)
            {
                maxPenetrationIndex = i;
                maxPenetration = pointCache[i].distance1;
            }
        }

        btScalar res0 = 0, res1 = 0, res2 = 0, res3 = 0;

        if (GlobalContactCalcArea3Points)
        {
            if (maxPenetrationIndex != 0)
            {
                Vector3 a0 = pt.localPointA - pointCache[1].localPointA;
                Vector3 b0 = pointCache[3].localPointA - pointCache[2].localPointA;
                res0 = a0.Cross(b0).LengthSquared();
            }
            if (maxPenetrationIndex != 1)
            {
                Vector3 a1 = pt.localPointA - pointCache[0].localPointA;
                Vector3 b1 = pointCache[3].localPointA - pointCache[2].localPointA;
                res1 = a1.Cross(b1).LengthSquared();
            }
            if (maxPenetrationIndex != 2)
            {
                Vector3 a2 = pt.localPointA - pointCache[0].localPointA;
                Vector3 b2 = pointCache[3].localPointA - pointCache[1].localPointA;
                res2 = a2.Cross(b2).LengthSquared();
            }
            if (maxPenetrationIndex != 3)
            {
                Vector3 a3 = pt.localPointA - pointCache[0].localPointA;
                Vector3 b3 = pointCache[2].localPointA - pointCache[1].localPointA;
                res3 = a3.Cross(b3).LengthSquared();
            }
        }
        else
        {
            if (maxPenetrationIndex != 0)
                res0 = CalcArea4Points(pt.localPointA, pointCache[1].localPointA, pointCache[2].localPointA, pointCache[3].localPointA);
            if (maxPenetrationIndex != 1)
                res1 = CalcArea4Points(pt.localPointA, pointCache[0].localPointA, pointCache[2].localPointA, pointCache[3].localPointA);
            if (maxPenetrationIndex != 2)
                res2 = CalcArea4Points(pt.localPointA, pointCache[0].localPointA, pointCache[1].localPointA, pointCache[3].localPointA);
            if (maxPenetrationIndex != 3)
                res3 = CalcArea4Points(pt.localPointA, pointCache[0].localPointA, pointCache[1].localPointA, pointCache[2].localPointA);
        }

        // btVector4(res0..res3).closestAxis4() = index of the component with the largest absolute value
        btScalar a0Abs = BtMath.Abs(res0), a1Abs = BtMath.Abs(res1);
        btScalar a2Abs = BtMath.Abs(res2), a3Abs = BtMath.Abs(res3);
        int maxIndex = 0;
        btScalar maxVal = a0Abs;
        if (a1Abs > maxVal) { maxVal = a1Abs; maxIndex = 1; }
        if (a2Abs > maxVal) { maxVal = a2Abs; maxIndex = 2; }
        if (a3Abs > maxVal) { maxIndex = 3; }
        return maxIndex;
    }

    internal int GetCacheEntry(ManifoldPoint newPoint)
    {
        btScalar shortestDist = ContactBreakingThreshold * ContactBreakingThreshold;
        int size = NumContacts;
        int nearestPoint = -1;
        for (int i = 0; i < size; i++)
        {
            ManifoldPoint mp = pointCache[i];
            Vector3 diffA = mp.localPointA - newPoint.localPointA;
            btScalar distToManiPoint = diffA.Dot(diffA);
            if (distToManiPoint < shortestDist)
            {
                shortestDist = distToManiPoint;
                nearestPoint = i;
            }
        }
        return nearestPoint;
    }

    internal int AddManifoldPoint(ManifoldPoint newPoint, bool isPredictive = false)
    {
        int insertIndex = NumContacts;
        if (insertIndex == ManifoldCacheSize)
        {
            insertIndex = SortCachedPoints(newPoint);
            ClearUserCache(pointCache[insertIndex]);
        }
        else
        {
            cachedPoints++;
        }
        if (insertIndex < 0)
            insertIndex = 0;

        pointCache[insertIndex].CopyFrom(newPoint);
        return insertIndex;
    }

    internal void RemoveContactPoint(int index)
    {
        ClearUserCache(pointCache[index]);

        int lastUsedIndex = NumContacts - 1;
        if (index != lastUsedIndex)
        {
            pointCache[index].CopyFrom(pointCache[lastUsedIndex]);
            pointCache[lastUsedIndex].userPersistentData = null;
            pointCache[lastUsedIndex].appliedImpulse = 0;
            pointCache[lastUsedIndex].prevRhs = 0;
            pointCache[lastUsedIndex].contactPointFlags = 0;
            pointCache[lastUsedIndex].appliedImpulseLateral1 = 0;
            pointCache[lastUsedIndex].appliedImpulseLateral2 = 0;
            pointCache[lastUsedIndex].lifeTime = 0;
        }

        cachedPoints--;
    }

    internal void ReplaceContactPoint(ManifoldPoint newPoint, int insertIndex)
    {
        int lifeTime = pointCache[insertIndex].lifeTime;
        btScalar appliedImpulse = pointCache[insertIndex].appliedImpulse;
        btScalar prevRhs = pointCache[insertIndex].prevRhs;
        btScalar appliedLateralImpulse1 = pointCache[insertIndex].appliedImpulseLateral1;
        btScalar appliedLateralImpulse2 = pointCache[insertIndex].appliedImpulseLateral2;

        bool replacePoint = true;
        if ((newPoint.contactPointFlags & ContactPointFlags.LateralFrictionInitialized) != 0)
        {
            btScalar mu = pointCache[insertIndex].combinedFriction;
            btScalar eps = 0;
            btScalar a = appliedLateralImpulse1 * appliedLateralImpulse1 + appliedLateralImpulse2 * appliedLateralImpulse2;
            btScalar b = eps + mu * appliedImpulse;
            b = b * b;
            replacePoint = a > b;
        }

        if (replacePoint)
        {
            object cache = pointCache[insertIndex].userPersistentData;
            pointCache[insertIndex].CopyFrom(newPoint);
            pointCache[insertIndex].userPersistentData = cache;
            pointCache[insertIndex].appliedImpulse = appliedImpulse;
            pointCache[insertIndex].prevRhs = prevRhs;
            pointCache[insertIndex].appliedImpulseLateral1 = appliedLateralImpulse1;
            pointCache[insertIndex].appliedImpulseLateral2 = appliedLateralImpulse2;
        }

        pointCache[insertIndex].lifeTime = lifeTime;
    }

    internal bool ValidContactDistance(ManifoldPoint pt) =>
        pt.distance1 <= ContactBreakingThreshold;

    /// <summary>Recomputes world positions and depths, dropping points beyond the collision margin</summary>
    internal void RefreshContactPoints(in Transform trA, in Transform trB)
    {
        for (int i = NumContacts - 1; i >= 0; i--)
        {
            ManifoldPoint manifoldPoint = pointCache[i];
            manifoldPoint.positionWorldOnA = trA * manifoldPoint.localPointA;
            manifoldPoint.positionWorldOnB = trB * manifoldPoint.localPointB;
            manifoldPoint.distance1 = (manifoldPoint.positionWorldOnA - manifoldPoint.positionWorldOnB).Dot(manifoldPoint.normalWorldOnB);
            manifoldPoint.lifeTime++;
        }

        for (int i = NumContacts - 1; i >= 0; i--)
        {
            ManifoldPoint manifoldPoint = pointCache[i];
            if (!ValidContactDistance(manifoldPoint))
            {
                RemoveContactPoint(i);
            }
            else
            {
                Vector3 projectedPoint = manifoldPoint.positionWorldOnA - manifoldPoint.normalWorldOnB * manifoldPoint.distance1;
                Vector3 projectedDifference = manifoldPoint.positionWorldOnB - projectedPoint;
                btScalar distance2d = projectedDifference.Dot(projectedDifference);
                if (distance2d > ContactBreakingThreshold * ContactBreakingThreshold)
                {
                    RemoveContactPoint(i);
                }
            }
        }
    }

    public void ClearManifold()
    {
        for (int i = 0; i < cachedPoints; i++)
        {
            ClearUserCache(pointCache[i]);
        }
        cachedPoints = 0;
    }
}
