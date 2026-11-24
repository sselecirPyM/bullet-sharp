using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btSphereSphereCollisionAlgorithm: sphere-sphere collision detection,
/// supporting frame-to-frame coherence (persistent contacts) and collision response;
/// also the most basic example of a custom algorithm.
/// </summary>
public class SphereSphereCollisionAlgorithm : ActivatingCollisionAlgorithm
{
    internal bool ownManifold;
    internal PersistentManifold manifoldPtr;

    public SphereSphereCollisionAlgorithm(ConstructionInfo ci)
        : base(ci)
    {
    }

    public SphereSphereCollisionAlgorithm(PersistentManifold mf, ConstructionInfo ci,
        CollisionObjectWrapper col0Wrap, CollisionObjectWrapper col1Wrap)
        : base(ci, col0Wrap, col1Wrap)
    {
        manifoldPtr = mf;
        if (manifoldPtr == null)
        {
            manifoldPtr = dispatcher.GetNewManifold(col0Wrap.CollisionObject, col1Wrap.CollisionObject);
            ownManifold = true;
        }
    }

    /// <summary>btSphereSphereCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm</summary>
    public static CollisionAlgorithm Create(ConstructionInfo ci, CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new SphereSphereCollisionAlgorithm(null, ci, body0Wrap, body1Wrap);

    /// <summary>btSphereSphereCollisionAlgorithm::processCollision</summary>
    public override void ProcessCollision(CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        if (manifoldPtr == null)
            return;

        resultOut.SetPersistentManifold(manifoldPtr);

        var sphere0 = (SphereShape)body0Wrap.CollisionShape;
        var sphere1 = (SphereShape)body1Wrap.CollisionShape;

        Vector3 diff = body0Wrap.WorldTransformInternal.Origin - body1Wrap.WorldTransformInternal.Origin;
        btScalar len = diff.Length();
        btScalar radius0 = sphere0.Radius;
        btScalar radius1 = sphere1.Radius;

        // don't add a new contact if the distance is positive
        // (C++ also adds resultOut->m_closestPointDistanceThreshold here; the current base layer has no such field, treated as 0)
        if (len > radius0 + radius1)
        {
            resultOut.RefreshContactPoints();
            return;
        }
        // distance (negative means penetration)
        btScalar dist = len - (radius0 + radius1);

        Vector3 normalOnSurfaceB = new(1, 0, 0);
        if (len > BtMath.SimdEpsilon)
        {
            normalOnSurfaceB = diff / len;
        }

        // point on B (world space)
        Vector3 pos1 = body1Wrap.WorldTransformInternal.Origin + radius1 * normalOnSurfaceB;

        // report the contact point; internally it is persisted and reduced
        resultOut.AddContactPoint(normalOnSurfaceB, pos1, dist);

        resultOut.RefreshContactPoints();
    }

    public override btScalar CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        // not yet
        return 1;
    }

    public override void GetAllContactManifolds(List<PersistentManifold> manifoldArray)
    {
        if (manifoldPtr != null && ownManifold)
            manifoldArray.Add(manifoldPtr);
    }

    internal override void Cleanup(Dispatcher dispatcher)
    {
        if (ownManifold && manifoldPtr != null)
        {
            dispatcher.ReleaseManifold(manifoldPtr);
            manifoldPtr = null;
        }
    }
}
