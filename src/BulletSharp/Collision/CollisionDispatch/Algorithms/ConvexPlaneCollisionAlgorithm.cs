using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btConvexPlaneCollisionAlgorithm: convex vs. static plane collision detection.
/// </summary>
public class ConvexPlaneCollisionAlgorithm : CollisionAlgorithm
{
    internal bool ownManifold;
    internal PersistentManifold manifoldPtr;
    internal bool isSwapped;
    internal int numPerturbationIterations;
    internal int minimumPointsPerturbationThreshold;

    public ConvexPlaneCollisionAlgorithm(PersistentManifold mf, ConstructionInfo ci,
        CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap, bool isSwapped,
        int numPerturbationIterations = 1, int minimumPointsPerturbationThreshold = 0)
        : base(ci)
    {
        manifoldPtr = mf;
        this.isSwapped = isSwapped;
        this.numPerturbationIterations = numPerturbationIterations;
        this.minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;

        CollisionObjectWrapper convexObjWrap = this.isSwapped ? body1Wrap : body0Wrap;
        CollisionObjectWrapper planeObjWrap = this.isSwapped ? body0Wrap : body1Wrap;

        if (manifoldPtr == null && dispatcher.NeedsCollision(convexObjWrap.CollisionObject, planeObjWrap.CollisionObject))
        {
            manifoldPtr = dispatcher.GetNewManifold(convexObjWrap.CollisionObject, planeObjWrap.CollisionObject);
            ownManifold = true;
        }
    }

    /// <summary>btConvexPlaneCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm (not swapped)</summary>
    public static CollisionAlgorithm Create(ConstructionInfo ci, CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new ConvexPlaneCollisionAlgorithm(null, ci, body0Wrap, body1Wrap, false);

    /// <summary>btConvexPlaneCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm (swapped, m_swapped=true)</summary>
    public static CollisionAlgorithm SwappedCreate(ConstructionInfo ci, CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new ConvexPlaneCollisionAlgorithm(null, ci, body0Wrap, body1Wrap, true);

    public int NumPerturbationIterations
    {
        get => numPerturbationIterations;
        set => numPerturbationIterations = value;
    }

    public int MinimumPointsPerturbationThreshold
    {
        get => minimumPointsPerturbationThreshold;
        set => minimumPointsPerturbationThreshold = value;
    }

    /// <summary>btConvexPlaneCollisionAlgorithm::collideSingleContact</summary>
    internal void CollideSingleContact(Quaternion perturbeRot, CollisionObjectWrapper body0Wrap,
        CollisionObjectWrapper body1Wrap, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        CollisionObjectWrapper convexObjWrap = isSwapped ? body1Wrap : body0Wrap;
        CollisionObjectWrapper planeObjWrap = isSwapped ? body0Wrap : body1Wrap;

        var convexShape = (ConvexShape)convexObjWrap.CollisionShape;
        var planeShape = (StaticPlaneShape)planeObjWrap.CollisionShape;

        Vector3 planeNormal = planeShape.PlaneNormal;
        btScalar planeConstant = planeShape.PlaneConstant;

        Transform convexWorldTransform = convexObjWrap.WorldTransformInternal;
        Transform convexInPlaneTrans = planeObjWrap.WorldTransformInternal.Inverse() * convexWorldTransform;
        // now perturb the world transform of the convex shape
        convexWorldTransform.Basis *= new Matrix3x3(perturbeRot);
        Transform planeInConvex = convexWorldTransform.Inverse() * planeObjWrap.WorldTransformInternal;

        Vector3 vtx = convexShape.LocalGetSupportingVertex(planeInConvex.Basis * -planeNormal);

        Vector3 vtxInPlane = convexInPlaneTrans * vtx;
        btScalar distance = planeNormal.Dot(vtxInPlane) - planeConstant;

        Vector3 vtxInPlaneProjected = vtxInPlane - distance * planeNormal;
        Vector3 vtxInPlaneWorld = planeObjWrap.WorldTransformInternal * vtxInPlaneProjected;

        bool hasCollision = distance < manifoldPtr.ContactBreakingThreshold;
        resultOut.SetPersistentManifold(manifoldPtr);
        if (hasCollision)
        {
            // report the contact point; internally it is persisted and reduced
            Vector3 normalOnSurfaceB = planeObjWrap.WorldTransformInternal.Basis * planeNormal;
            Vector3 pOnB = vtxInPlaneWorld;
            resultOut.AddContactPoint(normalOnSurfaceB, pOnB, distance);
        }
    }

    /// <summary>btConvexPlaneCollisionAlgorithm::processCollision</summary>
    public override void ProcessCollision(CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        if (manifoldPtr == null)
            return;

        CollisionObjectWrapper convexObjWrap = isSwapped ? body1Wrap : body0Wrap;
        CollisionObjectWrapper planeObjWrap = isSwapped ? body0Wrap : body1Wrap;

        var convexShape = (ConvexShape)convexObjWrap.CollisionShape;
        var planeShape = (StaticPlaneShape)planeObjWrap.CollisionShape;

        Vector3 planeNormal = planeShape.PlaneNormal;
        btScalar planeConstant = planeShape.PlaneConstant;
        Transform planeInConvex = convexObjWrap.WorldTransformInternal.Inverse() * planeObjWrap.WorldTransformInternal;
        Transform convexInPlaneTrans = planeObjWrap.WorldTransformInternal.Inverse() * convexObjWrap.WorldTransformInternal;

        Vector3 vtx = convexShape.LocalGetSupportingVertex(planeInConvex.Basis * -planeNormal);
        Vector3 vtxInPlane = convexInPlaneTrans * vtx;
        btScalar distance = planeNormal.Dot(vtxInPlane) - planeConstant;

        Vector3 vtxInPlaneProjected = vtxInPlane - distance * planeNormal;
        Vector3 vtxInPlaneWorld = planeObjWrap.WorldTransformInternal * vtxInPlaneProjected;

        // (C++ also adds resultOut->m_closestPointDistanceThreshold here; the current base layer has no such field, treated as 0)
        bool hasCollision = distance < manifoldPtr.ContactBreakingThreshold;
        resultOut.SetPersistentManifold(manifoldPtr);
        if (hasCollision)
        {
            // report the contact point; internally it is persisted and reduced
            Vector3 normalOnSurfaceB = planeObjWrap.WorldTransformInternal.Basis * planeNormal;
            Vector3 pOnB = vtxInPlaneWorld;
            resultOut.AddContactPoint(normalOnSurfaceB, pOnB, distance);
        }

        // the perturbation algorithm doesn't work well with implicit surfaces
        // (sphere, cylinder, cone): the extra off-center contact points would make
        // them roll forever, so the feature is only enabled for polyhedral shapes
        // (btBoxShape, btConvexHullShape etc.)
        if (convexShape.IsPolyhedral && resultOut.PersistentManifold.NumContacts < minimumPointsPerturbationThreshold)
        {
            BtMath.PlaneSpace1(in planeNormal, out Vector3 v0, out Vector3 v1);
            // now perform numPerturbationIterations collision queries in perturbed orientations

            const btScalar angleLimit = (btScalar)0.125 * BtMath.Pi;
            btScalar radius = convexShape.GetAngularMotionDisc();
            btScalar perturbeAngle = PersistentManifold.GlobalContactBreakingThreshold / radius;
            if (perturbeAngle > angleLimit)
                perturbeAngle = angleLimit;

            var perturbeRot = new Quaternion(v0, perturbeAngle);
            for (int i = 0; i < numPerturbationIterations; i++)
            {
                btScalar iterationAngle = i * (BtMath.Simd2Pi / numPerturbationIterations);
                var rotq = new Quaternion(planeNormal, iterationAngle);
                CollideSingleContact(rotq.Inverse() * perturbeRot * rotq, body0Wrap, body1Wrap, dispatchInfo, resultOut);
            }
        }

        if (ownManifold)
        {
            if (manifoldPtr.NumContacts != 0)
            {
                resultOut.RefreshContactPoints();
            }
        }
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
