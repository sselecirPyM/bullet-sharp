namespace BulletSharp.SoftBody;

/// <summary>
/// Collision detection between a <see cref="SoftBody"/> and a convex rigid/collision object.
/// Ported from BulletSoftBody/btSoftRigidCollisionAlgorithm.{h,cpp}.
/// </summary>
public class SoftRigidCollisionAlgorithm : CollisionAlgorithm, IDisposable
{
    internal bool isSwapped;

    public SoftRigidCollisionAlgorithm(PersistentManifold mf, ConstructionInfo ci,
        CollisionObjectWrapper col0, CollisionObjectWrapper col1Wrap, bool isSwapped)
        : base(ci)
    {
        this.isSwapped = isSwapped;
    }

    public bool IsSwapped => isSwapped;

    /// <summary>btSoftRigidCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm (not swapped: soft vs convex)</summary>
    public static CollisionAlgorithm Create(ConstructionInfo ci,
        CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new SoftRigidCollisionAlgorithm(null, ci, body0Wrap, body1Wrap, false);

    /// <summary>btSoftRigidCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm (swapped: convex vs soft)</summary>
    public static CollisionAlgorithm SwappedCreate(ConstructionInfo ci,
        CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new SoftRigidCollisionAlgorithm(null, ci, body0Wrap, body1Wrap, true);

    public override void ProcessCollision(CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        SoftBody softBody = isSwapped
            ? (SoftBody)body1Wrap.CollisionObject
            : (SoftBody)body0Wrap.CollisionObject;
        CollisionObjectWrapper rigidCollisionObjectWrap = isSwapped ? body0Wrap : body1Wrap;

        if (!softBody.collisionDisabledObjects.Contains(rigidCollisionObjectWrap.CollisionObject))
        {
            softBody.SoftBodySolver.ProcessCollision(softBody, rigidCollisionObjectWrap);
        }
    }

    public override btScalar CalculateTimeOfImpact(CollisionObject col0, CollisionObject col1,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        return 1;
    }

    public override void GetAllContactManifolds(List<PersistentManifold> manifoldArray)
    {
        // we don't add any manifolds
    }

    public void Dispose()
    {
        Cleanup(dispatcher);
        GC.SuppressFinalize(this);
    }
}
