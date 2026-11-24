namespace BulletSharp.SoftBody;

/// <summary>
/// Collision detection between two <see cref="SoftBody"/> shapes.
/// Ported from BulletSoftBody/btSoftSoftCollisionAlgorithm.{h,cpp}.
/// </summary>
public class SoftSoftCollisionAlgorithm : CollisionAlgorithm, IDisposable
{
    public SoftSoftCollisionAlgorithm(ConstructionInfo ci)
        : base(ci)
    {
    }

    public SoftSoftCollisionAlgorithm(PersistentManifold mf, ConstructionInfo ci,
        CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap)
        : base(ci)
    {
    }

    /// <summary>btSoftSoftCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm</summary>
    public static CollisionAlgorithm Create(ConstructionInfo ci,
        CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new SoftSoftCollisionAlgorithm(null, ci, body0Wrap, body1Wrap);

    public override void ProcessCollision(CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
        var soft0 = (SoftBody)body0Wrap.CollisionObject;
        var soft1 = (SoftBody)body1Wrap.CollisionObject;
        soft0.SoftBodySolver.ProcessCollision(soft0, soft1);
    }

    public override btScalar CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1,
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
