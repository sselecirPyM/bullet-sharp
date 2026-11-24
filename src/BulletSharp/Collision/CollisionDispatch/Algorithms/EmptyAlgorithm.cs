namespace BulletSharp;

/// <summary>
/// Ported from btEmptyAlgorithm: placeholder algorithm for unsupported collision pairs.
/// The dispatcher may persist an EmptyAlgorithm to avoid repeated lookups each frame.
/// </summary>
public class EmptyAlgorithm : CollisionAlgorithm
{
    public EmptyAlgorithm(ConstructionInfo ci)
        : base(ci)
    {
    }

    /// <summary>btEmptyAlgorithm::CreateFunc::CreateCollisionAlgorithm</summary>
    public static CollisionAlgorithm Create(ConstructionInfo ci, CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap) =>
        new EmptyAlgorithm(ci);

    public override void ProcessCollision(CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut)
    {
    }

    public override btScalar CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1,
        DispatcherInfo dispatchInfo, ManifoldResult resultOut) => 1;

    public override void GetAllContactManifolds(List<PersistentManifold> manifoldArray)
    {
    }
}
