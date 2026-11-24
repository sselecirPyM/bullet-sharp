namespace BulletSharp;

/// <summary>
/// Ported from btActivatingCollisionAlgorithm (a work-in-progress base class for
/// "more aggressive object activation", not yet enabled in Bullet).
/// </summary>
public abstract class ActivatingCollisionAlgorithm : CollisionAlgorithm
{
    protected ActivatingCollisionAlgorithm(ConstructionInfo ci)
        : base(ci)
    {
    }

    protected ActivatingCollisionAlgorithm(ConstructionInfo ci, CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap)
        : base(ci)
    {
    }
}
