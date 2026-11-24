namespace BulletSharp;

/// <summary>Delegate form of btCollisionAlgorithmCreateFunc.</summary>
public delegate CollisionAlgorithm CreateCollisionAlgorithmFunc(
    CollisionAlgorithm.ConstructionInfo ci, CollisionObjectWrapper body0Wrap, CollisionObjectWrapper body1Wrap);

/// <summary>
/// Ported from btCollisionConfiguration. Abstract base class CollisionConfiguration in BulletSharp.
/// </summary>
public abstract class CollisionConfiguration : IDisposable
{
    internal abstract CreateCollisionAlgorithmFunc GetCollisionAlgorithmCreateFunc(
        BroadphaseNativeType proxyType0, BroadphaseNativeType proxyType1);

    internal abstract CreateCollisionAlgorithmFunc GetClosestPointsAlgorithmCreateFunc(
        BroadphaseNativeType proxyType0, BroadphaseNativeType proxyType1);

    public virtual void Dispose()
    {
    }
}
