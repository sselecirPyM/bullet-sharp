namespace BulletSharp.SoftBody;

/// <summary>
/// Adds soft body collision algorithm creation functions on top of <see cref="DefaultCollisionConfiguration"/>.
/// Ported from BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.{h,cpp}.
/// </summary>
public class SoftBodyRigidBodyCollisionConfiguration : DefaultCollisionConfiguration
{
    public SoftBodyRigidBodyCollisionConfiguration()
    {
    }

    public SoftBodyRigidBodyCollisionConfiguration(DefaultCollisionConstructionInfo constructionInfo)
        : base(constructionInfo)
    {
    }

    internal override CreateCollisionAlgorithmFunc GetCollisionAlgorithmCreateFunc(
        BroadphaseNativeType proxyType0, BroadphaseNativeType proxyType1)
    {
        if (proxyType0 == BroadphaseNativeType.SoftBodyShape && proxyType1 == BroadphaseNativeType.SoftBodyShape)
            return SoftSoftCollisionAlgorithm.Create;

        if (proxyType0 == BroadphaseNativeType.SoftBodyShape && BroadphaseProxy.IsConvex(proxyType1))
            return SoftRigidCollisionAlgorithm.Create;

        if (BroadphaseProxy.IsConvex(proxyType0) && proxyType1 == BroadphaseNativeType.SoftBodyShape)
            return SoftRigidCollisionAlgorithm.SwappedCreate;

        if (proxyType0 == BroadphaseNativeType.SoftBodyShape && BroadphaseProxy.IsConcave(proxyType1))
            return SoftBodyConcaveCollisionAlgorithm.Create;

        if (BroadphaseProxy.IsConcave(proxyType0) && proxyType1 == BroadphaseNativeType.SoftBodyShape)
            return SoftBodyConcaveCollisionAlgorithm.SwappedCreate;

        return base.GetCollisionAlgorithmCreateFunc(proxyType0, proxyType1);
    }
}
