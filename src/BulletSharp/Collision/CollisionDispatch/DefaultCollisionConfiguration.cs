namespace BulletSharp;

/// <summary>
/// Ported from btDefaultCollisionConfiguration: the default collision algorithm double-dispatch table.
/// Pool allocation is meaningless in C# (GC-managed); the related constructor parameters are kept only for compatibility.
/// </summary>
public class DefaultCollisionConfiguration : CollisionConfiguration
{
    public DefaultCollisionConfiguration()
    {
        // btDefaultCollisionConfiguration: UseEpaPenetrationAlgorithm selects the penetration depth solver.
        // This port implements only GJK/EPA (btGjkEpaPenetrationDepthSolver);
        // the btMinkowskiPenetrationDepthSolver path is not ported (equivalent results, EPA is more stable).
    }

    public DefaultCollisionConfiguration(DefaultCollisionConstructionInfo constructionInfo)
        : this()
    {
    }

    // btDefaultCollisionConfiguration::getCollisionAlgorithmCreateFunc
    internal override CreateCollisionAlgorithmFunc GetCollisionAlgorithmCreateFunc(
        BroadphaseNativeType proxyType0, BroadphaseNativeType proxyType1)
    {
        if (proxyType0 == BroadphaseNativeType.SphereShape && proxyType1 == BroadphaseNativeType.SphereShape)
            return SphereSphereCollisionAlgorithm.Create;

        if (proxyType0 == BroadphaseNativeType.SphereShape && proxyType1 == BroadphaseNativeType.TriangleShape)
            return SphereTriangleCollisionAlgorithm.Create;

        if (proxyType0 == BroadphaseNativeType.TriangleShape && proxyType1 == BroadphaseNativeType.SphereShape)
            return SphereTriangleCollisionAlgorithm.SwappedCreate;

        if (proxyType0 == BroadphaseNativeType.BoxShape && proxyType1 == BroadphaseNativeType.BoxShape)
            return BoxBoxCollisionAlgorithm.Create;

        if (BroadphaseProxy.IsConvex(proxyType0) && proxyType1 == BroadphaseNativeType.StaticPlaneShape)
            return ConvexPlaneCollisionAlgorithm.Create;

        if (BroadphaseProxy.IsConvex(proxyType1) && proxyType0 == BroadphaseNativeType.StaticPlaneShape)
            return ConvexPlaneCollisionAlgorithm.SwappedCreate;

        if (BroadphaseProxy.IsConvex(proxyType0) && BroadphaseProxy.IsConvex(proxyType1))
            return ConvexConvexAlgorithm.Create;

        if (BroadphaseProxy.IsConvex(proxyType0) && BroadphaseProxy.IsConcave(proxyType1))
            return ConvexConcaveCollisionAlgorithm.Create;

        if (BroadphaseProxy.IsConvex(proxyType1) && BroadphaseProxy.IsConcave(proxyType0))
            return ConvexConcaveCollisionAlgorithm.SwappedCreate;

        if (BroadphaseProxy.IsCompound(proxyType0) && BroadphaseProxy.IsCompound(proxyType1))
            return CompoundCompoundCollisionAlgorithm.Create;

        if (BroadphaseProxy.IsCompound(proxyType0))
            return CompoundCollisionAlgorithm.Create;

        if (BroadphaseProxy.IsCompound(proxyType1))
            return CompoundCollisionAlgorithm.SwappedCreate;

        // No matching algorithm
        return EmptyAlgorithm.Create;
    }

    // btDefaultCollisionConfiguration::getClosestPointsAlgorithmCreateFunc
    // (same as the contact version, but without the dedicated box-box algorithm)
    internal override CreateCollisionAlgorithmFunc GetClosestPointsAlgorithmCreateFunc(
        BroadphaseNativeType proxyType0, BroadphaseNativeType proxyType1)
    {
        if (proxyType0 == BroadphaseNativeType.SphereShape && proxyType1 == BroadphaseNativeType.SphereShape)
            return SphereSphereCollisionAlgorithm.Create;

        if (proxyType0 == BroadphaseNativeType.SphereShape && proxyType1 == BroadphaseNativeType.TriangleShape)
            return SphereTriangleCollisionAlgorithm.Create;

        if (proxyType0 == BroadphaseNativeType.TriangleShape && proxyType1 == BroadphaseNativeType.SphereShape)
            return SphereTriangleCollisionAlgorithm.SwappedCreate;

        if (BroadphaseProxy.IsConvex(proxyType0) && proxyType1 == BroadphaseNativeType.StaticPlaneShape)
            return ConvexPlaneCollisionAlgorithm.Create;

        if (BroadphaseProxy.IsConvex(proxyType1) && proxyType0 == BroadphaseNativeType.StaticPlaneShape)
            return ConvexPlaneCollisionAlgorithm.SwappedCreate;

        if (BroadphaseProxy.IsConvex(proxyType0) && BroadphaseProxy.IsConvex(proxyType1))
            return ConvexConvexAlgorithm.Create;

        if (BroadphaseProxy.IsConvex(proxyType0) && BroadphaseProxy.IsConcave(proxyType1))
            return ConvexConcaveCollisionAlgorithm.Create;

        if (BroadphaseProxy.IsConvex(proxyType1) && BroadphaseProxy.IsConcave(proxyType0))
            return ConvexConcaveCollisionAlgorithm.SwappedCreate;

        if (BroadphaseProxy.IsCompound(proxyType0) && BroadphaseProxy.IsCompound(proxyType1))
            return CompoundCompoundCollisionAlgorithm.Create;

        if (BroadphaseProxy.IsCompound(proxyType0))
            return CompoundCollisionAlgorithm.Create;

        if (BroadphaseProxy.IsCompound(proxyType1))
            return CompoundCollisionAlgorithm.SwappedCreate;

        return EmptyAlgorithm.Create;
    }
}
