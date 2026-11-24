namespace BulletSharp;

/// <summary>btDefaultCollisionConstructionInfo (pool sizes are meaningless in C#; fields kept for compatibility).</summary>
public class DefaultCollisionConstructionInfo
{
    public int DefaultMaxPersistentManifoldPoolSize = 4096;
    public int DefaultMaxCollisionAlgorithmPoolSize = 4096;
    public int CustomCollisionAlgorithmMaxElementSize;
    public bool UseEpaPenetrationAlgorithm = true;
}
