using BulletSharp.Math;

namespace BulletSharp.SoftBody;

/// <summary>Ported from btSoftBodyWorldInfo.</summary>
public class SoftBodyWorldInfo : IDisposable
{
    internal btScalar airDensity = (btScalar)1.2;
    internal btScalar waterDensity = 0;
    internal btScalar waterOffset = 0;
    internal btScalar maxDisplacement = (btScalar)1000;
    internal Vector3 waterNormal = Vector3.Zero;
    internal BroadphaseInterface broadphase;
    internal Dispatcher dispatcher;
    internal Vector3 gravity = new(0, -10, 0);
    internal readonly SparseSdf sparseSdf = new();

    public SoftBodyWorldInfo()
    {
        sparseSdf.Initialize();
    }

    public btScalar AirDensity
    {
        get => airDensity;
        set => airDensity = value;
    }

    public btScalar WaterDensity
    {
        get => waterDensity;
        set => waterDensity = value;
    }

    public btScalar WaterOffset
    {
        get => waterOffset;
        set => waterOffset = value;
    }

    public btScalar MaxDisplacement
    {
        get => maxDisplacement;
        set => maxDisplacement = value;
    }

    public Vector3 WaterNormal
    {
        get => waterNormal;
        set => waterNormal = value;
    }

    public BroadphaseInterface Broadphase
    {
        get => broadphase;
        set => broadphase = value;
    }

    public Dispatcher Dispatcher
    {
        get => dispatcher;
        set => dispatcher = value;
    }

    public Vector3 Gravity
    {
        get => gravity;
        set => gravity = value;
    }

    public SparseSdf SparseSdf => sparseSdf;

    public void Dispose()
    {
        sparseSdf.Reset();
    }
}
