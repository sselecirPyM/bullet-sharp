namespace BulletSharp;

/// <summary>
/// Ported from btDispatcherInfo.
/// </summary>
public class DispatcherInfo
{
    internal btScalar timeStep;
    internal int stepCount;
    internal DispatchFunc dispatchFunc;
    internal btScalar timeOfImpact;
    internal bool useContinuous;
    internal IDebugDraw debugDraw;
    internal bool enableSatConvex;
    internal bool enableSpu = true;
    internal bool useEpa = true;
    internal btScalar allowedCcdPenetration = (btScalar)0.04;
    internal bool useConvexConservativeDistanceUtil;
    internal btScalar convexConservativeDistanceThreshold;
    internal bool deterministicOverlappingPairs;

    public DispatcherInfo()
    {
        timeStep = 0;
        stepCount = 0;
        dispatchFunc = DispatchFunc.Discrete;
        timeOfImpact = 1;
    }

    public btScalar TimeStep
    {
        get => timeStep;
        set => timeStep = value;
    }

    public int StepCount
    {
        get => stepCount;
        set => stepCount = value;
    }

    public DispatchFunc DispatchMethod
    {
        get => dispatchFunc;
        set => dispatchFunc = value;
    }

    public btScalar TimeOfImpact
    {
        get => timeOfImpact;
        set => timeOfImpact = value;
    }

    public bool UseContinuous
    {
        get => useContinuous;
        set => useContinuous = value;
    }

    public IDebugDraw DebugDraw
    {
        get => debugDraw;
        set => debugDraw = value;
    }

    public bool EnableSatConvex
    {
        get => enableSatConvex;
        set => enableSatConvex = value;
    }

    public bool EnableSpu
    {
        get => enableSpu;
        set => enableSpu = value;
    }

    public bool UseEpa
    {
        get => useEpa;
        set => useEpa = value;
    }

    public btScalar AllowedCcdPenetration
    {
        get => allowedCcdPenetration;
        set => allowedCcdPenetration = value;
    }

    public bool UseConvexConservativeDistanceUtil
    {
        get => useConvexConservativeDistanceUtil;
        set => useConvexConservativeDistanceUtil = value;
    }

    public btScalar ConvexConservativeDistanceThreshold
    {
        get => convexConservativeDistanceThreshold;
        set => convexConservativeDistanceThreshold = value;
    }

    public bool DeterministicOverlappingPairs
    {
        get => deterministicOverlappingPairs;
        set => deterministicOverlappingPairs = value;
    }
}
