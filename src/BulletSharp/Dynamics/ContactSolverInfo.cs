namespace BulletSharp;

/// <summary>
/// Ported from btContactSolverInfo.
/// </summary>
public class ContactSolverInfo
{
    internal btScalar tau = (btScalar)0.6;
    internal btScalar damping = 1;
    internal btScalar friction = (btScalar)0.3;
    internal btScalar timeStep;
    internal btScalar restitution = 0;
    internal int numIterations = 10;
    internal btScalar maxErrorReduction = 20;
    internal btScalar sor = 1;
    internal btScalar erp = (btScalar)0.2;
    internal btScalar erp2 = (btScalar)0.2;
    internal btScalar globalCfm = 0;
    internal bool splitImpulse = true;
    internal btScalar splitImpulsePenetrationThreshold = (btScalar)(-0.04);
    internal btScalar splitImpulseTurnErp = (btScalar)0.1;
    internal btScalar linearSlop = 0;
    internal btScalar warmstartingFactor = (btScalar)0.85;
    internal btScalar maxGyroscopicForce = 100;
    internal btScalar singleAxisRollingFrictionThreshold = (btScalar)1e30;
    internal SolverModes solverMode = SolverModes.UseWarmStarting | SolverModes.Simd;
    internal btScalar restingContactRestitutionThreshold = 2;
    internal int minimumSolverBatchSize = 128;
    internal btScalar leastSquaresResidualThreshold = 0;
    internal int restackIterations = 1;

    public btScalar Tau { get => tau; set => tau = value; }
    public btScalar Damping { get => damping; set => damping = value; }
    public btScalar Friction { get => friction; set => friction = value; }
    public btScalar TimeStep { get => timeStep; set => timeStep = value; }
    public btScalar Restitution { get => restitution; set => restitution = value; }
    public int NumIterations { get => numIterations; set => numIterations = value; }
    public btScalar MaxErrorReduction { get => maxErrorReduction; set => maxErrorReduction = value; }
    public btScalar Sor { get => sor; set => sor = value; }
    public btScalar Erp { get => erp; set => erp = value; }
    public btScalar Erp2 { get => erp2; set => erp2 = value; }
    public btScalar GlobalCfm { get => globalCfm; set => globalCfm = value; }
    public bool SplitImpulse { get => splitImpulse; set => splitImpulse = value; }
    public btScalar SplitImpulsePenetrationThreshold { get => splitImpulsePenetrationThreshold; set => splitImpulsePenetrationThreshold = value; }
    public btScalar SplitImpulseTurnErp { get => splitImpulseTurnErp; set => splitImpulseTurnErp = value; }
    public btScalar LinearSlop { get => linearSlop; set => linearSlop = value; }
    public btScalar WarmstartingFactor { get => warmstartingFactor; set => warmstartingFactor = value; }
    public btScalar MaxGyroscopicForce { get => maxGyroscopicForce; set => maxGyroscopicForce = value; }
    public btScalar SingleAxisRollingFrictionThreshold { get => singleAxisRollingFrictionThreshold; set => singleAxisRollingFrictionThreshold = value; }
    public SolverModes SolverMode { get => solverMode; set => solverMode = value; }
    public btScalar RestingContactRestitutionThreshold { get => restingContactRestitutionThreshold; set => restingContactRestitutionThreshold = value; }
    public int MinimumSolverBatchSize { get => minimumSolverBatchSize; set => minimumSolverBatchSize = value; }
    public btScalar LeastSquaresResidualThreshold { get => leastSquaresResidualThreshold; set => leastSquaresResidualThreshold = value; }
    public int RestackIterations { get => restackIterations; set => restackIterations = value; }
}
