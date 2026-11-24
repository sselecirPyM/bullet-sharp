namespace BulletSharp;

/// <summary>btBroadphaseProxy::CollisionShapeTypes</summary>
public enum BroadphaseNativeType
{
    BoxShape = 0,
    TriangleShape = 1,
    TetrahedralShape = 2,
    ConvexTriangleMeshShape = 3,
    ConvexHullShape = 4,
    ConvexPointCloudShape = 5,
    CustomPolyhedralShape = 6,
    ImplicitConvexShapesStartHere = 7,
    SphereShape = 8,
    MultiSphereShape = 9,
    CapsuleShape = 10,
    ConeShape = 11,
    ConvexShape = 12,
    CylinderShape = 13,
    UniformScalingShape = 14,
    MinkowskiSumShape = 15,
    MinkowskiDifferenceShape = 16,
    Box2DShape = 17,
    Convex2DShape = 18,
    CustomConvexShape = 19,
    ConcaveShapesStartHere = 20,
    TriangleMeshShape = 21,
    ScaledTriangleMeshShape = 22,
    FastConcaveMeshShape = 23,
    TerrainShape = 24,
    GImpactShape = 25,
    MultiMaterialTriangleMesh = 26,
    EmptyShape = 27,
    StaticPlaneShape = 28,
    CustomConcaveShape = 29,
    ConcaveShapesEndHere = 30,
    CompoundShape = 31,
    SoftBodyShape = 32,
    HfFluidShape = 33,
    HfFluidBuoyantConvexShape = 34,
    InvalidShape = 35,
    MaxBroadphaseCollisionTypes = 36,
}

/// <summary>btCollisionObject::CollisionFlags</summary>
[Flags]
public enum CollisionFlags
{
    None = 0,
    StaticObject = 1,
    KinematicObject = 2,
    NoContactResponse = 4,
    CustomMaterialCallback = 8,
    CharacterObject = 16,
    DisableVisualizeObject = 32,
    DisableSpuCollisionProcessing = 64,
    HasContactStiffnessDamping = 128,
    HasCustomDebugRenderingColor = 256,
    HasFrictionAnchor = 512,
    HasCollisionSoundTrigger = 1024,
}

/// <summary>btCollisionObject activation states</summary>
public enum ActivationState
{
    Undefined = 0,
    ActiveTag = 1,
    IslandSleeping = 2,
    WantsDeactivation = 3,
    DisableDeactivation = 4,
    DisableSimulation = 5,
}

/// <summary>btBroadphaseProxy::CollisionFilterGroups</summary>
[Flags]
public enum CollisionFilterGroups
{
    None = 0,
    DefaultFilter = 1,
    StaticFilter = 2,
    KinematicFilter = 4,
    DebrisFilter = 8,
    SensorTrigger = 16,
    CharacterFilter = 32,
    AllFilter = -1,
}

/// <summary>btCollisionObject::CollisionObjectTypes</summary>
public enum CollisionObjectTypes
{
    CollisionObject = 1,
    RigidBody = 2,
    GhostObject = 4,
    SoftBody = 8,
    HfFluid = 16,
    UserType = 32,
    FeatherstoneLink = 64,
}

/// <summary>btDispatcherInfo::DispatchFunc</summary>
public enum DispatchFunc
{
    Discrete = 1,
    Continuous = 2,
}

/// <summary>btCollisionDispatcher::btDispatcherFlags</summary>
[Flags]
public enum DispatcherFlags
{
    None = 0,
    UseRelativeContactBreakingThreshold = 1,
    UseConvexConservativeDistanceUtil = 2,
}

/// <summary>btTypedConstraint::btConstraintParams</summary>
public enum ConstraintParams
{
    Erp = 1,
    StopErp = 2,
    Cfm = 3,
    StopCfm = 4,
}

/// <summary>btTypedConstraintType</summary>
public enum TypedConstraintType
{
    Point2Point = 3,
    Hinge = 4,
    ConeTwist = 5,
    D6 = 6,
    Slider = 7,
    Contact = 8,
    D6Spring = 9,
    Gear = 10,
    Fixed = 11,
    D6Spring2 = 12,
    Max = 13,
}

/// <summary>btContactSolverInfo::btSolverMode</summary>
[Flags]
public enum SolverModes
{
    None = 0,
    RandomizeOrder = 1,
    FrictionSeparate = 2,
    UseWarmStarting = 4,
    Use2FrictionDirections = 16,
    EnableFrictionDirectionCaching = 32,
    DisableVelocityDependentFrictionDirection = 64,
    CacheFriendly = 128,
    Simd = 256,
    InterleaveContactAndFriction = 512,
    AllowZeroLengthFrictionDirections = 1024,
    DisableImplicitConeFriction = 2048,
}

/// <summary>btManifoldPoint::btContactPointFlags</summary>
[Flags]
public enum ContactPointFlags
{
    None = 0,
    LateralFrictionInitialized = 1,
    HasContactCfm = 2,
    HasContactErp = 4,
    ContactStiffnessDamping = 8,
    FrictionAnchor = 16,
}

/// <summary>Helper for btTypedConstraint::btConstraintInfo1 (row count limit)</summary>
public enum ConstraintInfo1SizeLimit
{
    Default = 6,
}
