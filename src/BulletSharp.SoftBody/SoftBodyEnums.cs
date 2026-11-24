namespace BulletSharp.SoftBody;

/// <summary>Ported from btSoftBody::eAeroModel.</summary>
public enum AeroModel
{
    VPoint = 0,
    VTwoSided = 1,
    VTwoSidedLiftDrag = 2,
    VOneSided = 3,
    FTwoSided = 4,
    FTwoSidedLiftDrag = 5,
    FOneSided = 6,
    End = 7,
}

/// <summary>Ported from btSoftBody::eVSolver (velocity solvers).</summary>
public enum VSolver
{
    Linear = 0,
}

/// <summary>Ported from btSoftBody::ePSolver (position solvers).</summary>
public enum PSolver
{
    Linear = 0,
    Anchors = 1,
    RContacts = 2,
    SContacts = 3,
}

/// <summary>Ported from btSoftBody::eSolverPresets.</summary>
public enum SolverPresets
{
    Positions = 0,
    Velocities = 1,
    Default = Positions,
}

/// <summary>Ported from btSoftBody::eFeature.</summary>
public enum FeatureType
{
    None = 0,
    Node = 1,
    Link = 2,
    Face = 3,
    Tetra = 4,
}

/// <summary>Ported from btSoftBody::fCollision (classic soft body flags; deformable flags omitted).</summary>
[Flags]
public enum Collisions
{
    None = 0x0000,
    RVSmask = 0x000f,
    SDF_RS = 0x0001,
    CL_RS = 0x0002,
    SVSmask = 0x00f0,
    VF_SS = 0x0010,
    CL_SS = 0x0020,
    CL_SELF = 0x0040,
    Default = SDF_RS,
}

/// <summary>Ported from btSoftBody::fMaterial.</summary>
[Flags]
public enum MaterialFlags
{
    None = 0x0000,
    DebugDraw = 0x0001,
    Default = DebugDraw,
}

/// <summary>Ported from btSoftBodyHelpers::fDrawFlags.</summary>
[Flags]
public enum DrawFlags
{
    None = 0x0000,
    Nodes = 0x0001,
    Links = 0x0002,
    Faces = 0x0004,
    Tetras = 0x0008,
    Normals = 0x0010,
    Contacts = 0x0020,
    Anchors = 0x0040,
    Notes = 0x0080,
    Clusters = 0x0100,
    NodeTree = 0x0200,
    FaceTree = 0x0400,
    ClusterTree = 0x0800,
    Joints = 0x1000,
    Std = Links | Faces | Tetras | Anchors | Notes | Joints,
    StdTetra = Std & ~Faces,
}

/// <summary>Ported from btSoftBody::Joint::eType.</summary>
public enum JointType
{
    Linear = 0,
    Angular = 1,
    Contact = 2,
}
