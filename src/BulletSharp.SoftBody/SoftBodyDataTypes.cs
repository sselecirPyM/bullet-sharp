using BulletSharp.Math;

namespace BulletSharp.SoftBody;

/// <summary>Ported from btSoftBody::sRayCast.</summary>
public struct sRayCast
{
    internal SoftBody body;
    internal FeatureType feature;
    internal int index;
    internal btScalar fraction;

    public SoftBody Body
    {
        readonly get => body;
        set => body = value;
    }

    public FeatureType Feature
    {
        readonly get => feature;
        set => feature = value;
    }

    public int Index
    {
        readonly get => index;
        set => index = value;
    }

    public btScalar Fraction
    {
        readonly get => fraction;
        set => fraction = value;
    }
}

/// <summary>Ported from btSoftBody::ImplicitFn.</summary>
public abstract class ImplicitFn
{
    public abstract btScalar Eval(in Vector3 x);
}

/// <summary>Ported from btSoftBody::sCti (soft body contact info).</summary>
public struct sCti
{
    internal CollisionObject colObj;
    internal Vector3 normal;
    internal Vector3 impulse;
    internal btScalar offset;
    internal Vector3 bary;

    public CollisionObject CollisionObject
    {
        readonly get => colObj;
        set => colObj = value;
    }

    public Vector3 Normal
    {
        readonly get => normal;
        set => normal = value;
    }

    public Vector3 Impulse
    {
        readonly get => impulse;
        set => impulse = value;
    }

    public btScalar Offset
    {
        readonly get => offset;
        set => offset = value;
    }

    public Vector3 Barycentric
    {
        readonly get => bary;
        set => bary = value;
    }
}

/// <summary>Ported from btSoftBody::sMedium.</summary>
public struct sMedium
{
    internal Vector3 velocity;
    internal btScalar pressure;
    internal btScalar density;

    public Vector3 Velocity
    {
        readonly get => velocity;
        set => velocity = value;
    }

    public btScalar Pressure
    {
        readonly get => pressure;
        set => pressure = value;
    }

    public btScalar Density
    {
        readonly get => density;
        set => density = value;
    }
}

/// <summary>Ported from btSoftBody::Element.</summary>
public class Element
{
    internal object tag;

    public object Tag
    {
        get => tag;
        set => tag = value;
    }
}

/// <summary>Ported from btSoftBody::Material.</summary>
public class Material : Element
{
    internal btScalar kLST;
    internal btScalar kAST;
    internal btScalar kVST;
    internal int flags;

    public btScalar Lst
    {
        get => kLST;
        set => kLST = value;
    }

    public btScalar Ast
    {
        get => kAST;
        set => kAST = value;
    }

    public btScalar Vst
    {
        get => kVST;
        set => kVST = value;
    }

    public MaterialFlags Flags
    {
        get => (MaterialFlags)flags;
        set => flags = (int)value;
    }
}

/// <summary>Ported from btSoftBody::Feature.</summary>
public class Feature : Element
{
    internal Material material;

    public Material Material
    {
        get => material;
        set => material = value;
    }
}

/// <summary>Ported from btSoftBody::RenderNode.</summary>
public class RenderNode
{
    internal Vector3 x;
    internal Vector3 uv1;
    internal Vector3 normal;

    public Vector3 Position
    {
        get => x;
        set => x = value;
    }

    public Vector3 Uv1
    {
        get => uv1;
        set => uv1 = value;
    }

    public Vector3 Normal
    {
        get => normal;
        set => normal = value;
    }
}

/// <summary>Ported from btSoftBody::Node.</summary>
public class Node : Feature
{
    internal Vector3 x;
    internal Vector3 q;
    internal Vector3 v;
    internal Vector3 vn;
    internal Vector3 f;
    internal Vector3 n;
    internal btScalar im;
    internal btScalar area;
    internal DbvtNode leaf;
    internal int battach;
    internal int index;

    public Vector3 Position
    {
        get => x;
        set => x = value;
    }

    public Vector3 Q
    {
        get => q;
        set => q = value;
    }

    public Vector3 Velocity
    {
        get => v;
        set => v = value;
    }

    public Vector3 ForceAccumulator
    {
        get => f;
        set => f = value;
    }

    public Vector3 Normal
    {
        get => n;
        set => n = value;
    }

    public btScalar InverseMass
    {
        get => im;
        set => im = value;
    }

    public btScalar Area
    {
        get => area;
        set => area = value;
    }

    public DbvtNode Leaf
    {
        get => leaf;
        set => leaf = value;
    }

    public int IsAttached
    {
        get => battach;
        set => battach = value;
    }

    public int Index
    {
        get => index;
        set => index = value;
    }
}

/// <summary>Ported from btSoftBody::Link.</summary>
public class Link : Feature
{
    internal Vector3 c3;
    internal readonly Node[] n = new Node[2];
    internal btScalar rl;
    internal int bbending;
    internal btScalar c0;
    internal btScalar c1;
    internal btScalar c2;

    public Node[] Nodes => n;

    public btScalar RestLength
    {
        get => rl;
        set => rl = value;
    }

    public int IsBending
    {
        get => bbending;
        set => bbending = value;
    }

    public btScalar C0
    {
        get => c0;
        set => c0 = value;
    }

    public btScalar C1
    {
        get => c1;
        set => c1 = value;
    }

    public btScalar C2
    {
        get => c2;
        set => c2 = value;
    }

    public Vector3 C3
    {
        get => c3;
        set => c3 = value;
    }
}

/// <summary>Ported from btSoftBody::Face.</summary>
public class Face : Feature
{
    internal readonly Node[] n = new Node[3];
    internal Vector3 normal;
    internal btScalar ra;
    internal DbvtNode leaf;
    internal readonly btScalar[] pcontact = new btScalar[4];
    internal Vector3 n0;
    internal Vector3 n1;
    internal Vector3 vn;
    internal int index;

    public Node[] Nodes => n;

    public Vector3 Normal
    {
        get => normal;
        set => normal = value;
    }

    public btScalar RestArea
    {
        get => ra;
        set => ra = value;
    }

    public DbvtNode Leaf
    {
        get => leaf;
        set => leaf = value;
    }

    public int Index
    {
        get => index;
        set => index = value;
    }
}

/// <summary>Ported from btSoftBody::Tetra.</summary>
public class Tetra : Feature
{
    internal readonly Node[] n = new Node[4];
    internal btScalar rv;
    internal DbvtNode leaf;
    internal readonly Vector3[] c0 = new Vector3[4];
    internal btScalar c1;
    internal btScalar c2;
    internal Matrix3x3 dmInverse;
    internal Matrix3x3 f;
    internal btScalar elementMeasure;
    internal readonly btScalar[][] pInv = new btScalar[3][]
    {
        new btScalar[4],
        new btScalar[4],
        new btScalar[4],
    };

    public Node[] Nodes => n;

    public btScalar RestVolume
    {
        get => rv;
        set => rv = value;
    }

    public DbvtNode Leaf
    {
        get => leaf;
        set => leaf = value;
    }

    public Vector3[] C0 => c0;

    public btScalar C1
    {
        get => c1;
        set => c1 = value;
    }

    public btScalar C2
    {
        get => c2;
        set => c2 = value;
    }

    public Matrix3x3 DmInverse
    {
        get => dmInverse;
        set => dmInverse = value;
    }

    public Matrix3x3 F
    {
        get => f;
        set => f = value;
    }

    public btScalar ElementMeasure
    {
        get => elementMeasure;
        set => elementMeasure = value;
    }
}

/// <summary>Ported from btSoftBody::TetraScratch.</summary>
public struct TetraScratch
{
    internal Matrix3x3 f;
    internal btScalar trace;
    internal btScalar j;
    internal Matrix3x3 cofF;
    internal Matrix3x3 corotation;

    public Matrix3x3 F
    {
        readonly get => f;
        set => f = value;
    }

    public btScalar Trace
    {
        readonly get => trace;
        set => trace = value;
    }

    public btScalar J
    {
        readonly get => j;
        set => j = value;
    }

    public Matrix3x3 CofF
    {
        readonly get => cofF;
        set => cofF = value;
    }

    public Matrix3x3 Corotation
    {
        readonly get => corotation;
        set => corotation = value;
    }
}

/// <summary>Ported from btSoftBody::RContact (MultiBody fields omitted).</summary>
public struct RContact
{
    internal sCti cti;
    internal Node node;
    internal Matrix3x3 c0;
    internal Vector3 c1;
    internal btScalar c2;
    internal btScalar c3;
    internal btScalar c4;
    internal Vector3 t1;
    internal Vector3 t2;

    public sCti Cti
    {
        readonly get => cti;
        set => cti = value;
    }

    public Node Node
    {
        readonly get => node;
        set => node = value;
    }

    public Matrix3x3 C0
    {
        readonly get => c0;
        set => c0 = value;
    }

    public Vector3 C1
    {
        readonly get => c1;
        set => c1 = value;
    }

    public btScalar C2
    {
        readonly get => c2;
        set => c2 = value;
    }

    public btScalar C3
    {
        readonly get => c3;
        set => c3 = value;
    }

    public btScalar C4
    {
        readonly get => c4;
        set => c4 = value;
    }
}

/// <summary>Inline two-element scalar storage for value-type contact structs without heap allocation.</summary>
public struct ScalarPair
{
    public btScalar V0;
    public btScalar V1;

    public btScalar this[int index]
    {
        readonly get => index == 0 ? V0 : V1;
        set
        {
            if (index == 0)
                V0 = value;
            else
                V1 = value;
        }
    }
}

/// <summary>Ported from btSoftBody::SContact.</summary>
public struct SContact
{
    internal Node node;
    internal Face face;
    internal Vector3 weights;
    internal Vector3 normal;
    internal btScalar margin;
    internal btScalar friction;
    internal ScalarPair cfm;

    public Node Node
    {
        readonly get => node;
        set => node = value;
    }

    public Face Face
    {
        readonly get => face;
        set => face = value;
    }

    public Vector3 Weights
    {
        readonly get => weights;
        set => weights = value;
    }

    public Vector3 Normal
    {
        readonly get => normal;
        set => normal = value;
    }

    public btScalar Margin
    {
        readonly get => margin;
        set => margin = value;
    }

    public btScalar Friction
    {
        readonly get => friction;
        set => friction = value;
    }

    public ScalarPair Cfm
    {
        readonly get => cfm;
        set => cfm = value;
    }
}

/// <summary>Ported from btSoftBody::Anchor.</summary>
public class Anchor
{
    internal Node node;
    internal Vector3 local;
    internal RigidBody body;
    internal btScalar influence;
    internal Matrix3x3 c0;
    internal Vector3 c1;
    internal btScalar c2;

    public Node Node
    {
        get => node;
        set => node = value;
    }

    public Vector3 Local
    {
        get => local;
        set => local = value;
    }

    public RigidBody Body
    {
        get => body;
        set => body = value;
    }

    public btScalar Influence
    {
        get => influence;
        set => influence = value;
    }

    public Matrix3x3 C0
    {
        get => c0;
        set => c0 = value;
    }

    public Vector3 C1
    {
        get => c1;
        set => c1 = value;
    }

    public btScalar C2
    {
        get => c2;
        set => c2 = value;
    }
}

/// <summary>Ported from btSoftBody::Note.</summary>
public class Note : Element
{
    internal string text;
    internal Vector3 offset;
    internal int rank;
    internal readonly Node[] nodes = new Node[4];
    internal readonly btScalar[] coords = new btScalar[4];

    public string Text
    {
        get => text;
        set => text = value;
    }

    public Vector3 Offset
    {
        get => offset;
        set => offset = value;
    }

    public int Rank
    {
        get => rank;
        set => rank = value;
    }

    public Node[] Nodes => nodes;

    public btScalar[] Coords => coords;
}

/// <summary>Ported from btSoftBody::Pose.</summary>
public class Pose
{
    internal bool bvolume;
    internal bool bframe;
    internal btScalar volume;
    internal readonly List<Vector3> pos = new();
    internal readonly List<btScalar> wgh = new();
    internal Vector3 com;
    internal Matrix3x3 rot = Matrix3x3.Identity;
    internal Matrix3x3 scl = Matrix3x3.Identity;
    internal Matrix3x3 aqq;

    public bool IsVolume
    {
        get => bvolume;
        set => bvolume = value;
    }

    public bool IsFrame
    {
        get => bframe;
        set => bframe = value;
    }

    public btScalar Volume
    {
        get => volume;
        set => volume = value;
    }

    public List<Vector3> Positions => pos;

    public List<btScalar> Weights => wgh;

    public Vector3 Com
    {
        get => com;
        set => com = value;
    }

    public Matrix3x3 Rotation
    {
        get => rot;
        set => rot = value;
    }

    public Matrix3x3 Scale
    {
        get => scl;
        set => scl = value;
    }

    public Matrix3x3 Aqq
    {
        get => aqq;
        set => aqq = value;
    }
}

/// <summary>Ported from btSoftBody::Cluster.</summary>
public class Cluster
{
    internal readonly List<btScalar> masses = new();
    internal readonly List<Node> nodes = new();
    internal readonly List<Vector3> framerefs = new();
    internal Transform framexform = Transform.Identity;
    internal btScalar idmass;
    internal btScalar imass;
    internal Matrix3x3 locii;
    internal Matrix3x3 invwi;
    internal Vector3 com;
    internal readonly Vector3[] vimpulses = new Vector3[2];
    internal readonly Vector3[] dimpulses = new Vector3[2];
    internal int nvimpulses;
    internal int ndimpulses;
    internal Vector3 lv;
    internal Vector3 av;
    internal DbvtNode leaf;
    internal btScalar ndamping;
    internal btScalar ldamping;
    internal btScalar adamping;
    internal btScalar matching;
    internal btScalar maxSelfCollisionImpulse = (btScalar)100;
    internal btScalar selfCollisionImpulseFactor = (btScalar)0.01;
    internal bool containsAnchor;
    internal bool collide;
    internal int clusterIndex;

    public List<btScalar> Masses => masses;

    public List<Node> Nodes => nodes;

    public List<Vector3> FrameRefs => framerefs;

    public Transform FrameTransform
    {
        get => framexform;
        set => framexform = value;
    }

    public btScalar IdMass
    {
        get => idmass;
        set => idmass = value;
    }

    public btScalar InverseMass
    {
        get => imass;
        set => imass = value;
    }

    public Matrix3x3 LocalInertia
    {
        get => locii;
        set => locii = value;
    }

    public Matrix3x3 InverseWorldInertia
    {
        get => invwi;
        set => invwi = value;
    }

    public Vector3 Com
    {
        get => com;
        set => com = value;
    }

    public Vector3[] VImpulses => vimpulses;

    public Vector3[] DImpulses => dimpulses;

    public int NumVImpulses
    {
        get => nvimpulses;
        set => nvimpulses = value;
    }

    public int NumDImpulses
    {
        get => ndimpulses;
        set => ndimpulses = value;
    }

    public Vector3 LinearVelocity
    {
        get => lv;
        set => lv = value;
    }

    public Vector3 AngularVelocity
    {
        get => av;
        set => av = value;
    }

    public DbvtNode Leaf
    {
        get => leaf;
        set => leaf = value;
    }

    public btScalar NodeDamping
    {
        get => ndamping;
        set => ndamping = value;
    }

    public btScalar LinearDamping
    {
        get => ldamping;
        set => ldamping = value;
    }

    public btScalar AngularDamping
    {
        get => adamping;
        set => adamping = value;
    }

    public btScalar Matching
    {
        get => matching;
        set => matching = value;
    }

    public btScalar MaxSelfCollisionImpulse
    {
        get => maxSelfCollisionImpulse;
        set => maxSelfCollisionImpulse = value;
    }

    public btScalar SelfCollisionImpulseFactor
    {
        get => selfCollisionImpulseFactor;
        set => selfCollisionImpulseFactor = value;
    }

    public bool ContainsAnchor
    {
        get => containsAnchor;
        set => containsAnchor = value;
    }

    public bool Collide
    {
        get => collide;
        set => collide = value;
    }

    public int ClusterIndex
    {
        get => clusterIndex;
        set => clusterIndex = value;
    }

    internal void ApplyVImpulse(in Vector3 rpos, in Vector3 impulse)
    {
        Vector3 li = impulse * imass;
        Vector3 cross = new(
            rpos.Y * impulse.Z - rpos.Z * impulse.Y,
            rpos.Z * impulse.X - rpos.X * impulse.Z,
            rpos.X * impulse.Y - rpos.Y * impulse.X);
        Vector3 ai = invwi * cross;
        vimpulses[0] += li;
        lv += li;
        vimpulses[1] += ai;
        av += ai;
        nvimpulses++;
    }

    internal void ApplyDImpulse(in Vector3 rpos, in Vector3 impulse)
    {
        Vector3 li = impulse * imass;
        Vector3 cross = new(
            rpos.Y * impulse.Z - rpos.Z * impulse.Y,
            rpos.Z * impulse.X - rpos.X * impulse.Z,
            rpos.X * impulse.Y - rpos.Y * impulse.X);
        Vector3 ai = invwi * cross;
        dimpulses[0] += li;
        dimpulses[1] += ai;
        ndimpulses++;
    }

    internal void ApplyVAImpulse(in Vector3 impulse)
    {
        Vector3 ai = invwi * impulse;
        vimpulses[1] += ai;
        av += ai;
        nvimpulses++;
    }

    internal void ApplyDAImpulse(in Vector3 impulse)
    {
        Vector3 ai = invwi * impulse;
        dimpulses[1] += ai;
        ndimpulses++;
    }

    internal void ApplyDCImpulse(in Vector3 impulse)
    {
        dimpulses[0] += impulse * imass;
        ndimpulses++;
    }
}

/// <summary>Ported from btSoftBody::Impulse.</summary>
public struct Impulse
{
    internal Vector3 velocity;
    internal Vector3 drift;
    internal int asVelocity;
    internal int asDrift;

    public Vector3 Velocity
    {
        readonly get => velocity;
        set => velocity = value;
    }

    public Vector3 Drift
    {
        readonly get => drift;
        set => drift = value;
    }

    public int AsVelocity
    {
        readonly get => asVelocity;
        set => asVelocity = value;
    }

    public int AsDrift
    {
        readonly get => asDrift;
        set => asDrift = value;
    }

    public static Impulse operator -(in Impulse i) =>
        new()
        {
            velocity = -i.velocity,
            drift = -i.drift,
            asVelocity = i.asVelocity,
            asDrift = i.asDrift,
        };

    public static Impulse operator *(in Impulse i, btScalar x) =>
        new()
        {
            velocity = i.velocity * x,
            drift = i.drift * x,
            asVelocity = i.asVelocity,
            asDrift = i.asDrift,
        };

    public static Impulse operator *(btScalar x, in Impulse i) => i * x;
}

/// <summary>Ported from btSoftBody::Body (Cluster / RigidBody / CollisionObject branches).</summary>
public class Body
{
    internal Cluster soft;
    internal RigidBody rigid;
    internal CollisionObject collisionObject;

    public Body()
    {
    }

    public Body(Cluster p)
    {
        soft = p;
    }

    public Body(CollisionObject colObj)
    {
        collisionObject = colObj;
        rigid = colObj as RigidBody;
    }

    public Cluster Soft
    {
        get => soft;
        set => soft = value;
    }

    public RigidBody Rigid
    {
        get => rigid;
        set => rigid = value;
    }

    public CollisionObject CollisionObject
    {
        get => collisionObject;
        set => collisionObject = value;
    }

    public void Activate()
    {
        rigid?.Activate();
        collisionObject?.Activate();
    }

    public Matrix3x3 InvWorldInertia()
    {
        if (rigid != null)
            return rigid.invInertiaTensorWorld;
        if (soft != null)
            return soft.invwi;
        return default;
    }

    public btScalar InvMass()
    {
        if (rigid != null)
            return rigid.InvMass;
        if (soft != null)
            return soft.imass;
        return 0;
    }

    public Transform Xform()
    {
        if (collisionObject != null)
            return collisionObject.worldTransform;
        if (soft != null)
            return soft.framexform;
        return Transform.Identity;
    }

    public Vector3 LinearVelocity()
    {
        if (rigid != null)
            return rigid.linearVelocity;
        if (soft != null)
            return soft.lv;
        return Vector3.Zero;
    }

    public Vector3 AngularVelocity(in Vector3 rpos)
    {
        if (rigid != null)
        {
            Vector3 av = rigid.angularVelocity;
            return new Vector3(
                av.Y * rpos.Z - av.Z * rpos.Y,
                av.Z * rpos.X - av.X * rpos.Z,
                av.X * rpos.Y - av.Y * rpos.X);
        }
        if (soft != null)
        {
            Vector3 av = soft.av;
            return new Vector3(
                av.Y * rpos.Z - av.Z * rpos.Y,
                av.Z * rpos.X - av.X * rpos.Z,
                av.X * rpos.Y - av.Y * rpos.X);
        }
        return Vector3.Zero;
    }

    public Vector3 AngularVelocity()
    {
        if (rigid != null)
            return rigid.angularVelocity;
        if (soft != null)
            return soft.av;
        return Vector3.Zero;
    }

    public Vector3 Velocity(in Vector3 rpos) => LinearVelocity() + AngularVelocity(in rpos);

    public void ApplyVImpulse(in Vector3 impulse, in Vector3 rpos)
    {
        if (rigid != null)
        {
            Vector3 imp = impulse;
            Vector3 rel = rpos;
            rigid.ApplyImpulse(ref imp, ref rel);
        }
        if (soft != null)
            soft.ApplyVImpulse(in rpos, in impulse);
    }

    public void ApplyDImpulse(in Vector3 impulse, in Vector3 rpos)
    {
        if (rigid != null)
        {
            Vector3 imp = impulse;
            Vector3 rel = rpos;
            rigid.ApplyImpulse(ref imp, ref rel);
        }
        if (soft != null)
            soft.ApplyDImpulse(in rpos, in impulse);
    }

    public void ApplyImpulse(in Impulse impulse, in Vector3 rpos)
    {
        if (impulse.asVelocity != 0)
            ApplyVImpulse(in impulse.velocity, in rpos);
        if (impulse.asDrift != 0)
            ApplyDImpulse(in impulse.drift, in rpos);
    }

    public void ApplyVAImpulse(in Vector3 impulse)
    {
        if (rigid != null)
        {
            Vector3 imp = impulse;
            rigid.ApplyTorqueImpulse(ref imp);
        }
        if (soft != null)
            soft.ApplyVAImpulse(in impulse);
    }

    public void ApplyDAImpulse(in Vector3 impulse)
    {
        if (rigid != null)
        {
            Vector3 imp = impulse;
            rigid.ApplyTorqueImpulse(ref imp);
        }
        if (soft != null)
            soft.ApplyDAImpulse(in impulse);
    }

    public void ApplyAImpulse(in Impulse impulse)
    {
        if (impulse.asVelocity != 0)
            ApplyVAImpulse(in impulse.velocity);
        if (impulse.asDrift != 0)
            ApplyDAImpulse(in impulse.drift);
    }

    public void ApplyDCImpulse(in Vector3 impulse)
    {
        if (rigid != null)
        {
            Vector3 imp = impulse;
            rigid.ApplyCentralImpulse(ref imp);
        }
        if (soft != null)
            soft.ApplyDCImpulse(in impulse);
    }
}

/// <summary>Ported from btSoftBody::Joint.</summary>
public abstract partial class Joint
{
    public class Specs
    {
        internal btScalar erp = 1;
        internal btScalar cfm = 1;
        internal btScalar split = 1;

        public btScalar Erp
        {
            get => erp;
            set => erp = value;
        }

        public btScalar Cfm
        {
            get => cfm;
            set => cfm = value;
        }

        public btScalar Split
        {
            get => split;
            set => split = value;
        }
    }

    internal readonly Body[] bodies = new Body[] { new(), new() };
    internal readonly Vector3[] refs = new Vector3[2];
    internal btScalar cfm;
    internal btScalar erp;
    internal btScalar split;
    internal Vector3 drift;
    internal Vector3 sdrift;
    internal Matrix3x3 massmatrix;
    internal bool delete;

    public Body[] Bodies => bodies;

    public Vector3[] Refs => refs;

    public btScalar Cfm
    {
        get => cfm;
        set => cfm = value;
    }

    public btScalar Erp
    {
        get => erp;
        set => erp = value;
    }

    public btScalar Split
    {
        get => split;
        set => split = value;
    }

    public Vector3 Drift
    {
        get => drift;
        set => drift = value;
    }

    public Vector3 SDrift
    {
        get => sdrift;
        set => sdrift = value;
    }

    public Matrix3x3 MassMatrix
    {
        get => massmatrix;
        set => massmatrix = value;
    }

    public bool Delete
    {
        get => delete;
        set => delete = value;
    }

    public abstract JointType Type { get; }
}

/// <summary>Ported from btSoftBody::LJoint.</summary>
public partial class LJoint : Joint
{
    public new class Specs : Joint.Specs
    {
        internal Vector3 position;

        public Vector3 Position
        {
            get => position;
            set => position = value;
        }
    }

    internal readonly Vector3[] rpos = new Vector3[2];

    public Vector3[] RPos => rpos;

    public override JointType Type => JointType.Linear;
}

/// <summary>Ported from btSoftBody::AJoint.</summary>
public partial class AJoint : Joint
{
    public class IControl
    {
        private static readonly IControl s_default = new();

        public static IControl Default() => s_default;

        public virtual void Prepare(AJoint joint)
        {
        }

        public virtual btScalar Speed(AJoint joint, btScalar current) => current;
    }

    public new class Specs : Joint.Specs
    {
        internal Vector3 axis;
        internal IControl icontrol = IControl.Default();

        public Vector3 Axis
        {
            get => axis;
            set => axis = value;
        }

        public IControl Control
        {
            get => icontrol;
            set => icontrol = value;
        }
    }

    internal readonly Vector3[] axis = new Vector3[2];
    internal IControl icontrol = IControl.Default();

    public Vector3[] Axis => axis;

    public IControl Control
    {
        get => icontrol;
        set => icontrol = value;
    }

    public override JointType Type => JointType.Angular;
}

/// <summary>Ported from btSoftBody::CJoint.</summary>
public partial class CJoint : Joint
{
    internal int life;
    internal int maxlife;
    internal readonly Vector3[] rpos = new Vector3[2];
    internal Vector3 normal;
    internal btScalar friction;

    public int Life
    {
        get => life;
        set => life = value;
    }

    public int MaxLife
    {
        get => maxlife;
        set => maxlife = value;
    }

    public Vector3[] RPos => rpos;

    public Vector3 Normal
    {
        get => normal;
        set => normal = value;
    }

    public btScalar Friction
    {
        get => friction;
        set => friction = value;
    }

    public override JointType Type => JointType.Contact;
}

/// <summary>Ported from btSoftBody::Config.</summary>
public class Config
{
    internal AeroModel aeromodel;
    internal btScalar kVCF;
    internal btScalar kDP;
    internal btScalar kDG;
    internal btScalar kLF;
    internal btScalar kPR;
    internal btScalar kVC;
    internal btScalar kDF;
    internal btScalar kMT;
    internal btScalar kCHR;
    internal btScalar kKHR;
    internal btScalar kSHR;
    internal btScalar kAHR;
    internal btScalar kSRHR_CL;
    internal btScalar kSKHR_CL;
    internal btScalar kSSHR_CL;
    internal btScalar kSR_SPLT_CL;
    internal btScalar kSK_SPLT_CL;
    internal btScalar kSS_SPLT_CL;
    internal btScalar maxvolume;
    internal btScalar timescale;
    internal int viterations;
    internal int piterations;
    internal int diterations;
    internal int citerations;
    internal int collisions;
    internal readonly List<VSolver> vsequence = new();
    internal readonly List<PSolver> psequence = new();
    internal readonly List<PSolver> dsequence = new();

    public AeroModel AeroModel
    {
        get => aeromodel;
        set => aeromodel = value;
    }

    public btScalar Vcf
    {
        get => kVCF;
        set => kVCF = value;
    }

    public btScalar DP
    {
        get => kDP;
        set => kDP = value;
    }

    public btScalar DG
    {
        get => kDG;
        set => kDG = value;
    }

    public btScalar LF
    {
        get => kLF;
        set => kLF = value;
    }

    public btScalar PR
    {
        get => kPR;
        set => kPR = value;
    }

    public btScalar VC
    {
        get => kVC;
        set => kVC = value;
    }

    public btScalar DF
    {
        get => kDF;
        set => kDF = value;
    }

    public btScalar MT
    {
        get => kMT;
        set => kMT = value;
    }

    public btScalar Chr
    {
        get => kCHR;
        set => kCHR = value;
    }

    public btScalar Khr
    {
        get => kKHR;
        set => kKHR = value;
    }

    public btScalar Shr
    {
        get => kSHR;
        set => kSHR = value;
    }

    public btScalar Ahr
    {
        get => kAHR;
        set => kAHR = value;
    }

    public btScalar SrhrCl
    {
        get => kSRHR_CL;
        set => kSRHR_CL = value;
    }

    public btScalar SkhrCl
    {
        get => kSKHR_CL;
        set => kSKHR_CL = value;
    }

    public btScalar SshrCl
    {
        get => kSSHR_CL;
        set => kSSHR_CL = value;
    }

    public btScalar SrSplitCl
    {
        get => kSR_SPLT_CL;
        set => kSR_SPLT_CL = value;
    }

    public btScalar SkSplitCl
    {
        get => kSK_SPLT_CL;
        set => kSK_SPLT_CL = value;
    }

    public btScalar SsSplitCl
    {
        get => kSS_SPLT_CL;
        set => kSS_SPLT_CL = value;
    }

    public btScalar MaxVolume
    {
        get => maxvolume;
        set => maxvolume = value;
    }

    public btScalar TimeScale
    {
        get => timescale;
        set => timescale = value;
    }

    public int VIterations
    {
        get => viterations;
        set => viterations = value;
    }

    public int PIterations
    {
        get => piterations;
        set => piterations = value;
    }

    public int DIterations
    {
        get => diterations;
        set => diterations = value;
    }

    public int CIterations
    {
        get => citerations;
        set => citerations = value;
    }

    public Collisions Collisions
    {
        get => (Collisions)collisions;
        set => collisions = (int)value;
    }

    public List<VSolver> VSequence => vsequence;

    public List<PSolver> PSequence => psequence;

    public List<PSolver> DSequence => dsequence;
}

/// <summary>Ported from btSoftBody::SolverState.</summary>
public struct SolverState
{
    internal btScalar sdt;
    internal btScalar isdt;
    internal btScalar velmrg;
    internal btScalar radmrg;
    internal btScalar updmrg;

    public btScalar Sdt
    {
        readonly get => sdt;
        set => sdt = value;
    }

    public btScalar InverseSdt
    {
        readonly get => isdt;
        set => isdt = value;
    }

    public btScalar VelocityMargin
    {
        readonly get => velmrg;
        set => velmrg = value;
    }

    public btScalar RadialMargin
    {
        readonly get => radmrg;
        set => radmrg = value;
    }

    public btScalar UpdateMargin
    {
        readonly get => updmrg;
        set => updmrg = value;
    }
}
