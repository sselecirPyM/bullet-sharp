using System.Diagnostics;
using BulletSharp.Math;

namespace BulletSharp.SoftBody;

/// <summary>
/// Ported from BulletSoftBody/btSoftBody.h and btSoftBody.cpp (construction, topology, mass/pose, and transforms).
/// </summary>
public partial class SoftBody : CollisionObject
{
    internal readonly List<CollisionObject> collisionDisabledObjects = new();
    internal SoftBodySolver softBodySolver;

    internal readonly Config cfg = new();
    internal SolverState sst;
    internal readonly Pose pose = new();
    internal object tag;
    internal SoftBodyWorldInfo worldInfo;
    internal readonly List<Note> notes = new();
    internal readonly List<Node> nodes = new();
    internal readonly List<Link> links = new();
    internal readonly List<Face> faces = new();
    internal readonly List<Tetra> tetras = new();
    internal readonly List<TetraScratch> tetraScratches = new();
    internal readonly List<TetraScratch> tetraScratchesTn = new();
    internal readonly List<Anchor> anchors = new();
    internal readonly List<RContact> rcontacts = new();
    internal readonly List<SContact> scontacts = new();
    internal readonly List<Joint> joints = new();
    internal readonly List<Material> materials = new();
    internal btScalar timeacc;
    internal readonly Vector3[] bounds = new Vector3[2];
    internal bool bUpdateRtCst;
    internal readonly Dbvt ndbvt = new();
    internal readonly Dbvt fdbvt = new();
    internal readonly Dbvt cdbvt = new();
    internal readonly List<Cluster> clusters = new();
    internal readonly List<bool> clusterConnectivity = new();
    internal BulletSharp.Math.Transform initialWorldTransform = BulletSharp.Math.Transform.Identity;
    internal Vector3 windVelocity;
    internal btScalar restLengthScale;
    internal readonly List<Vector3> x = new();
    internal uint randSeed = 243703u;
    internal readonly List<RenderNode> renderNodes = new();
    internal readonly List<btScalar[]> renderNodesInterpolationWeights = new();
    internal readonly List<List<Node>> renderNodesParents = new();
    internal readonly List<btScalar> z = new();

    public SoftBody()
        : this(null)
    {
    }

    public SoftBody(SoftBodyWorldInfo worldInfo)
    {
        this.worldInfo = worldInfo;
        InitDefaults();
    }

    public SoftBody(SoftBodyWorldInfo worldInfo, IReadOnlyList<Vector3> x, IReadOnlyList<btScalar> m)
        : this(worldInfo, x?.Count ?? m?.Count ?? 0, x, m)
    {
    }

    public SoftBody(SoftBodyWorldInfo worldInfo, int nodeCount, IReadOnlyList<Vector3> x, IReadOnlyList<btScalar> m)
    {
        softBodySolver = null;
        this.worldInfo = worldInfo;

        InitDefaults();

        // Default material
        Material pm = AppendMaterial();
        pm.kLST = 1;
        pm.kAST = 1;
        pm.kVST = 1;
        pm.flags = (int)MaterialFlags.Default;

        // Nodes
        btScalar margin = collisionShape.Margin;
        if (nodeCount > 0)
        {
            nodes.Capacity = nodeCount;
            this.x.Capacity = nodeCount;
        }
        for (int i = 0; i < nodeCount; ++i)
        {
            var n = new Node();
            n.x = x != null ? x[i] : Vector3.Zero;
            n.q = n.x;
            btScalar mass = m != null ? m[i] : 1;
            n.im = mass > 0 ? 1 / mass : 0;
            n.leaf = ndbvt.Insert(DbvtVolume.FromCR(n.x, margin), n);
            n.material = pm;
            n.index = i;
            nodes.Add(n);
            this.x.Add(n.x);
        }
        UpdateBounds();
    }

    public List<CollisionObject> CollisionDisabledObjects => collisionDisabledObjects;

    public SoftBodySolver SoftBodySolver
    {
        get => softBodySolver;
        set => softBodySolver = value;
    }

    public Config Cfg => cfg;

    public SolverState SolverState
    {
        get => sst;
        set => sst = value;
    }

    public Pose Pose => pose;

    public object Tag
    {
        get => tag;
        set => tag = value;
    }

    public SoftBodyWorldInfo WorldInfo
    {
        get => worldInfo;
        set => worldInfo = value;
    }

    public List<Note> Notes => notes;

    public List<Node> Nodes => nodes;

    public List<Link> Links => links;

    public List<Face> Faces => faces;

    public List<Tetra> Tetras => tetras;

    public List<Anchor> Anchors => anchors;

    public List<RContact> RContacts => rcontacts;

    public List<SContact> SContacts => scontacts;

    public List<Joint> Joints => joints;

    public List<Material> Materials => materials;

    public btScalar Timeacc
    {
        get => timeacc;
        set => timeacc = value;
    }

    public Vector3[] Bounds => bounds;

    public Dbvt NDbvt => ndbvt;

    public Dbvt FDbvt => fdbvt;

    public Dbvt CDbvt => cdbvt;

    public List<Cluster> Clusters => clusters;

    public List<RenderNode> RenderNodes => renderNodes;

    public List<btScalar[]> RenderNodesInterpolationWeights => renderNodesInterpolationWeights;

    public List<List<Node>> RenderNodesParents => renderNodesParents;

    public List<btScalar> Z => z;

    public Vector3 WindVelocity
    {
        get => windVelocity;
        set => windVelocity = value;
    }

    public btScalar RestLengthScale
    {
        get => GetRestLengthScale();
        set => SetRestLengthScale(value);
    }

    public btScalar TotalMass
    {
        get => GetTotalMass();
        set => SetTotalMass(value);
    }

    public btScalar Volume => GetVolume();

    public Vector3 CenterOfMass => EvaluateCom();

    public Vector3 LinearVelocity
    {
        get => GetLinearVelocity();
        set => SetLinearVelocity(in value);
    }

    public static SoftBody Upcast(CollisionObject colObj) =>
        colObj != null && colObj.internalType == CollisionObjectTypes.SoftBody
            ? (SoftBody)colObj
            : null;

    private void InitDefaults()
    {
        internalType = CollisionObjectTypes.SoftBody;
        cfg.aeromodel = AeroModel.VPoint;
        cfg.kVCF = 1;
        cfg.kDG = 0;
        cfg.kLF = 0;
        cfg.kDP = 0;
        cfg.kPR = 0;
        cfg.kVC = 0;
        cfg.kDF = (btScalar)0.2;
        cfg.kMT = 0;
        cfg.kCHR = (btScalar)1.0;
        cfg.kKHR = (btScalar)0.1;
        cfg.kSHR = (btScalar)1.0;
        cfg.kAHR = (btScalar)0.7;
        cfg.kSRHR_CL = (btScalar)0.1;
        cfg.kSKHR_CL = (btScalar)1;
        cfg.kSSHR_CL = (btScalar)0.5;
        cfg.kSR_SPLT_CL = (btScalar)0.5;
        cfg.kSK_SPLT_CL = (btScalar)0.5;
        cfg.kSS_SPLT_CL = (btScalar)0.5;
        cfg.maxvolume = (btScalar)1;
        cfg.timescale = 1;
        cfg.viterations = 0;
        cfg.piterations = 1;
        cfg.diterations = 0;
        cfg.citerations = 4;
        cfg.collisions = (int)Collisions.Default;
        pose.bvolume = false;
        pose.bframe = false;
        pose.volume = 0;
        pose.com = Vector3.Zero;
        pose.rot = Matrix3x3.Identity;
        pose.scl = Matrix3x3.Identity;
        tag = null;
        timeacc = 0;
        bUpdateRtCst = true;
        bounds[0] = Vector3.Zero;
        bounds[1] = Vector3.Zero;
        worldTransform = BulletSharp.Math.Transform.Identity;
        SetSolver(SolverPresets.Positions);

        // Collision shape
        collisionShape = new SoftBodyCollisionShape(this);
        collisionShape.Margin = (btScalar)0.25;

        worldTransform = BulletSharp.Math.Transform.Identity;

        windVelocity = Vector3.Zero;
        restLengthScale = (btScalar)1.0;
    }

    public bool CheckLink(int node0, int node1) =>
        CheckLink(nodes[node0], nodes[node1]);

    public bool CheckLink(Node node0, Node node1)
    {
        for (int i = 0, ni = links.Count; i < ni; ++i)
        {
            Link l = links[i];
            if ((ReferenceEquals(l.n[0], node0) && ReferenceEquals(l.n[1], node1)) ||
                (ReferenceEquals(l.n[0], node1) && ReferenceEquals(l.n[1], node0)))
            {
                return true;
            }
        }
        return false;
    }

    public bool CheckFace(int node0, int node1, int node2)
    {
        Node n0 = nodes[node0];
        Node n1 = nodes[node1];
        Node n2 = nodes[node2];
        for (int i = 0, ni = faces.Count; i < ni; ++i)
        {
            Face f = faces[i];
            int c = 0;
            for (int j = 0; j < 3; ++j)
            {
                Node fn = f.n[j];
                if (ReferenceEquals(fn, n0) ||
                    ReferenceEquals(fn, n1) ||
                    ReferenceEquals(fn, n2))
                {
                    c |= 1 << j;
                }
                else
                {
                    break;
                }
            }
            if (c == 7)
            {
                return true;
            }
        }
        return false;
    }

    public bool CheckContact(
        CollisionObjectWrapper colObjWrap,
        in Vector3 x,
        btScalar margin,
        ref sCti cti)
    {
        CollisionShape shp = colObjWrap.collisionShape;
        ref readonly BulletSharp.Math.Transform wtr = ref colObjWrap.worldTransform;
        Vector3 localX = wtr.InverseXForm(x);

        btScalar dst = worldInfo.sparseSdf.Evaluate(in localX, shp, out Vector3 nrm, margin);
        if (dst < 0)
        {
            cti.colObj = colObjWrap.collisionObject;
            cti.normal = wtr.Basis * nrm;
            cti.offset = -Vector3.Dot(cti.normal, x - cti.normal * dst);
            return true;
        }
        return false;
    }

    public Material AppendMaterial()
    {
        var pm = new Material();
        if (materials.Count > 0)
        {
            Material src = materials[0];
            pm.tag = src.tag;
            pm.kLST = src.kLST;
            pm.kAST = src.kAST;
            pm.kVST = src.kVST;
            pm.flags = src.flags;
        }
        materials.Add(pm);
        return pm;
    }

    public Material AppendMaterial(Material model)
    {
        var pm = new Material();
        if (model != null)
        {
            pm.tag = model.tag;
            pm.kLST = model.kLST;
            pm.kAST = model.kAST;
            pm.kVST = model.kVST;
            pm.flags = model.flags;
        }
        materials.Add(pm);
        return pm;
    }

    public void AppendNote(
        string text,
        in Vector3 o,
        btScalar c0,
        btScalar c1,
        btScalar c2,
        btScalar c3,
        Node n0 = null,
        Node n1 = null,
        Node n2 = null,
        Node n3 = null)
    {
        var n = new Note
        {
            rank = 0,
            text = text,
            offset = o,
        };
        n.coords[0] = c0;
        n.coords[1] = c1;
        n.coords[2] = c2;
        n.coords[3] = c3;
        n.nodes[0] = n0;
        n.rank += n0 != null ? 1 : 0;
        n.nodes[1] = n1;
        n.rank += n1 != null ? 1 : 0;
        n.nodes[2] = n2;
        n.rank += n2 != null ? 1 : 0;
        n.nodes[3] = n3;
        n.rank += n3 != null ? 1 : 0;
        notes.Add(n);
    }

    public void AppendNote(string text, in Vector3 o) =>
        AppendNote(text, in o, 1, 0, 0, 0);

    public void AppendNote(string text, in Vector3 o, Node feature) =>
        AppendNote(text, in o, 1, 0, 0, 0, feature);

    public void AppendNote(string text, in Vector3 o, Link feature)
    {
        btScalar w = 1 / (btScalar)2;
        AppendNote(text, in o, w, w, 0, 0, feature.n[0], feature.n[1]);
    }

    public void AppendNote(string text, in Vector3 o, Face feature)
    {
        btScalar w = 1 / (btScalar)3;
        AppendNote(text, in o, w, w, w, 0, feature.n[0], feature.n[1], feature.n[2]);
    }

    public void AppendNode(in Vector3 x, btScalar m)
    {
        btScalar margin = collisionShape.Margin;
        var n = new Node
        {
            x = x,
            q = x,
            im = m > 0 ? 1 / m : 0,
            material = materials.Count > 0 ? materials[0] : null,
            index = nodes.Count,
        };
        n.leaf = ndbvt.Insert(DbvtVolume.FromCR(n.x, margin), n);
        nodes.Add(n);
    }

    public void AppendLink(int model = -1, Material mat = null)
    {
        Link l;
        if (model >= 0)
        {
            Link src = links[model];
            l = new Link
            {
                tag = src.tag,
                material = src.material,
                rl = src.rl,
                bbending = src.bbending,
                c0 = src.c0,
                c1 = src.c1,
                c2 = src.c2,
                c3 = src.c3,
            };
            l.n[0] = src.n[0];
            l.n[1] = src.n[1];
        }
        else
        {
            l = new Link
            {
                material = mat ?? (materials.Count > 0 ? materials[0] : null),
            };
        }
        links.Add(l);
    }

    public void AppendLink(int node0, int node1, Material mat = null, bool bcheckexist = false) =>
        AppendLink(nodes[node0], nodes[node1], mat, bcheckexist);

    public void AppendLink(Node node0, Node node1, Material mat = null, bool bcheckexist = false)
    {
        if (!bcheckexist || !CheckLink(node0, node1))
        {
            AppendLink(-1, mat);
            Link l = links[links.Count - 1];
            l.n[0] = node0;
            l.n[1] = node1;
            l.rl = (l.n[0].x - l.n[1].x).Length();
            bUpdateRtCst = true;
        }
    }

    public void AppendFace(int model = -1, Material mat = null)
    {
        Face f;
        if (model >= 0)
        {
            Face src = faces[model];
            f = new Face
            {
                tag = src.tag,
                material = src.material,
                normal = src.normal,
                ra = src.ra,
                leaf = src.leaf,
                n0 = src.n0,
                n1 = src.n1,
                vn = src.vn,
            };
            f.n[0] = src.n[0];
            f.n[1] = src.n[1];
            f.n[2] = src.n[2];
            f.pcontact[0] = src.pcontact[0];
            f.pcontact[1] = src.pcontact[1];
            f.pcontact[2] = src.pcontact[2];
            f.pcontact[3] = src.pcontact[3];
        }
        else
        {
            f = new Face
            {
                material = mat ?? (materials.Count > 0 ? materials[0] : null),
            };
        }
        f.index = faces.Count;
        faces.Add(f);
    }

    public void AppendFace(int node0, int node1, int node2, Material mat = null)
    {
        if (node0 == node1)
            return;
        if (node1 == node2)
            return;
        if (node2 == node0)
            return;

        AppendFace(-1, mat);
        Face f = faces[faces.Count - 1];
        Debug.Assert(node0 != node1);
        Debug.Assert(node1 != node2);
        Debug.Assert(node2 != node0);
        f.n[0] = nodes[node0];
        f.n[1] = nodes[node1];
        f.n[2] = nodes[node2];
        f.ra = SoftBodyInternals.AreaOf(in f.n[0].x, in f.n[1].x, in f.n[2].x);
        bUpdateRtCst = true;
    }

    public void AppendTetra(int model = -1, Material mat = null)
    {
        Tetra t;
        if (model >= 0)
        {
            Tetra src = tetras[model];
            t = new Tetra
            {
                tag = src.tag,
                material = src.material,
                rv = src.rv,
                leaf = src.leaf,
                c1 = src.c1,
                c2 = src.c2,
                dmInverse = src.dmInverse,
                f = src.f,
                elementMeasure = src.elementMeasure,
            };
            for (int i = 0; i < 4; ++i)
            {
                t.n[i] = src.n[i];
                t.c0[i] = src.c0[i];
            }
            for (int r = 0; r < 3; ++r)
            {
                for (int c = 0; c < 4; ++c)
                {
                    t.pInv[r][c] = src.pInv[r][c];
                }
            }
        }
        else
        {
            t = new Tetra
            {
                material = mat ?? (materials.Count > 0 ? materials[0] : null),
            };
        }
        tetras.Add(t);
    }

    public void AppendTetra(int node0, int node1, int node2, int node3, Material mat = null)
    {
        AppendTetra(-1, mat);
        Tetra t = tetras[tetras.Count - 1];
        t.n[0] = nodes[node0];
        t.n[1] = nodes[node1];
        t.n[2] = nodes[node2];
        t.n[3] = nodes[node3];
        t.rv = SoftBodyInternals.VolumeOf(in t.n[0].x, in t.n[1].x, in t.n[2].x, in t.n[3].x);
        bUpdateRtCst = true;
    }

    public void AppendAnchor(
        int node,
        RigidBody body,
        bool disableCollisionBetweenLinkedBodies = false,
        btScalar influence = 1)
    {
        Vector3 local = body.worldTransform.Inverse() * nodes[node].x;
        AppendAnchor(node, body, in local, disableCollisionBetweenLinkedBodies, influence);
    }

    public void AppendAnchor(
        int node,
        RigidBody body,
        in Vector3 localPivot,
        bool disableCollisionBetweenLinkedBodies = false,
        btScalar influence = 1)
    {
        if (disableCollisionBetweenLinkedBodies && !collisionDisabledObjects.Contains(body))
        {
            collisionDisabledObjects.Add(body);
        }

        Node n = nodes[node];
        n.battach = 1;
        var a = new Anchor
        {
            node = n,
            body = body,
            local = localPivot,
            influence = influence,
        };
        anchors.Add(a);
    }

    public void AppendLinearJoint(LJoint.Specs specs, Cluster body0, Body body1)
    {
        var pj = new LJoint();
        pj.bodies[0] = new Body(body0);
        pj.bodies[1] = body1 != null
            ? new Body { soft = body1.soft, rigid = body1.rigid, collisionObject = body1.collisionObject }
            : new Body();
        pj.refs[0] = pj.bodies[0].Xform().Inverse() * specs.position;
        pj.refs[1] = pj.bodies[1].Xform().Inverse() * specs.position;
        pj.cfm = specs.cfm;
        pj.erp = specs.erp;
        pj.split = specs.split;
        joints.Add(pj);
    }

    public void AppendLinearJoint(LJoint.Specs specs, Body body) =>
        AppendLinearJoint(specs, clusters[0], body);

    public void AppendLinearJoint(LJoint.Specs specs, SoftBody body) =>
        AppendLinearJoint(specs, clusters[0], new Body(body.clusters[0]));

    public void AppendLinearJoint(LJoint.Specs specs, RigidBody body) =>
        AppendLinearJoint(specs, clusters[0], new Body(body));

    public void AppendAngularJoint(AJoint.Specs specs, Cluster body0, Body body1)
    {
        var pj = new AJoint();
        pj.bodies[0] = new Body(body0);
        pj.bodies[1] = body1 != null
            ? new Body { soft = body1.soft, rigid = body1.rigid, collisionObject = body1.collisionObject }
            : new Body();
        pj.refs[0] = pj.bodies[0].Xform().Inverse().Basis * specs.axis;
        pj.refs[1] = pj.bodies[1].Xform().Inverse().Basis * specs.axis;
        pj.cfm = specs.cfm;
        pj.erp = specs.erp;
        pj.split = specs.split;
        pj.icontrol = specs.icontrol;
        joints.Add(pj);
    }

    public void AppendAngularJoint(AJoint.Specs specs, Body body) =>
        AppendAngularJoint(specs, clusters[0], body);

    public void AppendAngularJoint(AJoint.Specs specs, SoftBody body) =>
        AppendAngularJoint(specs, clusters[0], new Body(body.clusters[0]));

    public void AppendAngularJoint(AJoint.Specs specs, RigidBody body) =>
        AppendAngularJoint(specs, clusters[0], new Body(body));

    public void AddForce(in Vector3 force)
    {
        for (int i = 0, ni = nodes.Count; i < ni; ++i)
        {
            AddForce(in force, i);
        }
    }

    public void AddForce(in Vector3 force, int node)
    {
        Node n = nodes[node];
        if (n.im > 0)
        {
            n.f += force;
        }
    }

    public void AddAeroForceToNode(in Vector3 windVelocity, int nodeIndex)
    {
        Debug.Assert(nodeIndex >= 0 && nodeIndex < nodes.Count);

        btScalar dt = sst.sdt;
        btScalar kLF = cfg.kLF;
        btScalar kDG = cfg.kDG;
        bool asLift = kLF > 0;
        bool asDrag = kDG > 0;
        bool asAero = asLift || asDrag;
        bool asVAero = asAero && (cfg.aeromodel < AeroModel.FTwoSided);

        Node n = nodes[nodeIndex];

        if (n.im > 0)
        {
            SoftBodyInternals.EvaluateMedium(worldInfo, in n.x, out sMedium medium);
            medium.velocity = windVelocity;
            medium.density = worldInfo.airDensity;

            // Aerodynamics
            if (asVAero)
            {
                Vector3 relV = n.v - medium.velocity;
                btScalar relVLen = relV.Length();
                btScalar relV2 = relV.LengthSquared();

                if (relV2 > BtMath.SimdEpsilon)
                {
                    Vector3 relVNrm = relV.Normalized();
                    Vector3 nrm = n.n;

                    if (cfg.aeromodel == AeroModel.VTwoSidedLiftDrag)
                    {
                        nrm *= (btScalar)(Vector3.Dot(nrm, relV) < 0 ? -1 : +1);
                        Vector3 fDrag;
                        Vector3 fLift = Vector3.Zero;

                        btScalar nDotV = nrm.Dot(relVNrm);
                        btScalar triArea = (btScalar)0.5 * n.area;

                        fDrag = (btScalar)0.5 * kDG * medium.density * relV2 * triArea * nDotV * (-relVNrm);

                        // Check angle of attack: cos(10 deg) = 0.98480
                        if (0 < nDotV && nDotV < (btScalar)0.98480f)
                        {
                            fLift = (btScalar)0.5 * kLF * medium.density * relVLen * triArea *
                                BtMath.Sqrt((btScalar)1.0 - nDotV * nDotV) *
                                nrm.Cross(relVNrm).Cross(relVNrm);
                        }

                        // Check if the velocity change resulted by aero drag force exceeds the current velocity of the node.
                        Vector3 delVByFDrag = fDrag * n.im * sst.sdt;
                        btScalar delVByFDragLen2 = delVByFDrag.LengthSquared();
                        btScalar vLen2 = n.v.LengthSquared();

                        if (delVByFDragLen2 >= vLen2 && delVByFDragLen2 > 0)
                        {
                            btScalar delVByFDragLen = delVByFDrag.Length();
                            btScalar vLen = n.v.Length();
                            fDrag *= (btScalar)0.8 * (vLen / delVByFDragLen);
                        }

                        n.f += fDrag;
                        n.f += fLift;
                    }
                    else if (cfg.aeromodel == AeroModel.VPoint ||
                             cfg.aeromodel == AeroModel.VOneSided ||
                             cfg.aeromodel == AeroModel.VTwoSided)
                    {
                        if (cfg.aeromodel == AeroModel.VTwoSided)
                        {
                            nrm *= (btScalar)(Vector3.Dot(nrm, relV) < 0 ? -1 : +1);
                        }

                        btScalar dvn = Vector3.Dot(relV, nrm);
                        if (dvn > 0)
                        {
                            Vector3 force = Vector3.Zero;
                            btScalar c0 = n.area * dvn * relV2 / 2;
                            btScalar c1 = c0 * medium.density;
                            force += nrm * (-c1 * kLF);
                            force += relV.Normalized() * (-c1 * kDG);
                            SoftBodyInternals.ApplyClampedForce(n, in force, dt);
                        }
                    }
                }
            }
        }
    }

    public void AddAeroForceToFace(in Vector3 windVelocity, int faceIndex)
    {
        btScalar dt = sst.sdt;
        btScalar kLF = cfg.kLF;
        btScalar kDG = cfg.kDG;
        bool asLift = kLF > 0;
        bool asDrag = kDG > 0;
        bool asAero = asLift || asDrag;
        bool asFAero = asAero && (cfg.aeromodel >= AeroModel.FTwoSided);

        if (asFAero)
        {
            Face f = faces[faceIndex];

            Vector3 v = (f.n[0].v + f.n[1].v + f.n[2].v) / 3;
            Vector3 faceX = (f.n[0].x + f.n[1].x + f.n[2].x) / 3;
            SoftBodyInternals.EvaluateMedium(worldInfo, in faceX, out sMedium medium);
            medium.velocity = windVelocity;
            medium.density = worldInfo.airDensity;
            Vector3 relV = v - medium.velocity;
            btScalar relVLen = relV.Length();
            btScalar relV2 = relV.LengthSquared();

            if (relV2 > BtMath.SimdEpsilon)
            {
                Vector3 relVNrm = relV.Normalized();
                Vector3 nrm = f.normal;

                if (cfg.aeromodel == AeroModel.FTwoSidedLiftDrag)
                {
                    nrm *= (btScalar)(Vector3.Dot(nrm, relV) < 0 ? -1 : +1);

                    Vector3 fDrag;
                    Vector3 fLift = Vector3.Zero;

                    btScalar nDotV = nrm.Dot(relVNrm);
                    btScalar triArea = (btScalar)0.5 * f.ra;

                    fDrag = (btScalar)0.5 * kDG * medium.density * relV2 * triArea * nDotV * (-relVNrm);

                    // Check angle of attack: cos(10 deg) = 0.98480
                    if (0 < nDotV && nDotV < (btScalar)0.98480f)
                    {
                        fLift = (btScalar)0.5 * kLF * medium.density * relVLen * triArea *
                            BtMath.Sqrt((btScalar)1.0 - nDotV * nDotV) *
                            nrm.Cross(relVNrm).Cross(relVNrm);
                    }

                    fDrag /= 3;
                    fLift /= 3;

                    for (int j = 0; j < 3; ++j)
                    {
                        if (f.n[j].im > 0)
                        {
                            Vector3 delVByFDrag = fDrag * f.n[j].im * sst.sdt;
                            btScalar delVByFDragLen2 = delVByFDrag.LengthSquared();
                            btScalar vLen2 = f.n[j].v.LengthSquared();

                            if (delVByFDragLen2 >= vLen2 && delVByFDragLen2 > 0)
                            {
                                btScalar delVByFDragLen = delVByFDrag.Length();
                                btScalar vLen = f.n[j].v.Length();
                                fDrag *= (btScalar)0.8 * (vLen / delVByFDragLen);
                            }

                            f.n[j].f += fDrag;
                            f.n[j].f += fLift;
                        }
                    }
                }
                else if (cfg.aeromodel == AeroModel.FOneSided ||
                         cfg.aeromodel == AeroModel.FTwoSided)
                {
                    if (cfg.aeromodel == AeroModel.FTwoSided)
                    {
                        nrm *= (btScalar)(Vector3.Dot(nrm, relV) < 0 ? -1 : +1);
                    }

                    btScalar dvn = Vector3.Dot(relV, nrm);
                    if (dvn > 0)
                    {
                        Vector3 force = Vector3.Zero;
                        btScalar c0 = f.ra * dvn * relV2;
                        btScalar c1 = c0 * medium.density;
                        force += nrm * (-c1 * kLF);
                        force += relV.Normalized() * (-c1 * kDG);
                        force /= 3;
                        for (int j = 0; j < 3; ++j)
                        {
                            SoftBodyInternals.ApplyClampedForce(f.n[j], in force, dt);
                        }
                    }
                }
            }
        }
    }

    public void AddVelocity(in Vector3 velocity)
    {
        for (int i = 0, ni = nodes.Count; i < ni; ++i)
        {
            AddVelocity(in velocity, i);
        }
    }

    public void SetVelocity(in Vector3 velocity)
    {
        for (int i = 0, ni = nodes.Count; i < ni; ++i)
        {
            Node n = nodes[i];
            if (n.im > 0)
            {
                n.v = velocity;
                n.vn = velocity;
            }
        }
    }

    public void AddVelocity(in Vector3 velocity, int node)
    {
        Node n = nodes[node];
        if (n.im > 0)
        {
            n.v += velocity;
        }
    }

    public void SetMass(int node, btScalar mass)
    {
        nodes[node].im = mass > 0 ? 1 / mass : 0;
        bUpdateRtCst = true;
    }

    public btScalar GetMass(int node) =>
        nodes[node].im > 0 ? 1 / nodes[node].im : 0;

    public btScalar GetTotalMass()
    {
        btScalar mass = 0;
        for (int i = 0; i < nodes.Count; ++i)
        {
            mass += GetMass(i);
        }
        return mass;
    }

    public void SetTotalMass(btScalar mass, bool fromfaces = false)
    {
        if (fromfaces)
        {
            for (int i = 0; i < nodes.Count; ++i)
            {
                nodes[i].im = 0;
            }
            for (int i = 0; i < faces.Count; ++i)
            {
                Face f = faces[i];
                btScalar twicearea = SoftBodyInternals.AreaOf(in f.n[0].x, in f.n[1].x, in f.n[2].x);
                for (int j = 0; j < 3; ++j)
                {
                    f.n[j].im += twicearea;
                }
            }
            for (int i = 0; i < nodes.Count; ++i)
            {
                nodes[i].im = 1 / nodes[i].im;
            }
        }
        btScalar tm = GetTotalMass();
        btScalar itm = 1 / tm;
        for (int i = 0; i < nodes.Count; ++i)
        {
            nodes[i].im /= itm * mass;
        }
        bUpdateRtCst = true;
    }

    public void SetTotalDensity(btScalar density) =>
        SetTotalMass(GetVolume() * density, true);

    public void SetVolumeMass(btScalar mass)
    {
        btScalar[] ranks = new btScalar[nodes.Count];

        for (int i = 0; i < nodes.Count; ++i)
        {
            nodes[i].im = 0;
        }
        for (int i = 0; i < tetras.Count; ++i)
        {
            Tetra t = tetras[i];
            for (int j = 0; j < 4; ++j)
            {
                t.n[j].im += BtMath.Abs(t.rv);
                ranks[t.n[j].index] += 1;
            }
        }
        for (int i = 0; i < nodes.Count; ++i)
        {
            if (nodes[i].im > 0)
            {
                nodes[i].im = ranks[i] / nodes[i].im;
            }
        }
        SetTotalMass(mass, false);
    }

    public void SetVolumeDensity(btScalar density)
    {
        btScalar volume = 0;
        for (int i = 0; i < tetras.Count; ++i)
        {
            Tetra t = tetras[i];
            for (int j = 0; j < 4; ++j)
            {
                volume += BtMath.Abs(t.rv);
            }
        }
        SetVolumeMass(volume * density / 6);
    }

    public Vector3 GetLinearVelocity()
    {
        Vector3 totalMomentum = Vector3.Zero;
        for (int i = 0; i < nodes.Count; ++i)
        {
            btScalar mass = nodes[i].im == 0 ? 0 : (btScalar)1.0 / nodes[i].im;
            totalMomentum += mass * nodes[i].v;
        }
        btScalar totalMass = GetTotalMass();
        return totalMass == 0 ? totalMomentum : totalMomentum / totalMass;
    }

    public void SetLinearVelocity(in Vector3 linVel)
    {
        Vector3 oldVel = GetLinearVelocity();
        Vector3 diff = linVel - oldVel;
        for (int i = 0; i < nodes.Count; ++i)
        {
            nodes[i].v += diff;
        }
    }

    public void SetAngularVelocity(in Vector3 angVel)
    {
        Vector3 oldVel = GetLinearVelocity();
        Vector3 com = GetCenterOfMass();
        for (int i = 0; i < nodes.Count; ++i)
        {
            nodes[i].v = angVel.Cross(nodes[i].x - com) + oldVel;
        }
    }

    public void SetWindVelocity(in Vector3 velocity) => windVelocity = velocity;

    public Vector3 GetWindVelocity() => windVelocity;

    public Vector3 GetCenterOfMass() => EvaluateCom();

    public Vector3 EvaluateCom()
    {
        Vector3 com = Vector3.Zero;
        if (pose.bframe)
        {
            for (int i = 0, ni = nodes.Count; i < ni; ++i)
            {
                com += nodes[i].x * pose.wgh[i];
            }
        }
        return com;
    }

    public void Transform(in BulletSharp.Math.Transform trs)
    {
        btScalar margin = collisionShape.Margin;

        for (int i = 0, ni = nodes.Count; i < ni; ++i)
        {
            Node n = nodes[i];
            n.x = trs * n.x;
            n.q = trs * n.q;
            n.n = trs.Basis * n.n;
            DbvtVolume vol = DbvtVolume.FromCR(n.x, margin);
            ndbvt.Update(n.leaf, vol);
        }
        UpdateNormals();
        UpdateBounds();
        UpdateConstants();
    }

    public void Transform(Matrix trs) =>
        Transform(BulletSharp.Math.Transform.FromMatrix(in trs));

    public void Translate(in Vector3 trs)
    {
        var t = BulletSharp.Math.Transform.Identity;
        t.Origin = trs;
        Transform(in t);
    }

    public void Translate(btScalar x, btScalar y, btScalar z) =>
        Translate(new Vector3(x, y, z));

    public void Rotate(in Quaternion rot)
    {
        var t = BulletSharp.Math.Transform.Identity;
        t.SetRotation(rot);
        Transform(in t);
    }

    public void Scale(in Vector3 scl)
    {
        btScalar margin = collisionShape.Margin;

        for (int i = 0, ni = nodes.Count; i < ni; ++i)
        {
            Node n = nodes[i];
            n.x = new Vector3(n.x.X * scl.X, n.x.Y * scl.Y, n.x.Z * scl.Z);
            n.q = new Vector3(n.q.X * scl.X, n.q.Y * scl.Y, n.q.Z * scl.Z);
            DbvtVolume vol = DbvtVolume.FromCR(n.x, margin);
            ndbvt.Update(n.leaf, vol);
        }
        UpdateNormals();
        UpdateBounds();
        UpdateConstants();
        InitializeDmInverse();
    }

    public void Scale(btScalar x, btScalar y, btScalar z) =>
        Scale(new Vector3(x, y, z));

    public btScalar GetRestLengthScale() => restLengthScale;

    public void SetRestLengthScale(btScalar restLengthScale)
    {
        for (int i = 0, ni = links.Count; i < ni; ++i)
        {
            Link l = links[i];
            l.rl = l.rl / this.restLengthScale * restLengthScale;
            l.c1 = l.rl * l.rl;
        }
        this.restLengthScale = restLengthScale;

        if (ActivationState == ActivationState.IslandSleeping)
        {
            Activate();
        }
    }

    public void SetPose(bool bvolume, bool bframe)
    {
        pose.bvolume = bvolume;
        pose.bframe = bframe;
        int ni = nodes.Count;

        // Weights
        btScalar omass = GetTotalMass();
        btScalar kmass = omass * nodes.Count * 1000;
        btScalar tmass = omass;
        ResizeList(pose.wgh, ni);
        for (int i = 0; i < ni; ++i)
        {
            if (nodes[i].im <= 0)
            {
                tmass += kmass;
            }
        }
        for (int i = 0; i < ni; ++i)
        {
            Node n = nodes[i];
            pose.wgh[i] = n.im > 0 ? 1 / (nodes[i].im * tmass) : kmass / tmass;
        }

        // Pos
        Vector3 com = EvaluateCom();
        ResizeList(pose.pos, ni);
        for (int i = 0; i < ni; ++i)
        {
            pose.pos[i] = nodes[i].x - com;
        }
        pose.volume = bvolume ? GetVolume() : 0;
        pose.com = com;
        pose.rot = Matrix3x3.Identity;
        pose.scl = Matrix3x3.Identity;

        // Aqq
        pose.aqq.Row0 = Vector3.Zero;
        pose.aqq.Row1 = Vector3.Zero;
        pose.aqq.Row2 = Vector3.Zero;
        for (int i = 0; i < ni; ++i)
        {
            Vector3 q = pose.pos[i];
            Vector3 mq = pose.wgh[i] * q;
            pose.aqq.Row0 += mq.X * q;
            pose.aqq.Row1 += mq.Y * q;
            pose.aqq.Row2 += mq.Z * q;
        }
        pose.aqq = pose.aqq.Inverse();

        UpdateConstants();
    }

    public void InitPose(bool bvolume, bool bframe) => SetPose(bvolume, bframe);

    public void ResetLinkRestLengths()
    {
        for (int i = 0, ni = links.Count; i < ni; ++i)
        {
            Link l = links[i];
            l.rl = (l.n[0].x - l.n[1].x).Length();
            l.c1 = l.rl * l.rl;
        }
    }

    public btScalar GetVolume()
    {
        btScalar vol = 0;
        if (nodes.Count > 0)
        {
            Vector3 org = nodes[0].x;
            for (int i = 0, ni = faces.Count; i < ni; ++i)
            {
                Face f = faces[i];
                vol += Vector3.Dot(f.n[0].x - org, Vector3.Cross(f.n[1].x - org, f.n[2].x - org));
            }
            vol /= (btScalar)6;
        }
        return vol;
    }

    public int GenerateBendingConstraints(int distance, Material mat = null)
    {
        if (distance > 1)
        {
            // Build graph
            int n = nodes.Count;
            const uint inf = (~0u) >> 1;
            uint[] adj = new uint[n * n];

            for (int j = 0; j < n; ++j)
            {
                for (int i = 0; i < n; ++i)
                {
                    if (i != j)
                    {
                        adj[j * n + i] = adj[i * n + j] = inf;
                    }
                    else
                    {
                        adj[j * n + i] = adj[i * n + j] = 0;
                    }
                }
            }
            for (int i = 0; i < links.Count; ++i)
            {
                int ia = links[i].n[0].index;
                int ib = links[i].n[1].index;
                adj[ib * n + ia] = 1;
                adj[ia * n + ib] = 1;
            }

            // Special optimized case for distance == 2
            if (distance == 2)
            {
                var nodeLinks = new List<int>[n];
                for (int i = 0; i < n; ++i)
                {
                    nodeLinks[i] = new List<int>();
                }

                for (int i = 0; i < links.Count; ++i)
                {
                    int ia = links[i].n[0].index;
                    int ib = links[i].n[1].index;
                    if (!nodeLinks[ia].Contains(ib))
                    {
                        nodeLinks[ia].Add(ib);
                    }
                    if (!nodeLinks[ib].Contains(ia))
                    {
                        nodeLinks[ib].Add(ia);
                    }
                }

                for (int ii = 0; ii < n; ++ii)
                {
                    int i = ii;
                    List<int> linksI = nodeLinks[ii];
                    for (int jj = 0; jj < linksI.Count; ++jj)
                    {
                        int k = linksI[jj];
                        List<int> linksK = nodeLinks[k];
                        for (int kk = 0; kk < linksK.Count; ++kk)
                        {
                            int j = linksK[kk];
                            if (i != j)
                            {
                                uint sum = adj[k * n + i] + adj[j * n + k];
                                Debug.Assert(sum == 2);
                                if (adj[j * n + i] > sum)
                                {
                                    adj[j * n + i] = adj[i * n + j] = sum;
                                }
                            }
                        }
                    }
                }
            }
            else
            {
                // Generic Floyd's algorithm
                for (int k = 0; k < n; ++k)
                {
                    for (int j = 0; j < n; ++j)
                    {
                        for (int i = j + 1; i < n; ++i)
                        {
                            uint sum = adj[k * n + i] + adj[j * n + k];
                            if (adj[j * n + i] > sum)
                            {
                                adj[j * n + i] = adj[i * n + j] = sum;
                            }
                        }
                    }
                }
            }

            // Build links
            int nlinks = 0;
            for (int j = 0; j < n; ++j)
            {
                for (int i = j + 1; i < n; ++i)
                {
                    if (adj[j * n + i] == (uint)distance)
                    {
                        AppendLink(i, j, mat);
                        links[links.Count - 1].bbending = 1;
                        ++nlinks;
                    }
                }
            }
            return nlinks;
        }
        return 0;
    }

    public void RandomizeConstraints()
    {
        uint seed = 243703u;
        for (int i = 0, ni = links.Count; i < ni; ++i)
        {
            unchecked
            {
                seed = 1664525u * seed + 1013904223u;
            }
            int swapIndex = (int)(seed % (uint)ni);
            (links[i], links[swapIndex]) = (links[swapIndex], links[i]);
        }
        for (int i = 0, ni = faces.Count; i < ni; ++i)
        {
            unchecked
            {
                seed = 1664525u * seed + 1013904223u;
            }
            int swapIndex = (int)(seed % (uint)ni);
            (faces[i], faces[swapIndex]) = (faces[swapIndex], faces[i]);
            faces[i].index = i;
            faces[swapIndex].index = swapIndex;
        }
    }

    public void GetAabb(out Vector3 aabbMin, out Vector3 aabbMax)
    {
        aabbMin = bounds[0];
        aabbMax = bounds[1];
    }

    private static void ResizeList<T>(List<T> list, int count)
    {
        if (list.Count > count)
        {
            list.RemoveRange(count, list.Count - count);
        }
        else if (list.Count < count)
        {
            if (list.Capacity < count)
            {
                list.Capacity = count;
            }
            while (list.Count < count)
            {
                list.Add(default);
            }
        }
    }
}
