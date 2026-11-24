using System.Diagnostics;
using BulletSharp.Math;

namespace BulletSharp.SoftBody;

/// <summary>
/// Ported from BulletSoftBody/btSoftRigidDynamicsWorld.h and btSoftRigidDynamicsWorld.cpp.
/// </summary>
public class SoftRigidDynamicsWorld : DiscreteDynamicsWorld
{
    private readonly List<SoftBody> softBodies = new();
    private DrawFlags drawFlags;
    private bool drawNodeTree;
    private bool drawFaceTree;
    private bool drawClusterTree;
    private readonly SoftBodyWorldInfo worldInfo;
    private SoftBodySolver softBodySolver;
    private readonly bool ownsSolver;
    private SoftSingleRayCallback softSingleRayCallback;

    public SoftRigidDynamicsWorld(
        Dispatcher dispatcher,
        BroadphaseInterface pairCache,
        ConstraintSolver constraintSolver,
        CollisionConfiguration collisionConfiguration,
        SoftBodySolver softBodySolver = null)
        : base(dispatcher, pairCache, constraintSolver, collisionConfiguration)
    {
        if (softBodySolver == null)
        {
            this.softBodySolver = new DefaultSoftBodySolver();
            ownsSolver = true;
        }
        else
        {
            this.softBodySolver = softBodySolver;
            ownsSolver = false;
        }

        drawFlags = DrawFlags.Std;
        drawNodeTree = true;
        drawFaceTree = false;
        drawClusterTree = false;

        worldInfo = new SoftBodyWorldInfo
        {
            broadphase = pairCache,
            dispatcher = dispatcher,
            airDensity = (btScalar)1.2,
            waterDensity = 0,
            waterOffset = 0,
            waterNormal = Vector3.Zero,
            gravity = new Vector3(0, -10, 0),
        };
        worldInfo.sparseSdf.Initialize();
        worldInfo.sparseSdf.Reset();
        worldInfo.sparseSdf.Initialize();
    }

    public DrawFlags DrawFlags
    {
        get => drawFlags;
        set => drawFlags = value;
    }

    public bool DrawNodeTree
    {
        get => drawNodeTree;
        set => drawNodeTree = value;
    }

    public bool DrawFaceTree
    {
        get => drawFaceTree;
        set => drawFaceTree = value;
    }

    public bool DrawClusterTree
    {
        get => drawClusterTree;
        set => drawClusterTree = value;
    }

    public SoftBodyWorldInfo WorldInfo => worldInfo;

    public override DynamicsWorldType WorldType => DynamicsWorldType.SoftRigid;

    public List<SoftBody> SoftBodyArray => softBodies;

    public SoftBodySolver SoftBodySolver => softBodySolver;

    protected override void PredictUnconstraintMotion(btScalar timeStep)
    {
        base.PredictUnconstraintMotion(timeStep);
        softBodySolver.PredictMotion(timeStep);
    }

    protected override void InternalSingleStepSimulation(btScalar timeStep)
    {
        // Let the solver grab the soft bodies and if necessary optimize for it
        softBodySolver.Optimize(softBodies);

        if (!softBodySolver.CheckInitialized())
        {
            Debug.Assert(false, "Solver initialization failed");
        }

        base.InternalSingleStepSimulation(timeStep);

        // Solve soft bodies constraints
        SolveSoftBodiesConstraints(timeStep);

        // Self collisions
        for (int i = 0; i < softBodies.Count; i++)
        {
            SoftBody psb = softBodies[i];
            psb.DefaultCollisionHandler(psb);
        }

        // Update soft bodies
        softBodySolver.UpdateSoftBodies();
    }

    protected virtual void SolveSoftBodiesConstraints(btScalar timeStep)
    {
        if (softBodies.Count > 0)
        {
            SoftBody.SolveClusters(softBodies);
        }

        // Solve constraints solver-wise
        softBodySolver.SolveConstraints(timeStep * softBodySolver.TimeScale);

        worldInfo.sparseSdf.GarbageCollect();
    }

    public void AddSoftBody(
        SoftBody body,
        CollisionFilterGroups collisionFilterGroup = CollisionFilterGroups.DefaultFilter,
        CollisionFilterGroups collisionFilterMask = CollisionFilterGroups.AllFilter)
    {
        softBodies.Add(body);

        // Set the soft body solver that will deal with this body to be the world's solver
        body.SoftBodySolver = softBodySolver;

        base.AddCollisionObject(body, collisionFilterGroup, collisionFilterMask);
    }

    public void AddSoftBody(SoftBody body, int collisionFilterGroup, int collisionFilterMask) =>
        AddSoftBody(body, (CollisionFilterGroups)collisionFilterGroup, (CollisionFilterGroups)collisionFilterMask);

    public void RemoveSoftBody(SoftBody body)
    {
        softBodies.Remove(body);
        base.RemoveCollisionObject(body);
    }

    public override void RemoveRigidBody(RigidBody body)
    {
        if (body?.CollisionShape != null)
        {
            worldInfo.sparseSdf.RemoveReferences(body.CollisionShape);
        }
        base.RemoveRigidBody(body);
    }

    public override void RemoveCollisionObject(CollisionObject collisionObject)
    {
        SoftBody body = SoftBody.Upcast(collisionObject);
        if (body != null)
        {
            RemoveSoftBody(body);
        }
        else
        {
            if (collisionObject?.CollisionShape != null && RigidBody.Upcast(collisionObject) == null)
            {
                worldInfo.sparseSdf.RemoveReferences(collisionObject.CollisionShape);
            }
            base.RemoveCollisionObject(collisionObject);
        }
    }

    public override void DebugDrawWorld()
    {
        base.DebugDrawWorld();

        IDebugDraw drawer = DebugDrawer;
        if (drawer != null)
        {
            for (int i = 0; i < softBodies.Count; i++)
            {
                SoftBody psb = softBodies[i];
                if ((drawer.DebugMode & DebugDrawModes.DrawWireframe) != 0)
                {
                    SoftBodyHelpers.DrawFrame(psb, drawer);
                    SoftBodyHelpers.Draw(psb, drawer, drawFlags);
                }

                if ((drawer.DebugMode & DebugDrawModes.DrawAabb) != 0)
                {
                    if (drawNodeTree) SoftBodyHelpers.DrawNodeTree(psb, drawer);
                    if (drawFaceTree) SoftBodyHelpers.DrawFaceTree(psb, drawer);
                    if (drawClusterTree) SoftBodyHelpers.DrawClusterTree(psb, drawer);
                }
            }
        }
    }

    public override void RayTest(Vector3 rayFromWorld, Vector3 rayToWorld, RayResultCallback resultCallback)
    {
        softSingleRayCallback ??= new SoftSingleRayCallback();
        softSingleRayCallback.Set(in rayFromWorld, in rayToWorld, this, resultCallback);
        broadphasePairCache.RayTest(rayFromWorld, rayToWorld, softSingleRayCallback);
    }

    public override void RayTestSingle(
        Matrix rayFromWorld,
        Matrix rayToWorld,
        CollisionObject collisionObject,
        CollisionShape collisionShape,
        Matrix colObjWorldTransform,
        RayResultCallback resultCallback)
    {
        RayTestSingleInternal(
            Transform.FromMatrix(in rayFromWorld),
            Transform.FromMatrix(in rayToWorld),
            collisionObject,
            collisionShape,
            Transform.FromMatrix(in colObjWorldTransform),
            resultCallback);
    }

    internal override void RayTestSingleInternal(
        in Transform rayFromTrans,
        in Transform rayToTrans,
        CollisionObject collisionObject,
        CollisionShape collisionShape,
        in Transform colObjWorldTransform,
        RayResultCallback resultCallback)
    {
        if (collisionShape.IsSoftBody)
        {
            SoftBody softBody = SoftBody.Upcast(collisionObject);
            if (softBody != null)
            {
                sRayCast softResult = default;
                Vector3 rayFrom = rayFromTrans.Origin;
                Vector3 rayTo = rayToTrans.Origin;
                if (softBody.RayTest(in rayFrom, in rayTo, ref softResult))
                {
                    if (softResult.fraction <= resultCallback.closestHitFraction)
                    {
                        var shapeInfo = new LocalShapeInfo
                        {
                            shapePart = 0,
                            triangleIndex = softResult.index,
                        };

                        // Get the normal
                        Vector3 rayDir = rayTo - rayFrom;
                        Vector3 normal = -rayDir;
                        normal.Normalize();

                        if (softResult.feature == FeatureType.Face)
                        {
                            normal = softBody.faces[softResult.index].normal;
                            if (normal.Dot(rayDir) > 0)
                            {
                                // Normal always points toward origin of the ray
                                normal = -normal;
                            }
                        }

                        var rayResult = new LocalRayResult(
                            collisionObject,
                            shapeInfo,
                            normal,
                            softResult.fraction);
                        resultCallback.AddSingleResult(rayResult, true);
                    }
                }
            }
        }
        else
        {
            base.RayTestSingleInternal(
                in rayFromTrans,
                in rayToTrans,
                collisionObject,
                collisionShape,
                in colObjWorldTransform,
                resultCallback);
        }
    }

    public override void Dispose()
    {
        if (ownsSolver && softBodySolver != null)
        {
            softBodySolver.Dispose();
            softBodySolver = null;
        }
        worldInfo.Dispose();
        base.Dispose();
    }

    private sealed class SoftSingleRayCallback : BroadphaseRayCallback
    {
        private Transform rayFromTrans;
        private Transform rayToTrans;
        private SoftRigidDynamicsWorld world;
        private RayResultCallback resultCallback;

        public void Set(
            in Vector3 rayFromWorld,
            in Vector3 rayToWorld,
            SoftRigidDynamicsWorld world,
            RayResultCallback resultCallback)
        {
            this.world = world;
            this.resultCallback = resultCallback;

            rayFromTrans = Transform.Identity;
            rayFromTrans.Origin = rayFromWorld;
            rayToTrans = Transform.Identity;
            rayToTrans.Origin = rayToWorld;

            Vector3 rayDir = rayToWorld - rayFromWorld;
            rayDir.Normalize();
            rayDirectionInverse = new Vector3(
                rayDir.X == 0 ? (btScalar)1e30 : (btScalar)1.0 / rayDir.X,
                rayDir.Y == 0 ? (btScalar)1e30 : (btScalar)1.0 / rayDir.Y,
                rayDir.Z == 0 ? (btScalar)1e30 : (btScalar)1.0 / rayDir.Z);
            signs[0] = rayDirectionInverse.X < 0 ? 1 : 0;
            signs[1] = rayDirectionInverse.Y < 0 ? 1 : 0;
            signs[2] = rayDirectionInverse.Z < 0 ? 1 : 0;

            lambdaMax = rayDir.Dot(rayToWorld - rayFromWorld);
        }

        public override bool Process(BroadphaseProxy proxy)
        {
            if (resultCallback.closestHitFraction == 0)
            {
                return false;
            }

            CollisionObject collisionObject = proxy.clientObject;
            if (resultCallback.NeedsCollision(collisionObject.BroadphaseHandle))
            {
                world.RayTestSingleInternal(
                    in rayFromTrans,
                    in rayToTrans,
                    collisionObject,
                    collisionObject.CollisionShape,
                    in collisionObject.worldTransform,
                    resultCallback);
            }
            return true;
        }
    }
}
