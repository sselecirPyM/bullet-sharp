using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btDiscreteDynamicsWorld: the discrete rigid body dynamics world.
/// Not ported: actions/character controller (btActionInterface), multithreading, serialization,
/// debugDrawWorld/debugDrawConstraint (the base CollisionWorld does not hold an IDebugDraw yet).
/// </summary>
public class DiscreteDynamicsWorld : DynamicsWorld
{
    /// <summary>gNumClampedCcdMotions (internal debug counter; should stay small)</summary>
    public static int NumClampedCcdMotions;

    protected readonly List<TypedConstraint> sortedConstraints = new();
    internal InplaceSolverIslandCallback solverIslandCallback;

    protected ConstraintSolver constraintSolver;

    protected SimulationIslandManager islandManager;

    protected readonly List<TypedConstraint> constraints = new();

    protected readonly List<RigidBody> nonStaticRigidBodies = new();

    protected Vector3 gravity;

    // Used for variable time steps
    protected btScalar localTime;
    protected btScalar fixedTimeStep;

    protected bool ownsIslandManager;
    protected bool ownsConstraintSolver;
    protected bool synchronizeAllMotionStates;
    protected bool applySpeculativeContactRestitution;

    protected int profileTimings;

    protected bool latencyMotionStateInterpolation = true;

    protected readonly List<PersistentManifold> predictiveManifolds = new();

    // Reused CCD sweep temporaries (stack objects in C++; the world is used single-threaded)
    private ClosestNotMeConvexResultCallback ccdSweepCallback;
    private SphereShape ccdSphere;

    // The base DynamicsWorld does not provide m_solverInfo (a btDynamicsWorld member), so it is held here
    protected readonly ContactSolverInfo solverInfo = new();

    protected object worldUserInfo;

    /// <summary>btDiscreteDynamicsWorld::btDiscreteDynamicsWorld (objects created by the user are not destroyed by this class)</summary>
    public DiscreteDynamicsWorld(Dispatcher dispatcher, BroadphaseInterface pairCache,
        ConstraintSolver constraintSolver, CollisionConfiguration collisionConfiguration)
        : base(dispatcher, pairCache, collisionConfiguration)
    {
        this.constraintSolver = constraintSolver;
        gravity = new Vector3(0, -10, 0);
        localTime = 0;
        fixedTimeStep = 0;
        synchronizeAllMotionStates = false;
        applySpeculativeContactRestitution = false;
        profileTimings = 0;
        latencyMotionStateInterpolation = true;

        if (this.constraintSolver == null)
        {
            this.constraintSolver = new SequentialImpulseConstraintSolver();
            ownsConstraintSolver = true;
        }
        else
        {
            ownsConstraintSolver = false;
        }

        islandManager = new SimulationIslandManager();
        ownsIslandManager = true;

        solverIslandCallback = new InplaceSolverIslandCallback(this.constraintSolver, dispatcher);
    }

    /// <summary>btDynamicsWorld::getSolverInfo (not present in the base layer; held by this class)</summary>
    public ContactSolverInfo SolverInfo => solverInfo;

    /// <summary>btDiscreteDynamicsWorld::saveKinematicState</summary>
    protected virtual void SaveKinematicState(btScalar timeStep)
    {
        // Would like to iterate over nonStaticRigidBodies, but the old API allowed switching
        // state after a kinematic object was added to the world
        for (int i = 0; i < collisionObjects.Count; i++)
        {
            CollisionObject colObj = collisionObjects[i];
            RigidBody body = RigidBody.Upcast(colObj);
            if (body != null && body.activationState1 != ActivationState.IslandSleeping)
            {
                if (body.IsKinematicObject)
                {
                    // Compute velocities for the next frame
                    body.SaveKinematicState(timeStep);
                }
            }
        }
    }

    /// <summary>btDiscreteDynamicsWorld::clearForces</summary>
    public override void ClearForces()
    {
        // @todo: should iterate over active simulation islands
        for (int i = 0; i < nonStaticRigidBodies.Count; i++)
        {
            RigidBody body = nonStaticRigidBodies[i];
            body.ClearForces();
        }
    }

    /// <summary>btDiscreteDynamicsWorld::applyGravity (called once per time step)</summary>
    public virtual void ApplyGravity()
    {
        // @todo: should iterate over active simulation islands
        for (int i = 0; i < nonStaticRigidBodies.Count; i++)
        {
            RigidBody body = nonStaticRigidBodies[i];
            if (body.IsActive)
            {
                body.ApplyGravity();
            }
        }
    }

    /// <summary>btDiscreteDynamicsWorld::synchronizeSingleMotionState</summary>
    public void SynchronizeSingleMotionState(RigidBody body)
    {
        if (body.MotionState != null && !body.IsStaticOrKinematicObject)
        {
            // Sleeping objects must be updated at least once, otherwise the graphics transform is not updated correctly
            TransformUtil.IntegrateTransform(in body.interpolationWorldTransform,
                in body.interpolationLinearVelocity, in body.interpolationAngularVelocity,
                latencyMotionStateInterpolation && fixedTimeStep != 0
                    ? localTime - fixedTimeStep
                    : localTime * body.hitFraction,
                out Transform interpolatedTransform);
            Matrix m = interpolatedTransform.ToMatrix();
            body.MotionState.SetWorldTransform(ref m);
        }
    }

    /// <summary>btDiscreteDynamicsWorld::synchronizeMotionStates</summary>
    public override void SynchronizeMotionStates()
    {
        if (synchronizeAllMotionStates)
        {
            // Iterate over all collision objects
            for (int i = 0; i < collisionObjects.Count; i++)
            {
                CollisionObject colObj = collisionObjects[i];
                RigidBody body = RigidBody.Upcast(colObj);
                if (body != null)
                    SynchronizeSingleMotionState(body);
            }
        }
        else
        {
            // Iterate over all active rigid bodies
            for (int i = 0; i < nonStaticRigidBodies.Count; i++)
            {
                RigidBody body = nonStaticRigidBodies[i];
                if (body.IsActive)
                    SynchronizeSingleMotionState(body);
            }
        }
    }

    /// <summary>btDiscreteDynamicsWorld::stepSimulation (interpolates with a fixed step when maxSubSteps > 0)</summary>
    public override int StepSimulation(btScalar timeStep, int maxSubSteps = 1, btScalar fixedTimeStep = (btScalar)(1.0 / 60.0))
    {
        int numSimulationSubSteps = 0;

        if (maxSubSteps != 0)
        {
            // Fixed time step + interpolation
            this.fixedTimeStep = fixedTimeStep;
            localTime += timeStep;
            if (localTime >= fixedTimeStep)
            {
                numSimulationSubSteps = (int)(localTime / fixedTimeStep);
                localTime -= numSimulationSubSteps * fixedTimeStep;
            }
        }
        else
        {
            // Variable time step
            fixedTimeStep = timeStep;
            localTime = latencyMotionStateInterpolation ? 0 : timeStep;
            this.fixedTimeStep = 0;
            if (BtMath.Abs(timeStep) < BtMath.SimdEpsilon) // btFuzzyZero
            {
                numSimulationSubSteps = 0;
                maxSubSteps = 0;
            }
            else
            {
                numSimulationSubSteps = 1;
                maxSubSteps = 1;
            }
        }

        if (numSimulationSubSteps != 0)
        {
            // Clamp the number of substeps to prevent the simulation from spiraling
            int clampedSimulationSteps = numSimulationSubSteps > maxSubSteps ? maxSubSteps : numSimulationSubSteps;

            SaveKinematicState(fixedTimeStep * clampedSimulationSteps);

            ApplyGravity();

            for (int i = 0; i < clampedSimulationSteps; i++)
            {
                InternalSingleStepSimulation(fixedTimeStep);
                SynchronizeMotionStates();
            }
        }
        else
        {
            SynchronizeMotionStates();
        }

        ClearForces();

        return numSimulationSubSteps;
    }

    /// <summary>btDiscreteDynamicsWorld::internalSingleStepSimulation</summary>
    protected virtual void InternalSingleStepSimulation(btScalar timeStep)
    {
        // Apply gravity and predict motion
        PredictUnconstraintMotion(timeStep);

        DispatcherInfo dispatchInfo = DispatchInfo;
        dispatchInfo.timeStep = timeStep;
        dispatchInfo.stepCount = 0;
        dispatchInfo.debugDraw = debugDrawer;

        CreatePredictiveContacts(timeStep);

        // Perform collision detection
        PerformDiscreteCollisionDetection();

        CalculateSimulationIslands();

        solverInfo.timeStep = timeStep;

        // Solve contacts and other joint constraints
        SolveConstraints(solverInfo);

        // Integrate transforms
        IntegrateTransforms(timeStep);

        // Update vehicle/character simulation (actions not ported; btDiscreteDynamicsWorld::updateActions)
        UpdateActions(timeStep);

        UpdateActivationState(timeStep);
    }

    /// <summary>btDiscreteDynamicsWorld::setGravity</summary>
    public override void SetGravity(ref Vector3 gravity)
    {
        this.gravity = gravity;
        for (int i = 0; i < nonStaticRigidBodies.Count; i++)
        {
            RigidBody body = nonStaticRigidBodies[i];
            if (body.IsActive && (body.Flags & (int)RigidBodyFlags.DisableWorldGravity) == 0)
            {
                body.SetGravity(ref gravity);
            }
        }
    }

    public override Vector3 Gravity
    {
        get => gravity;
        set => SetGravity(ref value);
    }

    /// <summary>btDiscreteDynamicsWorld::addCollisionObject</summary>
    public override void AddCollisionObject(CollisionObject collisionObject,
        CollisionFilterGroups collisionFilterGroup = CollisionFilterGroups.DefaultFilter,
        CollisionFilterGroups collisionFilterMask = CollisionFilterGroups.AllFilter)
    {
        base.AddCollisionObject(collisionObject, collisionFilterGroup, collisionFilterMask);
    }

    /// <summary>btDiscreteDynamicsWorld::removeCollisionObject (checks whether it is a rigid body first)</summary>
    public override void RemoveCollisionObject(CollisionObject collisionObject)
    {
        RigidBody body = RigidBody.Upcast(collisionObject);
        if (body != null)
            RemoveRigidBody(body);
        else
            base.RemoveCollisionObject(collisionObject);
    }

    /// <summary>btDiscreteDynamicsWorld::removeRigidBody</summary>
    public override void RemoveRigidBody(RigidBody body)
    {
        nonStaticRigidBodies.Remove(body);
        base.RemoveCollisionObject(body);
    }

    /// <summary>btDiscreteDynamicsWorld::addRigidBody</summary>
    public override void AddRigidBody(RigidBody body)
    {
        if (!body.IsStaticOrKinematicObject && (body.Flags & (int)RigidBodyFlags.DisableWorldGravity) == 0)
        {
            Vector3 g = gravity;
            body.SetGravity(ref g);
        }

        if (body.CollisionShape != null)
        {
            if (!body.IsStaticObject)
            {
                nonStaticRigidBodies.Add(body);
            }
            else
            {
                body.SetActivationState(ActivationState.IslandSleeping);
            }

            bool isDynamic = !(body.IsStaticObject || body.IsKinematicObject);
            CollisionFilterGroups collisionFilterGroup = isDynamic ? CollisionFilterGroups.DefaultFilter : CollisionFilterGroups.StaticFilter;
            CollisionFilterGroups collisionFilterMask = isDynamic ? CollisionFilterGroups.AllFilter : CollisionFilterGroups.AllFilter ^ CollisionFilterGroups.StaticFilter;

            AddCollisionObject(body, collisionFilterGroup, collisionFilterMask);
        }
    }

    /// <summary>btDiscreteDynamicsWorld::addRigidBody(body, group, mask)</summary>
    public override void AddRigidBody(RigidBody body, CollisionFilterGroups group, CollisionFilterGroups mask)
    {
        if (!body.IsStaticOrKinematicObject && (body.Flags & (int)RigidBodyFlags.DisableWorldGravity) == 0)
        {
            Vector3 g = gravity;
            body.SetGravity(ref g);
        }

        if (body.CollisionShape != null)
        {
            if (!body.IsStaticObject)
            {
                nonStaticRigidBodies.Add(body);
            }
            else
            {
                body.SetActivationState(ActivationState.IslandSleeping);
            }
            AddCollisionObject(body, group, mask);
        }
    }

    /// <summary>btDiscreteDynamicsWorld::updateActions (actions not ported; empty call site kept)</summary>
    protected void UpdateActions(btScalar timeStep)
    {
        // btActionInterface (vehicle/character controllers) is outside the scope of this port
    }

    /// <summary>btDiscreteDynamicsWorld::updateActivationState</summary>
    protected virtual void UpdateActivationState(btScalar timeStep)
    {
        for (int i = 0; i < nonStaticRigidBodies.Count; i++)
        {
            RigidBody body = nonStaticRigidBodies[i];
            if (body != null)
            {
                body.UpdateDeactivation(timeStep);

                if (body.WantsSleeping())
                {
                    if (body.IsStaticOrKinematicObject)
                    {
                        body.SetActivationState(ActivationState.IslandSleeping);
                    }
                    else
                    {
                        if (body.activationState1 == ActivationState.ActiveTag)
                            body.SetActivationState(ActivationState.WantsDeactivation);
                        if (body.activationState1 == ActivationState.IslandSleeping)
                        {
                            body.AngularVelocity = Vector3.Zero;
                            body.LinearVelocity = Vector3.Zero;
                        }
                    }
                }
                else
                {
                    if (body.activationState1 != ActivationState.DisableDeactivation)
                        body.SetActivationState(ActivationState.ActiveTag);
                }
            }
        }
    }

    /// <summary>btDiscreteDynamicsWorld::addConstraint</summary>
    public override void AddConstraint(TypedConstraint constraint, bool disableCollisionsBetweenLinkedBodies = false)
    {
        constraints.Add(constraint);

        if (disableCollisionsBetweenLinkedBodies)
        {
            constraint.rigidBodyA.AddConstraintRef(constraint);
            constraint.rigidBodyB.AddConstraintRef(constraint);
        }
    }

    /// <summary>btDiscreteDynamicsWorld::removeConstraint</summary>
    public override void RemoveConstraint(TypedConstraint constraint)
    {
        constraints.Remove(constraint);
        constraint.rigidBodyA.RemoveConstraintRef(constraint);
        constraint.rigidBodyB.RemoveConstraintRef(constraint);
    }

    /// <summary>btDiscreteDynamicsWorld::solveConstraints</summary>
    public virtual void SolveConstraints(ContactSolverInfo solverInfo)
    {
        sortedConstraints.Clear();
        sortedConstraints.AddRange(constraints);

        // btSortConstraintOnIslandPredicate: sort by island id
        // (btAlignedObjectArray::quickSort; equal keys must permute exactly like native Bullet)
        BtQuickSort.QuickSort(sortedConstraints, static (lhs, rhs) =>
            ConstraintIslandId.GetConstraintIslandId(lhs).CompareTo(ConstraintIslandId.GetConstraintIslandId(rhs)));

        solverIslandCallback.Setup(solverInfo, sortedConstraints, sortedConstraints.Count, dispatcherInfo.debugDraw);
        constraintSolver.PrepareSolve(NumCollisionObjects, Dispatcher.NumManifolds);

        // Solve all constraints of the island
        islandManager.BuildAndProcessIslands(Dispatcher, this, solverIslandCallback);

        solverIslandCallback.ProcessConstraints();

        constraintSolver.AllSolved(solverInfo, dispatcherInfo.debugDraw);
    }

    /// <summary>btDiscreteDynamicsWorld::calculateSimulationIslands</summary>
    protected virtual void CalculateSimulationIslands()
    {
        islandManager.UpdateActivationState(this, Dispatcher);

        {
            // Also merge islands based on predictive contact manifolds
            for (int i = 0; i < predictiveManifolds.Count; i++)
            {
                PersistentManifold manifold = predictiveManifolds[i];

                CollisionObject colObj0 = manifold.body0;
                CollisionObject colObj1 = manifold.body1;

                if (colObj0 != null && !colObj0.IsStaticOrKinematicObject &&
                    colObj1 != null && !colObj1.IsStaticOrKinematicObject)
                {
                    islandManager.unionFind.Unite(colObj0.islandTag1, colObj1.islandTag1);
                }
            }
        }

        {
            int numConstraints = constraints.Count;
            for (int i = 0; i < numConstraints; i++)
            {
                TypedConstraint constraint = constraints[i];
                if (constraint.IsEnabled)
                {
                    RigidBody colObj0 = constraint.rigidBodyA;
                    RigidBody colObj1 = constraint.rigidBodyB;

                    if (colObj0 != null && !colObj0.IsStaticOrKinematicObject &&
                        colObj1 != null && !colObj1.IsStaticOrKinematicObject)
                    {
                        islandManager.unionFind.Unite(colObj0.islandTag1, colObj1.islandTag1);
                    }
                }
            }
        }

        // Store the island id in each body
        islandManager.StoreIslandActivationState(this);
    }

    /// <summary>btClosestNotMeConvexResultCallback (CCD sweep result callback that excludes itself)</summary>
    private sealed class ClosestNotMeConvexResultCallback : ClosestConvexResultCallback
    {
        private CollisionObject me;
        internal btScalar allowedPenetration;
        private OverlappingPairCache pairCache;
        private Dispatcher dispatcher;

        public ClosestNotMeConvexResultCallback(CollisionObject me, Vector3 fromA, Vector3 toA,
            OverlappingPairCache pairCache, Dispatcher dispatcher)
            : base(fromA, toA)
        {
            Set(me, fromA, toA, pairCache, dispatcher, 0);
        }

        /// <summary>Reuses the instance in place (equivalent to C++ constructing it on the stack for each sweep)</summary>
        public void Set(CollisionObject me, Vector3 fromA, Vector3 toA,
            OverlappingPairCache pairCache, Dispatcher dispatcher, btScalar allowedPenetration)
        {
            this.me = me;
            convexFromWorld = fromA;
            convexToWorld = toA;
            closestHitFraction = 1;
            hitCollisionObject = null;
            hitNormalWorld = Vector3.Zero;
            hitPointWorld = Vector3.Zero;
            collisionFilterGroup = CollisionFilterGroups.DefaultFilter;
            collisionFilterMask = CollisionFilterGroups.AllFilter;
            this.allowedPenetration = allowedPenetration;
            this.pairCache = pairCache;
            this.dispatcher = dispatcher;
        }

        public override btScalar AddSingleResult(LocalConvexResult convexResult, bool normalInWorldSpace)
        {
            if (convexResult.collisionObject == me)
                return 1;

            // Ignore when there is no contact response
            if (!convexResult.collisionObject.HasContactResponse)
                return 1;

            Vector3 linVelA = convexToWorld - convexFromWorld;
            Vector3 linVelB = Vector3.Zero; // toB - fromB

            Vector3 relativeVelocity = linVelA - linVelB;
            // Do not report a time of impact when moving away from the contact normal (would cause slight penetration)
            if (convexResult.hitNormalLocal.Dot(relativeVelocity) >= -allowedPenetration)
                return 1;

            return base.AddSingleResult(convexResult, normalInWorldSpace);
        }

        public override bool NeedsCollision(BroadphaseProxy proxy0)
        {
            // Don't collide with itself
            if (proxy0.clientObject == me)
                return false;

            // Don't perform CCD when the collision filters don't match
            if (!base.NeedsCollision(proxy0))
                return false;
            // C++ additionally checks pairCache->getOverlapFilterCallback() (no such callback in the base layer yet)

            CollisionObject otherObj = proxy0.clientObject;

            if (!dispatcher.NeedsCollision(me, otherObj))
                return false;

            // Call needsResponse (see bullet issue #179)
            if (dispatcher.NeedsResponse(me, otherObj))
            {
                return true;
            }

            return false;
        }
    }

    /// <summary>btDiscreteDynamicsWorld::createPredictiveContactsInternal</summary>
    private void CreatePredictiveContactsInternal(List<RigidBody> bodies, int numBodies, btScalar timeStep)
    {
        for (int i = 0; i < numBodies; i++)
        {
            RigidBody body = bodies[i];
            body.hitFraction = 1;

            if (body.IsActive && !body.IsStaticOrKinematicObject)
            {
                if (DispatchInfo.useContinuous)
                {
                    body.PredictIntegratedTransformInternal(timeStep, out Transform predictedTrans);

                    btScalar squareMotion = (predictedTrans.Origin - body.worldTransform.Origin).LengthSquared();

                    if (body.CcdSquareMotionThreshold != 0 &&
                        body.CcdSquareMotionThreshold < squareMotion)
                    {
                        if (body.CollisionShape.IsConvex)
                        {
                            NumClampedCcdMotions++;
                            // Reuse the callback and sweep sphere (stack objects in C++)
                            ccdSweepCallback ??= new ClosestNotMeConvexResultCallback(body,
                                body.worldTransform.Origin, predictedTrans.Origin,
                                Broadphase.OverlappingPairCache, Dispatcher);
                            ccdSweepCallback.Set(body, body.worldTransform.Origin, predictedTrans.Origin,
                                Broadphase.OverlappingPairCache, Dispatcher, DispatchInfo.allowedCcdPenetration);

                            ccdSphere ??= new SphereShape(0);
                            ccdSphere.Margin = body.CcdSweptSphereRadius;

                            ccdSweepCallback.CollisionFilterGroup = body.BroadphaseProxy.collisionFilterGroup;
                            ccdSweepCallback.CollisionFilterMask = body.BroadphaseProxy.collisionFilterMask;
                            Transform modifiedPredictedTrans = predictedTrans;
                            modifiedPredictedTrans.Basis = body.worldTransform.Basis;

                            Matrix fromM = body.worldTransform.ToMatrix();
                            Matrix toM = modifiedPredictedTrans.ToMatrix();
                            ConvexSweepTest(ccdSphere, fromM, toM, ccdSweepCallback);
                            if (ccdSweepCallback.HasHit && ccdSweepCallback.closestHitFraction < 1)
                            {
                                Vector3 distVec = (predictedTrans.Origin - body.worldTransform.Origin) * ccdSweepCallback.closestHitFraction;
                                btScalar distance = distVec.Dot(-ccdSweepCallback.hitNormalWorld);

                                PersistentManifold manifold = dispatcher1.GetNewManifold(body, ccdSweepCallback.hitCollisionObject);
                                predictiveManifolds.Add(manifold);

                                Vector3 worldPointB = body.worldTransform.Origin + distVec;
                                Vector3 localPointB = ccdSweepCallback.hitCollisionObject.worldTransform.Inverse() * worldPointB;

                                var newPoint = new ManifoldPoint(Vector3.Zero, localPointB, ccdSweepCallback.hitNormalWorld, distance);

                                int index = manifold.AddManifoldPoint(newPoint, true);
                                ManifoldPoint pt = manifold.GetContactPoint(index);
                                pt.combinedRestitution = 0;
                                pt.combinedFriction = ManifoldResult.CalculateCombinedFriction(body, ccdSweepCallback.hitCollisionObject);
                                pt.positionWorldOnA = body.worldTransform.Origin;
                                pt.positionWorldOnB = worldPointB;
                            }
                        }
                    }
                }
            }
        }
    }

    /// <summary>btDiscreteDynamicsWorld::releasePredictiveContacts</summary>
    private void ReleasePredictiveContacts()
    {
        for (int i = 0; i < predictiveManifolds.Count; i++)
        {
            PersistentManifold manifold = predictiveManifolds[i];
            dispatcher1.ReleaseManifold(manifold);
        }
        predictiveManifolds.Clear();
    }

    /// <summary>btDiscreteDynamicsWorld::createPredictiveContacts</summary>
    protected virtual void CreatePredictiveContacts(btScalar timeStep)
    {
        ReleasePredictiveContacts();
        if (nonStaticRigidBodies.Count > 0)
        {
            CreatePredictiveContactsInternal(nonStaticRigidBodies, nonStaticRigidBodies.Count, timeStep);
        }
    }

    /// <summary>btDiscreteDynamicsWorld::integrateTransformsInternal</summary>
    private void IntegrateTransformsInternal(List<RigidBody> bodies, int numBodies, btScalar timeStep)
    {
        for (int i = 0; i < numBodies; i++)
        {
            RigidBody body = bodies[i];
            body.hitFraction = 1;

            if (body.IsActive && !body.IsStaticOrKinematicObject)
            {
                body.PredictIntegratedTransformInternal(timeStep, out Transform predictedTrans);

                btScalar squareMotion = (predictedTrans.Origin - body.worldTransform.Origin).LengthSquared();

                if (DispatchInfo.useContinuous && body.CcdSquareMotionThreshold != 0 &&
                    body.CcdSquareMotionThreshold < squareMotion)
                {
                    if (body.CollisionShape.IsConvex)
                    {
                        NumClampedCcdMotions++;
                        // Reuse the callback and sweep sphere (stack objects in C++)
                        ccdSweepCallback ??= new ClosestNotMeConvexResultCallback(body,
                            body.worldTransform.Origin, predictedTrans.Origin,
                            Broadphase.OverlappingPairCache, Dispatcher);
                        ccdSweepCallback.Set(body, body.worldTransform.Origin, predictedTrans.Origin,
                            Broadphase.OverlappingPairCache, Dispatcher, DispatchInfo.allowedCcdPenetration);

                        ccdSphere ??= new SphereShape(0);
                        ccdSphere.Margin = body.CcdSweptSphereRadius;

                        ccdSweepCallback.CollisionFilterGroup = body.BroadphaseProxy.collisionFilterGroup;
                        ccdSweepCallback.CollisionFilterMask = body.BroadphaseProxy.collisionFilterMask;
                        Transform modifiedPredictedTrans = predictedTrans;
                        modifiedPredictedTrans.Basis = body.worldTransform.Basis;

                        Matrix fromM = body.worldTransform.ToMatrix();
                        Matrix toM = modifiedPredictedTrans.ToMatrix();
                        ConvexSweepTest(ccdSphere, fromM, toM, ccdSweepCallback);
                        if (ccdSweepCallback.HasHit && ccdSweepCallback.closestHitFraction < 1)
                        {
                            body.hitFraction = ccdSweepCallback.closestHitFraction;
                            body.PredictIntegratedTransformInternal(timeStep * body.hitFraction, out predictedTrans);
                            body.hitFraction = 0;
                            body.ProceedToTransformInternal(in predictedTrans);

                            // Do not apply collision response yet; handled next frame (a zero-restitution
                            // resolveSingleCollision can be enabled here if needed)
                            continue;
                        }
                    }
                }

                body.ProceedToTransformInternal(in predictedTrans);
            }
        }
    }

    /// <summary>btDiscreteDynamicsWorld::integrateTransforms</summary>
    protected virtual void IntegrateTransforms(btScalar timeStep)
    {
        if (nonStaticRigidBodies.Count > 0)
        {
            IntegrateTransformsInternal(nonStaticRigidBodies, nonStaticRigidBodies.Count, timeStep);
        }

        // This switch is off by default (not sufficiently tested yet)
        if (applySpeculativeContactRestitution)
        {
            for (int i = 0; i < predictiveManifolds.Count; i++)
            {
                PersistentManifold manifold = predictiveManifolds[i];
                RigidBody body0 = RigidBody.Upcast(manifold.body0);
                RigidBody body1 = RigidBody.Upcast(manifold.body1);

                for (int p = 0; p < manifold.NumContacts; p++)
                {
                    ManifoldPoint pt = manifold.GetContactPoint(p);
                    btScalar combinedRestitution = ManifoldResult.CalculateCombinedRestitution(body0, body1);

                    if (combinedRestitution > 0 && pt.appliedImpulse != 0)
                    {
                        Vector3 imp = -pt.normalWorldOnB * pt.appliedImpulse * combinedRestitution;

                        Vector3 pos1 = pt.positionWorldOnA;
                        Vector3 pos2 = pt.positionWorldOnB;

                        Vector3 relPos0 = pos1 - body0.worldTransform.Origin;
                        Vector3 relPos1 = pos2 - body1.worldTransform.Origin;

                        if (body0 != null)
                            body0.ApplyImpulse(ref imp, ref relPos0);
                        if (body1 != null)
                        {
                            Vector3 negImp = -imp;
                            body1.ApplyImpulse(ref negImp, ref relPos1);
                        }
                    }
                }
            }
        }
    }

    /// <summary>btDiscreteDynamicsWorld::predictUnconstraintMotion</summary>
    protected virtual void PredictUnconstraintMotion(btScalar timeStep)
    {
        for (int i = 0; i < nonStaticRigidBodies.Count; i++)
        {
            RigidBody body = nonStaticRigidBodies[i];
            if (!body.IsStaticOrKinematicObject)
            {
                // Velocities are not integrated/updated here; that is done by the constraint solver
                body.ApplyDamping(timeStep);

                body.PredictIntegratedTransformInternal(timeStep, out Transform predicted);
                body.interpolationWorldTransform = predicted;
            }
        }
    }

    /// <summary>btDiscreteDynamicsWorld::setConstraintSolver</summary>
    public override ConstraintSolver Solver
    {
        get => constraintSolver;
        set
        {
            ownsConstraintSolver = false;
            constraintSolver = value;
            solverIslandCallback.solver = value;
        }
    }

    public override int NumConstraints => constraints.Count;

    public override TypedConstraint GetConstraint(int index) => constraints[index];

    public SimulationIslandManager SimulationIslandManager => islandManager;

    public List<RigidBody> NonStaticRigidBodies => nonStaticRigidBodies;

    public override object WorldUserInfo
    {
        get => worldUserInfo;
        set => worldUserInfo = value;
    }

    public bool SynchronizeAllMotionStates
    {
        get => synchronizeAllMotionStates;
        set => synchronizeAllMotionStates = value;
    }

    public bool ApplySpeculativeContactRestitution
    {
        get => applySpeculativeContactRestitution;
        set => applySpeculativeContactRestitution = value;
    }

    /// <summary>btDiscreteDynamicsWorld::setLatencyMotionStateInterpolation
    /// (interpolates between the previous and current frame, mitigating render jitter caused by penetration)</summary>
    public bool LatencyMotionStateInterpolation
    {
        get => latencyMotionStateInterpolation;
        set => latencyMotionStateInterpolation = value;
    }

    public override void Dispose()
    {
        // No native resources (the C++ destructor only frees the owned islandManager/solver; in C# the GC handles it)
    }
}
