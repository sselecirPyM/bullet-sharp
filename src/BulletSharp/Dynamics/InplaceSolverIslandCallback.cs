using BulletSharp.Math;

namespace BulletSharp;

/// <summary>btGetConstraintIslandId</summary>
internal static class ConstraintIslandId
{
    public static int GetConstraintIslandId(TypedConstraint lhs)
    {
        CollisionObject rcolObj0 = lhs.rigidBodyA;
        CollisionObject rcolObj1 = lhs.rigidBodyB;
        return rcolObj0.islandTag1 >= 0 ? rcolObj0.islandTag1 : rcolObj1.islandTag1;
    }
}

/// <summary>
/// Ported from InplaceSolverIslandCallback (inside btDiscreteDynamicsWorld.cpp):
/// feeds the ready bodies/manifolds/constraints of each island to the solver.
/// </summary>
internal sealed class InplaceSolverIslandCallback : ISimulationIslandCallback
{

    internal ContactSolverInfo solverInfo;
    internal ConstraintSolver solver;
    internal List<TypedConstraint> sortedConstraints;
    internal int numConstraints;
    internal IDebugDraw debugDrawer;
    internal Dispatcher dispatcher;

    internal readonly List<CollisionObject> bodies = new();
    internal readonly List<PersistentManifold> manifolds = new();
    internal readonly List<TypedConstraint> constraints = new();

    /// <summary>Reusable sub-list for an island's manifolds</summary>
    private readonly List<PersistentManifold> islandManifolds = new();

    public InplaceSolverIslandCallback(ConstraintSolver solver, Dispatcher dispatcher)
    {
        this.solver = solver;
        this.dispatcher = dispatcher;
    }

    /// <summary>InplaceSolverIslandCallback::setup</summary>
    public void Setup(ContactSolverInfo solverInfo, List<TypedConstraint> sortedConstraints, int numConstraints, IDebugDraw debugDrawer)
    {
        this.solverInfo = solverInfo;
        this.sortedConstraints = sortedConstraints;
        this.numConstraints = numConstraints;
        this.debugDrawer = debugDrawer;
        bodies.Clear();
        manifolds.Clear();
        constraints.Clear();
    }

    /// <summary>InplaceSolverIslandCallback::processIsland</summary>
    public void ProcessIsland(List<CollisionObject> islandBodies, int numBodies,
        List<PersistentManifold> islandmanifold, int startManifoldIndex, int numManifolds, int islandId)
    {
        // Assemble the manifold sub-list for this island (replaces the C++ pointer slice)
        islandManifolds.Clear();
        for (int i = 0; i < numManifolds; i++)
            islandManifolds.Add(islandmanifold[startManifoldIndex + i]);

        if (islandId < 0)
        {
            // No island splitting: all constraints/contact manifolds/bodies go directly to the solver
        solver.SolveGroup(islandBodies, numBodies, islandManifolds, numManifolds,
            sortedConstraints, numConstraints, solverInfo, debugDrawer, dispatcher);
        }
        else
        {
            // Also add all non-contact constraints/joints of this island
            int startConstraint = -1;
            int numCurConstraints = 0;
            int i;

            // Find the first constraint of this island (sortedConstraints is sorted by island id)
            for (i = 0; i < numConstraints; i++)
            {
                if (ConstraintIslandId.GetConstraintIslandId(sortedConstraints[i]) == islandId)
                {
                    startConstraint = i;
                    break;
                }
            }
            // Count the constraints of this island
            if (startConstraint >= 0)
            {
                for (; i < numConstraints; i++)
                {
                    if (ConstraintIslandId.GetConstraintIslandId(sortedConstraints[i]) == islandId)
                    {
                        numCurConstraints++;
                    }
                }
            }

            if (solverInfo.minimumSolverBatchSize <= 1)
            {
                // Solve directly: build the constraint sub-list for this island
                islandConstraints.Clear();
                for (i = 0; i < numCurConstraints; i++)
                    islandConstraints.Add(sortedConstraints[startConstraint + i]);
                solver.SolveGroup(islandBodies, numBodies, islandManifolds, numManifolds,
                    islandConstraints, numCurConstraints, solverInfo, debugDrawer, dispatcher);
            }
            else
            {
                for (i = 0; i < numBodies; i++)
                    bodies.Add(islandBodies[i]);
                for (i = 0; i < numManifolds; i++)
                    manifolds.Add(islandManifolds[i]);
                for (i = 0; i < numCurConstraints; i++)
                    constraints.Add(sortedConstraints[startConstraint + i]);
                if (constraints.Count + manifolds.Count > solverInfo.minimumSolverBatchSize)
                {
                    ProcessConstraints();
                }
            }
        }
    }

    private readonly List<TypedConstraint> islandConstraints = new();

    /// <summary>InplaceSolverIslandCallback::processConstraints</summary>
    public void ProcessConstraints()
    {
        solver.SolveGroup(bodies, bodies.Count, manifolds, manifolds.Count,
            constraints, constraints.Count, solverInfo, debugDrawer, dispatcher);
        bodies.Clear();
        manifolds.Clear();
        constraints.Clear();
    }
}
