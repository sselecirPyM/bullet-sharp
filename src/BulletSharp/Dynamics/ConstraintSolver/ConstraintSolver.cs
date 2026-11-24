namespace BulletSharp;

/// <summary>btConstraintSolverType</summary>
public enum ConstraintSolverType
{
    SequentialImpulseSolver = 1,
    MlcpSolver = 2,
    MultiBodySolver = 4,
}

/// <summary>
/// Ported from btConstraintSolver. Exposed in BulletSharp as the abstract ConstraintSolver class.
/// </summary>
public abstract class ConstraintSolver : IDisposable
{
    public virtual void PrepareSolve(int numBodies, int numManifolds)
    {
    }

    public virtual btScalar SolveGroup(List<CollisionObject> bodies, int numBodies,
        List<PersistentManifold> manifold, int numManifolds,
        List<TypedConstraint> constraints, int numConstraints,
        ContactSolverInfo info, IDebugDraw debugDrawer, Dispatcher dispatcher)
    {
        return 0;
    }

    public virtual void AllSolved(ContactSolverInfo info, IDebugDraw debugDrawer)
    {
    }

    public abstract btScalar SolveGroupCacheFriendlySetup(List<CollisionObject> bodies, int numBodies,
        List<PersistentManifold> manifoldPtr, int numManifolds,
        List<TypedConstraint> constraints, int numConstraints,
        ContactSolverInfo infoGlobal, IDebugDraw debugDrawer);

    public abstract btScalar SolveGroupCacheFriendlyIterations(List<CollisionObject> bodies, int numBodies,
        List<PersistentManifold> manifoldPtr, int numManifolds,
        List<TypedConstraint> constraints, int numConstraints,
        ContactSolverInfo infoGlobal, IDebugDraw debugDrawer);

    public abstract btScalar SolveGroupCacheFriendlyFinish(List<CollisionObject> bodies, int numBodies,
        ContactSolverInfo infoGlobal);

    public abstract void Reset();

    public abstract ConstraintSolverType SolverType { get; }

    public virtual void Dispose()
    {
    }
}
