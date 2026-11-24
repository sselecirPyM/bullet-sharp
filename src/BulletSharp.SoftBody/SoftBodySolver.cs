namespace BulletSharp.SoftBody;

/// <summary>Ported from btSoftBodySolver::SolverTypes.</summary>
public enum SolverTypes
{
    Default,
    Cpu,
    CL,
    CLSimd,
    DX,
    DXSimd,
}

/// <summary>Ported from btVertexBufferDescriptor::BufferTypes.</summary>
public enum BufferTypes
{
    Cpu,
    DX11,
    OpenGL,
}

/// <summary>
/// Ported from BulletSoftBody/btSoftBodySolverVertexBuffer.h (btVertexBufferDescriptor).
/// </summary>
public abstract class VertexBufferDescriptor : IDisposable
{
    protected bool hasVertexPositions;
    protected bool hasNormals;
    protected int vertexOffset;
    protected int vertexStride;
    protected int normalOffset;
    protected int normalStride;

    protected VertexBufferDescriptor()
    {
        hasVertexPositions = false;
        hasNormals = false;
        vertexOffset = 0;
        vertexStride = 0;
        normalOffset = 0;
        normalStride = 0;
    }

    public virtual bool HasVertexPositions => hasVertexPositions;

    public virtual bool HasNormals => hasNormals;

    public abstract BufferTypes BufferType { get; }

    /// <summary>Vertex offset in floats from the base pointer.</summary>
    public virtual int VertexOffset => vertexOffset;

    /// <summary>Vertex stride in number of floats between vertices.</summary>
    public virtual int VertexStride => vertexStride;

    /// <summary>Normal offset in floats from the base pointer.</summary>
    public virtual int NormalOffset => normalOffset;

    /// <summary>Normal stride in number of floats between vertices.</summary>
    public virtual int NormalStride => normalStride;

    public void Dispose()
    {
        Dispose(true);
        GC.SuppressFinalize(this);
    }

    protected virtual void Dispose(bool disposing)
    {
    }
}

/// <summary>
/// Pure managed CPU vertex buffer descriptor backed by a <see cref="float"/> array.
/// Ported from BulletSoftBody/btSoftBodySolverVertexBuffer.h (btCPUVertexBufferDescriptor).
/// </summary>
public class CpuVertexBufferDescriptor : VertexBufferDescriptor
{
    protected float[] basePointer;

    public CpuVertexBufferDescriptor(float[] basePointer, int vertexOffset, int vertexStride)
    {
        this.basePointer = basePointer;
        this.vertexOffset = vertexOffset;
        this.vertexStride = vertexStride;
        hasVertexPositions = true;
    }

    public CpuVertexBufferDescriptor(
        float[] basePointer,
        int vertexOffset,
        int vertexStride,
        int normalOffset,
        int normalStride)
    {
        this.basePointer = basePointer;
        this.vertexOffset = vertexOffset;
        this.vertexStride = vertexStride;
        hasVertexPositions = true;
        this.normalOffset = normalOffset;
        this.normalStride = normalStride;
        hasNormals = true;
    }

    public override BufferTypes BufferType => BufferTypes.Cpu;

    public virtual float[] BasePointer => basePointer;
}

/// <summary>
/// Ported from BulletSoftBody/btSoftBodySolvers.h (btSoftBodySolver).
/// </summary>
public abstract class SoftBodySolver : IDisposable
{
    protected int numberOfPositionIterations;
    protected int numberOfVelocityIterations;
    protected btScalar timeScale;

    protected SoftBodySolver()
    {
        numberOfVelocityIterations = 0;
        numberOfPositionIterations = 5;
        timeScale = 1;
    }

    public abstract SolverTypes SolverType { get; }

    public virtual int NumberOfPositionIterations
    {
        get => numberOfPositionIterations;
        set => numberOfPositionIterations = value;
    }

    public virtual int NumberOfVelocityIterations
    {
        get => numberOfVelocityIterations;
        set => numberOfVelocityIterations = value;
    }

    public btScalar TimeScale
    {
        get => timeScale;
        protected set => timeScale = value;
    }

    public abstract bool CheckInitialized();

    public abstract void Optimize(List<SoftBody> softBodies, bool forceUpdate = false);

    public abstract void CopyBackToSoftBodies(bool bMove = true);

    public abstract void PredictMotion(btScalar solverdt);

    public abstract void SolveConstraints(btScalar solverdt);

    public abstract void UpdateSoftBodies();

    public abstract void ProcessCollision(SoftBody softBody, CollisionObjectWrapper collisionObjectWrap);

    public abstract void ProcessCollision(SoftBody softBody, SoftBody otherSoftBody);

    public void Dispose()
    {
        Dispose(true);
        GC.SuppressFinalize(this);
    }

    protected virtual void Dispose(bool disposing)
    {
    }
}

/// <summary>
/// Ported from BulletSoftBody/btSoftBodySolvers.h (btSoftBodySolverOutput).
/// </summary>
public abstract class SoftBodySolverOutput : IDisposable
{
    protected SoftBodySolverOutput()
    {
    }

    public abstract void CopySoftBodyToVertexBuffer(SoftBody softBody, VertexBufferDescriptor vertexBuffer);

    public void Dispose()
    {
        Dispose(true);
        GC.SuppressFinalize(this);
    }

    protected virtual void Dispose(bool disposing)
    {
    }
}
