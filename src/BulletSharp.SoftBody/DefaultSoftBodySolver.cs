using BulletSharp.Math;

namespace BulletSharp.SoftBody;

/// <summary>
/// Ported from BulletSoftBody/btDefaultSoftBodySolver.h and btDefaultSoftBodySolver.cpp.
/// </summary>
public class DefaultSoftBodySolver : SoftBodySolver
{
    /// <summary>Variable to define whether we need to update solver constants on the next iteration.</summary>
    protected bool updateSolverConstants;

    protected readonly List<SoftBody> softBodySet = new();

    public DefaultSoftBodySolver()
    {
        updateSolverConstants = true;
    }

    public override SolverTypes SolverType => SolverTypes.Default;

    public override bool CheckInitialized() => true;

    public override void CopyBackToSoftBodies(bool bMove = true)
    {
    }

    public override void Optimize(List<SoftBody> softBodies, bool forceUpdate = false)
    {
        if (ReferenceEquals(softBodySet, softBodies))
            return;

        softBodySet.Clear();
        if (softBodies.Count > 0)
        {
            if (softBodySet.Capacity < softBodies.Count)
                softBodySet.Capacity = softBodies.Count;
            for (int i = 0; i < softBodies.Count; i++)
                softBodySet.Add(softBodies[i]);
        }
    }

    public override void UpdateSoftBodies()
    {
        for (int i = 0; i < softBodySet.Count; i++)
        {
            SoftBody psb = softBodySet[i];
            if (psb.IsActive)
            {
                psb.IntegrateMotion();
            }
        }
    }

    public override void SolveConstraints(btScalar solverdt)
    {
        for (int i = 0; i < softBodySet.Count; ++i)
        {
            SoftBody psb = softBodySet[i];
            if (psb.IsActive)
            {
                psb.SolveConstraints();
            }
        }
    }

    public override void PredictMotion(btScalar solverdt)
    {
        for (int i = 0; i < softBodySet.Count; ++i)
        {
            SoftBody psb = softBodySet[i];
            if (psb.IsActive)
            {
                psb.PredictMotion(solverdt);
            }
        }
    }

    public virtual void CopySoftBodyToVertexBuffer(SoftBody softBody, VertexBufferDescriptor vertexBuffer)
    {
        // Currently only support CPU output buffers.
        if (vertexBuffer.BufferType == BufferTypes.Cpu)
        {
            List<Node> clothVertices = softBody.nodes;
            int numVertices = clothVertices.Count;

            var cpuVertexBuffer = (CpuVertexBufferDescriptor)vertexBuffer;
            float[] basePointer = cpuVertexBuffer.BasePointer;

            if (vertexBuffer.HasVertexPositions)
            {
                int vertexOffset = cpuVertexBuffer.VertexOffset;
                int vertexStride = cpuVertexBuffer.VertexStride;
                int vertexPointer = vertexOffset;

                for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex)
                {
                    Vector3 position = clothVertices[vertexIndex].x;
                    basePointer[vertexPointer + 0] = (float)position.X;
                    basePointer[vertexPointer + 1] = (float)position.Y;
                    basePointer[vertexPointer + 2] = (float)position.Z;
                    vertexPointer += vertexStride;
                }
            }

            if (vertexBuffer.HasNormals)
            {
                int normalOffset = cpuVertexBuffer.NormalOffset;
                int normalStride = cpuVertexBuffer.NormalStride;
                int normalPointer = normalOffset;

                for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex)
                {
                    Vector3 normal = clothVertices[vertexIndex].n;
                    basePointer[normalPointer + 0] = (float)normal.X;
                    basePointer[normalPointer + 1] = (float)normal.Y;
                    basePointer[normalPointer + 2] = (float)normal.Z;
                    normalPointer += normalStride;
                }
            }
        }
    }

    public override void ProcessCollision(SoftBody softBody, SoftBody otherSoftBody)
    {
        softBody.DefaultCollisionHandler(otherSoftBody);
    }

    public override void ProcessCollision(SoftBody softBody, CollisionObjectWrapper collisionObjectWrap)
    {
        softBody.DefaultCollisionHandler(collisionObjectWrap);
    }
}
