using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// btMotionState / BulletSharp MotionState abstract class.
/// </summary>
public abstract class MotionState : IDisposable
{
    public abstract void GetWorldTransform(out Matrix worldTrans);

    public abstract void SetWorldTransform(ref Matrix worldTrans);

    public Matrix WorldTransform
    {
        get
        {
            GetWorldTransform(out Matrix m);
            return m;
        }
        set => SetWorldTransform(ref value);
    }

    public virtual void Dispose()
    {
    }
}

/// <summary>
/// btDefaultMotionState.
/// </summary>
public class DefaultMotionState : MotionState
{
    internal Matrix graphicsWorldTrans;
    internal Matrix centerOfMassOffset;

    public DefaultMotionState()
    {
        graphicsWorldTrans = Matrix.Identity;
        centerOfMassOffset = Matrix.Identity;
    }

    public DefaultMotionState(Matrix graphicsWorldTrans)
    {
        this.graphicsWorldTrans = graphicsWorldTrans;
        centerOfMassOffset = Matrix.Identity;
    }

    public DefaultMotionState(Matrix graphicsWorldTrans, Matrix centerOfMassOffset)
    {
        this.graphicsWorldTrans = graphicsWorldTrans;
        this.centerOfMassOffset = centerOfMassOffset;
    }

    /// <summary>m_graphicsWorldTrans * m_centerOfMassOffset.inverse()</summary>
    public override void GetWorldTransform(out Matrix worldTrans)
    {
        worldTrans = centerOfMassOffset.Inverse() * graphicsWorldTrans;
    }

    /// <summary>m_graphicsWorldTrans = worldTrans * m_centerOfMassOffset (offset applied first)</summary>
    public override void SetWorldTransform(ref Matrix worldTrans)
    {
        graphicsWorldTrans = centerOfMassOffset * worldTrans;
    }
}
