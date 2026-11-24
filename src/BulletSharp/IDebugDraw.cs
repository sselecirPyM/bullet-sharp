using BulletSharp.Math;

namespace BulletSharp;

/// <summary>btIDebugDraw::DebugDrawModes</summary>
[Flags]
public enum DebugDrawModes
{
    None = 0,
    DrawWireframe = 1,
    DrawAabb = 2,
    DrawFeaturesText = 4,
    DrawContactPoints = 8,
    NoDeactivation = 16,
    NoHelpText = 32,
    DrawText = 64,
    ProfileTimings = 128,
    EnableSatComparison = 256,
    DisableBulletLCP = 512,
    EnableCCD = 1024,
    DrawConstraints = 2048,
    DrawConstraintLimits = 4096,
    FastWireframe = 8192,
    DrawNormals = 16384,
    DrawFrames = 32768,
    Max = 65536,
}

/// <summary>
/// Ported from btIDebugDraw (optional debug drawing interface).
/// </summary>
public abstract class IDebugDraw : IDisposable
{
    public abstract DebugDrawModes DebugMode { get; set; }

    public abstract void DrawLine(ref Vector3 from, ref Vector3 to, ref Vector3 color);

    public virtual void DrawLine(ref Vector3 from, ref Vector3 to, ref Vector3 fromColor, ref Vector3 toColor) =>
        DrawLine(ref from, ref to, ref fromColor);

    public virtual void DrawSphere(ref Vector3 p, btScalar radius, ref Vector3 color)
    {
        // Default implementation: three axis-aligned crosses
        for (int axis = 0; axis < 3; axis++)
        {
            Vector3 a = p, b = p, c = p, d = p;
            a[axis] -= radius;
            b[axis] += radius;
            c[(axis + 1) % 3] -= radius;
            d[(axis + 1) % 3] += radius;
            DrawLine(ref a, ref b, ref color);
            DrawLine(ref c, ref d, ref color);
        }
    }

    public virtual void DrawTriangle(ref Vector3 a, ref Vector3 b, ref Vector3 c, ref Vector3 color, btScalar alpha)
    {
        DrawLine(ref a, ref b, ref color);
        DrawLine(ref b, ref c, ref color);
        DrawLine(ref c, ref a, ref color);
    }

    public virtual void DrawAabb(ref Vector3 from, ref Vector3 to, ref Vector3 color)
    {
        Vector3 halfExtents = (to - from) * (btScalar)0.5;
        Vector3 center = (to + from) * (btScalar)0.5;
        Vector3 edgecoord = new((btScalar)1, (btScalar)1, (btScalar)1);
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                Vector3 pa = new(edgecoord.X * halfExtents.X, edgecoord.Y * halfExtents.Y, edgecoord.Z * halfExtents.Z);
                pa += center;

                int othercoord = j % 3;
                edgecoord[othercoord] *= (btScalar)(-1);
                Vector3 pb = new(edgecoord.X * halfExtents.X, edgecoord.Y * halfExtents.Y, edgecoord.Z * halfExtents.Z);
                pb += center;

                DrawLine(ref pa, ref pb, ref color);
            }
            edgecoord = new Vector3((btScalar)(-1), (btScalar)(-1), (btScalar)(-1));
            if (i < 3)
            {
                edgecoord[i] *= (btScalar)(-1);
            }
        }
    }

    public virtual void DrawContactPoint(ref Vector3 pointOnB, ref Vector3 normalOnB, btScalar distance, int lifeTime, ref Vector3 color)
    {
    }

    public virtual void ReportErrorWarning(string warningString)
    {
    }

    public virtual void Draw3DText(ref Vector3 location, string textString)
    {
    }

    public virtual void SetDefaultColors(object defaultColors)
    {
    }

    public virtual void Dispose()
    {
    }
}
