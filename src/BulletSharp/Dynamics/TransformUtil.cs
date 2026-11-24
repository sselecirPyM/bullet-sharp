using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btTransformUtil (time-dependent transform utilities). Engine-internal.
/// </summary>
internal static class TransformUtil
{
    /// <summary>btTransformUtil::integrateTransform (integrates rotation via the exponential map)</summary>
    public static void IntegrateTransform(in Transform curTrans, in Vector3 linvel, in Vector3 angvel,
        btScalar timeStep, out Transform predictedTransform)
    {
        predictedTransform = curTrans;
        predictedTransform.Origin = curTrans.Origin + linvel * timeStep;

        // Exponential map (see "Practical Parameterization of Rotations Using the Exponential Map")
        Vector3 axis;
        btScalar fAngle2 = angvel.LengthSquared();
        btScalar fAngle = 0;
        if (fAngle2 > BtMath.SimdEpsilon)
        {
            fAngle = BtMath.Sqrt(fAngle2);
        }

        // Clamp the angular motion per step
        if (fAngle * timeStep > BtMath.AngularMotionThreshold)
        {
            fAngle = BtMath.AngularMotionThreshold / timeStep;
        }

        if (fAngle < (btScalar)0.001)
        {
            // Use the Taylor expansion of the sinc function
            axis = angvel * ((btScalar)0.5 * timeStep - timeStep * timeStep * timeStep * (btScalar)0.020833333333 * fAngle * fAngle);
        }
        else
        {
            // sinc(fAngle) = sin(c*fAngle)/t
            axis = angvel * (BtMath.Sin((btScalar)0.5 * fAngle * timeStep) / fAngle);
        }
        var dorn = new Quaternion(axis.X, axis.Y, axis.Z, BtMath.Cos(fAngle * timeStep * (btScalar)0.5));
        Quaternion orn0 = curTrans.GetRotation();

        Quaternion predictedOrn = dorn * orn0;
        predictedOrn.Normalize();
        if (predictedOrn.LengthSquared() > BtMath.SimdEpsilon)
        {
            predictedTransform.SetRotation(predictedOrn);
        }
        else
        {
            predictedTransform.Basis = curTrans.Basis;
        }
    }

    /// <summary>btTransformUtil::calculateVelocityQuaternion</summary>
    public static void CalculateVelocityQuaternion(in Vector3 pos0, in Vector3 pos1,
        in Quaternion orn0, in Quaternion orn1, btScalar timeStep,
        out Vector3 linVel, out Vector3 angVel)
    {
        linVel = (pos1 - pos0) / timeStep;
        if (orn0 != orn1)
        {
            CalculateDiffAxisAngleQuaternion(in orn0, in orn1, out Vector3 axis, out btScalar angle);
            angVel = axis * angle / timeStep;
        }
        else
        {
            angVel = Vector3.Zero;
        }
    }

    /// <summary>btTransformUtil::calculateDiffAxisAngleQuaternion</summary>
    public static void CalculateDiffAxisAngleQuaternion(in Quaternion orn0, in Quaternion orn1a,
        out Vector3 axis, out btScalar angle)
    {
        Quaternion orn1 = orn0.Nearest(orn1a);
        Quaternion dorn = orn1 * orn0.Inverse();
        angle = dorn.GetAngle();
        axis = new Vector3(dorn.X, dorn.Y, dorn.Z);
        // Check the axis length
        btScalar len = axis.LengthSquared();
        if (len < BtMath.SimdEpsilon * BtMath.SimdEpsilon)
            axis = new Vector3(1, 0, 0);
        else
            axis /= BtMath.Sqrt(len);
    }

    /// <summary>btTransformUtil::calculateVelocity</summary>
    public static void CalculateVelocity(in Transform transform0, in Transform transform1, btScalar timeStep,
        out Vector3 linVel, out Vector3 angVel)
    {
        linVel = (transform1.Origin - transform0.Origin) / timeStep;
        CalculateDiffAxisAngle(in transform0, in transform1, out Vector3 axis, out btScalar angle);
        angVel = axis * angle / timeStep;
    }

    /// <summary>btTransformUtil::calculateDiffAxisAngle</summary>
    public static void CalculateDiffAxisAngle(in Transform transform0, in Transform transform1,
        out Vector3 axis, out btScalar angle)
    {
        Matrix3x3 dmat = transform1.Basis * transform0.Basis.Inverse();
        Quaternion dorn = dmat.GetRotation();

        // Floating-point error can make the w component > 1
        dorn.Normalize();

        angle = dorn.GetAngle();
        axis = new Vector3(dorn.X, dorn.Y, dorn.Z);
        // Check the axis length
        btScalar len = axis.LengthSquared();
        if (len < BtMath.SimdEpsilon * BtMath.SimdEpsilon)
            axis = new Vector3(1, 0, 0);
        else
            axis /= BtMath.Sqrt(len);
    }
}
