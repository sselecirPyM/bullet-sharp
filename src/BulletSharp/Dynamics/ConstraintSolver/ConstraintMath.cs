using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Math helper functions shared by constraints (ported from static functions such as
/// btNormalizeAngle/btAdjustAngleToLimits in btScalar.h, btTypedConstraint::getMotorFactor,
/// and btVector3::getSkewSymmetricMatrix).
/// </summary>
internal static class ConstraintMath
{
    /// <summary>btNormalizeAngle: normalize to [-PI, PI]</summary>
    public static btScalar NormalizeAngle(btScalar angleInRadians)
    {
        angleInRadians = BtMath.Fmod(angleInRadians, BtMath.Simd2Pi);
        if (angleInRadians < -BtMath.Pi)
        {
            return angleInRadians + BtMath.Simd2Pi;
        }
        if (angleInRadians > BtMath.Pi)
        {
            return angleInRadians - BtMath.Simd2Pi;
        }
        return angleInRadians;
    }

    /// <summary>btAdjustAngleToLimits: returns the angle within the limits or closest to them (in [-2PI, 2PI]; all inputs are normalized angles)</summary>
    public static btScalar AdjustAngleToLimits(btScalar angleInRadians, btScalar angleLowerLimitInRadians, btScalar angleUpperLimitInRadians)
    {
        if (angleLowerLimitInRadians >= angleUpperLimitInRadians)
        {
            return angleInRadians;
        }
        else if (angleInRadians < angleLowerLimitInRadians)
        {
            btScalar diffLo = BtMath.Abs(NormalizeAngle(angleLowerLimitInRadians - angleInRadians));
            btScalar diffHi = BtMath.Abs(NormalizeAngle(angleUpperLimitInRadians - angleInRadians));
            return diffLo < diffHi ? angleInRadians : angleInRadians + BtMath.Simd2Pi;
        }
        else if (angleInRadians > angleUpperLimitInRadians)
        {
            btScalar diffHi = BtMath.Abs(NormalizeAngle(angleInRadians - angleUpperLimitInRadians));
            btScalar diffLo = BtMath.Abs(NormalizeAngle(angleInRadians - angleLowerLimitInRadians));
            return diffLo < diffHi ? angleInRadians - BtMath.Simd2Pi : angleInRadians;
        }
        else
        {
            return angleInRadians;
        }
    }

    /// <summary>btTypedConstraint::getMotorFactor (internal use by solvers)</summary>
    public static btScalar GetMotorFactor(btScalar pos, btScalar lowLim, btScalar uppLim, btScalar vel, btScalar timeFact)
    {
        if (lowLim > uppLim)
        {
            return 1;
        }
        else if (lowLim == uppLim)
        {
            return 0;
        }
        btScalar limFact = 1;
        btScalar deltaMax = vel / timeFact;
        if (deltaMax < 0)
        {
            if (pos >= lowLim && pos < lowLim - deltaMax)
            {
                limFact = (lowLim - pos) / deltaMax;
            }
            else if (pos < lowLim)
            {
                limFact = 0;
            }
            else
            {
                limFact = 1;
            }
        }
        else if (deltaMax > 0)
        {
            if (pos <= uppLim && pos > uppLim - deltaMax)
            {
                limFact = (uppLim - pos) / deltaMax;
            }
            else if (pos > uppLim)
            {
                limFact = 0;
            }
            else
            {
                limFact = 1;
            }
        }
        else
        {
            limFact = 0;
        }
        return limFact;
    }

    /// <summary>btFuzzyZero</summary>
    public static bool FuzzyZero(btScalar x) => BtMath.Abs(x) < BtMath.SimdEpsilon;

    /// <summary>btVector3::getSkewSymmetricMatrix (writes 3 rows of the skew-symmetric matrix into a J array with row stride rowskip)</summary>
    public static void WriteSkewSymmetric(btScalar[] dest, int rowskip, in Vector3 v)
    {
        dest[0] = 0; dest[1] = -v.Z; dest[2] = v.Y;
        dest[rowskip] = v.Z; dest[rowskip + 1] = 0; dest[rowskip + 2] = -v.X;
        dest[2 * rowskip] = -v.Y; dest[2 * rowskip + 1] = v.X; dest[2 * rowskip + 2] = 0;
    }

    /// <summary>Angular-constraint construction of btJacobianEntry (jointAxis version, for the legacy BuildJacobian paths of constraints)</summary>
    public static JacobianEntry CreateAngularJacobianEntry(in Vector3 jointAxis,
        in Matrix3x3 world2A, in Matrix3x3 world2B,
        in Vector3 inertiaInvA, in Vector3 inertiaInvB)
    {
        var e = new JacobianEntry
        {
            LinearJointAxis = Vector3.Zero,
        };
        e.AJ = world2A * jointAxis;
        e.BJ = world2B * -jointAxis;
        e.MinImpulse = -BtMath.BtLargeFloat;
        e.MaxImpulse = BtMath.BtLargeFloat;
        e.ADiag = e.AJ.Dot(inertiaInvA * e.AJ) + e.BJ.Dot(inertiaInvB * e.BJ);
        return e;
    }
}
