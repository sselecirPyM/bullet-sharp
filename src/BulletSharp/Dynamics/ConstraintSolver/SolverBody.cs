using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btSolverBody: internal data structure of the constraint solver,
/// packing only the necessary data for better cache coherency.
/// </summary>
internal class SolverBody
{
    public Transform worldTransform;
    public Vector3 deltaLinearVelocity;
    public Vector3 deltaAngularVelocity;
    public Vector3 angularFactor;
    public Vector3 linearFactor;
    public Vector3 invMass;
    public Vector3 pushVelocity;
    public Vector3 turnVelocity;
    public Vector3 linearVelocity;
    public Vector3 angularVelocity;
    public Vector3 externalForceImpulse;
    public Vector3 externalTorqueImpulse;

    public RigidBody originalBody;

    /// <summary>btSolverBody::getVelocityInLocalPointNoDelta</summary>
    public void GetVelocityInLocalPointNoDelta(in Vector3 relPos, out Vector3 velocity)
    {
        if (originalBody != null)
            velocity = linearVelocity + externalForceImpulse + (angularVelocity + externalTorqueImpulse).Cross(relPos);
        else
            velocity = Vector3.Zero;
    }

    /// <summary>btSolverBody::getVelocityInLocalPointObsolete</summary>
    public void GetVelocityInLocalPointObsolete(in Vector3 relPos, out Vector3 velocity)
    {
        if (originalBody != null)
            velocity = linearVelocity + deltaLinearVelocity + (angularVelocity + deltaAngularVelocity).Cross(relPos);
        else
            velocity = Vector3.Zero;
    }

    /// <summary>btSolverBody::getAngularVelocity</summary>
    public void GetAngularVelocity(out Vector3 angVel)
    {
        if (originalBody != null)
            angVel = angularVelocity + deltaAngularVelocity;
        else
            angVel = Vector3.Zero;
    }

    /// <summary>btSolverBody::applyImpulse (optimization for iterative solvers: avoids recomputing inertia/normal/relative-position constant terms)</summary>
    public void ApplyImpulse(in Vector3 linearComponent, in Vector3 angularComponent, btScalar impulseMagnitude)
    {
        if (originalBody != null)
        {
            deltaLinearVelocity += linearComponent * impulseMagnitude * linearFactor;
            deltaAngularVelocity += angularComponent * (impulseMagnitude * angularFactor);
        }
    }

    /// <summary>btSolverBody::internalApplyPushImpulse</summary>
    public void InternalApplyPushImpulse(in Vector3 linearComponent, in Vector3 angularComponent, btScalar impulseMagnitude)
    {
        if (originalBody != null)
        {
            pushVelocity += linearComponent * impulseMagnitude * linearFactor;
            turnVelocity += angularComponent * (impulseMagnitude * angularFactor);
        }
    }

    /// <summary>btSolverBody::internalGetVelocityInLocalPointObsolete</summary>
    public void InternalGetVelocityInLocalPointObsolete(in Vector3 relPos, out Vector3 velocity)
    {
        velocity = linearVelocity + deltaLinearVelocity + (angularVelocity + deltaAngularVelocity).Cross(relPos);
    }

    /// <summary>btSolverBody::internalGetAngularVelocity</summary>
    public void InternalGetAngularVelocity(out Vector3 angVel)
    {
        angVel = angularVelocity + deltaAngularVelocity;
    }

    /// <summary>btSolverBody::internalApplyImpulse</summary>
    public void InternalApplyImpulse(in Vector3 linearComponent, in Vector3 angularComponent, btScalar impulseMagnitude)
    {
        if (originalBody != null)
        {
            deltaLinearVelocity += linearComponent * impulseMagnitude * linearFactor;
            deltaAngularVelocity += angularComponent * (impulseMagnitude * angularFactor);
        }
    }

    /// <summary>btSolverBody::writebackVelocity</summary>
    public void WritebackVelocity()
    {
        if (originalBody != null)
        {
            linearVelocity += deltaLinearVelocity;
            angularVelocity += deltaAngularVelocity;
        }
    }

    /// <summary>btSolverBody::writebackVelocityAndTransform (corrects position and orientation from push/turn velocities)</summary>
    public void WritebackVelocityAndTransform(btScalar timeStep, btScalar splitImpulseTurnErp)
    {
        if (originalBody != null)
        {
            linearVelocity += deltaLinearVelocity;
            angularVelocity += deltaAngularVelocity;

            // Correct position/orientation from push/turn velocities
            if (pushVelocity.X != 0 || pushVelocity.Y != 0 || pushVelocity.Z != 0 ||
                turnVelocity.X != 0 || turnVelocity.Y != 0 || turnVelocity.Z != 0)
            {
                TransformUtil.IntegrateTransform(in worldTransform, in pushVelocity,
                    turnVelocity * splitImpulseTurnErp, timeStep, out Transform newTransform);
                worldTransform = newTransform;
            }
        }
    }
}
