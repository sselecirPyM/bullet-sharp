using BulletSharp.Math;

namespace BulletSharp.SoftBody;

/// <summary>
/// Ported from BulletSoftBody/btSoftBody.h and btSoftBody.cpp (defaultCollisionHandler and self-collision flags).
/// Note: CompoundShape and concave triangle meshes are decomposed upstream by CompoundCollisionAlgorithm and
/// SoftBodyConcaveCollisionAlgorithm before reaching DefaultCollisionHandler; deformable VF_DD normal-cone
/// self-collision is excluded per project scope (classic soft body self-collision uses CL_SS | CL_SELF).
/// </summary>
public partial class SoftBody
{
    internal bool useSelfCollision;

    private readonly CollideSdfRs collideSdfRs = new();
    private readonly CollideClRs collideClRs = new();
    private readonly CollideClSs collideClSs = new();
    private readonly CollideVfSs collideVfSs = new();
    internal readonly List<DbvtNode> collideTvStack = new(64);

    public void SetSelfCollision(bool useSelfCollision) =>
        this.useSelfCollision = useSelfCollision;

    public bool UseSelfCollision() => useSelfCollision;

    public void DefaultCollisionHandler(CollisionObjectWrapper pcoWrap)
    {
        if (collisionDisabledObjects.Count > 0 && collisionDisabledObjects.Contains(pcoWrap.collisionObject))
        {
            return;
        }

        switch ((Collisions)(cfg.collisions & (int)Collisions.RVSmask))
        {
            case Collisions.SDF_RS:
            {
                RigidBody prb1 = RigidBody.Upcast(pcoWrap.collisionObject);
                BulletSharp.Math.Transform wtr = pcoWrap.worldTransform;
                BulletSharp.Math.Transform ctr = pcoWrap.worldTransform;
                btScalar timemargin = (wtr.Origin - ctr.Origin).Length();
                btScalar basemargin = collisionShape.Margin;

                pcoWrap.collisionShape.GetAabb(in pcoWrap.worldTransform, out Vector3 mins, out Vector3 maxs);
                DbvtVolume volume = DbvtVolume.FromMM(mins, maxs);
                volume.Expand(new Vector3(basemargin, basemargin, basemargin));

                collideSdfRs.psb = this;
                collideSdfRs.colObj1Wrap = pcoWrap;
                collideSdfRs.rigidBody = prb1;
                collideSdfRs.dynmargin = basemargin + timemargin;
                collideSdfRs.stamargin = basemargin;
                ndbvt.CollideTVNoStackAlloc(ndbvt.root, volume, collideTvStack, collideSdfRs);
                collideSdfRs.psb = null;
                collideSdfRs.colObj1Wrap = null;
                collideSdfRs.rigidBody = null;
                break;
            }
            case Collisions.CL_RS:
            {
                collideClRs.ProcessColObj(this, pcoWrap);
                collideClRs.psb = null;
                collideClRs.colObjWrap = null;
                break;
            }
        }
    }

    public void DefaultCollisionHandler(SoftBody psb)
    {
        int cf = cfg.collisions & psb.cfg.collisions;
        switch ((Collisions)(cf & (int)(Collisions.SVSmask & ~Collisions.CL_SELF)))
        {
            case Collisions.CL_SS:
            {
                // Support self-collision if CL_SELF flag set
                if (!ReferenceEquals(this, psb) || (psb.cfg.collisions & (int)Collisions.CL_SELF) != 0)
                {
                    collideClSs.ProcessSoftSoft(this, psb);
                    collideClSs.bodies[0] = null;
                    collideClSs.bodies[1] = null;
                }
                break;
            }
            case Collisions.VF_SS:
            {
                // Only self-collision for Cluster, not Vertex-Face yet
                if (!ReferenceEquals(this, psb))
                {
                    // Common
                    collideVfSs.mrg = collisionShape.Margin + psb.collisionShape.Margin;

                    // psb0 nodes vs psb1 faces
                    collideVfSs.psb[0] = this;
                    collideVfSs.psb[1] = psb;
                    collideVfSs.psb[0].ndbvt.CollideTTpersistentStack(
                        collideVfSs.psb[0].ndbvt.root,
                        collideVfSs.psb[1].fdbvt.root,
                        collideVfSs);

                    // psb1 nodes vs psb0 faces
                    collideVfSs.psb[0] = psb;
                    collideVfSs.psb[1] = this;
                    collideVfSs.psb[0].ndbvt.CollideTTpersistentStack(
                        collideVfSs.psb[0].ndbvt.root,
                        collideVfSs.psb[1].fdbvt.root,
                        collideVfSs);

                    collideVfSs.psb[0] = null;
                    collideVfSs.psb[1] = null;
                }
                break;
            }
        }
    }
}
