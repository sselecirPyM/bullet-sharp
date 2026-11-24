// Native Bullet (bullet3) reference test: constraint behavior (especially 6dof) after moving bodies
// One-to-one correspondence with the mirrored scenes in tests/BulletSharp.SmokeTests (S1..S6)
// Build: see build-ref.ps1 / README

#include "btBulletDynamicsCommon.h"
#include <cstdio>

static btDiscreteDynamicsWorld* CreateWorld()
{
    auto* broadphase = new btDbvtBroadphase();
    auto* config = new btDefaultCollisionConfiguration();
    auto* dispatcher = new btCollisionDispatcher(config);
    auto* solver = new btSequentialImpulseConstraintSolver();
    auto* world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config);
    world->setGravity(btVector3(0, 0, 0)); // gravity off, isolating constraint behavior
    return world;
}

static btRigidBody* AddBody(btDiscreteDynamicsWorld* world, btScalar mass, const btVector3& pos)
{
    auto* shape = new btBoxShape(btVector3(0.5f, 0.5f, 0.5f));
    btVector3 inertia(0, 0, 0);
    if (mass != 0)
        shape->calculateLocalInertia(mass, inertia);
    btTransform start;
    start.setIdentity();
    start.setOrigin(pos);
    auto* ms = new btDefaultMotionState(start);
    btRigidBody::btRigidBodyConstructionInfo info(mass, ms, shape, inertia);
    auto* body = new btRigidBody(info);
    world->addRigidBody(body);
    return body;
}

static void Step(btDiscreteDynamicsWorld* world, int n)
{
    for (int i = 0; i < n; i++)
        world->stepSimulation(1.0f / 60.0f, 10);
}

static void PrintPos(const char* tag, const char* name, btRigidBody* body)
{
    btVector3 p = body->getWorldTransform().getOrigin();
    btTransform msT;
    body->getMotionState()->getWorldTransform(msT);
    btVector3 q = msT.getOrigin();
    printf("%s %s body=(%8.4f %8.4f %8.4f) motionstate=(%8.4f %8.4f %8.4f) active=%d\n",
        tag, name, p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), body->isActive() ? 1 : 0);
}

// setWorldTransform translation (btCollisionObject::setWorldTransform; interpolation transform untouched)
static void Teleport(btRigidBody* body, const btVector3& delta, bool activate)
{
    btTransform t = body->getWorldTransform();
    t.setOrigin(t.getOrigin() + delta);
    body->setWorldTransform(t);
    if (activate)
        body->activate();
}

// proceedToTransform translation (updates the interpolation transform as well)
static void TeleportProceed(btRigidBody* body, const btVector3& delta, bool activate)
{
    btTransform t = body->getWorldTransform();
    t.setOrigin(t.getOrigin() + delta);
    body->proceedToTransform(t);
    if (activate)
        body->activate();
}

static btGeneric6DofConstraint* AddLocked6Dof(btDiscreteDynamicsWorld* world, btRigidBody* a, btRigidBody* b,
    const btVector3& pivotInA, const btVector3& pivotInB)
{
    btTransform fa, fb;
    fa.setIdentity();
    fa.setOrigin(pivotInA);
    fb.setIdentity();
    fb.setOrigin(pivotInB);
    auto* joint = new btGeneric6DofConstraint(*a, *b, fa, fb, true);
    joint->setLinearLowerLimit(btVector3(0, 0, 0));
    joint->setLinearUpperLimit(btVector3(0, 0, 0));
    joint->setAngularLowerLimit(btVector3(0, 0, 0));
    joint->setAngularUpperLimit(btVector3(0, 0, 0));
    world->addConstraint(joint);
    return joint;
}

int main()
{
    const btVector3 d(10, 0, 0);

    // S1: locked 6dof, teleport both bodies together — frames are body-local, should follow
    {
        auto* world = CreateWorld();
        auto* a = AddBody(world, 1, btVector3(0, 5, 0));
        auto* b = AddBody(world, 1, btVector3(2, 5, 0));
        AddLocked6Dof(world, a, b, btVector3(1, 0, 0), btVector3(-1, 0, 0));
        Step(world, 60);
        Teleport(a, d, true);
        Teleport(b, d, true);
        Step(world, 120);
        PrintPos("S1", "A", a);
        PrintPos("S1", "B", b);
    }

    // S2: locked 6dof, teleport only B — constraint is violated, B should be pulled back to its relative pose
    {
        auto* world = CreateWorld();
        auto* a = AddBody(world, 1, btVector3(0, 5, 0));
        auto* b = AddBody(world, 1, btVector3(2, 5, 0));
        AddLocked6Dof(world, a, b, btVector3(1, 0, 0), btVector3(-1, 0, 0));
        Step(world, 60);
        Teleport(b, d, true);
        Step(world, 120);
        PrintPos("S2", "A", a);
        PrintPos("S2", "B", b);
    }

    // S3: single-body 6dof (world-anchored), teleport B — frameInA is fixed at creation, B should be pulled back to its creation position
    {
        auto* world = CreateWorld();
        auto* b = AddBody(world, 1, btVector3(0, 3, 0));
        btTransform fb;
        fb.setIdentity();
        auto* joint = new btGeneric6DofConstraint(*b, fb, false);
        joint->setLinearLowerLimit(btVector3(0, 0, 0));
        joint->setLinearUpperLimit(btVector3(0, 0, 0));
        joint->setAngularLowerLimit(btVector3(0, 0, 0));
        joint->setAngularUpperLimit(btVector3(0, 0, 0));
        world->addConstraint(joint);
        Step(world, 60);
        Teleport(b, d, true);
        Step(world, 120);
        PrintPos("S3", "B", b);
    }

    // S4: p2p constraint, teleport both bodies together — should follow
    {
        auto* world = CreateWorld();
        auto* a = AddBody(world, 1, btVector3(0, 5, 0));
        auto* b = AddBody(world, 1, btVector3(2, 5, 0));
        auto* joint = new btPoint2PointConstraint(*a, *b, btVector3(1, 0, 0), btVector3(-1, 0, 0));
        world->addConstraint(joint);
        Step(world, 60);
        Teleport(a, d, true);
        Teleport(b, d, true);
        Step(world, 120);
        PrintPos("S4", "A", a);
        PrintPos("S4", "B", b);
    }

    // S5: same as S1, but teleport via proceedToTransform (interpolation transform updated in sync)
    {
        auto* world = CreateWorld();
        auto* a = AddBody(world, 1, btVector3(0, 5, 0));
        auto* b = AddBody(world, 1, btVector3(2, 5, 0));
        AddLocked6Dof(world, a, b, btVector3(1, 0, 0), btVector3(-1, 0, 0));
        Step(world, 60);
        TeleportProceed(a, d, true);
        TeleportProceed(b, d, true);
        Step(world, 120);
        PrintPos("S5", "A", a);
        PrintPos("S5", "B", b);
    }

    // S6: same as S1, but let the bodies sleep first and teleport without activate — does the motionstate stay stuck at the old position?
    {
        auto* world = CreateWorld();
        auto* a = AddBody(world, 1, btVector3(0, 5, 0));
        auto* b = AddBody(world, 1, btVector3(2, 5, 0));
        AddLocked6Dof(world, a, b, btVector3(1, 0, 0), btVector3(-1, 0, 0));
        Step(world, 240); // falls asleep (default deactivation time 2s)
        PrintPos("S6-sleep", "A", a);
        PrintPos("S6-sleep", "B", b);
        Teleport(a, d, false);
        Teleport(b, d, false);
        Step(world, 60);
        PrintPos("S6-noact", "A", a);
        PrintPos("S6-noact", "B", b);
        a->activate();
        b->activate();
        Step(world, 60);
        PrintPos("S6-act", "A", a);
        PrintPos("S6-act", "B", b);
    }

    // S7: locked 6dof, teleport only A — B should be pulled toward the midpoint (center of mass conserved: A≈5, B≈7)
    {
        auto* world = CreateWorld();
        auto* a = AddBody(world, 1, btVector3(0, 5, 0));
        auto* b = AddBody(world, 1, btVector3(2, 5, 0));
        AddLocked6Dof(world, a, b, btVector3(1, 0, 0), btVector3(-1, 0, 0));
        Step(world, 60);
        Teleport(a, d, true);
        Step(world, 120);
        PrintPos("S7", "A", a);
        PrintPos("S7", "B", b);
    }

    // S8: p2p, teleport only A — B should be pulled along
    {
        auto* world = CreateWorld();
        auto* a = AddBody(world, 1, btVector3(0, 5, 0));
        auto* b = AddBody(world, 1, btVector3(2, 5, 0));
        auto* joint = new btPoint2PointConstraint(*a, *b, btVector3(1, 0, 0), btVector3(-1, 0, 0));
        world->addConstraint(joint);
        Step(world, 60);
        Teleport(a, d, true);
        Step(world, 120);
        PrintPos("S8", "A", a);
        PrintPos("S8", "B", b);
    }

    // S9: p2p, violated from the start (spacing 5, expected 2) — should pull together to A≈1.5, B≈3.5
    {
        auto* world = CreateWorld();
        auto* a = AddBody(world, 1, btVector3(0, 5, 0));
        auto* b = AddBody(world, 1, btVector3(5, 5, 0));
        auto* joint = new btPoint2PointConstraint(*a, *b, btVector3(1, 0, 0), btVector3(-1, 0, 0));
        world->addConstraint(joint);
        Step(world, 120);
        PrintPos("S9", "A", a);
        PrintPos("S9", "B", b);
    }

    // S10: kinematic A + locked 6dof spring, moved via motionstate (Coocoo3D MoveRigidBody path)
    {
        auto* world = CreateWorld();
        auto* a = AddBody(world, 0, btVector3(0, 5, 0));
        a->setCollisionFlags(a->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        a->setActivationState(DISABLE_DEACTIVATION);
        a->setSleepingThresholds(0, 0);
        auto* b = AddBody(world, 1, btVector3(2, 5, 0));
        b->setActivationState(DISABLE_DEACTIVATION);
        b->setSleepingThresholds(0, 0);
        auto* joint = new btGeneric6DofSpringConstraint(*a, *b, btTransform(btQuaternion::getIdentity(), btVector3(1, 0, 0)),
            btTransform(btQuaternion::getIdentity(), btVector3(-1, 0, 0)), true);
        joint->setLinearLowerLimit(btVector3(0, 0, 0));
        joint->setLinearUpperLimit(btVector3(0, 0, 0));
        joint->setAngularLowerLimit(btVector3(0, 0, 0));
        joint->setAngularUpperLimit(btVector3(0, 0, 0));
        world->addConstraint(joint);
        Step(world, 60);
        btTransform mt;
        a->getMotionState()->getWorldTransform(mt);
        mt.setOrigin(mt.getOrigin() + d);
        a->getMotionState()->setWorldTransform(mt);
        Step(world, 120);
        PrintPos("S10", "A", a);
        PrintPos("S10", "B", b);
    }

    // S11: same as S10, loose limits + springs on all DOF (stiffness 50), MMD style
    {
        auto* world = CreateWorld();
        auto* a = AddBody(world, 0, btVector3(0, 5, 0));
        a->setCollisionFlags(a->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        a->setActivationState(DISABLE_DEACTIVATION);
        a->setSleepingThresholds(0, 0);
        auto* b = AddBody(world, 1, btVector3(2, 5, 0));
        b->setActivationState(DISABLE_DEACTIVATION);
        b->setSleepingThresholds(0, 0);
        auto* joint = new btGeneric6DofSpringConstraint(*a, *b, btTransform(btQuaternion::getIdentity(), btVector3(1, 0, 0)),
            btTransform(btQuaternion::getIdentity(), btVector3(-1, 0, 0)), true);
        joint->setLinearLowerLimit(btVector3(-5, -5, -5));
        joint->setLinearUpperLimit(btVector3(5, 5, 5));
        joint->setAngularLowerLimit(btVector3(-1, -1, -1));
        joint->setAngularUpperLimit(btVector3(1, 1, 1));
        for (int i = 0; i < 6; i++)
        {
            joint->enableSpring(i, true);
            joint->setStiffness(i, 50);
        }
        world->addConstraint(joint);
        Step(world, 60);
        btTransform mt;
        a->getMotionState()->getWorldTransform(mt);
        mt.setOrigin(mt.getOrigin() + d);
        a->getMotionState()->setWorldTransform(mt);
        Step(world, 300);
        PrintPos("S11", "A", a);
        PrintPos("S11", "B", b);
    }

    // S12: rotated frames + locked 6dof; kinematic A moves and rotates (via motionstate), B should follow
    {
        auto* world = CreateWorld();
        btQuaternion qz(btVector3(0, 0, 1), SIMD_PI / 4);
        auto* a = AddBody(world, 0, btVector3(0, 5, 0));
        a->setCollisionFlags(a->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        a->setActivationState(DISABLE_DEACTIVATION);
        a->setSleepingThresholds(0, 0);
        auto* b = AddBody(world, 1, btVector3(2, 5, 0));
        b->setActivationState(DISABLE_DEACTIVATION);
        b->setSleepingThresholds(0, 0);
        btTransform fa(qz, btVector3(1, 0, 0));
        btTransform fb(qz, btVector3(-1, 0, 0));
        auto* joint = new btGeneric6DofSpringConstraint(*a, *b, fa, fb, true);
        joint->setLinearLowerLimit(btVector3(0, 0, 0));
        joint->setLinearUpperLimit(btVector3(0, 0, 0));
        joint->setAngularLowerLimit(btVector3(0, 0, 0));
        joint->setAngularUpperLimit(btVector3(0, 0, 0));
        world->addConstraint(joint);
        Step(world, 60);
        btTransform mt;
        a->getMotionState()->getWorldTransform(mt);
        btTransform rot(btQuaternion(btVector3(0, 1, 0), SIMD_PI / 2), btVector3(mt.getOrigin().x() + 10, mt.getOrigin().y(), mt.getOrigin().z()));
        a->getMotionState()->setWorldTransform(rot);
        Step(world, 300);
        PrintPos("S12", "A", a);
        PrintPos("S12", "B", b);
        btTransform bm;
        b->getMotionState()->getWorldTransform(bm);
        const btMatrix3x3& bb = bm.getBasis();
        printf("S12 B basis row0=(%8.4f %8.4f %8.4f) row1=(%8.4f %8.4f %8.4f)\n",
            bb[0][0], bb[0][1], bb[0][2], bb[1][0], bb[1][1], bb[1][2]);
    }

    return 0;
}
