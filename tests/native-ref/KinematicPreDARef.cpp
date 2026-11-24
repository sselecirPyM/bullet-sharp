// Native Bullet (bullet3) reference experiment: Coocoo3D's creation order —
// setActivationState(DISABLE_DEACTIVATION) first, then CF_KINEMATIC_OBJECT,
// and finally addRigidBody(body, group, mask); afterwards the body is moved
// only through its MotionState.
//
// Counterpart of S14 in tests/BulletSharp.SmokeTests.
//
// Key point: btDiscreteDynamicsWorld::addRigidBody calls
// body->setActivationState(ISLAND_SLEEPING) on mass=0 bodies carrying the static flag,
// but btCollisionObject::setActivationState is guarded (never overrides
// DISABLE_DEACTIVATION / DISABLE_SIMULATION), so DISABLE_DEACTIVATION survives
// when set before addRigidBody, and saveKinematicState still picks up MotionState
// writes — constraints should follow the kinematic body.

#include "btBulletDynamicsCommon.h"
#include <cstdio>

static btDiscreteDynamicsWorld* CreateWorld()
{
    auto* broadphase = new btDbvtBroadphase();
    auto* config = new btDefaultCollisionConfiguration();
    auto* dispatcher = new btCollisionDispatcher(config);
    auto* solver = new btSequentialImpulseConstraintSolver();
    auto* world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config);
    world->setGravity(btVector3(0, 0, 0));
    return world;
}

static btRigidBody* AddBodyRaw(btDiscreteDynamicsWorld* world, btScalar mass, const btVector3& pos)
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
    return new btRigidBody(info);
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
    printf("%s %s body=(%8.4f %8.4f %8.4f) motionstate=(%8.4f %8.4f %8.4f) active=%d state=%d\n",
        tag, name, p.x(), p.y(), p.z(), q.x(), q.y(), q.z(),
        body->isActive() ? 1 : 0, body->getActivationState());
}

int main()
{
    const btVector3 d(10, 0, 0);
    auto* world = CreateWorld();

    // A: Coocoo3D AddRigidBody order — DisableDeactivation and Kinematic flags set before add
    auto* a = AddBodyRaw(world, 0, btVector3(0, 5, 0));
    a->setActivationState(DISABLE_DEACTIVATION);
    a->setSleepingThresholds(0, 0);
    a->setCollisionFlags(a->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    world->addRigidBody(a, 1, -1);

    // B: dynamic body, also DisableDeactivation + SleepingThresholds(0,0) (Coocoo3D does this for all bodies)
    auto* b = AddBodyRaw(world, 1, btVector3(2, 5, 0));
    b->setActivationState(DISABLE_DEACTIVATION);
    b->setSleepingThresholds(0, 0);
    world->addRigidBody(b, 1, -1);

    btTransform fa, fb;
    fa.setIdentity();
    fa.setOrigin(btVector3(1, 0, 0));
    fb.setIdentity();
    fb.setOrigin(btVector3(-1, 0, 0));
    auto* joint = new btGeneric6DofConstraint(*a, *b, fa, fb, true);
    joint->setLinearLowerLimit(btVector3(0, 0, 0));
    joint->setLinearUpperLimit(btVector3(0, 0, 0));
    joint->setAngularLowerLimit(btVector3(0, 0, 0));
    joint->setAngularUpperLimit(btVector3(0, 0, 0));
    world->addConstraint(joint);

    Step(world, 60);
    PrintPos("S14-before", "A", a);

    // Coocoo3D MoveRigidBody: writes only the MotionState
    btTransform mt;
    a->getMotionState()->getWorldTransform(mt);
    mt.setOrigin(mt.getOrigin() + d);
    a->getMotionState()->setWorldTransform(mt);

    Step(world, 120);
    PrintPos("S14-after ", "A", a);
    PrintPos("S14-after ", "B", b);
    return 0;
}
