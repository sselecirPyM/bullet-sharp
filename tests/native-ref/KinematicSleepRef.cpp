// Native Bullet (bullet3) reference experiment: does moving a kinematic body via its
// MotionState work when DISABLE_DEACTIVATION is not set?
// (Counterpart of S13 in tests/BulletSharp.SmokeTests.)
//
// Background: a mass=0 body gets the CF_STATIC_OBJECT flag in setMassProps;
// btDiscreteDynamicsWorld::addRigidBody sets bodies with the static flag to
// ISLAND_SLEEPING, and saveKinematicState skips ISLAND_SLEEPING bodies —
// MotionState writes are never picked up.

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
    printf("%s %s body=(%8.4f %8.4f %8.4f) motionstate=(%8.4f %8.4f %8.4f) active=%d state=%d\n",
        tag, name, p.x(), p.y(), p.z(), q.x(), q.y(), q.z(),
        body->isActive() ? 1 : 0, body->getActivationState());
}

static void Run(const char* tag, bool disableDeactivation)
{
    const btVector3 d(10, 0, 0);
    auto* world = CreateWorld();
    auto* a = AddBody(world, 0, btVector3(0, 5, 0));
    a->setCollisionFlags(a->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    auto* b = AddBody(world, 1, btVector3(2, 5, 0));
    b->setActivationState(DISABLE_DEACTIVATION);
    b->setSleepingThresholds(0, 0);
    if (disableDeactivation)
    {
        a->setActivationState(DISABLE_DEACTIVATION);
        a->setSleepingThresholds(0, 0);
    }
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
    PrintPos(tag, "A-before", a);
    btTransform mt;
    a->getMotionState()->getWorldTransform(mt);
    mt.setOrigin(mt.getOrigin() + d);
    a->getMotionState()->setWorldTransform(mt);
    Step(world, 120);
    PrintPos(tag, "A-after ", a);
    PrintPos(tag, "B-after ", b);
}

int main()
{
    Run("S13-noDA ", false); // without DISABLE_DEACTIVATION: reproduce the "anchor fixed in space" symptom?
    Run("S13-withDA", true);  // with DISABLE_DEACTIVATION: should follow normally
    return 0;
}
