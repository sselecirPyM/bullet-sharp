// Native reference for the S7 smoke test:
// locked 6dof, teleport only A -> B should be pulled along (no gravity).
// Mirrors tests/BulletSharp.SmokeTests Program.cs S7 block.

#include <btBulletDynamicsCommon.h>
#include <stdio.h>

int main()
{
    btDefaultCollisionConfiguration* config = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(config);
    btDbvtBroadphase* broadphase = new btDbvtBroadphase();
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
    btDiscreteDynamicsWorld* world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config);
    world->setGravity(btVector3(0, 0, 0));

    auto makeBox = [&](const btVector3& pos) {
        btBoxShape* shape = new btBoxShape(btVector3(0.5f, 0.5f, 0.5f));
        btVector3 inertia(0, 0, 0);
        shape->calculateLocalInertia(1.0f, inertia);
        btTransform start = btTransform::getIdentity();
        start.setOrigin(pos);
        btDefaultMotionState* ms = new btDefaultMotionState(start);
        btRigidBody::btRigidBodyConstructionInfo info(1.0f, ms, shape, inertia);
        info.m_friction = 0.5f;
        info.m_restitution = 0.0f;
        btRigidBody* body = new btRigidBody(info);
        world->addRigidBody(body);
        return body;
    };

    btRigidBody* a = makeBox(btVector3(0, 5, 0));
    btRigidBody* b = makeBox(btVector3(2, 5, 0));

    btTransform frameA = btTransform::getIdentity(); frameA.setOrigin(btVector3(1, 0, 0));
    btTransform frameB = btTransform::getIdentity(); frameB.setOrigin(btVector3(-1, 0, 0));
    btGeneric6DofConstraint* joint = new btGeneric6DofConstraint(*a, *b, frameA, frameB, true);
    joint->setLinearLowerLimit(btVector3(0, 0, 0));
    joint->setLinearUpperLimit(btVector3(0, 0, 0));
    joint->setAngularLowerLimit(btVector3(0, 0, 0));
    joint->setAngularUpperLimit(btVector3(0, 0, 0));
    world->addConstraint(joint);

    for (int i = 0; i < 60; i++) world->stepSimulation(1.0f / 60.0f, 10);

    // Teleport a: translate via setWorldTransform, then activate
    {
        btTransform t = a->getWorldTransform();
        t.setOrigin(t.getOrigin() + btVector3(10, 0, 0));
        a->setWorldTransform(t);
        a->activate();
    }

    for (int i = 0; i < 120; i++) world->stepSimulation(1.0f / 60.0f, 10);

    btVector3 pa = a->getWorldTransform().getOrigin();
    btVector3 pb = b->getWorldTransform().getOrigin();
    btQuaternion qa = a->getWorldTransform().getRotation();
    btQuaternion qb = b->getWorldTransform().getRotation();
    printf("S7 A body=(%.7f %.7f %.7f) quat=(%.7f %.7f %.7f %.7f)\n", pa.x(), pa.y(), pa.z(), qa.x(), qa.y(), qa.z(), qa.w());
    printf("S7 B body=(%.7f %.7f %.7f) quat=(%.7f %.7f %.7f %.7f)\n", pb.x(), pb.y(), pb.z(), qb.x(), qb.y(), qb.z(), qb.w());
    return 0;
}
