// Bullet C++ benchmark — identical scenarios to benchmarks/BulletSharp.Benchmarks (single-threaded).
// Build: see benchmarks/README.md
#include "btBulletDynamicsCommon.h"
#include <chrono>
#include <cstdio>

static double NowMs()
{
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

static btDiscreteDynamicsWorld* CreateWorld()
{
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();
    btDefaultCollisionConfiguration* config = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(config);
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
    btDiscreteDynamicsWorld* world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config);
    world->setGravity(btVector3(0, btScalar(-9.81), 0));
    return world;
}

static btRigidBody* AddBody(btDiscreteDynamicsWorld* world, btCollisionShape* shape, btScalar mass,
    const btTransform& startTransform, btScalar friction = btScalar(0.5))
{
    btVector3 inertia(0, 0, 0);
    if (mass != 0)
        shape->calculateLocalInertia(mass, inertia);
    btDefaultMotionState* motionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo info(mass, motionState, shape, inertia);
    info.m_friction = friction;
    btRigidBody* body = new btRigidBody(info);
    world->addRigidBody(body);
    return body;
}

// Deterministic LCG
static unsigned int g_rngState = 12345;
static float NextFloat() // [0, 1)
{
    g_rngState = g_rngState * 1664525u + 1013904223u;
    return (float)(g_rngState >> 8) / 16777216.0f;
}

// ---------- Scenario 1: 1000 boxes fall onto the ground and stack, 600 steps ----------
static void BoxPile()
{
    btDiscreteDynamicsWorld* world = CreateWorld();

    btBoxShape* ground = new btBoxShape(btVector3(50, 1, 50));
    btTransform groundT; groundT.setIdentity(); groundT.setOrigin(btVector3(0, -1, 0));
    AddBody(world, ground, 0, groundT);

    btBoxShape* box = new btBoxShape(btVector3(btScalar(0.5), btScalar(0.5), btScalar(0.5)));
    for (int iy = 0; iy < 10; iy++)
        for (int ix = 0; ix < 10; ix++)
            for (int iz = 0; iz < 10; iz++)
            {
                btTransform t; t.setIdentity();
                t.setOrigin(btVector3(
                    btScalar((ix - 4.5) * 1.1),
                    btScalar(2.0 + iy * 1.1),
                    btScalar((iz - 4.5) * 1.1)));
                AddBody(world, box, 1, t);
            }

    const int steps = 600;
    // JIT warmup on the C# side; run the same segment here so both sides measure the
    // identical physics (steps 61..660, after the initial settling churn)
    for (int i = 0; i < 60; i++)
        world->stepSimulation(btScalar(1.0 / 60.0), 1, btScalar(1.0 / 60.0));
    double t0 = NowMs();
    for (int i = 0; i < steps; i++)
        world->stepSimulation(btScalar(1.0 / 60.0), 1, btScalar(1.0 / 60.0));
    double elapsed = NowMs() - t0;

    printf("boxpile : %8.1f ms  (%d steps, %.3f ms/step)\n", elapsed, steps, elapsed / steps);
}

// ---------- Scenario 2: static box pile + 50000 raycasts ----------
static void Raycast()
{
    btDiscreteDynamicsWorld* world = CreateWorld();

    btBoxShape* ground = new btBoxShape(btVector3(50, 1, 50));
    btTransform groundT; groundT.setIdentity(); groundT.setOrigin(btVector3(0, -1, 0));
    AddBody(world, ground, 0, groundT);

    btBoxShape* box = new btBoxShape(btVector3(btScalar(0.5), btScalar(0.5), btScalar(0.5)));
    for (int iy = 0; iy < 10; iy++)
        for (int ix = 0; ix < 10; ix++)
            for (int iz = 0; iz < 10; iz++)
            {
                btTransform t; t.setIdentity();
                t.setOrigin(btVector3(
                    btScalar((ix - 4.5) * 1.1),
                    btScalar(0.5 + iy * 1.1),
                    btScalar((iz - 4.5) * 1.1)));
                AddBody(world, box, 0, t); // static
            }

    world->stepSimulation(btScalar(1.0 / 60.0), 1, btScalar(1.0 / 60.0)); // build the broadphase

    g_rngState = 12345;
    const int rays = 50000;
    int hits = 0;
    double t0 = NowMs();
    for (int i = 0; i < rays; i++)
    {
        float x = NextFloat() * 30.0f - 15.0f;
        float z = NextFloat() * 30.0f - 15.0f;
        btVector3 from(x, 50, z), to(x, -5, z);
        btCollisionWorld::ClosestRayResultCallback cb(from, to);
        world->rayTest(from, to, cb);
        if (cb.hasHit()) hits++;
    }
    double elapsed = NowMs() - t0;

    printf("raycast : %8.1f ms  (%d rays, %.3f us/ray, hits=%d)\n",
        elapsed, rays, elapsed * 1000.0 / rays, hits);
}

int main()
{
    BoxPile();
    Raycast();
    return 0;
}
