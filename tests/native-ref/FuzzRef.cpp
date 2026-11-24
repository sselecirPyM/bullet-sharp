// Native Bullet 6dof spring constraint differential fuzz test (mirrors BulletSharp.SmokeTests Fuzz.cs)
// Usage: FuzzRef.exe <seed>; prints the final position and quaternion of body B
#include "btBulletDynamicsCommon.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cstdint>

static uint32_t g_s;
static float Next()
{
    g_s = g_s * 1664525u + 1013904223u;
    return (g_s >> 8) * (1.0f / 16777216.0f);
}

int main(int argc, char** argv)
{
    int seed = argc > 1 ? atoi(argv[1]) : 1;
    g_s = (uint32_t)seed * 2654435761u + 12345u;

    auto* broadphase = new btDbvtBroadphase();
    auto* config = new btDefaultCollisionConfiguration();
    auto* dispatcher = new btCollisionDispatcher(config);
    auto* solver = new btSequentialImpulseConstraintSolver();
    auto* world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config);
    world->setGravity(btVector3(0, -9.81f, 0));

    bool dumpCfg = argc > 2 && atoi(argv[2]) != 0;
#define DUMP(...) do { if (dumpCfg) printf(__VA_ARGS__); } while (0)
    bool traceMode = argc > 2 && strcmp(argv[2], "trace") == 0;

    // Note: C++ argument evaluation order is unspecified; pin the Next() call order with locals
    float r0, r1, r2, r3;

    // Random pose for A
    r0 = Next(); r1 = Next(); r2 = Next();
    btVector3 pA(r0 * 2 - 1, r1 * 2 - 1, r2 * 2 - 1);
    r0 = Next(); r1 = Next(); r2 = Next();
    btVector3 axisA(r0 * 2 - 1, r1 * 2 - 1, r2 * 2 - 1);
    float angleA = (Next() * 2 - 1) * 0.4f;
    if (axisA.length2() < 1e-6f) axisA = btVector3(1, 0, 0);
    axisA.normalize();
    btQuaternion qA(axisA, angleA);

    // B's relative offset and orientation
    r0 = Next(); r1 = Next(); r2 = Next();
    btVector3 offB((r0 * 2 - 1) * 0.3f, (r1 * 2 - 1) * 0.3f, (r2 * 2 - 1) * 0.3f);
    r0 = Next(); r1 = Next(); r2 = Next();
    btVector3 axisB(r0 * 2 - 1, r1 * 2 - 1, r2 * 2 - 1);
    float angleB = (Next() * 2 - 1) * 0.4f;
    if (axisB.length2() < 1e-6f) axisB = btVector3(1, 0, 0);
    axisB.normalize();
    btQuaternion qB(axisB, angleB);
    float massB = 0.5f + Next() * 2.0f;

    // Constraint frames (coincident in world space, random rotation)
    r0 = Next(); r1 = Next(); r2 = Next();
    btVector3 axisF(r0 * 2 - 1, r1 * 2 - 1, r2 * 2 - 1);
    float angleF = (Next() * 2 - 1) * 1.0f;
    if (axisF.length2() < 1e-6f) axisF = btVector3(1, 0, 0);
    axisF.normalize();
    btQuaternion qF(axisF, angleF);
    r0 = Next(); r1 = Next(); r2 = Next();
    btVector3 midF = (pA + pA + offB) * 0.5f + btVector3((r0 * 2 - 1) * 0.1f, (r1 * 2 - 1) * 0.1f, (r2 * 2 - 1) * 0.1f);
    btTransform fW(qF, midF);

    // Limit and spring configuration (6 DOF)
    btVector3 linLo, linHi, angLo, angHi;
    bool springOn[6];
    float stiff[6], damp[6];
    for (int i = 0; i < 3; i++)
    {
        float r = Next();
        if (r < 0.3f) { linLo[i] = 0; linHi[i] = 0; }
        else if (r < 0.6f) { float l = 0.1f + r; linLo[i] = -l; linHi[i] = l; }
        else { linLo[i] = -100; linHi[i] = 100; }
    }
    for (int i = 0; i < 3; i++)
    {
        float r = Next();
        if (r < 0.3f) { angLo[i] = 0; angHi[i] = 0; }
        else if (r < 0.6f) { float l = 0.1f + r; angLo[i] = -l; angHi[i] = l; }
        else { angLo[i] = -100; angHi[i] = 100; }
    }
    for (int i = 0; i < 6; i++)
    {
        float r = Next();
        springOn[i] = r >= 0.5f;
        stiff[i] = springOn[i] ? 1.0f + r * 99.0f : 0.0f;
        damp[i] = 0.5f + Next() * 0.5f;
    }

    auto AddBox = [&](btScalar mass, const btVector3& pos, const btQuaternion& q) {
        auto* shape = new btBoxShape(btVector3(0.3f, 0.3f, 0.3f));
        btVector3 inertia(0, 0, 0);
        if (mass != 0) shape->calculateLocalInertia(mass, inertia);
        btTransform start(q, pos);
        auto* ms = new btDefaultMotionState(start);
        btRigidBody::btRigidBodyConstructionInfo info(mass, ms, shape, inertia);
        auto* body = new btRigidBody(info);
        body->setActivationState(DISABLE_DEACTIVATION);
        body->setSleepingThresholds(0, 0);
        world->addRigidBody(body);
        return body;
    };

    auto* a = AddBox(0, pA, qA);
    a->setCollisionFlags(a->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    auto* b = AddBox(massB, pA + offB, qB);

    btTransform worldA(qA, pA), worldB(qB, pA + offB);
    btTransform fa = worldA.inverse() * fW;
    btTransform fb = worldB.inverse() * fW;
    auto* joint = new btGeneric6DofSpringConstraint(*a, *b, fa, fb, true);
    joint->setLinearLowerLimit(linLo);
    joint->setLinearUpperLimit(linHi);
    joint->setAngularLowerLimit(angLo);
    joint->setAngularUpperLimit(angHi);
    for (int i = 0; i < 6; i++)
    {
        joint->enableSpring(i, springOn[i]);
        if (springOn[i])
        {
            joint->setStiffness(i, stiff[i]);
            joint->setDamping(i, damp[i]);
        }
    }
    world->addConstraint(joint);

    DUMP("cfg pA=(%.5f %.5f %.5f) qA=(%.5f %.5f %.5f %.5f)\n", pA.x(), pA.y(), pA.z(), qA.x(), qA.y(), qA.z(), qA.w());
    DUMP("cfg pB=(%.5f %.5f %.5f) qB=(%.5f %.5f %.5f %.5f) mB=%.5f\n",
        (pA + offB).x(), (pA + offB).y(), (pA + offB).z(), qB.x(), qB.y(), qB.z(), qB.w(), massB);
    DUMP("cfg fW org=(%.5f %.5f %.5f) qF=(%.5f %.5f %.5f %.5f)\n", midF.x(), midF.y(), midF.z(), qF.x(), qF.y(), qF.z(), qF.w());
    DUMP("cfg fa org=(%.5f %.5f %.5f) fb org=(%.5f %.5f %.5f)\n",
        fa.getOrigin().x(), fa.getOrigin().y(), fa.getOrigin().z(), fb.getOrigin().x(), fb.getOrigin().y(), fb.getOrigin().z());
    DUMP("cfg linLo=(%.3f %.3f %.3f) linHi=(%.3f %.3f %.3f)\n", linLo.x(), linLo.y(), linLo.z(), linHi.x(), linHi.y(), linHi.z());
    DUMP("cfg angLo=(%.3f %.3f %.3f) angHi=(%.3f %.3f %.3f)\n", angLo.x(), angLo.y(), angLo.z(), angHi.x(), angHi.y(), angHi.z());
    DUMP("cfg spring=%d%d%d%d%d%d stiff=(%.2f %.2f %.2f %.2f %.2f %.2f) damp=(%.3f %.3f %.3f %.3f %.3f %.3f)\n",
        springOn[0], springOn[1], springOn[2], springOn[3], springOn[4], springOn[5],
        stiff[0], stiff[1], stiff[2], stiff[3], stiff[4], stiff[5],
        damp[0], damp[1], damp[2], damp[3], damp[4], damp[5]);

    auto StepN = [&](int n) {
        for (int i = 0; i < n; i++)
            world->stepSimulation(1.0f / 60.0f, 1);
    };

    int stepNo = 0;
    int dumpStep = -1;
    if (const char* ds = getenv("FUZZ_DUMP_STEP")) dumpStep = atoi(ds);
    extern bool gSolverDump;
    if (argc > 2 && strcmp(argv[2], "pre") == 0)
    {
        // Collision detection only, at the initial poses (no solver, no integration)
        world->performDiscreteCollisionDetection();
        btDispatcher* disp = world->getDispatcher();
        int nm = disp->getNumManifolds();
        printf("PRE nm=%d\n", nm);
        for (int i = 0; i < nm; i++)
        {
            btPersistentManifold* m = disp->getManifoldByIndexInternal(i);
            const btCollisionObject* ob0 = (const btCollisionObject*)m->getBody0();
            const btCollisionObject* ob1 = (const btCollisionObject*)m->getBody1();
            printf("M %d pts=%d\n", i, m->getNumContacts());
            for (int j = 0; j < m->getNumContacts(); j++)
            {
                const btManifoldPoint& cp = m->getContactPoint(j);
                printf("P %d %d d=%.8e imp=%.8e pw=(%.8e %.8e %.8e) n=(%.8e %.8e %.8e) lA=(%.8e %.8e %.8e) lB=(%.8e %.8e %.8e)\n",
                    i, j, cp.getDistance(), cp.m_appliedImpulse,
                    cp.m_positionWorldOnB.x(), cp.m_positionWorldOnB.y(), cp.m_positionWorldOnB.z(),
                    cp.m_normalWorldOnB.x(), cp.m_normalWorldOnB.y(), cp.m_normalWorldOnB.z(),
                    cp.m_localPointA.x(), cp.m_localPointA.y(), cp.m_localPointA.z(),
                    cp.m_localPointB.x(), cp.m_localPointB.y(), cp.m_localPointB.z());
            }
        }
        return 0;
    }
    auto DumpStep = [&]() {
        if (!traceMode) return;
        // Solver internals are dumped DURING the step that is about to run when stepNo == dumpStep.
        if (dumpStep >= 0 && stepNo == dumpStep) gSolverDump = false;  // the dumped step just finished
        btTransform wt = b->getWorldTransform();
        btVector3 p = wt.getOrigin();
        btQuaternion q = wt.getRotation();
        btDispatcher* disp = world->getDispatcher();
        int nm = disp->getNumManifolds();
        printf("S %d pos=(%.8e %.8e %.8e) quat=(%.8e %.8e %.8e %.8e) nm=%d\n", stepNo, p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w(), nm);
        for (int i = 0; i < nm; i++)
        {
            btPersistentManifold* m = disp->getManifoldByIndexInternal(i);
            printf("M %d pts=%d\n", i, m->getNumContacts());
            for (int j = 0; j < m->getNumContacts(); j++)
            {
                const btManifoldPoint& cp = m->getContactPoint(j);
                printf("P %d %d d=%.8e imp=%.8e pw=(%.8e %.8e %.8e) n=(%.8e %.8e %.8e)\n", i, j, cp.getDistance(), cp.m_appliedImpulse,
                    cp.m_positionWorldOnB.x(), cp.m_positionWorldOnB.y(), cp.m_positionWorldOnB.z(),
                    cp.m_normalWorldOnB.x(), cp.m_normalWorldOnB.y(), cp.m_normalWorldOnB.z());
            }
        }
        stepNo++;
        if (dumpStep >= 0 && stepNo == dumpStep) gSolverDump = true;  // enable for the upcoming step
    };

    if (dumpStep == 0) gSolverDump = true;

    for (int i = 0; i < 60; i++)
    {
        world->stepSimulation(1.0f / 60.0f, 1);
        DumpStep();
    }

    {
        btVector3 sp = b->getWorldTransform().getOrigin();
        btQuaternion sq = b->getWorldTransform().getRotation();
        printf("settle pos=(%.5f %.5f %.5f) quat=(%.5f %.5f %.5f %.5f)\n", sp.x(), sp.y(), sp.z(), sq.x(), sq.y(), sq.z(), sq.w());
    }

    // Continuously move kinematic A
    for (int k = 0; k < 180; k++)
    {
        double dx = 0.2 * sin(k * 0.11), dy = 0.15 * sin(k * 0.07), dz = 0.1 * cos(k * 0.09);
        double wob = 0.15 * sin(k * 0.05);
        btQuaternion qk(btVector3(0, 1, 0), (btScalar)wob);
        btTransform mt;
        a->getMotionState()->getWorldTransform(mt);
        btTransform nt(qA * qk, pA + btVector3((btScalar)dx, (btScalar)dy, (btScalar)dz));
        a->getMotionState()->setWorldTransform(nt);
        world->stepSimulation(1.0f / 60.0f, 1);
        DumpStep();
    }

    btVector3 p = b->getWorldTransform().getOrigin();
    btQuaternion qb = b->getWorldTransform().getRotation();
    printf("seed=%d pos=(%.5f %.5f %.5f) quat=(%.5f %.5f %.5f %.5f)\n",
        seed, p.x(), p.y(), p.z(), qb.x(), qb.y(), qb.z(), qb.w());
    return 0;
}
