// Native Bullet SoftBody differential reference runner (mirrors BulletSharp.SmokeTests SoftDump.cs).
// Usage: SoftRef.exe <s1|s2a|s2b|s3|s4|s5a|s5b|all> [steps]
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

struct SoftWorldBundle
{
    btDbvtBroadphase* broadphase = nullptr;
    btSoftBodyRigidBodyCollisionConfiguration* config = nullptr;
    btCollisionDispatcher* dispatcher = nullptr;
    btSequentialImpulseConstraintSolver* solver = nullptr;
    btSoftRigidDynamicsWorld* world = nullptr;
    std::vector<btRigidBody*> rigidBodies;
    std::vector<btCollisionShape*> shapes;
    std::vector<btTriangleMesh*> meshes;

    void Destroy()
    {
        if (world)
        {
            while (world->getSoftBodyArray().size() > 0)
            {
                btSoftBody* sb = world->getSoftBodyArray()[0];
                world->removeSoftBody(sb);
                delete sb;
            }
            for (btRigidBody* rb : rigidBodies)
            {
                world->removeRigidBody(rb);
                delete rb->getMotionState();
                delete rb;
            }
        }
        for (btCollisionShape* sh : shapes)
            delete sh;
        for (btTriangleMesh* m : meshes)
            delete m;
        delete world;
        delete solver;
        delete dispatcher;
        delete config;
        delete broadphase;
    }
};

static SoftWorldBundle CreateSoftWorld()
{
    SoftWorldBundle b;
    b.broadphase = new btDbvtBroadphase();
    b.config = new btSoftBodyRigidBodyCollisionConfiguration();
    b.dispatcher = new btCollisionDispatcher(b.config);
    b.solver = new btSequentialImpulseConstraintSolver();
    b.world = new btSoftRigidDynamicsWorld(b.dispatcher, b.broadphase, b.solver, b.config);
    const btVector3 g(0, btScalar(-9.81), 0);
    b.world->setGravity(g);
    b.world->getWorldInfo().m_gravity = g;
    return b;
}

static btRigidBody* AddBody(SoftWorldBundle& b, btCollisionShape* shape, btScalar mass,
    const btVector3& pos, btScalar friction = btScalar(0.5), btScalar restitution = btScalar(0))
{
    b.shapes.push_back(shape);
    btVector3 inertia(0, 0, 0);
    if (mass != btScalar(0))
        shape->calculateLocalInertia(mass, inertia);
    btTransform start;
    start.setIdentity();
    start.setOrigin(pos);
    auto* ms = new btDefaultMotionState(start);
    btRigidBody::btRigidBodyConstructionInfo info(mass, ms, shape, inertia);
    info.m_friction = friction;
    info.m_restitution = restitution;
    auto* body = new btRigidBody(info);
    b.world->addRigidBody(body);
    b.rigidBodies.push_back(body);
    return body;
}

static void DumpTopo(int bi, const btSoftBody* sb)
{
    printf("  sb%d topo=(%d %d %d %d %d)\n",
        bi,
        sb->m_nodes.size(),
        sb->m_links.size(),
        sb->m_faces.size(),
        sb->m_tetras.size(),
        sb->m_clusters.size());
}

static void DumpRigid(int idx, const btRigidBody* rb)
{
    const btTransform& wt = rb->getWorldTransform();
    const btVector3& p = wt.getOrigin();
    btQuaternion q = wt.getRotation();
    const btVector3& lv = rb->getLinearVelocity();
    const btVector3& av = rb->getAngularVelocity();
    printf("  rb%d pos=(%.8e %.8e %.8e) q=(%.8e %.8e %.8e %.8e) lv=(%.8e %.8e %.8e) av=(%.8e %.8e %.8e)\n",
        idx,
        (double)p.x(), (double)p.y(), (double)p.z(),
        (double)q.x(), (double)q.y(), (double)q.z(), (double)q.w(),
        (double)lv.x(), (double)lv.y(), (double)lv.z(),
        (double)av.x(), (double)av.y(), (double)av.z());
}

static void DumpClusters(int bi, const btSoftBody* sb)
{
    for (int ci = 0; ci < sb->m_clusters.size(); ++ci)
    {
        const btSoftBody::Cluster* cl = sb->m_clusters[ci];
        printf("  sb%d cl%d com=(%.8e %.8e %.8e) lv=(%.8e %.8e %.8e) av=(%.8e %.8e %.8e)\n",
            bi, ci,
            (double)cl->m_com.x(), (double)cl->m_com.y(), (double)cl->m_com.z(),
            (double)cl->m_lv.x(), (double)cl->m_lv.y(), (double)cl->m_lv.z(),
            (double)cl->m_av.x(), (double)cl->m_av.y(), (double)cl->m_av.z());
    }
}

static void DumpNodes(int bi, const btSoftBody* sb, const int* indices, int count)
{
    for (int i = 0; i < count; ++i)
    {
        int ni = indices[i];
        const btSoftBody::Node& n = sb->m_nodes[ni];
        printf("  sb%d n%d x=(%.8e %.8e %.8e) v=(%.8e %.8e %.8e)\n",
            bi, ni,
            (double)n.m_x.x(), (double)n.m_x.y(), (double)n.m_x.z(),
            (double)n.m_v.x(), (double)n.m_v.y(), (double)n.m_v.z());
    }
}

static void StepWorld(btSoftRigidDynamicsWorld* world)
{
    const btScalar dt = btScalar(1) / btScalar(60);
    world->stepSimulation(dt, 10, dt);
}

// s1: CreateRope fixed head swinging + AppendAnchor hanging rigid box
static void RunS1(int steps)
{
    if (steps <= 0) steps = 180;
    SoftWorldBundle b = CreateSoftWorld();
    AddBody(b, new btBoxShape(btVector3(50, 1, 50)), 0, btVector3(0, -1, 0));

    btSoftBody* rope = btSoftBodyHelpers::CreateRope(
        b.world->getWorldInfo(), btVector3(0, 8, 0), btVector3(3, 8, 0), 8, 1);
    rope->m_cfg.piterations = 8;
    int lastNode = rope->m_nodes.size() - 1;

    btRigidBody* box = AddBody(b, new btBoxShape(btVector3(btScalar(0.4), btScalar(0.4), btScalar(0.4))),
        btScalar(1.0), btVector3(3, btScalar(7.5), 0));
    rope->appendAnchor(lastNode, box, true);
    b.world->addSoftBody(rope);

    const int sampleIdx[3] = { 0, 4, lastNode };
    printf("scene s1 steps=%d\n", steps);
    DumpTopo(0, rope);
    printf("step -1\n");
    DumpRigid(0, box);
    DumpNodes(0, rope, sampleIdx, 3);

    for (int s = 0; s < steps; ++s)
    {
        StepWorld(b.world);
        printf("step %d\n", s);
        DumpRigid(0, box);
        DumpNodes(0, rope, sampleIdx, 3);
    }
    b.Destroy();
}

// s2a: CreatePatch 4 corners fixed catching falling rigid sphere (SDF)
static void RunS2a(int steps)
{
    if (steps <= 0) steps = 180;
    SoftWorldBundle b = CreateSoftWorld();
    AddBody(b, new btBoxShape(btVector3(50, 1, 50)), 0, btVector3(0, -1, 0));

    btSoftBody* cloth = btSoftBodyHelpers::CreatePatch(
        b.world->getWorldInfo(),
        btVector3(-2, 6, -2),
        btVector3(2, 6, -2),
        btVector3(-2, 6, 2),
        btVector3(2, 6, 2),
        9, 9, 15, true);
    cloth->generateBendingConstraints(2);
    cloth->m_cfg.piterations = 8;
    cloth->setTotalMass(btScalar(12.0));
    b.world->addSoftBody(cloth);

    btRigidBody* sphere = AddBody(b, new btSphereShape(btScalar(0.8)),
        btScalar(0.3), btVector3(0, btScalar(7.5), 0));

    const int sampleIdx[5] = { 0, 20, 40, 60, 80 };
    printf("scene s2a steps=%d\n", steps);
    DumpTopo(0, cloth);
    printf("step -1\n");
    DumpRigid(0, sphere);
    DumpNodes(0, cloth, sampleIdx, 5);

    for (int s = 0; s < steps; ++s)
    {
        StepWorld(b.world);
        printf("step %d\n", s);
        DumpRigid(0, sphere);
        DumpNodes(0, cloth, sampleIdx, 5);
    }
    b.Destroy();
}

// s2b: Free CreatePatch falling onto BvhTriangleMeshShape concave ground
static void RunS2b(int steps)
{
    if (steps <= 0) steps = 180;
    SoftWorldBundle b = CreateSoftWorld();

    auto* mesh = new btTriangleMesh();
    b.meshes.push_back(mesh);
    for (int gz = -4; gz < 4; ++gz)
    {
        for (int gx = -4; gx < 4; ++gx)
        {
            btScalar x0 = gx * btScalar(2), x1 = (gx + 1) * btScalar(2);
            btScalar z0 = gz * btScalar(2), z1 = (gz + 1) * btScalar(2);
            mesh->addTriangle(btVector3(x0, 0, z0), btVector3(x0, 0, z1), btVector3(x1, 0, z1));
            mesh->addTriangle(btVector3(x0, 0, z0), btVector3(x1, 0, z1), btVector3(x1, 0, z0));
        }
    }
    auto* meshShape = new btBvhTriangleMeshShape(mesh, true);
    AddBody(b, meshShape, 0, btVector3(0, 0, 0));

    btSoftBody* freeCloth = btSoftBodyHelpers::CreatePatch(
        b.world->getWorldInfo(),
        btVector3(btScalar(-1.5), btScalar(2.5), btScalar(-1.5)),
        btVector3(btScalar(1.5), btScalar(2.5), btScalar(-1.5)),
        btVector3(btScalar(-1.5), btScalar(2.5), btScalar(1.5)),
        btVector3(btScalar(1.5), btScalar(2.5), btScalar(1.5)),
        6, 6, 0, true);
    freeCloth->generateBendingConstraints(2);
    freeCloth->m_cfg.piterations = 4;
    freeCloth->setTotalMass(btScalar(3.0));
    b.world->addSoftBody(freeCloth);

    const int sampleIdx[5] = { 0, 7, 14, 21, 35 };
    printf("scene s2b steps=%d\n", steps);
    DumpTopo(0, freeCloth);
    printf("step -1\n");
    DumpNodes(0, freeCloth, sampleIdx, 5);

    for (int s = 0; s < steps; ++s)
    {
        StepWorld(b.world);
        printf("step %d\n", s);
        DumpNodes(0, freeCloth, sampleIdx, 5);
    }
    b.Destroy();
}

// s3: CreateEllipsoid positive pressure (kPR > 0) inflation & bounce
static void RunS3(int steps)
{
    if (steps <= 0) steps = 200;
    SoftWorldBundle b = CreateSoftWorld();
    AddBody(b, new btBoxShape(btVector3(50, 1, 50)), 0, btVector3(0, -1, 0));

    btSoftBody* ellipsoid = btSoftBodyHelpers::CreateEllipsoid(
        b.world->getWorldInfo(), btVector3(0, 4, 0), btVector3(1, 1, 1), 64);
    ellipsoid->m_materials[0]->m_kLST = btScalar(0.2);
    ellipsoid->m_cfg.kDF = btScalar(1.0);
    ellipsoid->m_cfg.kDP = btScalar(0.001);
    ellipsoid->m_cfg.kPR = btScalar(800);
    ellipsoid->m_cfg.piterations = 4;
    ellipsoid->setTotalMass(btScalar(30));
    b.world->addSoftBody(ellipsoid);

    int n = ellipsoid->m_nodes.size();
    const int sampleIdx[5] = { 0, n / 4, n / 2, (3 * n) / 4, n - 1 };
    printf("scene s3 steps=%d\n", steps);
    DumpTopo(0, ellipsoid);
    printf("step -1\n");
    printf("  sb0 vol=%.8e\n", (double)ellipsoid->getVolume());
    DumpNodes(0, ellipsoid, sampleIdx, 5);

    for (int s = 0; s < steps; ++s)
    {
        StepWorld(b.world);
        printf("step %d\n", s);
        printf("  sb0 vol=%.8e\n", (double)ellipsoid->getVolume());
        DumpNodes(0, ellipsoid, sampleIdx, 5);
    }
    b.Destroy();
}

// s4: Tetrahedral soft body free-fall, ground collision & pose recovery
static void RunS4(int steps)
{
    if (steps <= 0) steps = 180;
    SoftWorldBundle b = CreateSoftWorld();
    AddBody(b, new btBoxShape(btVector3(50, 1, 50)), 0, btVector3(0, -1, 0));

    // Mirrors CreateFromTetGenData(tetEleData, null, tetNodeData, false, true, true)
    // without native CreateFromTetGenData's debug printf("\r\n") lines.
    const btVector3 tetPos[8] = {
        btVector3(btScalar(-0.5), btScalar(-0.5), btScalar(-0.5)),
        btVector3(btScalar( 0.5), btScalar(-0.5), btScalar(-0.5)),
        btVector3(btScalar( 0.5), btScalar( 0.5), btScalar(-0.5)),
        btVector3(btScalar(-0.5), btScalar( 0.5), btScalar(-0.5)),
        btVector3(btScalar(-0.5), btScalar(-0.5), btScalar( 0.5)),
        btVector3(btScalar( 0.5), btScalar(-0.5), btScalar( 0.5)),
        btVector3(btScalar( 0.5), btScalar( 0.5), btScalar( 0.5)),
        btVector3(btScalar(-0.5), btScalar( 0.5), btScalar( 0.5)),
    };
    const int tetIdx[5][4] = {
        { 0, 1, 3, 4 },
        { 2, 3, 1, 6 },
        { 5, 1, 4, 6 },
        { 7, 4, 3, 6 },
        { 1, 3, 4, 6 },
    };
    auto* tetBody = new btSoftBody(&b.world->getWorldInfo(), 8, tetPos, nullptr);
    for (int i = 0; i < 5; ++i)
    {
        const int* ni = tetIdx[i];
        tetBody->appendTetra(ni[0], ni[1], ni[2], ni[3]);
        tetBody->appendLink(ni[0], ni[1], nullptr, true);
        tetBody->appendLink(ni[1], ni[2], nullptr, true);
        tetBody->appendLink(ni[2], ni[0], nullptr, true);
        tetBody->appendLink(ni[0], ni[3], nullptr, true);
        tetBody->appendLink(ni[1], ni[3], nullptr, true);
        tetBody->appendLink(ni[2], ni[3], nullptr, true);
    }
    tetBody->initializeDmInverse();
    tetBody->m_tetraScratches.resize(tetBody->m_tetras.size());
    tetBody->m_tetraScratchesTn.resize(tetBody->m_tetras.size());
    btSoftBodyHelpers::generateBoundaryFaces(tetBody);

    tetBody->m_materials[0]->m_kLST = btScalar(0.6);
    tetBody->m_materials[0]->m_kVST = btScalar(0.8);
    tetBody->m_cfg.piterations = 5;
    tetBody->m_cfg.kMT = btScalar(0.2);
    tetBody->setVolumeMass(btScalar(4.0));
    tetBody->setPose(false, true);

    btQuaternion tilt = btQuaternion(btScalar(0.25), btScalar(0.1), btScalar(0.2), btScalar(0.94)).normalized();
    tetBody->rotate(tilt);
    tetBody->translate(btVector3(0, btScalar(3.5), 0));
    tetBody->addVelocity(btVector3(btScalar(1.0), 0, 0), 0);
    b.world->addSoftBody(tetBody);

    const int sampleIdx[4] = { 0, 2, 5, 7 };
    printf("scene s4 steps=%d\n", steps);
    DumpTopo(0, tetBody);
    printf("step -1\n");
    printf("  sb0 pose_com=(%.8e %.8e %.8e)\n",
        (double)tetBody->m_pose.m_com.x(), (double)tetBody->m_pose.m_com.y(), (double)tetBody->m_pose.m_com.z());
    DumpNodes(0, tetBody, sampleIdx, 4);

    for (int s = 0; s < steps; ++s)
    {
        StepWorld(b.world);
        printf("step %d\n", s);
        printf("  sb0 pose_com=(%.8e %.8e %.8e)\n",
            (double)tetBody->m_pose.m_com.x(), (double)tetBody->m_pose.m_com.y(), (double)tetBody->m_pose.m_com.z());
        DumpNodes(0, tetBody, sampleIdx, 4);
    }
    b.Destroy();
}

// s5a: GenerateClusters + LJoint / AJoint coupling
static void RunS5a(int steps)
{
    if (steps <= 0) steps = 180;
    SoftWorldBundle b = CreateSoftWorld();
    AddBody(b, new btBoxShape(btVector3(50, 1, 50)), 0, btVector3(0, -1, 0));

    btSoftBody* softA = btSoftBodyHelpers::CreateEllipsoid(
        b.world->getWorldInfo(), btVector3(0, 6, 0), btVector3(btScalar(0.6), btScalar(0.6), btScalar(0.6)), 32);
    btSoftBody* softB = btSoftBodyHelpers::CreateEllipsoid(
        b.world->getWorldInfo(), btVector3(2, 6, 0), btVector3(btScalar(0.6), btScalar(0.6), btScalar(0.6)), 32);

    btSoftBody* pair[2] = { softA, softB };
    for (btSoftBody* sb : pair)
    {
        sb->m_materials[0]->m_kLST = btScalar(0.8);
        sb->m_cfg.kDF = btScalar(1.0);
        sb->m_cfg.piterations = 4;
        sb->m_cfg.citerations = 8;
        sb->m_cfg.collisions = btSoftBody::fCollision::CL_RS | btSoftBody::fCollision::CL_SS;
        sb->setTotalMass(btScalar(2.0));
    }

    softA->generateClusters(1);
    softB->generateClusters(1);
    b.world->addSoftBody(softA);
    b.world->addSoftBody(softB);

    btSoftBody::LJoint::Specs ljWorld;
    ljWorld.position = btVector3(0, 6, 0);
    softA->appendLinearJoint(ljWorld, btSoftBody::Body());

    btSoftBody::AJoint::Specs ajWorld;
    ajWorld.axis = btVector3(0, 0, 1);
    softA->appendAngularJoint(ajWorld, btSoftBody::Body());

    btSoftBody::LJoint::Specs ljAB;
    ljAB.position = btVector3(1, 6, 0);
    softA->appendLinearJoint(ljAB, softB);

    btSoftBody::AJoint::Specs ajAB;
    ajAB.axis = btVector3(0, 0, 1);
    softA->appendAngularJoint(ajAB, softB);

    int nA = softA->m_nodes.size();
    int nB = softB->m_nodes.size();
    const int idxA[3] = { 0, nA / 2, nA - 1 };
    const int idxB[3] = { 0, nB / 2, nB - 1 };

    printf("scene s5a steps=%d\n", steps);
    DumpTopo(0, softA);
    DumpTopo(1, softB);
    printf("step -1\n");
    DumpClusters(0, softA);
    DumpClusters(1, softB);
    DumpNodes(0, softA, idxA, 3);
    DumpNodes(1, softB, idxB, 3);

    for (int s = 0; s < steps; ++s)
    {
        StepWorld(b.world);
        printf("step %d\n", s);
        DumpClusters(0, softA);
        DumpClusters(1, softB);
        DumpNodes(0, softA, idxA, 3);
        DumpNodes(1, softB, idxB, 3);
    }
    b.Destroy();
}

// s5b: Multi-cluster CL_RS & CL_SS GJK/EPA soft-soft stacking collision
static void RunS5b(int steps)
{
    if (steps <= 0) steps = 180;
    SoftWorldBundle b = CreateSoftWorld();
    AddBody(b, new btBoxShape(btVector3(50, 1, 50)), 0, btVector3(0, -1, 0));

    btSoftBody* softBottom = btSoftBodyHelpers::CreateEllipsoid(
        b.world->getWorldInfo(), btVector3(0, btScalar(0.8), 0),
        btVector3(btScalar(1.6), btScalar(0.5), btScalar(1.6)), 48);
    btSoftBody* softTop = btSoftBodyHelpers::CreateEllipsoid(
        b.world->getWorldInfo(), btVector3(0, btScalar(2.2), 0),
        btVector3(btScalar(1.0), btScalar(0.5), btScalar(1.0)), 48);

    btSoftBody* pair[2] = { softBottom, softTop };
    for (btSoftBody* sb : pair)
    {
        sb->m_materials[0]->m_kLST = btScalar(0.8);
        sb->m_cfg.kDF = btScalar(1.0);
        sb->m_cfg.piterations = 4;
        sb->m_cfg.citerations = 6;
        sb->m_cfg.collisions = btSoftBody::fCollision::CL_RS | btSoftBody::fCollision::CL_SS;
        sb->setTotalMass(btScalar(3.0));
    }

    softBottom->generateClusters(4);
    softTop->generateClusters(4);
    b.world->addSoftBody(softBottom);
    b.world->addSoftBody(softTop);

    int nBot = softBottom->m_nodes.size();
    int nTop = softTop->m_nodes.size();
    const int idxBot[3] = { 0, nBot / 2, nBot - 1 };
    const int idxTop[3] = { 0, nTop / 2, nTop - 1 };

    printf("scene s5b steps=%d\n", steps);
    DumpTopo(0, softBottom);
    DumpTopo(1, softTop);
    printf("step -1\n");
    DumpClusters(0, softBottom);
    DumpClusters(1, softTop);
    DumpNodes(0, softBottom, idxBot, 3);
    DumpNodes(1, softTop, idxTop, 3);

    for (int s = 0; s < steps; ++s)
    {
        StepWorld(b.world);
        printf("step %d\n", s);
        DumpClusters(0, softBottom);
        DumpClusters(1, softTop);
        DumpNodes(0, softBottom, idxBot, 3);
        DumpNodes(1, softTop, idxTop, 3);
    }
    b.Destroy();
}

int main(int argc, char** argv)
{
    const char* scene = argc > 1 ? argv[1] : "all";
    int steps = argc > 2 ? atoi(argv[2]) : 0;

    if (strcmp(scene, "s1") == 0) RunS1(steps);
    else if (strcmp(scene, "s2a") == 0) RunS2a(steps);
    else if (strcmp(scene, "s2b") == 0) RunS2b(steps);
    else if (strcmp(scene, "s3") == 0) RunS3(steps);
    else if (strcmp(scene, "s4") == 0) RunS4(steps);
    else if (strcmp(scene, "s5a") == 0) RunS5a(steps);
    else if (strcmp(scene, "s5b") == 0) RunS5b(steps);
    else if (strcmp(scene, "all") == 0)
    {
        RunS1(steps);
        RunS2a(steps);
        RunS2b(steps);
        RunS3(steps);
        RunS4(steps);
        RunS5a(steps);
        RunS5b(steps);
    }
    else
    {
        fprintf(stderr, "Unknown scene '%s'. Expected s1|s2a|s2b|s3|s4|s5a|s5b|all\n", scene);
        return 1;
    }
    return 0;
}
