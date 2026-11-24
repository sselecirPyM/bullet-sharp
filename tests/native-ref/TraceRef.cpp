// Per-step trace of an MMD-like capsule-chain + 6dof spring scenario.
// Mirrors tests/BulletSharp.SmokeTests/Trace.cs — output must match line-for-line.
// Usage: TraceRef.exe [steps]
#include "btBulletDynamicsCommon.h"
#include <cstdio>
#include <cmath>

extern bool gSolverDump;

static void DumpBody(const char* tag, btRigidBody* b)
{
	const btVector3& p = b->getWorldTransform().getOrigin();
	const btQuaternion& q = b->getWorldTransform().getRotation();
	const btVector3& lv = b->getLinearVelocity();
	const btVector3& av = b->getAngularVelocity();
	const btVector3& pv = b->getPushVelocity();
	const btVector3& tv = b->getTurnVelocity();
	const btMatrix3x3& iit = b->getInvInertiaTensorWorld();
	printf("%s pos=(%.8e %.8e %.8e) q=(%.8e %.8e %.8e %.8e) lv=(%.8e %.8e %.8e) av=(%.8e %.8e %.8e) pv=(%.8e %.8e %.8e) tv=(%.8e %.8e %.8e) iit=(%.8e %.8e %.8e %.8e %.8e %.8e %.8e %.8e %.8e)\n",
		tag, p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w(),
		lv.x(), lv.y(), lv.z(), av.x(), av.y(), av.z(),
		pv.x(), pv.y(), pv.z(), tv.x(), tv.y(), tv.z(),
		iit[0].x(), iit[0].y(), iit[0].z(), iit[1].x(), iit[1].y(), iit[1].z(), iit[2].x(), iit[2].y(), iit[2].z());
}

int main(int argc, char** argv)
{
	int steps = argc > 1 ? atoi(argv[1]) : 30;

	auto* broadphase = new btDbvtBroadphase();
	auto* config = new btDefaultCollisionConfiguration();
	auto* dispatcher = new btCollisionDispatcher(config);
	auto* solver = new btSequentialImpulseConstraintSolver();
	auto* world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config);
	world->setGravity(btVector3(0, -9.801f, 0));

	btScalar dt = (btScalar)(1.0 / 60.0);

	// kinematic root (like a bone-driven MMD rigid body)
	auto* rootShape = new btSphereShape(0.1f);
	btTransform rootT(btQuaternion::getIdentity(), btVector3(0, 1.6f, 0));
	auto* root = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
		0, new btDefaultMotionState(rootT), rootShape, btVector3(0, 0, 0)));
	root->setActivationState(DISABLE_DEACTIVATION);
	root->setSleepingThresholds(0, 0);
	root->setCollisionFlags(root->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	world->addRigidBody(root, 1, 0);

	// chain of capsules (like hair/skirt links)
	const int N = 4;
	btRigidBody* chain[N];
	btScalar y = 1.6f;
	for (int i = 0; i < N; i++)
	{
		auto* shape = new btCapsuleShape(0.08f, 0.22f);
		btScalar mass = 1.0f;
		btVector3 inertia(0, 0, 0);
		shape->calculateLocalInertia(mass, inertia);
		btTransform t(btQuaternion::getIdentity(), btVector3(0, y - 0.15f, 0));
		btRigidBody::btRigidBodyConstructionInfo ci(mass, new btDefaultMotionState(t), shape, inertia);
		ci.m_friction = 0.5f;
		ci.m_restitution = 0.0f;
		ci.m_linearDamping = 0.5f;
		ci.m_angularDamping = 0.5f;
		chain[i] = new btRigidBody(ci);
		chain[i]->setActivationState(DISABLE_DEACTIVATION);
		chain[i]->setSleepingThresholds(0, 0);
		world->addRigidBody(chain[i], 2, 0);
		y -= 0.3f;
	}

	// joints: 6dof springs (MMD-like)
	btGeneric6DofSpringConstraint* joints[N];
	{
		btTransform prevFrame = rootT; // approx: joint at each body center
		for (int i = 0; i < N; i++)
		{
			btRigidBody* rbA = i == 0 ? root : chain[i - 1];
			btRigidBody* rbB = chain[i];
			// joint world transform: midpoint between the two body centers
			btVector3 jpos = (rbA->getWorldTransform().getOrigin() + rbB->getWorldTransform().getOrigin()) * 0.5f;
			btTransform jw(btQuaternion::getIdentity(), jpos);
			btTransform fa = rbA->getWorldTransform().inverse() * jw;
			btTransform fb = rbB->getWorldTransform().inverse() * jw;
			auto* j = new btGeneric6DofSpringConstraint(*rbA, *rbB, fa, fb, true);
			j->setLinearLowerLimit(btVector3(0, 0, 0));
			j->setLinearUpperLimit(btVector3(0, 0, 0));
			j->setAngularLowerLimit(btVector3(-0.3f, -0.5f, -0.3f));
			j->setAngularUpperLimit(btVector3(0.3f, 0.5f, 0.3f));
			// MMD: angular springs on
			for (int a = 3; a < 6; a++)
			{
				j->enableSpring(a, true);
				j->setStiffness(a, 100.0f);
			}
			world->addConstraint(j);
			joints[i] = j;
		}
	}

	for (int s = 0; s < steps; s++)
	{
		// drive the kinematic root through its motion state (like PrePhysicsSync)
		double k = s;
		double dx = 0.25 * sin(k * 0.05), dz = 0.25 * cos(k * 0.07);
		double wob = 0.3 * sin(k * 0.03);
		btQuaternion qk(btVector3(0, 1, 0), (btScalar)wob);
		btTransform nt(qk, btVector3((btScalar)dx, 1.6f, (btScalar)dz));
		root->getMotionState()->setWorldTransform(nt);
		gSolverDump = s == 2;  // dump solver internals for step 2 only
		if (s == 2)
		{
			for (int i = 0; i < world->getNumCollisionObjects(); i++)
			{
				btCollisionObject* o = world->getCollisionObjectArray()[i];
				btVector3 p = ((btRigidBody*)o)->getWorldTransform().getOrigin();
				printf("TAG y=%.5f tag=%d\n", p.y(), o->getIslandTag());
			}
			for (int i = 0; i < world->getNumConstraints(); i++)
			{
				btTypedConstraint* c = world->getConstraint(i);
				int id = c->getRigidBodyA().getIslandTag() >= 0 ? c->getRigidBodyA().getIslandTag() : c->getRigidBodyB().getIslandTag();
				printf("CID j=%d id=%d\n", i, id);
			}
		}
		world->stepSimulation(dt, 1);
		gSolverDump = false;

		printf("step %d\n", s);
		DumpBody("  root", root);
		for (int i = 0; i < N; i++)
		{
			char tag[32];
			sprintf(tag, "  c%d", i);
			DumpBody(tag, chain[i]);
		}
		// joint internals as used by the solver during this step
		for (int i = 0; i < N; i++)
		{
			btGeneric6DofSpringConstraint* j = joints[i];
			printf("  j%d ang=(%.8e %.8e %.8e) lin=(%.8e %.8e %.8e) ai=", i,
				j->getAngle(0), j->getAngle(1), j->getAngle(2),
				j->getRelativePivotPosition(0), j->getRelativePivotPosition(1), j->getRelativePivotPosition(2));
			for (int a = 0; a < 3; a++)
			{
				btRotationalLimitMotor* lm = j->getRotationalLimitMotor(a);
				printf("%d/%.8e/%.8e/%.8e/%.8e ", lm->m_currentLimit, lm->m_currentLimitError,
					lm->m_currentPosition, lm->m_targetVelocity, lm->m_maxMotorForce);
			}
			printf("ai=%.8e imp=%.8e\n", j->getAngle(1), (double)j->getAppliedImpulse());
			fflush(stdout);
		}
	}
	return 0;
}
