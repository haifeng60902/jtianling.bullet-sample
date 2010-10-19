/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "stdafx.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "GlutDemoApplication.h"
#include "btBulletDynamicsCommon.h"
#include <stdio.h>
#include <iostream>
#include <list>
using namespace std;
class CollideDetectionDemo : public GlutDemoApplication
{
	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
	btBroadphaseInterface*	m_broadphase;
	btCollisionDispatcher*	m_dispatcher;
	btConstraintSolver*	m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;

	typedef std::list<btCollisionObject*> CollisionObjects_t;
	CollisionObjects_t m_collisionObjects;
public:
	CollideDetectionDemo()
	{
	}
	virtual ~CollideDetectionDemo()
	{
		exitPhysics();
	}
	void	initPhysics();
	void	exitPhysics();
	virtual void clientMoveAndDisplay();
	virtual void displayCallback();

	void RemoveObject(btCollisionObject* obj);
};

void CollideDetectionDemo::clientMoveAndDisplay()
{
	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
	}

	//Assume world->stepSimulation or world->performDiscreteCollisionDetection has been called

	int numManifolds = m_dynamicsWorld->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++) {
		btPersistentManifold* contactManifold =  m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());

		int numContacts = contactManifold->getNumContacts();
		for (int j=0;j<numContacts;j++) {
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			if (pt.getDistance()<0.f) {
				if (!obA->isStaticObject() && !obB->isStaticObject()) {
					m_collisionObjects.push_back(obA);
					m_collisionObjects.push_back(obB);
				}
			}
		}
	}

	m_collisionObjects.sort();
	m_collisionObjects.unique();
	for (CollisionObjects_t::iterator itr = m_collisionObjects.begin();
		itr != m_collisionObjects.end();
		++itr) {
		RemoveObject(*itr);
	}

	m_collisionObjects.clear();


	displayCallback();
}

void CollideDetectionDemo::RemoveObject(btCollisionObject* obj) {

	btRigidBody* body = btRigidBody::upcast(obj);
	if (body && body->getMotionState())
	{
		delete body->getMotionState();
	}
	m_dynamicsWorld->removeCollisionObject( obj );
	delete obj;
}

void CollideDetectionDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}

void CollideDetectionDemo::initPhysics()
{
	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(20.0));

	// create the physics world
	m_collisionConfiguration = new btDefaultCollisionConfiguration;
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_broadphase = new btDbvtBroadphase;
	m_solver = new btSequentialImpulseConstraintSolver;
	btDiscreteDynamicsWorld* world = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	world->setGravity(btVector3(0,-10,0));

	m_dynamicsWorld = world;

	///create a few basic rigid bodies

	// create the ground
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));

		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-56,0));

		btScalar mass(0.);

		btVector3 localInertia(0,0,0);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		rbInfo.m_restitution = 0.5;
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		world->addRigidBody(body);
	}


	{
		//create a dynamic rigidbody(a ball)
		btCollisionShape* colShape = new btSphereShape(btScalar(1.0));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(btVector3(0,10,0));
		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
		rbInfo.m_restitution = 0.5;
		btRigidBody* body = new btRigidBody(rbInfo);
		world->addRigidBody(body);
	}


	//{
	//	//create a dynamic rigidbody(a box)

	//	btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
	//	m_collisionShapes.push_back(colShape);

	//	/// Create Dynamic Objects
	//	btTransform startTransform;
	//	startTransform.setIdentity();
	//	startTransform.setOrigin(btVector3(2,10,0));
	//	btScalar	mass(1.f);

	//	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	//	bool isDynamic = (mass != 0.f);

	//	btVector3 localInertia(0,0,0);
	//	if (isDynamic)
	//		colShape->calculateLocalInertia(mass,localInertia);

	//	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	//	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	//	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
	//	rbInfo.m_restitution = 0.5;
	//	btRigidBody* body = new btRigidBody(rbInfo);

	//	m_dynamicsWorld->addRigidBody(body);
	//}


	//{
	//	//create a dynamic rigidbody(capsule)
	//	btCollisionShape* colShape = new btCapsuleShape(btScalar(1.0), btScalar(3.0));
	//	m_collisionShapes.push_back(colShape);

	//	/// Create Dynamic Objects
	//	btTransform startTransform;
	//	startTransform.setIdentity();
	//	startTransform.setOrigin(btVector3(4,10,0));
	//	btScalar	mass(1.f);

	//	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	//	bool isDynamic = (mass != 0.f);

	//	btVector3 localInertia(0,0,0);
	//	if (isDynamic)
	//		colShape->calculateLocalInertia(mass,localInertia);

	//	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	//	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	//	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
	//	rbInfo.m_restitution = 0.5;
	//	btRigidBody* body = new btRigidBody(rbInfo);

	//	m_dynamicsWorld->addRigidBody(body);
	//}

	//{
	//	//create a dynamic rigidbody(a ball or box you choosed)

	//	btCollisionShape* colShape = new btCylinderShape(btVector3(1.0, 3.0, 1.0));
	//	m_collisionShapes.push_back(colShape);

	//	/// Create Dynamic Objects
	//	btTransform startTransform;
	//	startTransform.setIdentity();
	//	startTransform.setOrigin(btVector3(6,10,0));
	//	btScalar	mass(1.f);

	//	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	//	bool isDynamic = (mass != 0.f);

	//	btVector3 localInertia(0,0,0);
	//	if (isDynamic)
	//		colShape->calculateLocalInertia(mass,localInertia);

	//	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	//	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	//	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
	//	rbInfo.m_restitution = 0.5;
	//	btRigidBody* body = new btRigidBody(rbInfo);

	//	m_dynamicsWorld->addRigidBody(body);
	//}

	//{
	//	//create a dynamic rigidbody(a ball or box you choosed)

	//	btCollisionShape* colShape = new btConeShape(btScalar(1.0), btScalar(3.0));
	//	m_collisionShapes.push_back(colShape);

	//	/// Create Dynamic Objects
	//	btTransform startTransform;
	//	startTransform.setIdentity();
	//	startTransform.setOrigin(btVector3(8,10,0));
	//	btScalar	mass(1.f);

	//	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	//	bool isDynamic = (mass != 0.f);

	//	btVector3 localInertia(0,0,0);
	//	if (isDynamic)
	//		colShape->calculateLocalInertia(mass,localInertia);

	//	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	//	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	//	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
	//	rbInfo.m_restitution = 0.5;
	//	btRigidBody* body = new btRigidBody(rbInfo);

	//	m_dynamicsWorld->addRigidBody(body);
	//}

	clientResetScene();
}

void CollideDetectionDemo::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		RemoveObject(obj);
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	delete m_dynamicsWorld;

	delete m_solver;

	delete m_broadphase;

	delete m_dispatcher;

	delete m_collisionConfiguration;
}
/// This is a Hello World program for running a basic Bullet physics simulation

int main(int argc, char** argv)
{
	GLDebugDrawer debugDrawer;
	debugDrawer.setDebugMode(btIDebugDraw::DBG_DrawAabb | btIDebugDraw::DBG_DrawWireframe);
	CollideDetectionDemo demo;
	demo.initPhysics();
	demo.getDynamicsWorld()->setDebugDrawer(&debugDrawer);
	return glutmain(argc, argv, 640, 480,"First Bullet Physics Demo. http://blog.csdn.net/vagrixe", &demo);
}