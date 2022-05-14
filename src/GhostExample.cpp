#include <ros/package.h>
#include "GhostExample.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase_G.h"


#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include <iostream>
#include <unistd.h>

#include "../Importers/ImportSTLDemo/LoadMeshFromSTL.h"
#include "../Utils/b3ResourcePath.h"
#include "../Utils/b3BulletDefaultFileIO.h"
#include <vector>
#include <algorithm>

using namespace std;

//mamad note look at CommonRigidBodyBase_G 
// class Thumb{
// 
// }

class Cube{
	btDiscreteDynamicsWorld *m_dynamicsWorld;
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
	public:
	Cube(btDiscreteDynamicsWorld *m_dynamicsWorld,btAlignedObjectArray<btCollisionShape*> m_collisionShapes,vector<double> offset);
	
	btBoxShape* createBoxShape(const btVector3& halfExtents);
	btRigidBody* createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, const btVector4& color = btVector4(1, 0, 0, 1));
};
Cube::Cube(btDiscreteDynamicsWorld *dynamicsWorld,btAlignedObjectArray<btCollisionShape*> collisionShapes,vector<double> offset ){
	//setting class members
    m_dynamicsWorld   = dynamicsWorld;
	m_collisionShapes = collisionShapes;
	
	btBoxShape* colShape = createBoxShape(btVector3(0.02, 0.02, 0.02));

	m_collisionShapes.push_back(colShape);

	/// Create Dynamic Objects
	btTransform startTransform_cube;
	startTransform_cube.setIdentity();
	startTransform_cube.setRotation(btQuaternion(0.717,0.0,0.0,0.717));
	startTransform_cube.setOrigin(btVector3(0+offset[0], -0.3+offset[1], 0.765+offset[2]));

	btScalar mass_cube(0.f);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass_cube != 0.f);

	btVector3 localInertia_cube(0, 0, 0);
	if (isDynamic)
		colShape->calculateLocalInertia(mass_cube, localInertia_cube);

	createRigidBody(mass_cube, startTransform_cube, colShape);
}
btBoxShape* Cube::createBoxShape(const btVector3& halfExtents)
	{
		btBoxShape* box = new btBoxShape(halfExtents);
		return box;
	}
btRigidBody* Cube::createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, const btVector4& color)
	{
		btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			shape->calculateLocalInertia(mass, localInertia);

			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

		#define USE_MOTIONSTATE 1
		#ifdef USE_MOTIONSTATE
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

		btRigidBody* body = new btRigidBody(cInfo);
		//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

		#else
		btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
		body->setWorldTransform(startTransform);
		#endif  //

		body->setUserIndex(-1);
		m_dynamicsWorld->addRigidBody(body);
		return body;
	}


class Finger{
	
	btDiscreteDynamicsWorld *m_dynamicsWorld;
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
	public:
	btGhostObject* finger_ws ;
	btCollisionShape* meshShape;
	Finger(btDiscreteDynamicsWorld *m_dynamicsWorld,btAlignedObjectArray<btCollisionShape*> m_collisionShapes,vector<double> offset );

	
};
Finger::Finger(btDiscreteDynamicsWorld *dynamicsWorld,btAlignedObjectArray<btCollisionShape*> collisionShapes,vector<double> offset ){
	//setting class members
    m_dynamicsWorld   = dynamicsWorld;
	m_collisionShapes = collisionShapes;

	std::string path = ros::package::getPath("handGhostws");
	std::string finger_path = path +  "/model/WS_mesh.STL";
	std::cout<<"path"<<finger_path<<std::endl;
	const char* m_fileName = finger_path.c_str() ;
	//const char* m_fileName = "./WS_mesh.STL";
	char relativeFileName[1024];
	if (!b3ResourcePath::findResourcePath(m_fileName, relativeFileName, 1024,0))
	{
		b3Warning("Cannot find file %s\n", m_fileName);
		return;
	}
	b3BulletDefaultFileIO fileIO;
	GLInstanceGraphicsShape* gfxShape = LoadMeshFromSTL(relativeFileName,&fileIO);

	// Convert to collision shape
	const GLInstanceVertex& v = gfxShape->m_vertices->at(0);
	
	int numTriangles = int(gfxShape->m_numvertices / 3);
	int* triangleIndexBase = &gfxShape->m_indices->at(0);
	int triangleIndexStride = 12;
	int numVertices = gfxShape->m_numvertices;
	btScalar* vertexBase = &gfxShape->m_vertices->at(0).xyzw[0];
	int vertexStride = sizeof(v);
	btStridingMeshInterface* arrays = new btTriangleIndexVertexArray(numTriangles, triangleIndexBase, triangleIndexStride, numVertices, vertexBase, vertexStride);


	meshShape = new btBvhTriangleMeshShape(arrays, true);
	meshShape->setLocalScaling(btVector3(0.0015, 0.0015, 0.0015));
	m_collisionShapes.push_back(meshShape);

	btGhostPairCallback* ghostCall = new btGhostPairCallback();
	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(ghostCall);

	// Create the ghost object
	btGhostObject* finger_ws = new btGhostObject();

	//collisionObjects.push_back(ghostObject);
	finger_ws->setCollisionShape(meshShape);

	btTransform startTransform_ff;	
	startTransform_ff.setIdentity();
	startTransform_ff.setOrigin(btVector3(0.+offset[0], 5.+offset[1], 0.+offset[2]));
	finger_ws->setWorldTransform(startTransform_ff);
			
	finger_ws->setUserIndex(-1);
	m_dynamicsWorld->addCollisionObject(finger_ws, btBroadphaseProxy::SensorTrigger, btBroadphaseProxy::AllFilter & ~btBroadphaseProxy::SensorTrigger);

}

class Thumb{
	
	btDiscreteDynamicsWorld *m_dynamicsWorld;
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
	public:
	btGhostObject* finger_ws ;
	btCollisionShape* meshShape;
	Thumb(btDiscreteDynamicsWorld *m_dynamicsWorld,btAlignedObjectArray<btCollisionShape*> m_collisionShapes,vector<double> offset);

	
};
Thumb::Thumb(btDiscreteDynamicsWorld *dynamicsWorld,btAlignedObjectArray<btCollisionShape*> collisionShapes,vector<double> offset ){
	//setting class members
    m_dynamicsWorld   = dynamicsWorld;
	m_collisionShapes = collisionShapes;

	std::string path = ros::package::getPath("handGhostws");
	std::string th_path = path +  "/model/THWS2.stl";
	std::cout<<"path"<<th_path<<std::endl;
	const char* m_fileName = th_path.c_str() ;
	//const char* m_fileName = "./WS_mesh.STL";
	char relativeFileName[1024];
	if (!b3ResourcePath::findResourcePath(m_fileName, relativeFileName, 1024,0))
	{
		b3Warning("Cannot find file %s\n", m_fileName);
		return;
	}
	b3BulletDefaultFileIO fileIO;
	GLInstanceGraphicsShape* gfxShape_th = LoadMeshFromSTL(relativeFileName,&fileIO);

	// Convert to collision shape
	const GLInstanceVertex& v_th = gfxShape_th->m_vertices->at(0);
	int numTriangles_th = int(gfxShape_th->m_numvertices / 3);
	int* triangleIndexBase_th = &gfxShape_th->m_indices->at(0);
	int triangleIndexStride_th = 12;
	int numVertices_th = gfxShape_th->m_numvertices;
	btScalar* vertexBase_th = &gfxShape_th->m_vertices->at(0).xyzw[0];
	int vertexStride_th = sizeof(v_th);
	btStridingMeshInterface* arrays_th = new btTriangleIndexVertexArray(numTriangles_th, triangleIndexBase_th, triangleIndexStride_th, numVertices_th, vertexBase_th, vertexStride_th);

	btCollisionShape* meshShape_th = new btBvhTriangleMeshShape(arrays_th, true);
	meshShape_th->setLocalScaling(btVector3(0.0015, 0.0015, 0.0015));
		
	m_collisionShapes.push_back(meshShape_th);

	btGhostPairCallback* ghostCall_th = new btGhostPairCallback();
	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(ghostCall_th);

	// Create the ghost object for thumb
	btGhostObject* thWorkspace = new btGhostObject();
	thWorkspace->setCollisionShape(meshShape_th);

	btTransform startTransform;	
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0.+offset[0], 5.+offset[1], 0.+offset[2]));
	thWorkspace->setWorldTransform(startTransform);
			
	thWorkspace->setUserIndex(-1);

	m_dynamicsWorld->addCollisionObject(thWorkspace, btBroadphaseProxy::SensorTrigger, btBroadphaseProxy::AllFilter & ~btBroadphaseProxy::SensorTrigger);
		
}

class Env {
	btDiscreteDynamicsWorld *m_dynamicsWorld;
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
	public:
	int env_id=1;
	vector<int> env_indexs = {1,2,3,4,5};
	vector<double> env_offset = {0,0,0};
	
	Cube* cube;
	Finger* ff;
	Finger* mf;
	Finger* rf;
	Thumb* th;
	Env(btDiscreteDynamicsWorld *m_dynamicsWorld,btAlignedObjectArray<btCollisionShape*> m_collisionShapes,int id,vector<double> offset);
	vector<int> get_indexes();
	void MultiplyVectorByScaler(std::vector<double> &v,int scaler);
	void IndexesForEnv(std::vector<int> &v);
};
Env::Env(btDiscreteDynamicsWorld *dynamicsWorld,btAlignedObjectArray<btCollisionShape*> collisionShapes,int id,vector<double> offset){
	//setting class members
    m_dynamicsWorld   = dynamicsWorld;
	m_collisionShapes = collisionShapes;
	env_id = id;
	//offsetting the env
	env_offset = offset;
	MultiplyVectorByScaler(env_offset,env_id-1);
	IndexesForEnv(env_indexs);
	// cout<<env_id<<endl;
	// cout<<"env_offset[0]"<<env_offset[0]<<endl;
	cube = new Cube  (m_dynamicsWorld,m_collisionShapes,env_offset);
	ff   = new Finger(m_dynamicsWorld,m_collisionShapes,env_offset);
	mf   = new Finger(m_dynamicsWorld,m_collisionShapes,env_offset);
	rf   = new Finger(m_dynamicsWorld,m_collisionShapes,env_offset);
	th   = new Thumb (m_dynamicsWorld,m_collisionShapes,env_offset);
}
void Env::MultiplyVectorByScaler(std::vector<double> &v,int scaler){
	for(int i=0;i<v.size();i++){
		v[i] *=scaler;
	}
}
void Env::IndexesForEnv(std::vector<int> &v){

	vector<int> new_index;
	for(int i=(5*(env_id-1))+1;i<=5*env_id;i++){
		new_index.push_back(i);
	}
	v=new_index;
}
vector<int> Env::get_indexes(){
	return env_indexs;
}
class Envs{
	std::vector<int> env_ids;
	
	public:
	vector<Env> envs;
	Envs(btDiscreteDynamicsWorld *dynamicsWorld,btAlignedObjectArray<btCollisionShape*> collisionShapes,int num_env,vector<double> offset);
	Env get_env(int env_id);
};
Envs::Envs(btDiscreteDynamicsWorld *dynamicsWorld,btAlignedObjectArray<btCollisionShape*> collisionShapes,int num_env,vector<double> offset){
	
	
	for(int i=1;i<=num_env;i++){
		Env env(dynamicsWorld,collisionShapes,i,offset);
		envs.push_back(env);
	} 
}
Env Envs::get_env(int env_id){
 return envs[env_id-1];
}

struct GhostExample : public CommonRigidBodyBase_G
{
	GhostExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase_G(helper)
	{
	}
	
	virtual ~GhostExample() {}
	Envs* envs;
	
	virtual void initPhysics();
	virtual std::vector<bool> detailed_checkCollision(int objectIndex);
	virtual bool checkCollision(int objectIndex); //for ghost object
	virtual void changePos(int index, float x, float y, float z, float r, float p, float yy);
	virtual btQuaternion getQuaternionFromEuler(float& yawZ, float& pitchY, float& rollX);
	virtual btVector3 getOrigin(int index);
	virtual btVector3 getRotation(int index);
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 1;
		float pitch = 0;
		float yaw = 270;
		float targetPos[3] = {-1.26814e-05, -0.300092, 0.765229};
		// View for adjusting workspace 
		// float yaw = 180;
		// float targetPos[3] = {-1.26814e-05, -0.300092, 2.2};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};


void GhostExample::initPhysics()
{
	m_guiHelper->setUpAxis(2);

	createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0,0,-10));

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	
	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);
	
	
	//*********************Making the ground and adding the ground*****************************
		btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));

		m_collisionShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, -50, 0));
		groundTransform.setOrigin(btVector3(0, 0, -50));
		
		btScalar mass_ground(0.);
		btVector3 localInertia_ground(0, 0, 0);
		
		
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass_ground, myMotionState, groundShape, localInertia_ground);
		btRigidBody* ground = new btRigidBody(rbInfo);

		//add the ground to the dynamics world
		m_dynamicsWorld->addCollisionObject(ground, btBroadphaseProxy::SensorTrigger, btBroadphaseProxy::AllFilter & ~btBroadphaseProxy::SensorTrigger);
		
	//*****************************Adding envs to the scence***********************************

		envs = new Envs(m_dynamicsWorld,m_collisionShapes,2,{2,0,0});

	
	std::cout << "Length of collision object array: " << m_dynamicsWorld->getCollisionObjectArray().size() << std::endl;
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

}

bool GhostExample::checkCollision(int objectIndex)
{	

	btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[objectIndex];
	btGhostObject* ghost = btGhostObject::upcast(obj);
	int numOverlappingObjects = ghost->getNumOverlappingObjects();
	// std::cout << "Collision object id: " << objectIndex << "; "
	// 		  << "Number of overlapping Objects: " << numOverlappingObjects << std::endl;
	
	// Check contact manifold
	if(numOverlappingObjects > 0) {
	 	// count the manifolds
		int numManifolds = m_dynamicsWorld->getDispatcher()->getNumManifolds();
		// std::cout << "No. of Manifolds = " << numManifolds << std::endl;
		for (int i = 0; i < numManifolds; ++i) {
			btPersistentManifold* contactManifold = m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
			const btCollisionObject* obA = static_cast<const btCollisionObject*>(contactManifold->getBody0());
        	const btCollisionObject* obB = static_cast<const btCollisionObject*>(contactManifold->getBody1());
			int indexA = obA->getWorldArrayIndex();
			int indexB = obB->getWorldArrayIndex();
			int numContacts = contactManifold->getNumContacts();
			// std::cout << " manifold " << i << " has " << numContacts << " contacts" << std::endl;
			if(numContacts > 0){
				if(indexA == objectIndex || indexB == objectIndex) { return true;}
			}
		}
    }
	
	return false;
}

void GhostExample::changePos(int index, float x, float y, float z, float r, float p, float yy)
{	vector<int> indexes;
	
	vector<double> offset = {0,0,0};
	vector<Env> env_stack = envs->envs;
	bool index_exists;
	for(int i =0;i<env_stack.size();i++){
		indexes = env_stack[i].env_indexs;
		std::vector<int>::iterator it = std::find(indexes.begin(), indexes.end(), index);
		
		if (it !=indexes.end()){
			index_exists = true;
		}
		else{
			index_exists = false;
		}
		if (index_exists){
			offset=env_stack[i].env_offset;
		}
	}
	m_dynamicsWorld->getCollisionObjectArray()[index]->getWorldTransform().setOrigin(btVector3(x+offset[0], y+offset[1], z+offset[2]));
	btQuaternion q = getQuaternionFromEuler(r, p, yy);
	m_dynamicsWorld->getCollisionObjectArray()[index]->getWorldTransform().setRotation(q);

	cout<<"changePos::index:: "<<index<<endl;
	cout<<"changePos::indexes:: "<<indexes[0]<<endl;
	cout<<"changePos::indexes:: "<<indexes[1]<<endl;
	cout<<"changePos::indexes:: "<<indexes[2]<<endl;
	cout<<"changePos::indexes:: "<<indexes[3]<<endl;
	cout<<"changePos::indexes:: "<<indexes[4]<<endl;
	cout<<"changePos::index_exists:: "<<index_exists<<endl;

}

std::vector<bool> GhostExample::detailed_checkCollision(int objectIndex){
	vector<bool> result = {false,false,false,false,false,false};
	return result;
}


btQuaternion GhostExample::getQuaternionFromEuler(float& rollX, float& pitchY, float& yawZ)
{
	btQuaternion q;
	q.setEulerZYX(yawZ, pitchY, rollX);	
	return q;
}

btVector3 GhostExample::getOrigin(int index)
{
	return m_dynamicsWorld->getCollisionObjectArray()[index]->getWorldTransform().getOrigin();
}

btVector3 GhostExample::getRotation(int index)
{
	btQuaternion quat = m_dynamicsWorld->getCollisionObjectArray()[index]->getWorldTransform().getRotation();
	btScalar roll, pitch, yaw;
	quat.getEulerZYX(yaw, pitch, roll);
	btVector3 rpy2 = btVector3(roll, pitch, yaw);
	return rpy2;
}

void GhostExample::renderScene()
{
	CommonRigidBodyBase_G::renderScene();
}

CommonExampleInterface_G* GhostExampleCreateFunc(CommonExampleOptions_G& options)
{
	return new GhostExample(options.m_guiHelper);
}

//B3_STANDALONE_EXAMPLE(BasicExampleCreateFunc)
