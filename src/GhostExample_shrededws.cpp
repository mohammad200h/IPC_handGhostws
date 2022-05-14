

// #include <Com/Com.h>
#include "GhostExample_shrededws.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btAlignedObjectArray.h"

#include "GhostAndRigidBodyBase_G.h"


#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include <iostream>
#include <unistd.h>

#include "../Importers/ImportSTLDemo/LoadMeshFromSTL.h"
#include "../Utils/b3ResourcePath.h"
#include "../Utils/b3BulletDefaultFileIO.h"
#include <vector>
#include <algorithm>



#include <unistd.h>
#include <stdio.h>
#include <limits.h>

#include <string> 
using namespace std;

//mamad note look at CommonRigidBodyBase_G 



btQuaternion getQuaternionFromEuler(float rollX, float pitchY, float yawZ)
{
	btQuaternion q;
	q.setEulerZYX(yawZ, pitchY, rollX);	
	return q;
}
vector<double> getEuler(btQuaternion q){
	
	btScalar yawZ;
	btScalar pitchY;
	btScalar rollX;

	q.getEulerZYX( yawZ,pitchY,rollX);

	// cout<<"yawZ:: "<<yawZ<<endl;
	// cout<<"pitchY:: "<<pitchY<<endl;
	// cout<<"rollX:: "<<rollX<<endl;

	return {rollX,pitchY,yawZ};
}
 
//*******************************************Cube class definations**********************************

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

btBoxShape* Cube::createBoxShape(const btVector3& halfExtents){
		btBoxShape* box = new btBoxShape(halfExtents);
		return box;
}

btRigidBody* Cube::createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, const btVector4& color){
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

//*******************************************Finger class definations**********************************
Finger::Finger(btDiscreteDynamicsWorld *dynamicsWorld,btAlignedObjectArray<btCollisionShape*> collisionShapes,vector<double> offset ){
	//setting class members
    m_dynamicsWorld   = dynamicsWorld;
	m_collisionShapes = collisionShapes;

	std::string path = "/home/mamad/hand_RL_ws/IPC_handGhostws";
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
//*******************************************Thumb class definations**********************************

Thumb::Thumb(btDiscreteDynamicsWorld *dynamicsWorld,btAlignedObjectArray<btCollisionShape*> collisionShapes,vector<double> offset ){
	//setting class members
    m_dynamicsWorld   = dynamicsWorld;
	m_collisionShapes = collisionShapes;

	std::string path = "/home/mamad/hand_RL_ws/IPC_handGhostws";
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

//*******************************************Env class definations**********************************
Env::Env(btDiscreteDynamicsWorld *dynamicsWorld,btAlignedObjectArray<btCollisionShape*> collisionShapes,int id,vector<double> offset){
	//setting class members
    m_dynamicsWorld   = dynamicsWorld;
	m_collisionShapes = collisionShapes;
	env_id = id;

	//offsetting the env
	env_offset = offset;
	MultiplyVectorByScaler(env_offset,env_id-1);
	calcualte_Index_for_env();
	// cout<<env_id<<endl;
	// cout<<"env_offset[0]"<<env_offset[0]<<endl;
	cube = new Cube  (m_dynamicsWorld,m_collisionShapes,env_offset);
	ff   = new Finger(m_dynamicsWorld,m_collisionShapes,env_offset);
	mf   = new Finger(m_dynamicsWorld,m_collisionShapes,env_offset);
	rf   = new Finger(m_dynamicsWorld,m_collisionShapes,env_offset);
	th   = new Thumb (m_dynamicsWorld,m_collisionShapes,env_offset);
}

void Env::updateEnvGymState(GymworldState gymState){

	int i=0;
	for(int index= this->env_indexs[0];index<=this->env_indexs[this->env_indexs.size()-1];index++){
		obj_enum key = obj_dic[i];
		objState obj_state = get_coordinate(key,gymState);
		m_dynamicsWorld->getCollisionObjectArray()[index]->getWorldTransform().setOrigin(btVector3(obj_state.pos.x+env_offset[0], obj_state.pos.y+env_offset[1], obj_state.pos.z+env_offset[2]));
		btQuaternion q = getQuaternionFromEuler(obj_state.orn.r, obj_state.orn.p, obj_state.orn.y);
		m_dynamicsWorld->getCollisionObjectArray()[index]->getWorldTransform().setRotation(q);
		i++;
	}
	updateInternalState( gymState);


}
void Env::updateInternalState(GymworldState gymState){
	//key
	this->state->gymState.key.id  = gymState.key.id;
	//ff
	this->state->gymState.ff.pos.x  = gymState.ff.pos.x;
	this->state->gymState.ff.pos.y  = gymState.ff.pos.y;
	this->state->gymState.ff.pos.z  = gymState.ff.pos.z;
	this->state->gymState.ff.orn.r  = gymState.ff.orn.r;
	this->state->gymState.ff.orn.p  = gymState.ff.orn.p;
	this->state->gymState.ff.orn.y  = gymState.ff.orn.y;
	//mf
	this->state->gymState.mf.pos.x  = gymState.mf.pos.x;
	this->state->gymState.mf.pos.y  = gymState.mf.pos.y;
	this->state->gymState.mf.pos.z  = gymState.mf.pos.z;
	this->state->gymState.mf.orn.r  = gymState.mf.orn.r;
	this->state->gymState.mf.orn.p  = gymState.mf.orn.p;
	this->state->gymState.mf.orn.y  = gymState.mf.orn.y;
	//rf
	this->state->gymState.rf.pos.x  = gymState.rf.pos.x;
	this->state->gymState.rf.pos.y  = gymState.rf.pos.y;
	this->state->gymState.rf.pos.z  = gymState.rf.pos.z;
	this->state->gymState.rf.orn.r  = gymState.rf.orn.r;
	this->state->gymState.rf.orn.p  = gymState.rf.orn.p;
	this->state->gymState.rf.orn.y  = gymState.rf.orn.y;
	//th
	this->state->gymState.th.pos.x  = gymState.th.pos.x;
	this->state->gymState.th.pos.y  = gymState.th.pos.y;
	this->state->gymState.th.pos.z  = gymState.th.pos.z;
	this->state->gymState.th.orn.r  = gymState.th.orn.r;
	this->state->gymState.th.orn.p  = gymState.th.orn.p;
	this->state->gymState.th.orn.y  = gymState.th.orn.y;
	//cube
	this->state->gymState.cube.pos.x  = gymState.cube.pos.x;
	this->state->gymState.cube.pos.y  = gymState.cube.pos.y;
	this->state->gymState.cube.pos.z  = gymState.cube.pos.z;
	this->state->gymState.cube.orn.r  = gymState.cube.orn.r;
	this->state->gymState.cube.orn.p  = gymState.cube.orn.p;
	this->state->gymState.cube.orn.y  = gymState.cube.orn.y;
}

void Env::MultiplyVectorByScaler(std::vector<double> &v,int scaler){
	for(int i=0;i<v.size();i++){
		v[i] *=scaler;
	}
}
void Env::calcualte_Index_for_env(){

	vector<int> new_index;
	for(int i=(5*(env_id-1))+1;i<=5*env_id;i++){
		new_index.push_back(i);
	}
	this->env_indexs = new_index;
}
vector<int> Env::get_indexes(){
	return env_indexs;
}

objState Env::get_coordinate(obj_enum key,GymworldState gymState){
	objState obj_state;
 switch(key){
	 case FF:
	 	obj_state.pos.x = gymState.ff.pos.x;
		obj_state.pos.y = gymState.ff.pos.y;
		obj_state.pos.z = gymState.ff.pos.z;
		obj_state.orn.r = gymState.ff.orn.r;
		obj_state.orn.p = gymState.ff.orn.p;
		obj_state.orn.y = gymState.ff.orn.y;
		break;
	case MF:
	 	obj_state.pos.x = gymState.mf.pos.x;
		obj_state.pos.y = gymState.mf.pos.y;
		obj_state.pos.z = gymState.mf.pos.z;
		obj_state.orn.r = gymState.mf.orn.r;
		obj_state.orn.p = gymState.mf.orn.p;
		obj_state.orn.y = gymState.mf.orn.y;
		break;
	case RF:
	 	obj_state.pos.x = gymState.rf.pos.x;
		obj_state.pos.y = gymState.rf.pos.y;
		obj_state.pos.z = gymState.rf.pos.z;
		obj_state.orn.r = gymState.rf.orn.r;
		obj_state.orn.p = gymState.rf.orn.p;
		obj_state.orn.y = gymState.rf.orn.y;
		break;
	case TH:
	 	obj_state.pos.x = gymState.th.pos.x;
		obj_state.pos.y = gymState.th.pos.y;
		obj_state.pos.z = gymState.th.pos.z;
		obj_state.orn.r = gymState.th.orn.r;
		obj_state.orn.p = gymState.th.orn.p;
		obj_state.orn.y = gymState.th.orn.y;
		break;
	case CUBE:
	 	obj_state.pos.x = gymState.cube.pos.x;
		obj_state.pos.y = gymState.cube.pos.y;
		obj_state.pos.z = gymState.cube.pos.z;
		obj_state.orn.r = gymState.cube.orn.r;
		obj_state.orn.p = gymState.cube.orn.p;
		obj_state.orn.y = gymState.cube.orn.y;
		break;
	
	
	 	
 }
 return obj_state;
}

GhostWorldState Env::getEnvGhostState(){
	// cout<<"Env::getEnvGhostState::called"<<endl;

	//Get collision for each ws in env
	int figner_key = 1;
	
	for(int index= this->env_indexs[1];index<=this->env_indexs[this->env_indexs.size()-1];index++){
		// cout<<"getEnvGhostState::first for::index:: "<<index<<endl;
		cout<<"index "<<index<<endl;
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[index];
		// cout<<"getEnvGhostState::first for::btCollisionObject* obj:: "<<endl;
		btGhostObject* ghost = btGhostObject::upcast(obj);
		int numOverlappingObjects = ghost->getNumOverlappingObjects();
		cout<<"getEnvGhostState::first for::numOverlappingObjects:: "<<numOverlappingObjects<<endl;
		if(numOverlappingObjects > 0) {
			int numManifolds = m_dynamicsWorld->getDispatcher()->getNumManifolds();
			
			for (int i = 0; i < numManifolds; ++i) {
				btPersistentManifold* contactManifold = m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
				const btCollisionObject* obA = static_cast<const btCollisionObject*>(contactManifold->getBody0());
    	    	const btCollisionObject* obB = static_cast<const btCollisionObject*>(contactManifold->getBody1());
				int indexA = obA->getWorldArrayIndex();
				int indexB = obB->getWorldArrayIndex();
				int numContacts = contactManifold->getNumContacts();
				
				if(numContacts > 0){
					if(indexA == index || indexB == index) { 
						//TODO::update ghost state for this index
						obj_enum key = obj_dic[figner_key];
						cout<<"index "<<index<<" indexA:: "<<indexA<<" indexB:: "<<indexB<<endl;
						setWSCollisitionFlagToTrue(key);
						
					}
				
				
				}


			}
		}
	figner_key ++;
	// cout<<"ff::flag"<<std::boolalpha <<this->state->ghostState.ff<<endl;
	}
	cout<<"Env::getEnvGhostState::call::ended"<<endl;

	return this->state->ghostState;

}
void Env::setWSCollisitionFlagToTrue(obj_enum key){
	switch(key){
		case FF:
			// cout<<"setWSCollisitionFlagToTrue::FF"<<endl;
			this->state->ghostState.ff=true;
			break;
		case MF:
			this->state->ghostState.mf=true;
			break;
		case RF:
			this->state->ghostState.rf=true;
			break;
		case TH:
			this->state->ghostState.th=true;
			break;
		case CUBE:

			break;



	}
}
void Env::restGhostState(){

	state->ghostState.ff=false;
	state->ghostState.mf=false;
	state->ghostState.rf=false;
	state->ghostState.th=false;
}
//*******************************************Envs class definations**********************************
Envs::Envs(btDiscreteDynamicsWorld *dynamicsWorld,btAlignedObjectArray<btCollisionShape*> collisionShapes,int num_env,vector<double> offset){
	// cout<<"Envs::constructor::num_env:: "<<num_env<<endl;
	
	for(int i=1;i<=num_env;i++){
		Env env(dynamicsWorld,collisionShapes,i,offset);
		this->envs.push_back(env);
		this->env_ids.push_back(i);
	} 
	// cout<<"envs::len:: "<<envs.size()<<endl;
	envs_initalized = true;
}
Env Envs::get_env(int env_id){
 return envs[env_id-1];
}
void Envs::updateEnvGymState(boost::uuids::uuid client_id,GymworldState gymState){
	//find env that is serving the client
	int env_id = get_env_id_binded_to_client_id(client_id);
	envs[env_id-1].updateEnvGymState(gymState);
}

GhostWorldState Envs::getEnvGhostState(boost::uuids::uuid client_id){
	int env_id = get_env_id_binded_to_client_id(client_id);
	cout<<"Envs::getEnvGhostState::called"<<endl;
	cout<<"Envs::getEnvGhostState::env_id:: "<<env_id<<endl;
	GhostWorldState gostState= envs[env_id-1].getEnvGhostState();
	cout<<"ff::flag::envs"<<std::boolalpha <<gostState.ff<<endl;
	return gostState;
}

int Envs::get_env_id_binded_to_client_id(boost::uuids::uuid client_id){
	
	for(int i=0;i<client_id_binding.size();i++){
		boost::uuids::uuid env_client_id = client_id_binding[i].second;
		if(env_client_id == client_id){
			return client_id_binding[i].first;

		}
	}
}

bool Envs::is_this_id_assinged_to_env(boost::uuids::uuid client_id){
	if (client_id_binding.size() !=0){
		for(int i=0;i<client_id_binding.size();i++){
			boost::uuids::uuid client_id_belonging_to_binded_env_id =client_id_binding[i].second;
			if (client_id == client_id_belonging_to_binded_env_id)
				return true;
		}
	}
	return false;

}

void Envs::assign_client_to_env(boost::uuids::uuid client_id){
	//get list of envs which are serving clients
	vector<int> env_id_of_env_with_binding;
	for(int i=0;i<client_id_binding.size();i++){
		env_id_of_env_with_binding.push_back(client_id_binding[i].first);
	}
	// cout<<"assign_client_to_env::after for loop"<<endl;
	// cout<<"assign_client_to_env::after for loop::env_id_of_env_with_binding::len:: "<<env_id_of_env_with_binding.size()<<endl;
	//get id of all available envs 
	/**
	We do this by finding the difference between two vecotes
	https://stackoverflow.com/questions/14175858/c-subtract-vectors
	Then we assing client id to first free env_id and append it to client_id_binding
	**/
	std::sort(env_id_of_env_with_binding.begin(), env_id_of_env_with_binding.end());
	std::sort(this->env_ids.begin(), this->env_ids.end());

	std::vector<int> difference;
	std::set_difference(
    env_id_of_env_with_binding.begin(), env_id_of_env_with_binding.end(),
    this->env_ids.begin(), this->env_ids.end(),
    std::back_inserter( difference )
	);
	// cout<<"assign_client_to_env::after sorting"<<endl;
	// cout<<"assign_client_to_env::after sorting::difference::len "<<difference.size()<<endl;
	bool there_exist_a_previous_bindings = difference.size() !=0;
	if (there_exist_a_previous_bindings){
		int candidate_env_id = difference[0];
		//binding the env to client 
		this->client_id_binding.push_back(make_pair(candidate_env_id,client_id));
	}else{
		int candidate_env_id = this->env_ids[0];
		this->client_id_binding.push_back(make_pair(candidate_env_id,client_id));
	}
 
}
void Envs::restEnvGhostState(boost::uuids::uuid client_id){
	envs[get_env_id_binded_to_client_id(client_id)-1].restGhostState();
}

//*******************************************Main part of the code**********************************

struct GhostExample : public GhostAndRigidBodyBase_G
{
	GhostExample(struct GUIHelperInterface* helper,int num_envs)
		: GhostAndRigidBodyBase_G(helper)
	{
		cout<<"GhostExample::constructor::num_envs"<<num_envs<<endl;
		this->num_envs = num_envs;
	}
	
	virtual ~GhostExample() {}
	Envs* envs;
	int num_envs;
	Envs* getEnvs();
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
	cout<<"initPhysics"<<endl;
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
		// cout<<"GhostExample::initPhysics::num_envs:: "<<this->num_envs<<endl;
		this->envs = new Envs(m_dynamicsWorld,m_collisionShapes,this->num_envs,{2,0,0});
        // cout<<"GhostExample::initPhysics::this->envs::len:: "<<this->envs->envs.size()<<endl;
	
	// std::cout << "Length of collision object array: " << m_dynamicsWorld->getCollisionObjectArray().size() << std::endl;
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

}

bool GhostExample::checkCollision(int objectIndex)
{	
	cout<<"inside::checkCollision"<<endl;

	bool ws_flag = false;
	vector<bool> result = {false,false,false,false,false,false};

	std::vector<std::vector<int>> hand ={{2,3,4,5},{7,8,9,10}};
	
	
	int adjusted_objectIndex;
	switch (objectIndex)
	{
		case 2: 
		adjusted_objectIndex = 2;
		break;
		case 3: 
		adjusted_objectIndex = 2+6;
		break;
		case 4: 
		adjusted_objectIndex = 2+6*2;
		break;
		case 5: 
		adjusted_objectIndex =20;
		break;
		case 7: 
		adjusted_objectIndex = 27;
		break;
		case 8: 
		adjusted_objectIndex = 27+6;
		break;
		case 9: 
		adjusted_objectIndex = 27+6*2;
		break;
		case 10: 
		adjusted_objectIndex = 45;
		break;	
	}
	// cout<<"objectIndex::"<<objectIndex<<endl;
	int counter = 0;
	
	for(int j=adjusted_objectIndex;j<adjusted_objectIndex+6;j++){
		// cout<<"adjusted_objectIndex::"<<j<<endl;
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[j];
		// cout<<"I am here::1"<<endl;
		btGhostObject* ghost = btGhostObject::upcast(obj);
		int numOverlappingObjects = ghost->getNumOverlappingObjects();
		// cout<<"numOverlappingObjects::"<< numOverlappingObjects<<endl;
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
					// cout<<"if::checkCollision"<<endl;
					if(indexA == j || indexB == j) { 
						result[counter]=true;
						// cout<<"indexA == j || indexB == j:: "<<to_string(indexA == j || indexB == j)<<" j:: "<<j<<endl;
						// cout<<"result[counter]:: "<<to_string(result[counter])<<endl;
						ws_flag= true;
					}
				}
			}
    	}
		
		counter +=1;
	}
	// cout<<"end::checkCollision"<<endl;
	
	// cout<<"return";print_v(result);
	switch (objectIndex)
	{
		case 2: 
		envs->envs[0].ff->collision_flag = result;
		break;
		case 3: 
		envs->envs[0].mf->collision_flag = result;
		break;
		case 4: 
		envs->envs[0].rf->collision_flag = result;
		break;
		case 5: 
		envs->envs[0].th->collision_flag = result;
		break;
		case 7: 
		envs->envs[1].ff->collision_flag = result;
		break;
		case 8: 
		envs->envs[1].mf->collision_flag = result;
		break;
		case 9: 
		envs->envs[1].rf->collision_flag = result;
		break;
		case 10: 
		envs->envs[1].th->collision_flag = result;
		break;	
	}
	
	return ws_flag;
}

void GhostExample::changePos(int index, float x, float y, float z, float r, float p, float yy)
{	
	// cout<<"I am here::changePos"<<endl;
	vector<int> indexes;
	
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
	//changing the pos
	//index 1,6 are for for cube in env1 and env2
	//index 2,3,4 and 7,8,9 are ff,mf,rf for env1 and env2
	// finger_ws has 6 piece therefore 6 index which should be in same pos
	std::vector<std::vector<int>> fingers ={{2,3,4},{7,8,9}};
	std::vector<std::vector<int>> cube ={{1},{6}};
	std::vector<std::vector<int>> th ={{5},{10}};

	//#################Todo######################
	bool finger_in_env_one = std::find(std::begin(fingers[0]), std::end(fingers[0]), index) != std::end(fingers[0]);
	bool finger_in_env_two = std::find(std::begin(fingers[1]), std::end(fingers[1]), index) != std::end(fingers[1]);
	// cout<<"tested::finger_in_env_one"<<std::to_string(finger_in_env_one || finger_in_env_two)<<endl;
	bool cube_in_env_one = std::find(std::begin(cube[0]), std::end(cube[0]), index) != std::end(cube[0]);
	bool cube_in_env_two = std::find(std::begin(cube[1]), std::end(cube[1]), index) != std::end(cube[1]);
	// cout<<"tested::cube_in_env_one"<<std::to_string(cube_in_env_one || cube_in_env_two)<<endl;
	bool th_in_env_one = std::find(std::begin(th[0]), std::end(th[0]), index) != std::end(th[0]);
	bool th_in_env_two = std::find(std::begin(th[1]), std::end(th[1]), index) != std::end(th[1]);
	// cout<<"tested::th_in_env_one"<<std::to_string(th_in_env_one || th_in_env_two)<<endl;

	int adjusted_index;
	//ff,mf,rf
	if(finger_in_env_one || finger_in_env_two ){
		// cout<<"I am inside::finger_in_env_one || finger_in_env_two"<<"index::"<<index<<endl;
		switch (index){
			case 2: 
			adjusted_index = 2;
			break;
			case 3: 
			adjusted_index = 2+6;
			break;
			case 4: 
			adjusted_index = 2+6*2;
			break;
			case 7: 
			adjusted_index = 27;
			break;
			case 8: 
			adjusted_index = 27+6;
			break;
			case 9: 
			adjusted_index = 27+6*2;
			break;
			
		}
			
		for (int i=adjusted_index;i<adjusted_index+6;i++){
			//getCollisionObjectArray this is in btCollisionWorld.h
				// cout<<"I am inside::finger_in_env_one || finger_in_env_two"<<"adjusted_index::"<<i<<endl;
				
				m_dynamicsWorld->getCollisionObjectArray()[i]->getWorldTransform().setOrigin(btVector3(x+offset[0], y+offset[1], z+offset[2]));
				btQuaternion q = getQuaternionFromEuler(r, p, yy);
				m_dynamicsWorld->getCollisionObjectArray()[i]->getWorldTransform().setRotation(q);
		}
	}

	if(cube_in_env_one || cube_in_env_two ){
		switch (index){
			case 1: 
			adjusted_index = 1;
			break;
			case 6: 
			adjusted_index = 26;
			break;	
		}
			
		
		m_dynamicsWorld->getCollisionObjectArray()[adjusted_index]->getWorldTransform().setOrigin(btVector3(x+offset[0], y+offset[1], z+offset[2]));
		btQuaternion q = getQuaternionFromEuler(r, p, yy);
		m_dynamicsWorld->getCollisionObjectArray()[adjusted_index]->getWorldTransform().setRotation(q);
		
	}

	if(th_in_env_one || th_in_env_two ){
		// cout<<"I am inside::th_in_env_one || th_in_env_two"<<"index::"<<index<<endl;
		switch (index){
			case 5: 
			adjusted_index =20;
			break;
			case 10: 
			adjusted_index = 45;
			break;	
		}
			
		for (int i=adjusted_index;i<adjusted_index+6;i++){
				// cout<<"I am inside::th_in_env_one || th_in_env_two"<<"adjusted_index::"<<i<<endl;
				m_dynamicsWorld->getCollisionObjectArray()[i]->getWorldTransform().setOrigin(btVector3(x+offset[0], y+offset[1], z+offset[2]));
				btQuaternion q = getQuaternionFromEuler(r, p, yy);
				m_dynamicsWorld->getCollisionObjectArray()[i]->getWorldTransform().setRotation(q);
		}
	}

	//##############End Todo###################


}

std::vector<bool> GhostExample::detailed_checkCollision(int objectIndex){
	
	GhostExample::checkCollision(objectIndex);
	vector<bool> result;
		switch (objectIndex)
	{
		case 2: 
		result=envs->envs[0].ff->collision_flag;
		break;
		case 3: 
		result=envs->envs[0].mf->collision_flag;
		break;
		case 4: 
		result=envs->envs[0].rf->collision_flag;
		break;
		case 5: 
		result=envs->envs[0].th->collision_flag;
		break;
		case 7: 
		result=envs->envs[1].ff->collision_flag;
		break;
		case 8: 
		result=envs->envs[1].mf->collision_flag;
		break;
		case 9: 
		result=envs->envs[1].rf->collision_flag;
		break;
		case 10: 
		result=envs->envs[1].th->collision_flag;
		break;	
	}
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
	GhostAndRigidBodyBase_G::renderScene();
}

Envs* GhostExample::getEnvs(){
	return this->envs;
}

GhostInterface_G* GhostExampleCreateFunc(CommonExampleOptions_G& options,int num_envs)
{	
	// cout<<"GhostExampleCreateFunc::num_envs::"<<num_envs<<endl;
	GhostExample* GE = new GhostExample(options.m_guiHelper,num_envs);
	return GE;
}


