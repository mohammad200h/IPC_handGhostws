

#ifndef COMMON_EXAMPLE_INTERFACE_G_H
#define COMMON_EXAMPLE_INTERFACE_G_H

#include "btBulletDynamicsCommon.h"
#include <vector>
#include "GhostExample_shrededws.h"

#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include <Com/Com.h>


class State{
	public:
		GymworldState    gymState;
		GhostWorldState  ghostState;

};

class Cube{
	btDiscreteDynamicsWorld *m_dynamicsWorld;
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
	public:
	Cube(btDiscreteDynamicsWorld *m_dynamicsWorld,btAlignedObjectArray<btCollisionShape*> m_collisionShapes,vector<double> offset);
	
	btBoxShape* createBoxShape(const btVector3& halfExtents);
	btRigidBody* createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, const btVector4& color = btVector4(1, 0, 0, 1));
};

class Finger{
	
	btDiscreteDynamicsWorld *m_dynamicsWorld;
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
	public:
	btGhostObject* finger_ws ;
	btCollisionShape* meshShape;
	Finger(btDiscreteDynamicsWorld *m_dynamicsWorld,btAlignedObjectArray<btCollisionShape*> m_collisionShapes,vector<double> offset );
	vector<bool> collision_flag;
	
};

class Thumb{
	
	btDiscreteDynamicsWorld *m_dynamicsWorld;
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
	public:
	btGhostObject* finger_ws ;
	btCollisionShape* meshShape;
	Thumb(btDiscreteDynamicsWorld *m_dynamicsWorld,btAlignedObjectArray<btCollisionShape*> m_collisionShapes,vector<double> offset);
	vector<bool> collision_flag;
	
};

class Env {
	private:
		
		btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
		
		

	public:
		enum obj_enum{CUBE,FF,MF,RF,TH};
		map<int,obj_enum> obj_dic = {
			{0,CUBE},
			{1,FF},
			{2,MF},
			{3,RF},
			{4,TH}

		}; 
		btDiscreteDynamicsWorld *m_dynamicsWorld;
		int env_id=1;
		vector<int> env_indexs = {1,2,3,4,5};//fingers and cube
		vector<double> env_offset = {0,0,0};
		State* state = new State();

		Cube* cube;
		Finger* ff;
		Finger* mf;
		Finger* rf;
		Thumb* th;

		Env(btDiscreteDynamicsWorld *m_dynamicsWorld,btAlignedObjectArray<btCollisionShape*> m_collisionShapes,int id,vector<double> offset);
		vector<int> get_indexes();
		void MultiplyVectorByScaler(std::vector<double> &v,int scaler);
		void calcualte_Index_for_env();
		void updateEnvGymState(GymworldState gymState);
		void updateInternalState(GymworldState gymState);
		GhostWorldState getEnvGhostState();
		objState get_coordinate(obj_enum key,GymworldState gymState);
		void setWSCollisitionFlagToTrue(obj_enum key);
		void restGhostState();
};

class Envs{
	vector<int> env_ids;
	//the first element of pair in envid and the second is client id
	
	public:
	vector <pair<int,boost::uuids::uuid>> client_id_binding;
	vector<Env> envs;
	Envs(btDiscreteDynamicsWorld *dynamicsWorld,btAlignedObjectArray<btCollisionShape*> collisionShapes,int num_env,vector<double> offset);
	Env get_env(int env_id);
	bool envs_initalized = false;
	void assign_client_to_env(boost::uuids::uuid client_id);
	bool is_this_id_assinged_to_env(boost::uuids::uuid client_id);
	void updateEnvGymState(boost::uuids::uuid client_id,GymworldState gymState);
	int get_env_id_binded_to_client_id(boost::uuids::uuid client_id);
	GhostWorldState getEnvGhostState(boost::uuids::uuid client_id);
	void restEnvGhostState(boost::uuids::uuid client_id);
};


struct CommandProcessorCreationInterface_G
{
	virtual ~CommandProcessorCreationInterface_G() {}
	virtual class CommandProcessorInterface* createCommandProcessor() = 0;
	virtual void deleteCommandProcessor(CommandProcessorInterface*) = 0;
};

struct CommonExampleOptions_G
{
	struct GUIHelperInterface* m_guiHelper;

	//Those are optional, some examples will use them others don't. Each example should work with them being 0.
	int m_option;
	const char* m_fileName;
	class SharedMemoryInterface* m_sharedMem;
	CommandProcessorCreationInterface_G* m_commandProcessorCreation;
	bool m_skipGraphicsUpdate;

	CommonExampleOptions_G(struct GUIHelperInterface* helper, int option = 0)
		: m_guiHelper(helper),
		  m_option(option),
		  m_fileName(0),
		  m_sharedMem(0),
		  m_commandProcessorCreation(0),
		  m_skipGraphicsUpdate(false)
	{
	}
};



class GhostInterface_G
{



	public:

	// Envs* envs;


	typedef class GhostInterface_G*(CreateFunc)(CommonExampleOptions_G& options,int num_env);

	virtual ~GhostInterface_G()
	{
	}
	
	virtual Envs* getEnvs()=0;
	virtual void initPhysics() = 0;
	virtual void exitPhysics() = 0;
	virtual std::vector<bool> detailed_checkCollision(int index){};
	virtual bool checkCollision(int objectIndex) = 0;
	virtual void changePos(int index, float x, float y, float z, float r, float p, float yy) = 0;
	virtual btVector3 getOrigin(int index) = 0;
	virtual btVector3 getRotation(int index) = 0;
	virtual void updateGraphics() {}
	virtual void stepSimulation(float deltaTime) = 0;
	virtual void renderScene() = 0;
	virtual void physicsDebugDraw(int debugFlags) = 0;  //for now we reuse the flags in Bullet/src/LinearMath/btIDebugDraw.h
	//reset camera is only called when switching demo. this way you can restart (initPhysics) and watch in a specific location easier
	virtual void resetCamera(){};
	virtual bool mouseMoveCallback(float x, float y) = 0;
	virtual bool mouseButtonCallback(int button, int state, float x, float y) = 0;
	virtual bool keyboardCallback(int key, int state) = 0;

	virtual void vrControllerMoveCallback(int controllerId, float pos[4], float orientation[4], float analogAxis, float auxAnalogAxes[10]) {}
	virtual void vrControllerButtonCallback(int controllerId, int button, int state, float pos[4], float orientation[4]) {}
	virtual void vrHMDMoveCallback(int controllerId, float pos[4], float orientation[4]) {}
	virtual void vrGenericTrackerMoveCallback(int controllerId, float pos[4], float orientation[4]) {}

	virtual void processCommandLineArgs(int argc, char* argv[]){};
};

class ExampleEntries_G
{
public:
	virtual ~ExampleEntries_G() {}

	virtual void initExampleEntries() = 0;

	virtual void initOpenCLExampleEntries() = 0;

	virtual int getNumRegisteredExamples() = 0;

	virtual GhostInterface_G::CreateFunc* getExampleCreateFunc(int index) = 0;

	virtual const char* getExampleName(int index) = 0;

	virtual const char* getExampleDescription(int index) = 0;

	virtual int getExampleOption(int index) = 0;
};



#endif  //COMMON_EXAMPLE_INTERFACE_H
