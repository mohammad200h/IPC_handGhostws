

#ifndef COMMON_EXAMPLE_INTERFACE_G_H
#define COMMON_EXAMPLE_INTERFACE_G_H

#include "btBulletDynamicsCommon.h"

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

class CommonExampleInterface_G
{
public:
	typedef class CommonExampleInterface_G*(CreateFunc)(CommonExampleOptions_G& options);

	virtual ~CommonExampleInterface_G()
	{
	}

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

	virtual CommonExampleInterface_G::CreateFunc* getExampleCreateFunc(int index) = 0;

	virtual const char* getExampleName(int index) = 0;

	virtual const char* getExampleDescription(int index) = 0;

	virtual int getExampleOption(int index) = 0;
};

CommonExampleInterface_G* GhostExampleCreateFunc(CommonExampleOptions_G& options);

// #ifdef B3_USE_STANDALONE_EXAMPLE
// #define B3_STANDALONE_EXAMPLE(ExampleFunc)                                             \
// 	CommonExampleInterface* StandaloneExampleCreateFunc(CommonExampleOptions& options) \
// 	{                                                                                  \
// 		return ExampleFunc(options);                                                   \
// 	}
// #else  //B3_USE_STANDALONE_EXAMPLE
// #define B3_STANDALONE_EXAMPLE(ExampleFunc)
// #endif  //B3_USE_STANDALONE_EXAMPLE

#endif  //COMMON_EXAMPLE_INTERFACE_H
