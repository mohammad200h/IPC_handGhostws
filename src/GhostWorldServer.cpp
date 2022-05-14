/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include <sstream>

#include "GhostExample_shrededws.h"


#include "GhostInterface.h"
#include "CommonGUIHelperInterface.h"
#include "b3Clock.h"

#include "SimpleOpenGL3App.h"
#include <stdio.h>
#include "OpenGLGuiHelper.h"


#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

#include "LinearMath/btTransform.h"
#include "LinearMath/btHashMap.h"

#include <iostream>
#include <vector>
using namespace std;


GhostInterface_G* sim;

int gSharedMemoryKey = -1;

b3MouseMoveCallback prevMouseMoveCallback = 0;
static void OnMouseMove(float x, float y)
{
	bool handled = false;
	handled = sim->mouseMoveCallback(x, y);
	if (!handled)
	{
		if (prevMouseMoveCallback)
			prevMouseMoveCallback(x, y);
	}
}

b3MouseButtonCallback prevMouseButtonCallback = 0;
static void OnMouseDown(int button, int state, float x, float y)
{
	bool handled = false;

	handled = sim->mouseButtonCallback(button, state, x, y);
	if (!handled)
	{
		if (prevMouseButtonCallback)
			prevMouseButtonCallback(button, state, x, y);
	}
}

int main(int argc, char* argv[])
{

	SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Ghost Example", 1024, 768, true);

	prevMouseButtonCallback = app->m_window->getMouseButtonCallback();
	prevMouseMoveCallback = app->m_window->getMouseMoveCallback();

	app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
	app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);

	OpenGLGuiHelper gui(app, false);

	CommonExampleOptions_G options(&gui);

	sim = GhostExampleCreateFunc(options,2);
	sim->processCommandLineArgs(argc, argv);

	sim->initPhysics();
	sim->resetCamera();

	b3Clock clock;

	//Server
	Server_MQ* server = new Server_MQ();
	//ENVs
	Envs* envs=sim->getEnvs();

	bool hasRender = true;

	int counter = 0;

	server->init();
	cout<<"I am before the server run"<<endl;
	server->create_run_thread();
	cout<<"I am after the server run"<<endl;

	// move the ghost object
	while (!app->m_window->requestedExit())
	{
		if(hasRender){
			app->m_instancingRenderer->init();
			app->m_instancingRenderer->updateCamera(app->getUpAxis());
		}

		btScalar dtSec = 1./500.;

		//*******************update gym state for all the envs********************
		/****
		Server serves one client at a time
		we get list of clients that should be served by calling getClientsInQueue
		****/
		vector <boost::uuids::uuid> clients_id = server->getClientsInQueue();
		/*****
		Assign Client to env if client id is not assotiated with any env
		******/
		for(auto & id: clients_id){
			if (!envs->is_this_id_assinged_to_env(id))
				envs->assign_client_to_env(id);
			/**
			Pass new state to env so change in Gym state will be 
			reflected in next simulation step
			**/
			GymworldState gymState = server->getGymStateforId(id);
			envs->updateEnvGymState(id,gymState);
			server->removeStateFromBuffer(id);
			
		}
		//*******************Step simulation**********************


		sim->stepSimulation(dtSec);

		//****************Get new Ghost State for envs************************
		cout<<"sim->stepSimulation(dtSec);"<<endl;
		vector <pair<boost::uuids::uuid,GhostWorldState>> ghost_vec;
		for(int i;i<clients_id.size();i++){
			ghost_vec.push_back(make_pair(clients_id[i],envs->getEnvGhostState(clients_id[i])));
		}
		//*****************Send Ghost stat to clients*****************************
		//Client that sent new Gym state will be waiting for Ghost state to be send to them
		//After Ghost state is sent to client. it's socket will be removed until ask for new connection from server
		for(auto it = ghost_vec.begin(); it != ghost_vec.end(); ++it){
			auto v_temp = *it;
			boost::uuids::uuid client_id = v_temp.first;
			GhostWorldState ghostState = v_temp.second;
			
			server->send( client_id, ghostState);
			//reset ghost state for env
			envs->restEnvGhostState(client_id);
		}
		//************************************************************************

		if(hasRender){
			sim->renderScene();
		}

		DrawGridData dg;
		dg.upAxis = app->getUpAxis();
		app->drawGrid(dg);

		app->swapBuffer();
	

	

	}

	sim->exitPhysics();
	delete sim;
	delete app;
	return 0;
}
