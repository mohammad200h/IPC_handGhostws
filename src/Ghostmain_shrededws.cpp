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


#include "std_msgs/String.h"
#include "handGhostws/objState.h"
#include "handGhostws/ghostws_shreded.h"
#include "handGhostws/ghostwses_shreded.h"

#include "handGhostws/worldState.h"
#include <sstream>

#include "GhostExample_shrededws.h"

#include "CommonExampleInterface_G.h"
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




void MultiplyVectorByScaler(std::vector<int> &v,int scaler){
	for(int i=0;i<v.size();i++){
		v[i] *=scaler;
	}
}
CommonExampleInterface_G* example;
int gSharedMemoryKey = -1;

b3MouseMoveCallback prevMouseMoveCallback = 0;
static void OnMouseMove(float x, float y)
{
	bool handled = false;
	handled = example->mouseMoveCallback(x, y);
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

	handled = example->mouseButtonCallback(button, state, x, y);
	if (!handled)
	{
		if (prevMouseButtonCallback)
			prevMouseButtonCallback(button, state, x, y);
	}
}


void print_v(std::vector<auto> const &input)
{
    for (int i = 0; i < input.size(); i++) {
        std::cout << input.at(i) << ' ';
    }
	std::cout<<endl;
}

vector<uint8_t> convert_bool_to_uint8_t_vector(vector<bool> x){
	
	vector<uint8_t> result;

	for(int i=0;i<x.size();i++){
		result.push_back(uint8_t(x[i]));
	}
	return result;

}

int main(int argc, char* argv[])
{
	std::cout << "hello"<< std::endl;


	SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Ghost Example", 1024, 768, true);

	prevMouseButtonCallback = app->m_window->getMouseButtonCallback();
	prevMouseMoveCallback = app->m_window->getMouseMoveCallback();

	app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
	app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);

	OpenGLGuiHelper gui(app, false);


	CommonExampleOptions_G options(&gui);

	//example = StandaloneExampleCreateFunc(options);
	sim = GhostExampleCreateFunc(options,2);
	sim->processCommandLineArgs(argc, argv);

	sim->initPhysics();
	sim->resetCamera();

	b3Clock clock;



	
	// move the ghost object
	while (!app->m_window->requestedExit() && ros::ok())
	{
		if(hasRender){
			app->m_instancingRenderer->init();
			app->m_instancingRenderer->updateCamera(app->getUpAxis());
		}


		btScalar dtSec = 1./500.;
		sim->stepSimulation(dtSec);

	

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
