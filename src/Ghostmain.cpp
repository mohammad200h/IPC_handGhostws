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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "handGhostws/objState.h"
#include "handGhostws/ghostws.h"
#include "handGhostws/ghostwses.h"

#include "handGhostws/worldState.h"
#include <sstream>

#include "GhostExample.h"

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
using namespace std;
class WorldState{

	ros::ServiceServer GetghostCollision;   
	ros::Subscriber sub; 
	public:
	int env_id;
	ghostwses ghWSes;
	worldState WState;
	WorldState(int env_id,ros::NodeHandle n);
	bool get_ws_update( handGhostws::ghostwses::Request &req,handGhostws::ghostwses::Response &res );
	void set_pos_update(const handGhostws::worldState msg);
	void set_shared_values_from_Ghost_wrold(ros::Time rostime,
										btVector3& ffOrigin, btVector3& ffRotation, bool& ffCollision,
										btVector3& mfOrigin, btVector3& mfRotation, bool& mfCollision,
										btVector3& rfOrigin, btVector3& rfRotation, bool& rfCollision,
										btVector3& thOrigin, btVector3& thRotation, bool& thCollision );

};
WorldState::WorldState(int env_id,ros::NodeHandle n){
	GetghostCollision   = n.advertiseService("GetghostCollision_"+to_string(env_id),&WorldState::get_ws_update,this);
	sub 				= n.subscribe("WorldStateModularRLGYM_"+to_string(env_id), 1000, &WorldState::set_pos_update,this);
}
bool WorldState::get_ws_update( handGhostws::ghostwses::Request &req,handGhostws::ghostwses::Response &res )
{
	handGhostws::ghostws sub_msg;
	handGhostws::objState objState;

	sub_msg.pos_x = 1;
	sub_msg.pos_y = 1;
	sub_msg.pos_z = 1;
	sub_msg.r = 1;
	sub_msg.p = 1;
	sub_msg.y = 1;
	sub_msg.collision_flag = 0;

  	objState.pos_x = 1;
  	objState.pos_y = 1;
  	objState.pos_z = 1;
  	objState.r = 1;
  	objState.p = 1;
  	objState.y = 1;

  	res.header.stamp = ghWSes.time;
	
  	res.ff.pos_x               =ghWSes.ff.pos_x;
  	res.ff.pos_y               =ghWSes.ff.pos_y;
  	res.ff.pos_z               =ghWSes.ff.pos_z;
  	res.ff.r               	 =ghWSes.ff.r;
  	res.ff.p               	 =ghWSes.ff.p;
  	res.ff.y               	 =ghWSes.ff.y;
  	res.ff.collision_flag      =ghWSes.ff.collision_flag;

  	res.mf.pos_x               =ghWSes.mf.pos_x;
  	res.mf.pos_y               =ghWSes.mf.pos_y;
  	res.mf.pos_z               =ghWSes.mf.pos_z;
  	res.mf.r               	 =ghWSes.mf.r;
  	res.mf.p               	 =ghWSes.mf.p;
  	res.mf.y               	 =ghWSes.mf.y;
  	res.mf.collision_flag      =ghWSes.mf.collision_flag;

  	res.rf.pos_x               =ghWSes.rf.pos_x;
  	res.rf.pos_y               =ghWSes.rf.pos_y;
  	res.rf.pos_z               =ghWSes.rf.pos_z;
  	res.rf.r               	 =ghWSes.rf.r;
  	res.rf.p               	 =ghWSes.rf.p;
  	res.rf.y               	 =ghWSes.rf.y;
  	res.rf.collision_flag      =ghWSes.rf.collision_flag;

  	res.th.pos_x               =ghWSes.th.pos_x;
  	res.th.pos_y               =ghWSes.th.pos_y;
  	res.th.pos_z               =ghWSes.th.pos_z;
  	res.th.r               	 =ghWSes.th.r;
  	res.th.p               	 =ghWSes.th.p;
  	res.th.y               	 =ghWSes.th.y;
  	res.th.collision_flag      =ghWSes.th.collision_flag;
  
 	return true;
}

void WorldState::set_pos_update(const handGhostws::worldState msg)
{	//shared values
	//set_pos for world
	//cube
	//workspaces
	WState.ff.pos_x = msg.ff.pos_x;
	WState.ff.pos_y = msg.ff.pos_y;
	WState.ff.pos_z = msg.ff.pos_z;
	WState.ff.r = msg.ff.r;
	WState.ff.p = msg.ff.p;
	WState.ff.y = msg.ff.y;

	//mf
	WState.mf.pos_x = msg.mf.pos_x;
	WState.mf.pos_y = msg.mf.pos_y;
	WState.mf.pos_z = msg.mf.pos_z;
	WState.mf.r = msg.mf.r;
	WState.mf.p = msg.mf.p;
	WState.mf.y = msg.mf.y;

	//rf
	WState.rf.pos_x = msg.rf.pos_x;
	WState.rf.pos_y = msg.rf.pos_y;
	WState.rf.pos_z = msg.rf.pos_z;
	WState.rf.r = msg.rf.r;
	WState.rf.p = msg.rf.p;
	WState.rf.y = msg.rf.y;

	//th
	WState.th.pos_x = msg.th.pos_x;
	WState.th.pos_y = msg.th.pos_y;
	WState.th.pos_z = msg.th.pos_z;
	WState.th.r = msg.th.r;
	WState.th.p = msg.th.p;
	WState.th.y = msg.th.y;

	//cube
	WState.cube.pos_x = msg.cube.pos_x;
	WState.cube.pos_y = msg.cube.pos_y;
	WState.cube.pos_z = msg.cube.pos_z;
	WState.cube.r = msg.cube.r;
	WState.cube.p = msg.cube.p;
	WState.cube.y = msg.cube.y;
}

void WorldState::set_shared_values_from_Ghost_wrold(ros::Time rostime,
		btVector3& ffOrigin, btVector3& ffRotation, bool& ffCollision,
		btVector3& mfOrigin, btVector3& mfRotation, bool& mfCollision,
		btVector3& rfOrigin, btVector3& rfRotation, bool& rfCollision,
		btVector3& thOrigin, btVector3& thRotation, bool& thCollision ){

	//calls getPos() to get current state of simulation
	//assing thos values to shared values
	ghWSes.ff.pos_x = ffOrigin[0];
	ghWSes.ff.pos_y = ffOrigin[1];
	ghWSes.ff.pos_z = ffOrigin[2];
	ghWSes.ff.r = ffRotation[0];
	ghWSes.ff.p = ffRotation[1];
	ghWSes.ff.y = ffRotation[2];
	ghWSes.ff.collision_flag = ffCollision;

	ghWSes.mf.pos_x = mfOrigin[0];
	ghWSes.mf.pos_y = mfOrigin[1];
	ghWSes.mf.pos_z = mfOrigin[2];
	ghWSes.mf.r = mfRotation[0];
	ghWSes.mf.p = mfRotation[1];
	ghWSes.mf.y = mfRotation[2];
	ghWSes.mf.collision_flag = mfCollision;

	ghWSes.rf.pos_x = rfOrigin[0];
	ghWSes.rf.pos_y = rfOrigin[1];
	ghWSes.rf.pos_z = rfOrigin[2];
	ghWSes.rf.r = rfRotation[0];
	ghWSes.rf.p = rfRotation[1];
	ghWSes.rf.y = rfRotation[2];
	ghWSes.rf.collision_flag = rfCollision;

	ghWSes.th.pos_x = thOrigin[0];
	ghWSes.th.pos_y = thOrigin[1];
	ghWSes.th.pos_z = thOrigin[2];
	ghWSes.th.r = thRotation[0];
	ghWSes.th.p = thRotation[1];
	ghWSes.th.y = thRotation[2];
	ghWSes.th.collision_flag = thCollision;

	ghWSes.time = rostime;

}




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


int main(int argc, char* argv[])
{
	std::cout << "hello"<< std::endl;


	SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Ghost Example", 1024, 768, true);

	prevMouseButtonCallback = app->m_window->getMouseButtonCallback();
	prevMouseMoveCallback = app->m_window->getMouseMoveCallback();

	app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
	app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);

	OpenGLGuiHelper gui(app, false);
	//LessDummyGuiHelper gui(app);
	//DummyGUIHelper gui;

	CommonExampleOptions_G options(&gui);

	//example = StandaloneExampleCreateFunc(options);
	example = GhostExampleCreateFunc(options);
	example->processCommandLineArgs(argc, argv);

	example->initPhysics();
	example->resetCamera();

	b3Clock clock;

	//ROS
	ros::init(argc, argv, "handGhostws");
	ros::NodeHandle n;

	//world state for env1
	WorldState env_one_WorldState(1,n);
	//world state for env2
	WorldState env_two_WorldState(2,n);
	
	ros::Rate loop_rate(10); //  1/1000s
	ros::AsyncSpinner spinner(2);
 	spinner.start();

	bool hasRender = true;
	int objIndex = 1;
	int ffIndex = 2;
	int mfIndex = 3;
	int rfIndex = 4;
	int thIndex = 5;
	btVector3 objOrigin, objRotation;
	btVector3 ffOrigin, ffRotation, mfOrigin, mfRotation, rfOrigin, rfRotation, thOrigin, thRotation;
	bool ffCollision, mfCollision, rfCollision, thCollision;


	// object
	//example->changePos(objIndex,...);

	int counter = 0;
	// move the ghost object
	while (!app->m_window->requestedExit() && ros::ok())
	{
		if(hasRender){
			app->m_instancingRenderer->init();
			app->m_instancingRenderer->updateCamera(app->getUpAxis());
		}


		btScalar dtSec = 1./500.;
		example->stepSimulation(dtSec);


		//updating frist enviornemnt
		vector<vector<int>> index_of_obj_in_the_scence_for_envs = {{1,2,3,4,5},{6,7,8,9,10}};
	

		worldState ws;
		ghostwses gh_ws;
	
		for (int i=0;i<2;i++){
			cout<<"env_num::"<<i<<endl;
			if (i==0){
				ws = env_one_WorldState.WState;
			}	
			else{
				ws = env_two_WorldState.WState;
			}
				
			//*********************************updating env*********************************
			//cube
			example->changePos(index_of_obj_in_the_scence_for_envs[i][0],ws.cube.pos_x,ws.cube.pos_y,ws.cube.pos_z,ws.cube.r,ws.cube.p,ws.cube.y);
			//ff
			example->changePos(index_of_obj_in_the_scence_for_envs[i][1],ws.ff.pos_x,ws.ff.pos_y,ws.ff.pos_z,ws.ff.r,ws.ff.p,ws.ff.y);
			//mf
			example->changePos(index_of_obj_in_the_scence_for_envs[i][2],ws.mf.pos_x,ws.mf.pos_y,ws.mf.pos_z,ws.mf.r,ws.mf.p,ws.mf.y);
			//rf
			example->changePos(index_of_obj_in_the_scence_for_envs[i][3],ws.rf.pos_x,ws.rf.pos_y,ws.rf.pos_z,ws.rf.r,ws.rf.p,ws.rf.y);
			//th
			example->changePos(index_of_obj_in_the_scence_for_envs[i][4],ws.th.pos_x,ws.th.pos_y,ws.th.pos_z,ws.th.r,ws.th.p,ws.th.y);
			//************************getting env state *****************
			ffCollision = example->checkCollision(index_of_obj_in_the_scence_for_envs[i][1]);
			mfCollision = example->checkCollision(index_of_obj_in_the_scence_for_envs[i][2]);
			rfCollision = example->checkCollision(index_of_obj_in_the_scence_for_envs[i][3]);
			thCollision = example->checkCollision(index_of_obj_in_the_scence_for_envs[i][4]);
			
			if (i==0){
				env_one_WorldState.set_shared_values_from_Ghost_wrold(ros::Time::now(),
									ffOrigin, ffRotation, ffCollision,
									mfOrigin, mfRotation, mfCollision,
									rfOrigin, rfRotation, rfCollision,
									thOrigin, thRotation, thCollision);
			}
			else{
				env_two_WorldState.set_shared_values_from_Ghost_wrold(ros::Time::now(),
											ffOrigin, ffRotation, ffCollision,
											mfOrigin, mfRotation, mfCollision,
											rfOrigin, rfRotation, rfCollision,
											thOrigin, thRotation, thCollision);
			}
		
		}


	

	

		if(hasRender){
			example->renderScene();
		}

		DrawGridData dg;
		dg.upAxis = app->getUpAxis();
		app->drawGrid(dg);

		app->swapBuffer();
	
		//ROS
		  
	

	}

	example->exitPhysics();
	delete example;
	delete app;
	return 0;
}
