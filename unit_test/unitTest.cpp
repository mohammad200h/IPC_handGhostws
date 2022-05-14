//https://www.youtube.com/watch?v=Lp1ifh9TuFI

//cntrl^shift+p run ctest
#include <gtest/gtest.h>  

#include <sstream>
#include "GhostExample.h"
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

TEST(General,getQuaternionFromEuler){
    //[x,y,z]
    vector <double> rpy = {0,0,1.5};//in radaians
    //[x,y,z,w]
    vector <double> expexted_quat={0.000, 0.000, 0.682,0.732};
    for(int i =0;i<expexted_quat.size();i++){
        expexted_quat[i]=std::ceil(expexted_quat[i] * 100.0) / 100.0;
    }
    btQuaternion q = getQuaternionFromEuler(rpy[0],rpy[1],rpy[2]);
    double x = std::ceil(q.x() * 100.0) / 100.0;
    double y = std::ceil(q.y() * 100.0) / 100.0;
    double z = std::ceil(q.z() * 100.0) / 100.0;
    double w = std::ceil(q.w() * 100.0) / 100.0;

    

    EXPECT_EQ(expexted_quat[0],x);
    EXPECT_EQ(expexted_quat[1],y);
    EXPECT_EQ(expexted_quat[2],z);
    EXPECT_EQ(expexted_quat[3],w);

    cout<<"test is runing::getQuaternionFromEuler"<<endl;
    
}

TEST(General,getEuler){
    //[x,y,z,w]
    vector <double> quat={0.000, 0.000, 0.682,0.732};
    //[x,y,z]
    vector <double> expected_rpy = {0,0,1.5};//in radaians
    btQuaternion q =btQuaternion(quat[0],quat[1],quat[2],quat[3]);

    vector<double> euler = getEuler(q);


    for(int i=0;i<euler.size();i++){
        euler[i]= std::floor(euler[i] * 10.0) / 10.0;
    }

    EXPECT_EQ(euler[0],expected_rpy[0]);
    EXPECT_EQ(euler[1],expected_rpy[1]);
    EXPECT_EQ(euler[2],expected_rpy[2]);

    cout<<"test is runing::getEuler"<<endl;

}

TEST(GhostExampleCreateFunc,NumberOfEnv){
    SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Ghost Example", 1024, 768, true);
    GhostInterface_G* sim;
    OpenGLGuiHelper gui(app, false);
    CommonExampleOptions_G options(&gui);
    sim = GhostExampleCreateFunc(options,2);
    sim->initPhysics();

    int len = sim->getEnvs()->envs.size(); 
    EXPECT_TRUE(len==2)<<"len:: "<<len<<endl;

    sim->exitPhysics();
	delete sim;
	delete app;

    cout<<"test is runing::NumberOfEnv"<<endl;
}

TEST(ENV,env_offset){
    SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Ghost Example", 1024, 768, true);
    GhostInterface_G* sim;
    OpenGLGuiHelper gui(app, false);
    CommonExampleOptions_G options(&gui);
    sim = GhostExampleCreateFunc(options,2);
    sim->initPhysics();
    Envs* envs= sim->getEnvs();

    /**
     The scond env should placed 2m to the left of the first env
     offset for dirst env should be {0,0,0}
     offset for dirst env should be {2,0,0}
    **/

    //***********checking first env offset*************
    vector<double> env_offset_one = envs->get_env(1).env_offset;
    for(int i=0; i<3;i++){
        EXPECT_EQ(env_offset_one[i],0);
    }
    //***********checking second env offset************
    vector<double> env_offset_two = envs->get_env(2).env_offset;
    for(int i=0; i<3;i++){
        if (i==0)
            EXPECT_EQ(env_offset_two[i],2);
        else
            EXPECT_EQ(env_offset_two[i],0);
    }
    sim->exitPhysics();
	delete sim;
	delete app;

    cout<<"test is runing::env_offset"<<endl;
}

TEST(ENV,env_index){
    SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Ghost Example", 1024, 768, true);
    GhostInterface_G* sim;
    OpenGLGuiHelper gui(app, false);
    CommonExampleOptions_G options(&gui);
    sim = GhostExampleCreateFunc(options,2);
    sim->initPhysics();
    Envs* envs= sim->getEnvs();

    //***********checking first env indexs*************
    vector<int> env_indexs_one_expected = {1,2,3,4,5};
    vector<int> env_indexs_one = envs->get_env(1).env_indexs;
    for(int i=0; i<5;i++){
        EXPECT_EQ(env_indexs_one[i],env_indexs_one_expected[i]);
    }
    //***********checking second env indexs************
    vector<int> env_indexs_two_expected = {6,7,8,9,10};
    vector<int> env_indexs_two = envs->get_env(2).env_indexs;
    for(int i=0; i<5;i++){
        EXPECT_EQ(env_indexs_two[i],env_indexs_two_expected[i]);
    }
    sim->exitPhysics();
	delete sim;
	delete app;

    cout<<"test is runing::env_index"<<endl;
}

TEST(ENV,env_updateEnvGymState){
    SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Ghost Example", 1024, 768, true);
    GhostInterface_G* sim;
    OpenGLGuiHelper gui(app, false);
    CommonExampleOptions_G options(&gui);
    sim = GhostExampleCreateFunc(options,2);
    sim->initPhysics();
    Envs* envs= sim->getEnvs();
    Env env = envs->get_env(1);
    //***********************Checking Internal state*******updateInternalState*****
    //*********checking state before assignment**************
    State* state  = env.state;
   
    EXPECT_EQ(state->gymState.ff.pos.x,0);
    EXPECT_EQ(state->gymState.ff.pos.y,0);
    EXPECT_EQ(state->gymState.ff.pos.z,0);
    EXPECT_EQ(state->gymState.ff.orn.r,0);
    EXPECT_EQ(state->gymState.ff.orn.p,0);
    EXPECT_EQ(state->gymState.ff.orn.y,0);

    EXPECT_EQ(state->gymState.mf.pos.x,0);
    EXPECT_EQ(state->gymState.mf.pos.y,0);
    EXPECT_EQ(state->gymState.mf.pos.z,0);
    EXPECT_EQ(state->gymState.mf.orn.r,0);
    EXPECT_EQ(state->gymState.mf.orn.p,0);
    EXPECT_EQ(state->gymState.mf.orn.y,0);

    EXPECT_EQ(state->gymState.rf.pos.x,0);
    EXPECT_EQ(state->gymState.rf.pos.y,0);
    EXPECT_EQ(state->gymState.rf.pos.z,0);
    EXPECT_EQ(state->gymState.rf.orn.r,0);
    EXPECT_EQ(state->gymState.rf.orn.p,0);
    EXPECT_EQ(state->gymState.rf.orn.y,0);

    EXPECT_EQ(state->gymState.th.pos.x,0);
    EXPECT_EQ(state->gymState.th.pos.y,0);
    EXPECT_EQ(state->gymState.th.pos.z,0);
    EXPECT_EQ(state->gymState.th.orn.r,0);
    EXPECT_EQ(state->gymState.th.orn.p,0);
    EXPECT_EQ(state->gymState.th.orn.y,0);

    EXPECT_EQ(state->gymState.cube.pos.x,0);
    EXPECT_EQ(state->gymState.cube.pos.y,0);
    EXPECT_EQ(state->gymState.cube.pos.z,0);
    EXPECT_EQ(state->gymState.cube.orn.r,0);
    EXPECT_EQ(state->gymState.cube.orn.p,0);
    EXPECT_EQ(state->gymState.cube.orn.y,0);

    //*********checking state after assignment**************
    GymworldState new_state;

    new_state.ff.pos.x = 20;
    new_state.ff.pos.z = 10;
    new_state.cube.orn.r=1.5; //radian

    env.updateEnvGymState(new_state);

    EXPECT_EQ(state->gymState.ff.pos.x,20);
    EXPECT_EQ(state->gymState.ff.pos.z,10);
    EXPECT_EQ(state->gymState.cube.orn.r,1.5);

    //********************Checking change in state in physic engine*******updateEnvGymState
    //*********checking physic engine state*****first finger****
     
    int ff_index = 2;
    btVector3 coordinate_ff =env.m_dynamicsWorld->getCollisionObjectArray()[ff_index]->getWorldTransform().getOrigin();
    double ff_pos_x = coordinate_ff[0];
    double ff_pos_z = coordinate_ff[2];

    EXPECT_EQ(new_state.ff.pos.x,ff_pos_x);
    EXPECT_EQ(new_state.ff.pos.z,ff_pos_z);
    //*********checking physic engine state*****cube****
    int cube_index = 1;
    btQuaternion coordinate_cube =env.m_dynamicsWorld->getCollisionObjectArray()[cube_index]->getWorldTransform().getRotation();
    vector<double>  euler= getEuler(coordinate_cube);
    double cube_orn_r = euler[0];
    EXPECT_EQ(new_state.cube.orn.r,std::floor(cube_orn_r * 10.0) / 10.0)<<"cube_orn_r:: "<<cube_orn_r;

    sim->exitPhysics();
	delete sim;
	delete app;

    cout<<"test is runing::env_updateEnvGymState"<<endl;
     
}

TEST(ENVs,is_this_id_assinged_to_env){
    SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Ghost Example", 1024, 768, true);
    GhostInterface_G* sim;
    OpenGLGuiHelper gui(app, false);
    CommonExampleOptions_G options(&gui);
    sim = GhostExampleCreateFunc(options,2);
    sim->initPhysics();
    Envs* envs= sim->getEnvs();
    //************************
    boost::uuids::uuid client_id = boost::uuids::random_generator()();
    GymworldState gymState;
    gymState.key.id = client_id;
    /***********adding first client to env
     *There is no client assigned to any env in the begining
     ************************************/
    bool expectation = false;
    bool res=envs->is_this_id_assinged_to_env(client_id);

    EXPECT_EQ(expectation,res);

    cout<<"test is runing::is_this_id_assinged_to_env"<<endl;
}

TEST(ENVs,assign_client_to_env){
    SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Ghost Example", 1024, 768, true);
    GhostInterface_G* sim;
    OpenGLGuiHelper gui(app, false);
    CommonExampleOptions_G options(&gui);
    sim = GhostExampleCreateFunc(options,2);
    sim->initPhysics();
    Envs* envs= sim->getEnvs();
    //************************
    boost::uuids::uuid client_id = boost::uuids::random_generator()();
    GymworldState gymState;
    gymState.key.id = client_id;
    /***********adding first client to env
     *There is no client assigned to any env in the begining
     ************************************/
    bool expectation = false;
    bool res=envs->is_this_id_assinged_to_env(client_id);
    
    ASSERT_EQ(expectation,res);
    //There is no binding at the start of simulation
    EXPECT_EQ(envs->client_id_binding.size(),0);
    //Bind a new clent to an unassigned env
    envs->assign_client_to_env(client_id);
    // EXPECT_EQ(envs->client_id_binding.size(),1);

    sim->exitPhysics();
	delete sim;
	delete app;

    cout<<"test is runing::assign_client_to_env"<<endl;

}

TEST(ENVs,updateEnvGymState){
    SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Ghost Example", 1024, 768, true);
    GhostInterface_G* sim;
    OpenGLGuiHelper gui(app, false);
    CommonExampleOptions_G options(&gui);
    sim = GhostExampleCreateFunc(options,2);
    sim->initPhysics();
    Envs* envs= sim->getEnvs();
    //#########################
    boost::uuids::uuid client_id = boost::uuids::random_generator()();
    GymworldState gymState;
    gymState.key.id = client_id;
    //setting ff
    gymState.ff.pos.x = 10;
    gymState.ff.pos.y = 0;
    gymState.ff.pos.z = 0;
    //setting cube
    gymState.cube.pos.x = 20;
    gymState.cube.pos.y = 0;
    gymState.cube.pos.z = 0;
    //updating envs
    envs->assign_client_to_env(client_id);
    envs->updateEnvGymState(client_id,gymState);
    /**
      client should be assinged to first env
    **/
    int env_id = envs->client_id_binding[0].first;
    EXPECT_EQ(env_id,1);
    /**
     * gym state for first env should be changed
     * we will check this by checking client ic
    **/
    boost::uuids::uuid env_one_gym_state_id  = envs->get_env(env_id).state->gymState.key.id;
    EXPECT_EQ(client_id,env_one_gym_state_id);
    /**
    * Testing gym state change in pos
    **/

    double exp_gymState_ff_pos_x = envs->get_env(env_id).state->gymState.ff.pos.x;
    double exp_gymState_ff_pos_y = envs->get_env(env_id).state->gymState.ff.pos.y;
    double exp_gymState_ff_pos_z = envs->get_env(env_id).state->gymState.ff.pos.z;
    //setting cube
    double exp_gymState_cube_pos_x = envs->get_env(env_id).state->gymState.cube.pos.x;
    double exp_gymState_cube_pos_y = envs->get_env(env_id).state->gymState.cube.pos.y;
    double exp_gymState_cube_pos_z = envs->get_env(env_id).state->gymState.cube.pos.z;

    EXPECT_EQ(exp_gymState_ff_pos_x,gymState.ff.pos.x );
    EXPECT_EQ(exp_gymState_ff_pos_y,gymState.ff.pos.y );
    EXPECT_EQ(exp_gymState_ff_pos_z,gymState.ff.pos.z );

    EXPECT_EQ(exp_gymState_cube_pos_x,gymState.cube.pos.x );
    EXPECT_EQ(exp_gymState_cube_pos_y,gymState.cube.pos.y );
    EXPECT_EQ(exp_gymState_cube_pos_z,gymState.cube.pos.z );


    sim->exitPhysics();
	delete sim;
	delete app;

    cout<<"test is runing::updateEnvGymState"<<endl;

}


TEST(GhostExampleCreateFunc,setWSCollisitionFlagToTrue){
    SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Ghost Example", 1024, 768, true);
    GhostInterface_G* sim;
    OpenGLGuiHelper gui(app, false);
    CommonExampleOptions_G options(&gui);
    sim = GhostExampleCreateFunc(options,2);
    sim->initPhysics();
    Envs* envs= sim->getEnvs();
    Env env = envs->get_env(1);
    //********************
    bool res = false;
    for (int i=1;i<=4;i++){
        env.setWSCollisitionFlagToTrue(env.obj_dic[i]);
        switch (i){
            case 1: //FF
                res = env.state->ghostState.ff;
                break;
            case 2: //MF
                res = env.state->ghostState.mf;
                break;
            case 3: //RF
                res = env.state->ghostState.rf;
                break;
            case 4: //Th
                res = env.state->ghostState.th;
                break;
        }
        EXPECT_TRUE(res);
    }


    sim->exitPhysics();
	delete sim;
	delete app;

    cout<<"test is runing::setWSCollisitionFlagToTrue"<<endl;
   
}

TEST(GhostExampleCreateFunc,getEnvGhostState){
    SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Ghost Example", 1024, 768, true);
    GhostInterface_G* sim;
    OpenGLGuiHelper gui(app, false);
    CommonExampleOptions_G options(&gui);
    sim = GhostExampleCreateFunc(options,2);
    sim->initPhysics();
    Envs* envs= sim->getEnvs();
    Env env = envs->get_env(1);

    bool hasRender = true;

    /*
    1. set GymState in a way to cause collision with box
    2. step simulation
    3. get ghost state
    */
    int counter = 0;
    boost::uuids::uuid client_id = boost::uuids::random_generator()();
    GhostWorldState ghostState ;
   	while (!app->m_window->requestedExit() && counter !=10)
	{   counter++;
		if(hasRender){
			app->m_instancingRenderer->init();
			app->m_instancingRenderer->updateCamera(app->getUpAxis());
		}

		btScalar dtSec = 1./500.;
        //#######Assign client to env and update state#########
        if (!envs->is_this_id_assinged_to_env(client_id)){
            
            GymworldState gymState;
            gymState.key.id = client_id;
            //setting ff
            gymState.ff.pos.x = 0;
            gymState.ff.pos.y = 0;
            gymState.ff.pos.z = 0.05;

            gymState.mf.pos.x = 0;
            gymState.mf.pos.y = 0;
            gymState.mf.pos.z = 0.5;

            gymState.rf.pos.x = 0;
            gymState.rf.pos.y = 0;
            gymState.rf.pos.z = 0.5;

            gymState.th.pos.x = 0;
            gymState.th.pos.y = 0;
            gymState.th.pos.z = 0.5;
            //setting cube
            gymState.cube.pos.x =0 ;
            gymState.cube.pos.y = 0;
            gymState.cube.pos.z = 0;
            //updating envs
            envs->assign_client_to_env(client_id);
            envs->updateEnvGymState(client_id,gymState);
        }

		//###################Step simulation###################

		sim->stepSimulation(dtSec);
        //###################Get Ghost state##################
		ghostState = envs->getEnvGhostState(client_id);
        /**
         * the flag for first finger should be true since they are colliding
        **/



		if(hasRender)
			sim->renderScene();
        


		DrawGridData dg;
		dg.upAxis = app->getUpAxis();
		app->drawGrid(dg);

		app->swapBuffer();
	}
    bool expexted_ff_flag = true;
    EXPECT_EQ(expexted_ff_flag,ghostState.ff);

	sim->exitPhysics();
	delete sim;
	delete app;

    cout<<"test is runing::getEnvGhostState"<<endl;
	
}


// get_env_id_binded_to_client_id

TEST(GhostExampleCreateFunc,restGhostState){
    SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Ghost Example", 1024, 768, true);
    GhostInterface_G* sim;
    OpenGLGuiHelper gui(app, false);
    CommonExampleOptions_G options(&gui);
    sim = GhostExampleCreateFunc(options,2);
    sim->initPhysics();
    Envs* envs= sim->getEnvs();
    Env env = envs->get_env(1);
    //*******************
    boost::uuids::uuid client_id = boost::uuids::random_generator()();
    GymworldState gymState;
    gymState.key.id = client_id;
    /***********adding first client to env
     *There is no client assigned to any env in the begining
     ************************************/
  
    bool res=envs->is_this_id_assinged_to_env(client_id);
    if (res ==false)
        envs->assign_client_to_env(client_id);

    env.state->ghostState.ff=true;

    envs->restEnvGhostState(client_id);

    EXPECT_FALSE(env.state->ghostState.ff);

    sim->exitPhysics();
	delete sim;
	delete app;
    cout<<"test is runing::restGhostState"<<endl;
}
