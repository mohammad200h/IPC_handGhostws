#include <math.h>
#include <vector>
#include <string>
#include <pandaFramework.h>
#include <collisionTraverser.h>
#include <collisionHandlerEvent.h>
#include <eventHandler.h>
#include <collisionNode.h>
#include <collisionBox.h>
#include <nodePath.h>
#include <pandaSystem.h>
#include <animControlCollection.h>
#include <asyncTaskManager.h>
#include <auto_bind.h>
#include "cIntervalManager.h"


using namespace std;

struct collision_box{
    LPoint3 center;
    PN_stdfloat x;
    PN_stdfloat y;
    PN_stdfloat z;
    collision_box(LPoint3 center,PN_stdfloat x,PN_stdfloat y,PN_stdfloat z){
        this->center = center;
        this->x = x;
        this->y = y;
        this->z = z;
    }
};

class Finger{
    Finger();
    NodePath finger_ws;
    CollisionTraverser* cTrav;
    CollisionHandlerEvent* notifier;
    public:
      Finger(WindowFramework* win, CollisionTraverser* cTrav,CollisionHandlerEvent* notifier,vector<float> offset,string finger_name);
      void ws_collision_boxes();
      vector<float> get_finger_bias(string finger_name);
};

class Thumb{
    Thumb();
    NodePath thumb_ws;
    CollisionTraverser* cTrav;
    CollisionHandlerEvent* notifier;
    
    public:
      Thumb(WindowFramework* win,CollisionTraverser* cTrav,CollisionHandlerEvent* notifier,vector<float> offset);
      void ws_collision_boxes();
};

class Cube{

    
    public:
      Cube(WindowFramework* win,vector<float> offset);
};

class Env{
      
        Cube*   cube;
        Finger* ff;
        Finger* mf;
        Finger* rf;
        Thumb*  th;
        int env_id;
        WindowFramework* win;
        CollisionTraverser cTrav ;
      
        CollisionHandlerEvent notifier;
        PT(AsyncTaskManager) taskMgr; 
        inline static vector<tuple<Env *,int>> me_vec;
    public:

        Env(WindowFramework* win,vector<float> offset,int env_id);
        void setupCollision();
        void update();
        static void onCollision(const Event *event);
        static AsyncTask::DoneStatus collisionTask(GenericAsyncTask* task, void* thisPtr);
};


class GhostWorld{

    GhostWorld();
    NodePath* models;
    WindowFramework* win;
    int num_envs;
    vector<float> offset_size;
    vector<Env> envs;
    vector<int> env_ids;
    public:
        GhostWorld(int num_envs,vector<float> offset_size,int argc, char* argv[]);
        vector<vector<float>>  calculate_offset_for_env();
        void setup_camera();
        void run();
        void closeFramework();
};
