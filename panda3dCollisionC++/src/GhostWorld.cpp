
#include "GhostWorld.h"
PandaFramework framework;
// WindowFramework* WIN = framework.open_window();

//************************Finger***********************
Finger::Finger(WindowFramework* win, CollisionTraverser* cTrav,CollisionHandlerEvent* notifier,vector<float> offset,string finger_name){
    

    this->cTrav = cTrav;
    this->notifier = notifier;

    string path = "/home/mamad/IPC_handGhostws/panda3dCollisionC++/src/";
    finger_ws = win->load_model(framework.get_models(),path+"FF.egg");
    // finger_ws = win->load_model(framework.get_models(),"panda");
    vector<float> finger_bias = this->get_finger_bias(finger_name);
    finger_ws.set_pos(  0+offset[0]+finger_bias[0],
                        0+offset[1]+finger_bias[1],
                        0+offset[2]+finger_bias[2]);
    finger_ws.reparent_to(win->get_render());

    //  collision 
    this->ws_collision_boxes();
}

void Finger::ws_collision_boxes(){
    vector<collision_box> CBs;

    CBs.push_back(collision_box(LPoint3(-0.03336,-0.003321,0.219165),0.035255,0.013681,0.002792));
    CBs.push_back(collision_box(LPoint3(-0.03336,-0.010602,0.224399),0.035255,0.013681,0.002792));
    CBs.push_back(collision_box(LPoint3(-0.03336,-0.010991,0.244547),0.035255,0.013681,0.033984));
    CBs.push_back(collision_box(LPoint3(-0.03336,-0.010991,0.268406),0.035255,0.013681,0.011236));
    CBs.push_back(collision_box(LPoint3(-0.03336,0.004424,0.253941),0.021793,0.013681,0.062659));
    CBs.push_back(collision_box(LPoint3(-0.032871 ,0.02094,0.274267),0.00616,0.013681,0.043832));
    CBs.push_back(collision_box(LPoint3(-0.032871,0.037748,0.281319),0.004354,0.013681,0.043832));
    CBs.push_back(collision_box(LPoint3(-0.032871,0.053085,0.289576),0.01066,0.013681,0.024994));
    CBs.push_back(collision_box(LPoint3(-0.032871,0.068155,0.290964),0.025145,0.013681,0.016356));
    CBs.push_back(collision_box(LPoint3(-0.032871,0.082526,0.290964),0.025145,0.013681,0.003581));

    for(collision_box CB:CBs){
        //https://discourse.panda3d.org/t/help-with-adding-collisionnodes-as-children-to-a-node-edited-title/28538
        // CollisionNode("ws");   
        // PT(CollisionBox) cbox= new CollisionBox(CB.center,CB.x,CB.y,CB.z);
        // node.add_solid(cbox);
        NodePath col = finger_ws.attach_new_node(new CollisionNode("ws"));
        CollisionNode *cNode;
        cNode = (CollisionNode*) col.node();
        cNode->add_solid(new CollisionBox(CB.center,CB.x,CB.y,CB.z));
        col.show();
        cTrav->add_collider(col,notifier);
        


        
        
    }
}

vector<float> Finger::get_finger_bias(string finger_name){
    map <string,vector<float>> biases;
    biases.insert(pair<string,vector<float>>("ff",{0,0,0}));
    biases.insert(pair<string,vector<float>>("mf",{0.022,0.002536,0.003068}));
    biases.insert(pair<string,vector<float>>("rf",{0.044,0,0}));
    
    return biases[finger_name];
}
//************************Thumb************************
Thumb::Thumb(WindowFramework* win,CollisionTraverser* cTrav,CollisionHandlerEvent* notifier,vector<float> offset){
    this->cTrav = cTrav;
    this->notifier = notifier;

    string path = "/home/mamad/IPC_handGhostws/panda3dCollisionC++/src/";
    thumb_ws = win->load_model(framework.get_models(),path+"TH.egg");
    // finger_ws = win->load_model(framework.get_models(),"panda");
   
    thumb_ws.set_pos(  0+offset[0],
                        0+offset[1],
                        0+offset[2]);
    thumb_ws.reparent_to(win->get_render());

    //  collision 
    this->ws_collision_boxes();
}
void Thumb::ws_collision_boxes(){
    vector<collision_box> CBs;

    CBs.push_back(collision_box(LPoint3(-0.03336,-0.003321,0.219165),0.035255,0.013681,0.002792));
    CBs.push_back(collision_box(LPoint3(-0.03857,0.035046,0.24799),0.050777,0.014184,0.009054));
    CBs.push_back(collision_box(LPoint3(-0.03857,0.031686,0.254842),0.029433,0.014184,0.00337));
    CBs.push_back(collision_box(LPoint3(-0.03857,0.053628,0.234354),0.050777,0.006749,0.009054));
    CBs.push_back(collision_box(LPoint3(-0.025502,0.043943,0.238845),0.050777,0.006749,0.005137));
    CBs.push_back(collision_box(LPoint3( -0.005499,0.022437,0.250206),0.010429,0.028513,0.004289)); 
    CBs.push_back(collision_box(LPoint3( -0.05316 ,0.059074,0.226753),0.028725,0.001,0.009054)); 
    CBs.push_back(collision_box(LPoint3( -0.027932,0.059074,0.226753),0.01719,0.002725,0.009054));
    CBs.push_back(collision_box(LPoint3( -0.009648,0.047519,0.233693),0.01719,0.002725,0.003962));
    CBs.push_back(collision_box(LPoint3( -0.08516,0,0.242063),0.005974,0.045269,0.005066));

    CBs.push_back(collision_box(LPoint3( -0.0773 ,0 ,0.247971 ),0.005974 ,0.045269 ,0.009168 )); 
    CBs.push_back(collision_box(LPoint3( -0.069499,-0.001781 ,0.249856 ),0.005974 ,0.043934 ,0.013232 )); 
    CBs.push_back(collision_box(LPoint3( -0.061971,-0.029332 ,0.24799 ),0.021347 ,0.014184,0.006203)); 
    CBs.push_back(collision_box(LPoint3(  -0.042739,-0.030442 ,0.253914 ),0.021347,0.014184,0.003761  )); 
    CBs.push_back(collision_box(LPoint3(  -0.048978,-0.042207 ,0.24799 ),0.021347,0.01,0.006203)); 
    CBs.push_back(collision_box(LPoint3( -0.070232,-0.041907,0.236535),0.016397,0.012933,0.001963)); 
    CBs.push_back(collision_box(LPoint3( -0.066965,-0.041907,0.243616),0.006381,0.01,0.007064)); 
    CBs.push_back(collision_box(LPoint3( -0.065435,-0.052215,0.235366),0.021347,0.006,0.006203)); 
    CBs.push_back(collision_box(LPoint3( -0.048978,-0.042207,0.24799),0.021348,0.01,0.006203)); 
    CBs.push_back(collision_box(LPoint3( -0.048978,-0.046478,0.243002),0.021347,0.01,0.002305));

    CBs.push_back(collision_box(LPoint3( -0.051402 ,-0.050888 ,0.239707 ),0.021347 ,0.006 ,0.002305 )); 
    CBs.push_back(collision_box(LPoint3(  -0.065435 ,-0.052215  ,0.235366  ), 0.021347 ,0.006  , 0.006203 )); 
    CBs.push_back(collision_box(LPoint3( -0.048821  ,-0.055404  , 0.235366 ), 0.010576 ,0.002  , 0.006203 )); 
    CBs.push_back(collision_box(LPoint3(  -0.056343 ,0.063603  ,0.216837  ), 0.022291 ,0.002725  ,0.009054  )); 
    CBs.push_back(collision_box(LPoint3(  -0.062885 ,0.043943  ,0.238845  ),0.022225  ,0.006749  ,0.005137  )); 
    CBs.push_back(collision_box(LPoint3( -0.070289  , -0.057788 ,0.224329  ),0.021347  , 0.006 ,0.006203  )); 
    CBs.push_back(collision_box(LPoint3(  -0.07023 ,-0.062591  ,0.216792  ),0.019066  ,0.003802  , 0.006203 )); 
    CBs.push_back(collision_box(LPoint3( -0.092849  ,0  ,0.236717  ), 0.005974 ,0.045269  ,0.005066  )); 
    CBs.push_back(collision_box(LPoint3(  -0.101208 ,-0.003902  ,0.230869  ), 0.005974 ,0.045269 ,0.005066  )); 
    CBs.push_back(collision_box(LPoint3( -0.108491  ,-0.003902  ,0.222479  ),0.003405  ,0.041394 ,0.005066  )); 

    CBs.push_back(collision_box(LPoint3(  -0.110514 ,-0.003902  ,0.217364  ),0.005974 ,0.045269  ,0.005066  )); 
    CBs.push_back(collision_box(LPoint3( -0.113856  , -0.003902 ,0.211484  ),0.002  , 0.045268 , 0.005066 )); 
    CBs.push_back(collision_box(LPoint3( -0.077508  ,-0.031635  ,0.243616  ), 0.006381 ,0.01746  ,0.007064  )); 
    CBs.push_back(collision_box(LPoint3( -0.08354  ,-0.036632  ,0.234664  ),0.006381  ,0.007215  ,0.007064  )); 
    CBs.push_back(collision_box(LPoint3(  -0.08354 ,-0.028306  ,0.236847  ), 0.006381 ,0.007215  ,0.001852  )); 
    CBs.push_back(collision_box(LPoint3(  -0.090588 ,-0.034218  ,0.233431  ),0.006381  ,0.012  ,0.007064  )); 
    CBs.push_back(collision_box(LPoint3(  -0.08354 ,-0.043097  ,0.234664  ), 0.006381 ,0.00439  ,0.007064  )); 
    CBs.push_back(collision_box(LPoint3( -0.10014  ,-0.034218  ,0.223415  ),0.003943  ,0.012  ,0.009406  )); 
    CBs.push_back(collision_box(LPoint3(  -0.095726 ,-0.034218  ,0.22792  ),0.002663  ,0.012  ,0.007064  )); 
    CBs.push_back(collision_box(LPoint3( -0.10501  ,-0.034218  , 0.215893 ),0.003943  ,0.012  ,0.009406  )); 

    CBs.push_back(collision_box(LPoint3( -0.109916   ,-0.034218  ,0.20814  ),0.003943  ,0.012 ,0.009406 )); 
    CBs.push_back(collision_box(LPoint3(  -0.07472 ,-0.065101  ,0.209584  ), 0.019066 ,0.001855  ,0.006203  )); 
    CBs.push_back(collision_box(LPoint3( -0.080926  ,-0.065101  ,0.202227  ),0.012172  ,0.001855  ,0.006203 )); 
    CBs.push_back(collision_box(LPoint3(  -0.109084 ,-0.041974  ,0.197096  ),0.001993  ,0.012 ,0.009406  )); 
    CBs.push_back(collision_box(LPoint3(  -0.11293 ,-0.034559 ,0.197096  ),0.001993  ,0.012  ,0.009406  )); 
    CBs.push_back(collision_box(LPoint3( -0.109916  ,-0.034218  ,0.20814  ),0.003943  , 0.012 ,0.009406  )); 
    CBs.push_back(collision_box(LPoint3( -0.091944  ,-0.060199  ,0.201091  ), 0.012172 , 0.001855 ,0.006203  )); 
    CBs.push_back(collision_box(LPoint3( -0.087079  ,-0.058475  ,0.209983  ),0.012172  ,0.004155  ,0.006203  )); 
    CBs.push_back(collision_box(LPoint3( -0.087079  , -0.0544 ,0.218198  ),0.012172  ,0.00598  ,0.006203  )); 
    CBs.push_back(collision_box(LPoint3(  -0.087079 ,-0.047396  ,0.228214  ),0.012172  ,0.003881  ,0.006203  )); 
            
    CBs.push_back(collision_box(LPoint3(  -0.087079  ,-0.051723  ,0.225312  ),0.012172  ,0.001252  ,0.006203  )); 
    CBs.push_back(collision_box(LPoint3(   -0.084301 , -0.055176 ,0.225312  ), 0.006367 ,0.003817  , 0.006203 ));
    CBs.push_back(collision_box(LPoint3(   -0.09587 ,-0.056952  , 0.201091 ), 0.012172 ,0.001855  ,0.006203  )); 
    CBs.push_back(collision_box(LPoint3(  -0.100727  ,-0.053688  ,0.197349 ),0.007804 , 0.001855 ,0.006203  )); 
    CBs.push_back(collision_box(LPoint3(   -0.096065 ,-0.052488  ,0.209983  ),0.005686  , 0.004155 , 0.006203 )); 
    CBs.push_back(collision_box(LPoint3(  -0.099433  ,-0.052488  ,0.204605 ),0.005686  ,0.004155  ,0.006203  )); 
    CBs.push_back(collision_box(LPoint3(   -0.093078 ,-0.047396  ,0.220792  ),0.007922  ,0.004273  ,0.006203 )); 
    CBs.push_back(collision_box(LPoint3(   -0.095976 ,-0.048959  ,0.214071  ),0.007922  ,0.004147  ,0.006203 )); 
    CBs.push_back(collision_box(LPoint3(   -0.102235 ,-0.047131  ,0.206631  ),0.005021  ,0.004147 , 0.006203 ));
    CBs.push_back(collision_box(LPoint3(   -0.101896 ,-0.042453  ,0.214071  ),0.007922  ,0.004147  ,0.006203  )); 

    CBs.push_back(collision_box(LPoint3( -0.105584  ,-0.042453  ,0.207589  ),0.007922  ,0.004147  ,0.006203  )); 
    CBs.push_back(collision_box(LPoint3( -0.096551  ,-0.042674  ,0.221787  ),0.007922 ,0.004147  ,0.006203 )); 
    CBs.push_back(collision_box(LPoint3( -0.092677  ,-0.042674  ,0.228904  ), 0.007922 ,0.004147  ,0.006203  )); 
    CBs.push_back(collision_box(LPoint3(  -0.063064 ,-0.057058  ,0.22981  ),0.031824  ,0.006  ,0.003974  ));
    CBs.push_back(collision_box(LPoint3(  -0.105709 ,-0.003902  ,0.22774  ),0.002534  ,0.041394  ,0.005066  )); 
    CBs.push_back(collision_box(LPoint3(  -0.098892 ,-0.003902  , 0.235683 ),0.003059  ,0.042865  ,0.003178  )); 
    CBs.push_back(collision_box(LPoint3(  -0.068779 ,0.032311  ,0.245256  ),0.005423  , 0.014184 , 0.006777 )); 
    CBs.push_back(collision_box(LPoint3( -0.078776  ,0.032311  ,0.241  ),0.005423  ,0.014184  ,0.005273  )); 
    CBs.push_back(collision_box(LPoint3( -0.086747  , 0.030035 ,0.236354  ),0.005423  ,0.008613  ,0.006777  )); 
    CBs.push_back(collision_box(LPoint3( -0.097575  , 0.02278 ,0.232427  ),0.003208  ,0.003842  ,0.006777  )); 

    CBs.push_back(collision_box(LPoint3( -0.102797  ,0.022025  ,0.225196  ),0.005375  ,0.005256 ,0.006777  )); 
    CBs.push_back(collision_box(LPoint3( -0.107098  ,0.022025  ,0.217462  ),0.005375 ,0.005256  ,0.006777  ));
    CBs.push_back(collision_box(LPoint3(  -0.111058 ,0.022025  ,0.211951  ),0.003154  ,0.005256  ,0.006777  )); 
    CBs.push_back(collision_box(LPoint3(  -0.111058 ,0.02845  ,0.211951  ),0.003154  , 0.005256 ,0.006777  )); 
    CBs.push_back(collision_box(LPoint3( -0.107443  ,0.02845  ,0.217373  ),0.003154 ,0.005256  , 0.003771 )); 
    CBs.push_back(collision_box(LPoint3(  -0.092537 ,0.02845  ,0.234531  ),0.003154  , 0.005256 ,0.003771  )); 
    CBs.push_back(collision_box(LPoint3( -0.079091  ,0.042388  ,0.234509  ),0.008673 ,0.005082  ,0.004438 )); 
    CBs.push_back(collision_box(LPoint3(  -0.069628 ,0.049743  ,0.233575  ),0.008673 ,0.008613  ,0.004557  )); 
    CBs.push_back(collision_box(LPoint3( -0.079198  ,0.047867  ,0.230892  ),0.008673  ,0.003347  ,0.004557  ));
    CBs.push_back(collision_box(LPoint3( -0.074457  ,0.052055  ,0.226789  ),0.014938  ,0.003347  ,0.004557  )); 

    CBs.push_back(collision_box(LPoint3( -0.073831  ,0.055547  ,0.221965  ), 0.014938 ,0.003347  , 0.004557 )); 
    CBs.push_back(collision_box(LPoint3( -0.073879  ,0.058229  ,0.217676  ), 0.014938 ,0.003347  ,0.002147  )); 
    CBs.push_back(collision_box(LPoint3(  -0.073879 , 0.059608 , 0.215207 ),0.014938  , 0.003347 ,0.002147  )); 
    CBs.push_back(collision_box(LPoint3( -0.072994  ,0.061826  ,0.212167  ),0.010177  ,0.003347  ,0.002147  )); 
    CBs.push_back(collision_box(LPoint3( -0.035415  ,-0.042235  ,0.247589  ), 0.004363 ,0.008704  ,0.004504 )); 
    CBs.push_back(collision_box(LPoint3(  -0.028904 ,-0.038525  ,0.247794  ),0.004363  ,0.008704  ,0.00626  )); 
    CBs.push_back(collision_box(LPoint3(  -0.025929 ,-0.028981  ,0.251355  ),0.01005  ,0.008704  ,0.005986  )); 
    CBs.push_back(collision_box(LPoint3(  -0.005499 ,0.001561  ,0.250208  ),0.010429  ,0.011338  ,0.004289  )); 
    CBs.push_back(collision_box(LPoint3(  -0.005499 ,0.001561  ,0.250994  ),0.010429  ,0.011338  ,0.004289  ));
    CBs.push_back(collision_box(LPoint3(-0.009162 ,-0.01219 ,0.250344  ),0.005665  ,0.011338  ,0.004289     )); 

    CBs.push_back(collision_box(LPoint3( -0.010751  ,0.008519  ,0.255339  ),0.012187  ,0.035386  ,0.004289  )); 
    CBs.push_back(collision_box(LPoint3( -0.014448  ,0.055186  ,0.226753  ),0.007289  , 0.002725 ,0.009054  )); 
    CBs.push_back(collision_box(LPoint3( -0.005499  ,0.031849  ,0.245093  ),0.010429  ,0.011078  ,0.004289  )); 
    CBs.push_back(collision_box(LPoint3(  0.002345 ,0.036143  ,0.238839  ),0.010429  ,0.011078  ,0.004289   )); 
    CBs.push_back(collision_box(LPoint3( 0.00602  ,0.022749  ,0.238839  ),0.00425  ,0.011078  ,0.004289     )); 
    CBs.push_back(collision_box(LPoint3(  0.002688 ,0.020848  ,0.244782  ),0.00425 ,0.031496  ,0.004289     )); 
    CBs.push_back(collision_box(LPoint3( -0.06255  ,-0.001781  ,0.253123  ), 0.005974 ,0.043934  ,0.008629  )); 
    CBs.push_back(collision_box(LPoint3( -0.03857  ,-0.00293  , 0.263793 ),0.030688  , 0.034158 ,0.002446   )); 

    for(collision_box CB:CBs){
        NodePath col = thumb_ws.attach_new_node(new CollisionNode("ws"));
        CollisionNode *cNode;
        cNode = (CollisionNode*) col.node();
        cNode->add_solid(new CollisionBox(CB.center,CB.x,CB.y,CB.z));
        col.show();
        cTrav->add_collider(col,notifier);
    }

}
//************************Cube*************************
Cube::Cube(WindowFramework* win,vector<float> offset){
    string path = "/home/mamad/IPC_handGhostws/panda3dCollisionC++/src/";
    NodePath cube = win->load_model(framework.get_models(),path+"Cube.egg");
    cube.set_scale(0.01);
    cube.set_pos(   0+offset[0] ,
                    0+offset[1] ,
                    0+offset[2] );
    cube.reparent_to(win->get_render());
}
//************************Env**************************
Env::Env(WindowFramework* win,vector<float> offset,int env_id){
        this->me_vec.push_back(tuple<Env *,int>(this,env_id));
        this->env_id = env_id;
        cout<<"this->env_id:: "<<this->env_id<<endl;
        this->win = win;
   
        this->cube = new Cube  (win,offset);
        this->ff   = new Finger(win,&cTrav,&notifier,offset,"ff");
        this->mf   = new Finger(win,&cTrav,&notifier,offset,"mf");
        this->rf   = new Finger(win,&cTrav,&notifier,offset,"rf");
        this->th   = new Thumb (win,&cTrav,&notifier,offset);

       
        this->setupCollision();

        //check collision in the loop
   
	

}
void Env::setupCollision(){

    //https://discourse.panda3d.org/t/python-to-c/9587/3
    //https://discourse.panda3d.org/t/simple-collision-example/8267/11
    //https://discourse.panda3d.org/t/collisions/6793
    //https://discourse.panda3d.org/t/retrieving-collisionentry-from-collisionhandlerevent/6650
    //https://discourse.panda3d.org/t/who-handles-the-events-loop-in-the-background/10818/11
    
    cTrav =CollisionTraverser();
    cTrav.show_collisions(win->get_render());
    notifier = CollisionHandlerEvent();
    notifier.add_in_pattern("%fn-in-%in");
    framework.get_event_handler().add_hook("ff-in-cube",onCollision);
}
void Env::onCollision(const Event *event){
    cout<<"I am colliding!!!!!!"<<endl;
}
void Env::update(){
    this->cTrav.traverse(win->get_render());
}
//************************GhostWorld*******************
GhostWorld::GhostWorld(int num_envs,vector<float> offset_size,int argc, char* argv[]){
    framework.open_framework(argc, argv);
    this->win=  framework.open_window();
    this->num_envs=num_envs;
    this->offset_size=offset_size;
    
    
    vector<vector<float>> offsets = this->calculate_offset_for_env();
    
    for(int i=0;i<num_envs;i++){
        int env_id = i+1;
        Env env = Env(win,offsets[i],env_id);
        envs.push_back(env);
        env_ids.push_back(env_id);
    }
}
vector<vector<float>>  GhostWorld::calculate_offset_for_env(){
    //  we stack all envs along x and z axis
    
    vector<vector<float>> offsets;
    vector<float>offset = {0,0,0};
    int x_counter = 0;
    int z_counter = 0;
    float z_offset  = 0;
    float x_offset  = 0;
    float y_offset  = 0;

    int num_env_on_each_axis = ceil(this->num_envs/2);

    for (int env_num=0; env_num <this->num_envs ;env_num++){
        if (num_env_on_each_axis!=0 && env_num%num_env_on_each_axis==0 && env_num !=0){
            z_counter +=1;
            x_counter = 0;
            z_offset = this->offset_size[2];
            z_offset  *=z_counter;
        }

        x_offset = this->offset_size[0];
        x_offset *= x_counter;
        x_counter +=1;
      
        offset[0] = x_offset;
        offset[1] = y_offset;
        offset[2] = z_offset;

        offsets.push_back(offset);
    }


    return offsets;
}
 
void GhostWorld::setup_camera(){
    NodePath camera = win->get_camera_group();
    camera.set_pos(0, -3, 0);
    camera.look_at(0,0,0);
}
void GhostWorld::run(){
      Thread *current_thread = Thread::get_current_thread();
      while(framework.do_frame(current_thread)) {
          CIntervalManager::get_global_ptr()->step();
          for(int i=0;i<this->num_envs;i++){
            envs[i].update();
             
          }

      }
}
void GhostWorld::closeFramework(){
     framework.close_framework();
}