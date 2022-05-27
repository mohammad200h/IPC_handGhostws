from panda3d.core import *
from direct.showbase.ShowBase import ShowBase
import math

class EnvManger():
    def __init__(self,game,num_envs=1):

        self.offset = [1,1,1]
        self.game = game
       
      
        self.num_envs =num_envs
        self.envs = []
        self.env_ids = []
        self.client_id_binding = []

        self.constructor(num_envs)

    def constructor(self,num_envs):
        offsets = self.calculate_offset_for_env()
        print("offsets:: ",offsets)
        for i in range(num_envs):
            env_id = i+1
            env = Env(self.game,env_id,offsets[i])
            self.envs.append(env)
            self.env_ids.append(env_id)

    def get_env(self,env_id):
        pass 
    def assign_client_to_env(self,client_id):
        pass
    def is_this_id_assinged_to_env(self,client_id):
        pass
    def updateEnvGymState(self,client_id,gymState):
        pass
    def get_env_id_binded_to_client_id(self,client_id):
        pass
    def getEnvGhostState(self,client_id):
        pass
    def restEnvGhostState(self,client_id):
        pass
    # utility
    def calculate_offset_for_env(self):
        # we stack all envs along x and z axis
        offsets = []
        x_counter = 0
        z_counter = 0
        z_offset  = 0
        x_offset  = 0
        y_offset  = 0
        num_env_on_each_axis = math.ceil(self.num_envs/2)
        for env_num in range(self.num_envs):
            if env_num%num_env_on_each_axis==0 and env_num !=0:
                z_counter +=1
                x_counter = 0
                z_offset = self.offset[-1]
                z_offset  *=z_counter
            x_offset = self.offset[0]
            x_offset *= x_counter
            x_counter +=1
            offsets.append([x_offset,y_offset,z_offset])
        
        return offsets


            


    
class Env():
 
    def __init__(self,game,env_id,offset):

        env_id = None
        self.game = game
        self.obj_in_scene = None
        self.constructor(offset)
        

    def constructor(self,offset):
        self.setupCollision()
        cube = Cube(self.game,offset)
        ff = Finger(self.game,offset,"ff")
        mf = Finger(self.game,offset,"mf")
        rf = Finger(self.game,offset,"rf")
        th =  Thumb(self.game,offset)

        self.obj_in_scene = [cube,ff,mf,rf,th]
    
    def onCollision(self, entry):
        self.counter+=1
        print("onCollision::counter ",self.counter)
        print("onCollision::entry:: ",entry)
   
    def setupCollision(self):
        base.cTrav = CollisionTraverser()
        base.cTrav.showCollisions(render)
        self.game.notifier = CollisionHandlerEvent()
        self.game.notifier.addInPattern("%fn-in-%in")
        self.game.accept("ff-in-cube", self.onCollision)

    def updateEnvGymState(self):
        pass
    def getEnvGhostState(self):
        pass

class Finger():
    def __init__(self,game,offset,finger_name ="ff"):
    
        self.game = game
        self.constructor(offset,finger_name)

    def constructor(self,offset,finger_name):
        # finger workspace
        finger_bias=self.get_finger_bias(finger_name)
        self.add_finger(offset,finger_bias)

    def add_finger(self,offset,finger_bias):
        self.ws = self.game.loader.loadModel("FF.egg")
        self.ws.setPos(0+offset[0]+finger_bias[0],
                       0+offset[1]+finger_bias[1],
                       0+offset[2]+finger_bias[2])

        self.ws.reparentTo(self.game.render)
        self.ws.setPythonTag("velocity", 0)

        # collision 
        self.ws_collision_boxes()

    def ws_collision_boxes(self):
        
        collision_boxs ={
            "c1":((-0.03336,-0.003321,0.219165),0.035255,0.013681,0.002792),
            "c2":((-0.03336,-0.010602,0.224399),0.035255,0.013681,0.002792),
            "c3":((-0.03336,-0.010991,0.244547),0.035255,0.013681,0.033984),
            "c4":((-0.03336,-0.010991,0.268406),0.035255,0.013681,0.011236),
            "c5":((-0.03336,0.004424,0.253941),0.021793,0.013681,0.062659),
            "c6":((-0.032871 ,0.02094,0.274267),0.00616,0.013681,0.043832),
            "c7":((-0.032871,0.037748,0.281319),0.004354,0.013681,0.043832),
            "c8":((-0.032871,0.053085,0.289576),0.01066,0.013681,0.024994),
            "c9":((-0.032871,0.068155,0.290964),0.025145,0.013681,0.016356),
            "c10":((-0.032871,0.082526,0.290964),0.025145,0.013681,0.003581),
        }
        for key in collision_boxs.keys():
        
            col = self.ws.attachNewNode(CollisionNode("ws"))
            center,x,y,z =collision_boxs[key]
            col.node().addSolid(CollisionBox(center, x/2, y/2, z/2))
            col.show()
            base.cTrav.addCollider(col, self.game.notifier)



    def update_pos(self,gymState):
        pos = gymState.pos
        orn = gymState.orn
        self.ws.setPos(pos.x,pos.y,pos.z)
        # this is in degree
        self.ws.setHpr(orn.r,orn.p,orn.y)

    def get_finger_bias(self,finger_bias):
        biases = {
            "ff":[0,0,0],
            "mf":[0.022,0.002536,0.003068],
            "rf":[0.044,0,0]

        }
        return biases[finger_bias]

class Thumb():
    def __init__(self,game,offset):
     
        self.game = game
        self.constructor(offset)

    def constructor(self,offset):
        # finger workspace
        self.add_finger(offset)

    def add_finger(self,offset):
        self.ws = self.game.loader.loadModel("TH.egg")
        self.ws.setPos(0+offset[0],
                       0+offset[1],
                       0+offset[2])

        self.ws.reparentTo(self.game.render)
        self.ws.setPythonTag("velocity", 0)

        # collision 
        self.ws_collision_boxes()

    def ws_collision_boxes(self):
        
        collision_boxs ={
            "c1":((-0.03857,0.035046,0.24799),0.050777,0.014184,0.009054),
            "c2":((-0.03857,0.031686,0.254842),0.029433,0.014184,0.00337),
            "c3":((-0.03857,0.053628,0.234354),0.050777,0.006749,0.009054),
            "c4":((-0.03857,0.053628,0.234354),0.050777,0.006749,0.009054), 
            # it seems like the previous two are duplicate
            "c5":((-0.025502,0.043943,0.238845),0.050777,0.006749,0.005137),
            "c6":(( -0.005499,0.022437,0.250206),0.010429,0.028513,0.004289), 
            "c7":(( -0.05316 ,0.059074,0.226753),0.028725,0.001,0.009054), 
            "c8":(( -0.027932,0.059074,0.226753),0.01719,0.002725,0.009054),
            "c9":(( -0.009648,0.047519,0.233693),0.01719,0.002725,0.003962),
            "c10":(( -0.08516,0,0.242063),0.005974,0.045269,0.005066),

            "c11":(( -0.0773 ,0 ,0.247971 ),0.005974 ,0.045269 ,0.009168 ), 
            "c12":(( -0.069499,-0.001781 ,0.249856 ),0.005974 ,0.043934 ,0.013232 ), 
            "c13":(( -0.061971,-0.029332 ,0.24799 ),0.021347 ,0.014184,0.006203), 
            "c14":((  -0.042739,-0.030442 ,0.253914 ),0.021347,0.014184,0.003761  ), 
            "c15":((  -0.048978,-0.042207 ,0.24799 ),0.021347,0.01,0.006203), 
            "c16":(( -0.070232,-0.041907,0.236535),0.016397,0.012933,0.001963), 
            "c17":(( -0.066965,-0.041907,0.243616),0.006381,0.01,0.007064), 
            "c18":(( -0.065435,-0.052215,0.235366),0.021347,0.006,0.006203), 
            "c19":(( -0.048978,-0.042207,0.24799),0.021348,0.01,0.006203), 
            "c20":(( -0.048978,-0.046478,0.243002),0.021347,0.01,0.002305),

            "c21":(( -0.051402 ,-0.050888 ,0.239707 ),0.021347 ,0.006 ,0.002305 ), 
            "c22":((  -0.065435 ,-0.052215  ,0.235366  ), 0.021347 ,0.006  , 0.006203 ), 
            "c23":(( -0.048821  ,-0.055404  , 0.235366 ), 0.010576 ,0.002  , 0.006203 ), 
            "c24":((  -0.056343 ,0.063603  ,0.216837  ), 0.022291 ,0.002725  ,0.009054  ), 
            "c25":((  -0.062885 ,0.043943  ,0.238845  ),0.022225  ,0.006749  ,0.005137  ), 
            "c26":(( -0.070289  , -0.057788 ,0.224329  ),0.021347  , 0.006 ,0.006203  ), 
            "c27":((  -0.07023 ,-0.062591  ,0.216792  ),0.019066  ,0.003802  , 0.006203 ), 
            "c28":(( -0.092849  ,0  ,0.236717  ), 0.005974 ,0.045269  ,0.005066  ), 
            "c29":((  -0.101208 ,-0.003902  ,0.230869  ), 0.005974 ,0.045269 ,0.005066  ), 
            "c30":(( -0.108491  ,-0.003902  ,0.222479  ),0.003405  ,0.041394 ,0.005066  ), 

            "c31":((  -0.110514 ,-0.003902  ,0.217364  ),0.005974 ,0.045269  ,0.005066  ), 
            "c32":(( -0.113856  , -0.003902 ,0.211484  ),0.002  , 0.045268 , 0.005066 ), 
            "c33":(( -0.077508  ,-0.031635  ,0.243616  ), 0.006381 ,0.01746  ,0.007064  ), 
            "c34":(( -0.08354  ,-0.036632  ,0.234664  ),0.006381  ,0.007215  ,0.007064  ), 
            "c35":((  -0.08354 ,-0.028306  ,0.236847  ), 0.006381 ,0.007215  ,0.001852  ), 
            "c36":((  -0.090588 ,-0.034218  ,0.233431  ),0.006381  ,0.012  ,0.007064  ), 
            "c37":((  -0.08354 ,-0.043097  ,0.234664  ), 0.006381 ,0.00439  ,0.007064  ), 
            "c38":(( -0.10014  ,-0.034218  ,0.223415  ),0.003943  ,0.012  ,0.009406  ), 
            "c39":((  -0.095726 ,-0.034218  ,0.22792  ),0.002663  ,0.012  ,0.007064  ), 
            "c40":(( -0.10501  ,-0.034218  , 0.215893 ),0.003943  ,0.012  ,0.009406  ), 

            "c41":((-0.109916   ,-0.034218  ,0.20814  ),0.003943  ,0.012 ,0.009406 ), 
            "c42":((  -0.07472 ,-0.065101  ,0.209584  ), 0.019066 ,0.001855  ,0.006203  ), 
            "c43":(( -0.080926  ,-0.065101  ,0.202227  ),0.012172  ,0.001855  ,0.006203 ), 
            "c44":((  -0.109084 ,-0.041974  ,0.197096  ),0.001993  ,0.012 ,0.009406  ), 
            "c45":((  -0.11293 ,-0.034559 ,0.197096  ),0.001993  ,0.012  ,0.009406  ), 
            "c46":(( -0.109916  ,-0.034218  ,0.20814  ),0.003943  , 0.012 ,0.009406  ), 
            "c47":(( -0.091944  ,-0.060199  ,0.201091  ), 0.012172 , 0.001855 ,0.006203  ), 
            "c48":(( -0.087079  ,-0.058475  ,0.209983  ),0.012172  ,0.004155  ,0.006203  ), 
            "c49":(( -0.087079  , -0.0544 ,0.218198  ),0.012172  ,0.00598  ,0.006203  ), 
            "c50":((  -0.087079 ,-0.047396  ,0.228214  ),0.012172  ,0.003881  ,0.006203  ), 
            
            "c51":(( -0.087079  ,-0.051723  ,0.225312  ),0.012172  ,0.001252  ,0.006203  ), 
            "c52":((  -0.084301 , -0.055176 ,0.225312  ), 0.006367 ,0.003817  , 0.006203 ), 
            "c53":((  -0.09587 ,-0.056952  , 0.201091 ), 0.012172 ,0.001855  ,0.006203  ), 
            "c54":(( -0.100727  ,-0.053688  ,0.197349 ),0.007804 , 0.001855 ,0.006203  ), 
            "c55":((  -0.096065 ,-0.052488  ,0.209983  ),0.005686  , 0.004155 , 0.006203 ), 
            "c56":(( -0.099433  ,-0.052488  ,0.204605 ),0.005686  ,0.004155  ,0.006203  ), 
            "c57":((  -0.093078 ,-0.047396  ,0.220792  ),0.007922  ,0.004273  ,0.006203 ), 
            "c58":((  -0.095976 ,-0.048959  ,0.214071  ),0.007922  ,0.004147  ,0.006203 ), 
            "c59":((  -0.102235 ,-0.047131  ,0.206631  ),0.005021  ,0.004147 , 0.006203 ), 
            "c60":((  -0.101896 ,-0.042453  ,0.214071  ),0.007922  ,0.004147  ,0.006203  ), 

            "c61":(( -0.105584  ,-0.042453  ,0.207589  ),0.007922  ,0.004147  ,0.006203  ), 
            "c62":(( -0.096551  ,-0.042674  ,0.221787  ),0.007922 ,0.004147  ,0.006203 ), 
            "c63":(( -0.092677  ,-0.042674  ,0.228904  ), 0.007922 ,0.004147  ,0.006203  ), 
            "c64":((  -0.063064 ,-0.057058  ,0.22981  ),0.031824  ,0.006  ,0.003974  ), 
            "c65":((  -0.105709 ,-0.003902  ,0.22774  ),0.002534  ,0.041394  ,0.005066  ), 
            "c66":((  -0.098892 ,-0.003902  , 0.235683 ),0.003059  ,0.042865  ,0.003178  ), 
            "c67":((  -0.068779 ,0.032311  ,0.245256  ),0.005423  , 0.014184 , 0.006777 ), 
            "c68":(( -0.078776  ,0.032311  ,0.241  ),0.005423  ,0.014184  ,0.005273  ), 
            "c69":(( -0.086747  , 0.030035 ,0.236354  ),0.005423  ,0.008613  ,0.006777  ), 
            "c70":(( -0.097575  , 0.02278 ,0.232427  ),0.003208  ,0.003842  ,0.006777  ), 

            "c71":(( -0.102797  ,0.022025  ,0.225196  ),0.005375  ,0.005256 ,0.006777  ), 
            "c72":(( -0.107098  ,0.022025  ,0.217462  ),0.005375 ,0.005256  ,0.006777  ), 
            "c73":((  -0.111058 ,0.022025  ,0.211951  ),0.003154  ,0.005256  ,0.006777  ), 
            "c74":((  -0.111058 ,0.02845  ,0.211951  ),0.003154  , 0.005256 ,0.006777  ), 
            "c75":(( -0.107443  ,0.02845  ,0.217373  ),0.003154 ,0.005256  , 0.003771 ), 
            "c76":((  -0.092537 ,0.02845  ,0.234531  ),0.003154  , 0.005256 ,0.003771  ), 
            "c77":(( -0.079091  ,0.042388  ,0.234509  ),0.008673 ,0.005082  ,0.004438 ), 
            "c78":((  -0.069628 ,0.049743  ,0.233575  ),0.008673 ,0.008613  ,0.004557  ), 
            "c79":(( -0.079198  ,0.047867  ,0.230892  ),0.008673  ,0.003347  ,0.004557  ),
            "c80":(( -0.074457  ,0.052055  ,0.226789  ),0.014938  ,0.003347  ,0.004557  ), 

            "c81":(( -0.073831  ,0.055547  ,0.221965  ), 0.014938 ,0.003347  , 0.004557 ), 
            "c82":(( -0.073879  ,0.058229  ,0.217676  ), 0.014938 ,0.003347  ,0.002147  ), 
            "c83":((  -0.073879 , 0.059608 , 0.215207 ),0.014938  , 0.003347 ,0.002147  ), 
            "c84":(( -0.072994  ,0.061826  ,0.212167  ),0.010177  ,0.003347  ,0.002147  ), 
            "c85":(( -0.035415  ,-0.042235  ,0.247589  ), 0.004363 ,0.008704  ,0.004504  ), 
            "c86":((  -0.028904 ,-0.038525  ,0.247794  ),0.004363  ,0.008704  ,0.00626  ), 
            "c87":((  -0.025929 ,-0.028981  ,0.251355  ),0.01005  ,0.008704  ,0.005986  ), 
            "c88":((  -0.005499 ,0.001561  ,0.250208  ),0.010429  ,0.011338  ,0.004289  ), 
            "c89":((  -0.005499 ,0.001561  ,0.250994  ),0.010429  ,0.011338  ,0.004289  ),
            "c90":((-0.009162 ,-0.01219 ,0.250344  ),0.005665  ,0.011338  ,0.004289  ), 

            "c91":(( -0.010751  ,0.008519  ,0.255339  ),0.012187  ,0.035386  ,0.004289  ), 
            "c92":(( -0.014448  ,0.055186  ,0.226753  ),0.007289  , 0.002725 ,0.009054  ), 
            "c93":(( -0.005499  ,0.031849  ,0.245093  ),0.010429  ,0.011078  ,0.004289  ), 
            "c94":((  0.002345 ,0.036143  ,0.238839  ),0.010429  ,0.011078  ,0.004289  ), 
            "c95":(( 0.00602  ,0.022749  ,0.238839  ),0.00425  ,0.011078  ,0.004289  ), 
            "c96":((  0.002688 ,0.020848  ,0.244782  ),0.00425 ,0.031496  ,0.004289  ), 
            "c97":(( -0.06255  ,-0.001781  ,0.253123  ), 0.005974 ,0.043934  ,0.008629  ), 
            "c98":(( -0.03857  ,-0.00293  , 0.263793 ),0.030688  , 0.034158 ,0.002446  ), 





        }
        for key in collision_boxs.keys():
        
            col = self.ws.attachNewNode(CollisionNode("ws"))
            center,x,y,z =collision_boxs[key]
            col.node().addSolid(CollisionBox(center, x/2, y/2, z/2))
            col.show()
            base.cTrav.addCollider(col, self.game.notifier)



    def update_pos(self,gymState):
        pos = gymState.pos
        orn = gymState.orn
        self.ws.setPos(pos.x,pos.y,pos.z)
        # this is in degree
        self.ws.setHpr(orn.r,orn.p,orn.y)

class Cube():
    def __init__(self,game,offset):
  
        self.game = game
 
        self.constructor(offset)
     
    def constructor(self,offset):
        self.add_cube(offset)

    def add_cube(self,offset):

        # print("base:: ",base)
        self.cube = self.game.loader.loadModel("Cube.egg")
        self.cube.setScale(0.01)
        self.cube.setPos(0+offset[0],0+offset[1],0+offset[2])
        self.cube.reparentTo(self.game.render)

        # collision 
        col = self.cube.attachNewNode(CollisionNode("cube"))
        center,dx,dy,dz =((0,0,0),2,2,2)
        col.node().addSolid(CollisionBox(center, dx, dy, dz))
        col.show()
        base.cTrav.addCollider(col, self.game.notifier)
