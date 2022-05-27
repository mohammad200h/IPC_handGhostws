from panda3d.core import *
from direct.showbase.ShowBase import ShowBase

from EnvManager import *



loadPrcFileData("", "load-file-type p3assimp")


cameraMap={
    "up":False,
    "down":False,
    "left":False,
    "right":False,
    "zoom_in":False,
    "zoom_out":False
}
# callback function to update cameraMap
def updateCameraMap(key,state):
    cameraMap[key]=state
  
class Game(ShowBase):

    def __init__(self,num_envs):
        super().__init__()
        self.speed = 10
   
        self.envs = None
        self.constructor(num_envs)
        # keyboard event setup
        self.keyboard_event_init()
        # camera setup
        self.setup_camera()
        # add task
        taskMgr.add(self.update, "Update")

    def constructor(self,num_envs):
        self.envs = EnvManger(game=self,num_envs=num_envs)

    def setup_camera(self):
        self.cam.setPos(0,-5,1)
        self.cam.lookAt(0,0,0)
    
    def keyboard_event_init(self):
        # camera zoom 
        self.accept("arrow_up",updateCameraMap,["up" ,True])
        self.accept("arrow_down",updateCameraMap,["down" ,True])
        self.accept("arrow_left",updateCameraMap,["left" ,True])
        self.accept("arrow_right",updateCameraMap,["right" ,True])
        self.accept("g",updateCameraMap,["zoom_out" ,True])
        self.accept("h",updateCameraMap,["zoom_in" ,True])

    def update(self,task):
        # self.update_ws()
        self.update_state()
        self.update_camera_zoom()

        return task.cont
    def update_state(self):
        pass

    def update_camera_zoom(self):
        dt = globalClock.getDt()
        pos = self.cam.getPos()
        if cameraMap["up"]:
            pos.z += self.speed *dt 
        if cameraMap["down"]:
            pos.z -= self.speed *dt 
        if cameraMap["right"]:
            pos.x += self.speed *dt 
        if cameraMap["left"]:
            pos.x -= self.speed *dt 
        if cameraMap["zoom_out"]:
            pos.y -= self.speed *dt 
        if cameraMap["zoom_in"]:
            pos.y += self.speed *dt 
        
        self.cam.setPos(pos)
        for key in cameraMap.keys():
            cameraMap[key]=False
     

game = Game(5)
game.run()
