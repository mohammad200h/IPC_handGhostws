from panda3d.core import *

from direct.showbase.ShowBase import ShowBase
import random

loadPrcFileData("", "load-file-type p3assimp")
class Game(ShowBase):

    def __init__(self):
        super().__init__()
        # isolation parameters
        self.limit ={
            "up":1,
            "down":-1
        }
        self.move_down_flag=True

        # camera setup
        self.cam.setPos(0,-1,1)
        self.cam.lookAt(0,0,0)
        # collison setup
        self.setupCollision()
        # finger workspace
        self.add_ws()

        # cube
        self.add_target()
        # add task
        taskMgr.add(self.update_ws, "UpdateWS")



    def add_ws(self):
        self.ws = self.loader.loadModel("FF.egg")
        self.ws.setPos(0,0,0)
        self.ws.reparentTo(self.render)
        self.ws.setPythonTag("velocity", 0)

        # collision 
        col = self.ws.attachNewNode(CollisionNode("ws"))
        col.node().addSolid(CollisionSphere(0, 0, 0, 1.1))
        col.show()
        base.cTrav.addCollider(col, self.notifier)
        
        

    def add_target(self):
        self.cube = self.loader.loadModel("FF.egg")
        self.cube.setPos(0,0,0)
        self.cube.reparentTo(self.render)

        # collision 
        col = self.cube.attachNewNode(CollisionNode("target"))
        col.node().addSolid(CollisionSphere(0, 0, 0, 1.1))
        col.show()
        base.cTrav.addCollider(col, self.notifier)

    def onCollision(self, entry):
        print("onCollision::entry:: ",entry)
   


    def setupCollision(self):
        base.cTrav = CollisionTraverser()
        base.cTrav.showCollisions(render)
        self.notifier = CollisionHandlerEvent()
        self.notifier.addInPattern("%fn-in-%in")
        self.accept("ws-in-target", self.onCollision)
    

    def update_ws(self,task):
        vel = self.ws.getPythonTag("velocity")
        if self.move_down_flag:
            self.moveDown(vel)
            z=self.ws.getZ()

            if z <self.limit["down"]:
               self.move_down_flag = False 
            
       
        
        if not self.move_down_flag:
            self.moveUp(vel)
            z=self.ws.getZ()
            if z >self.limit["down"]:
               self.move_down_flag = True 
           
   

      
        return task.cont

    def moveDown(self,vel):
       
        z=self.ws.getZ()
        self.ws.setZ(z+vel)
        new_vel = vel-0.01
        self.ws.setPythonTag("velocity", new_vel)
        
    def moveUp(self,vel):
               
        z=self.ws.getZ()
        self.ws.setZ(z+vel)
        new_vel = vel+0.01
        self.ws.setPythonTag("velocity", new_vel)

    


game = Game()
game.run()
