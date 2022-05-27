from panda3d.core import *

from direct.showbase.ShowBase import ShowBase
import random

loadPrcFileData("", "load-file-type p3assimp")

keyMap={
    # translation
    "up":False,
    "down":False,
    "left":False,
    "right":False,
    # rotation
    "z_left":False,
    "z_right":False,
    "y_left":False,
    "y_right":False

}
cameraMap={
    "up":False,
    "down":False,
    "left":False,
    "right":False,
    "zoom_in":False,
    "zoom_out":False
}
# callback function to update keyMap
def updateKeyMap(key,state):
    keyMap[key]=state
    # print("keyMap:: ",keyMap)

def updateCameraMap(key,state):
    cameraMap[key]=state

class Game(ShowBase):

    def __init__(self):
        super().__init__()
        self.counter =0
        self.speed = 10
        # isolation parameters
        self.limit ={
            "up":1,
            "down":-1
        }
        self.move_down_flag=True

        # keyboard event setup
        self.keyboard_event_init()
        # camera setup
        self.setup_camera()
        # collison setup
        self.setupCollision()
        # finger workspace
        self.add_ws()

        # cube
        self.add_target()
   

        # add task
        taskMgr.add(self.update, "Update")

    def setup_camera(self):
        self.cam.setPos(0,-2,1)
        self.cam.lookAt(0,0,0)
    
    def keyboard_event_init(self):
        # translation
        self.accept("w",updateKeyMap,["up"   ,True])
        self.accept("s",updateKeyMap,["down" ,True])
        self.accept("a",updateKeyMap,["left" ,True])
        self.accept("d",updateKeyMap,["right",True])
        # rotation
        self.accept("q",updateKeyMap,["z_left"  ,True])
        self.accept("e",updateKeyMap,["z_right" ,True])
        self.accept("z",updateKeyMap,["y_left"  ,True])
        self.accept("x",updateKeyMap,["y_right" ,True])
        # camera zoom 
        self.accept("arrow_up",updateCameraMap,["up" ,True])
        self.accept("arrow_down",updateCameraMap,["down" ,True])
        self.accept("arrow_left",updateCameraMap,["left" ,True])
        self.accept("arrow_right",updateCameraMap,["right" ,True])
        self.accept("g",updateCameraMap,["zoom_out" ,True])
        self.accept("h",updateCameraMap,["zoom_in" ,True])


    def add_ws(self):
        self.ws = self.loader.loadModel("FF.egg")
        self.ws.setPos(0.03,0,0)
        self.ws.reparentTo(self.render)
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
            base.cTrav.addCollider(col, self.notifier)


    def add_target(self):
        self.cube = self.loader.loadModel("Cube.egg")
        self.cube.setScale(0.01)
        self.cube.setPos(0,0,0)
        self.cube.reparentTo(self.render)

        # collision 
        col = self.cube.attachNewNode(CollisionNode("target"))
        center,dx,dy,dz =((0,0,0),2,2,2)
        col.node().addSolid(CollisionBox(center, dx, dy, dz))
        col.show()
        base.cTrav.addCollider(col, self.notifier)

    def onCollision(self, entry):
        self.counter+=1
        print("onCollision::counter ",self.counter)
        print("onCollision::entry:: ",entry)
   
    def setupCollision(self):
        base.cTrav = CollisionTraverser()
        base.cTrav.showCollisions(render)
        self.notifier = CollisionHandlerEvent()
        self.notifier.addInPattern("%fn-in-%in")
        self.accept("ws-in-target", self.onCollision)
    
    def update(self,task):
        # self.update_ws()
        print("base::",base)
        self.update_pos()
        self.update_camera_zoom()

        return task.cont

    def update_ws(self):
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
    
    def update_pos(self):
        dt = globalClock.getDt()
        # current position
        pos = self.ws.getPos()
        # current orientation
        orn = self.ws.getHpr()

        # update state
        if keyMap["up"]:
            pos.z += self.speed/10 *dt  
        if keyMap["down"]:
            pos.z -= self.speed/10 *dt

        if keyMap["right"]:
            pos.x += self.speed *dt       
        if keyMap["left"]:
            pos.x -= self.speed *dt
        
        if keyMap["z_right"]:
            orn.x += 20*self.speed *dt       
        if keyMap["z_left"]:
            orn.x -= 20*self.speed *dt

        if keyMap["y_right"]:
            orn.z += 20*self.speed *dt       
        if keyMap["y_left"]:
            orn.z -= 20*self.speed *dt


        self.ws.setPos(pos)
        self.ws.setHpr(orn)

        # restting keyMap
        for key in keyMap.keys():
            keyMap[key]=False

        # print("orn::",orn)
    
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
     

    def moveDown(self,vel):
       
        z=self.ws.getZ()
        self.ws.setZ(z+vel)
        new_vel = vel-0.001
        self.ws.setPythonTag("velocity", new_vel)
        
    def moveUp(self,vel):
               
        z=self.ws.getZ()
        self.ws.setZ(z+vel)
        new_vel = vel+0.001
        self.ws.setPythonTag("velocity", new_vel)

    def rebuildGeomNodesToColPolys(self,incomingNode):
        '''
        Converts GeomNodes into CollisionPolys in a straight 1-to-1 conversion
        Returns a new NodePath containing the CollisionNodes
        '''
        print("node:: ",incomingNode)
        parent = NodePath('cGeomConversionParent')
        for c in incomingNode.findAllMatches('**/+GeomNode'):
            gni = 0
            geomNode = c.node()
            for g in range(geomNode.getNumGeoms()):
                geom = geomNode.getGeom(g).decompose()
                vdata = geom.getVertexData()
                vreader = GeomVertexReader(vdata, 'vertex')
                cChild = CollisionNode('cGeom-%s-gni%i' % (c.getName(), gni))
                gni += 1
                for p in range(geom.getNumPrimitives()):
                    prim = geom.getPrimitive(p)
                    for p2 in range(prim.getNumPrimitives()):
                        s = prim.getPrimitiveStart(p2)
                        e = prim.getPrimitiveEnd(p2)
                        v = []
                        for vi in range (s, e):
                            vreader.setRow(prim.getVertex(vi))
                            v.append (vreader.getData3f())
                        colPoly = CollisionPolygon(*v)
                        cChild.addSolid(colPoly)
    
                parent.attachNewNode (cChild)
    
        return parent


game = Game()
game.run()
