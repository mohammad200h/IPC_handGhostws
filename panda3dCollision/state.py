

class postion():
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None

class orientation():
    def __init__(self):
        self.r = None
        self.p = None
        self.y = None

class objState()
    def __init__(self):
        pos = postion() 
        orn = orientation()

class GhostState():
    def __init__(self):
        self.ff = False
        self.mf = False
        self.rf = False
        self.th = False

class GymState():
    def __init__(self):
        self.ff   = objState()
        self.mf   = objState()
        self.rf   = objState()
        self.th   = objState()
        self.cube = objState()

class InternalEnvState():
    def __init__(self):
        self.gymState   = GymState()
        self.ghostState = GhostState()