class NodeTB():
    def __init__(self, x, y, tht, u1, u2, dt):
        self.x = x
        self.y = y
        self.tht = tht
        self.u1 = u1
        self.u2 = u2
        self.dt = dt
    
    def setnode(self, x, y, tht, u1, u2, dt):
        self.x = x
        self.y = y
        self.tht = tht
        self.u1 = u1
        self.u2 = u2
        self.dt = dt

    def printNode(self):
        print('x = '+ str(self.x))
        print('y = '+ str(self.y))
        print('tht = '+ str(self.tht))
        print('u1 = '+ str(self.u1))
        print('u2 = '+ str(self.u2))
        print('dt = '+ str(self.dt))


