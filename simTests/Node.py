class Node():
    def __init__(self, x, y, z, sc, tht):
        self.x = x
        self.y = y
        self.z = z
        self.sc = sc
        self.tht = tht
    
    def setNode(self, x, y, z, sc, tht):
        self.x = x
        self.y = y
        self.z = z
        self.sc = sc
        self.tht = tht

    def printNode(self):
        print('x = ', self.x)
        print('y = ', self.y)
        print('z = ', self.z)
        print('sc = ', self.sc)
        print('tht = ', self.tht)


