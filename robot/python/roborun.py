import g2g

#PARAMS USED FOR OBJECT SEEKING
minDistI = 58
minDistU = 50

g2g.initOdometry()

while True:
        for dist in g2g.allUltra():
                if dist < minDistU:
                        stop()
        for dist in g2g.allInfra():
                if dist < minDistI:
                        stop()
