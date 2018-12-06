import g2g

#PARAMS USED FOR OBJECT SEEKING
minDistI = 58
minDistU = 50

g2g.initOdometry()

def foundObject():
    int degree = 25
    if True:
        return degree
    else:
        return None

def withinMinDistance():
    for dist in g2g.allUltra():
            if dist < minDistU:
                    return false
    for dist in g2g.allInfra():
            if dist < minDistI:
                    false
    return true

def getArcDist():
    ultraArr = spinScanUltra()
    irArr = spinScanInfrared()

def spin(degree):
    move(0,0,degree)

def spinScanUltra():
    int incrementSpin = 6
    int sensorToUse = 0
    scanArr = []
    for i in range(increementSpin):
        spin(int(360/incrementSpin)
        scanArr.append(g2g.ultrasound(sensorToUse))
        deg = foundObject()
        if deg is not -1:
            eureka(deg)
    return scanArr

def other():
    scanArr = []
    explore()
    scanArr.extend(g2g.allInfra())
    scanArr.extend(g2g.allUltra())
    spin(180)
    explore()
    scanArr.extend(g2g.allInfra())
    scanArr.extend(g2g.allUltra())
    sping(180)

def explore():
    for cam in range(4)
        if imageMatch(cam)
            break

def imageMatch(cam):
    takePic = imageCapture()
    if imageCompare(takePic, destination)

def spinScanInfrared():
    int incrementSpin = 6
    int sensorToUse = 0
    scanArr = []
    for i in range(increementSpin):
        spin(180)
        scanArr.append(g2g.infrared(sensorToUse))
        deg = foundObject()
        if deg is not -1:
            eureka(deg)
    return scanArr

def eureka():
    print("destination object found")
    move(destX, destY)
    print("found it!!!")
    exit()

while True:
    if foundObject():
        move(maxDistX, maxDistY, angle)
        print("success")
        exit()
    if getArcDist()
