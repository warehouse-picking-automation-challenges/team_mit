## script to call topple then scoop then drop into bin sequentially

import topple
import scoop
import goToBin
import goToHome

temp_bias = 0.04  #TO-DO: Remove this bias, calculate object height correctly
topple.topple(objPose = [1.55620419979, 0.281148612499, 1.14214038849-temp_bias,0,0,0,0],
        objId = 0, 
        binNum=0, 
        robotConfig=None,
        shelfPosition = [1.9019,0.00030975,-0.503],
        forceThreshold = 1,
        effect = 'topple',
        binDepthReach = 0.37,
        isExecute = True,
        withPause = False)
scoop.scoop(
    objPose = [1.55620419979, 0.281148612499, 1.14214038849,0,0,0,0],
    binNum=0, 
    robotConfig=None, 
    shelfPosition = [1.9019,0.00030975,-0.503], 
    isExecute = True,
    withPause = False)

goToBin.goToBin(robotConfig=None,
        objectiveBinPos = [0.9,0,0], 
        isExecute = True,
        withPause = False)

goToHome.goToHome(robotConfig = None,
            homePos = [1,0,1.2], 
            isExecute = True,
            withPause = False)
