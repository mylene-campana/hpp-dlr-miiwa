#/usr/bin/env python
# Script which goes with ur_description package.
# Load 6-DoF arm robot to test methods.

from hpp.corbaserver.dlr_miiwa import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer

robot = Robot ('dlr_miiwa')
cl = robot.client
ps = ProblemSolver (robot)
r = Viewer (ps)
pp = PathPlayer (robot.client, r)

r.loadObstacleModel ("dlr_miiwa","table","table")
r.loadObstacleModel ("dlr_miiwa","large_wall","large_wall")
r.loadObstacleModel ("dlr_miiwa","floor","floor")

robot.setJointBounds('miiwa_joint_x', [-7, 5])
robot.setJointBounds('miiwa_joint_y', [-1.9, 1.9])

# q = ['miiwa_joint_x', 'miiwa_joint_y', 'miiwa_joint_theta', 'lbr_iiwa_joint_1', 'lbr_iiwa_joint_2', 'lbr_iiwa_joint_3', 'lbr_iiwa_joint_4', 'lbr_iiwa_joint_5', 'lbr_iiwa_joint_6', 'lbr_iiwa_joint_7', 'schunk_wsg50_joint_left_jaw', 'schunk_wsg50_joint_right_jaw', 'schunk_pw70_joint_pan', 'schunk_pw70_joint_tilt'] 15 DoF#

q1 = [4, 0.4, 0.8761403383, 0.4820561249, 0.51, -1.1, 0, -0.44, -0.78, 0.44, -0.53, 0, 0, 0, 0]
qinterm12 = [-1, 0.4, 0.2721349469, -0.9622590975, -1.22, 1.26, -2.28, -0.84, -1.796, 0.12, -0.53, 0, 0, 0, 0]
q2 = [-6.06, 0.465, 0.2721349469, -0.9622590975, -1.22, 1.26, -2.28, -0.84, -1.796, 0.12, -0.53, 0, 0, 0, 0]
r(qinterm12)
r(q2)

ps.selectPathPlanner ("VisibilityPrmPlanner")
ps.selectPathValidation ("Dichotomy", 0.)

robot.setJointBounds('miiwa_joint_x', [-2, 5])
ps.setInitialConfig (q1); ps.addGoalConfig (qinterm12); ps.solve (); ps.resetGoalConfigs ()

robot.setJointBounds('miiwa_joint_x', [-7, 0])
ps.setInitialConfig (qinterm12); ps.addGoalConfig (q2); ps.solve (); ps.resetGoalConfigs ()

robot.setJointBounds('miiwa_joint_x', [-7, 5])
ps.setInitialConfig (q1); ps.addGoalConfig (q2); ps.solve ()

#ps.saveRoadmap ('/local/mcampana/devel/hpp/data/miiwa-PRM.rdm')
#ps.saveRoadmap ('/local/mcampana/devel/hpp/data/miiwa-RRT.rdm')

ps.pathLength(2)

ps.saveRoadmap ('/local/mcampana/devel/hpp/data/miiwa-PRM.rdm') # built with interm

ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (2)
ps.pathLength(ps.numberPaths()-1)

ps.clearPathOptimizers()
ps.addPathOptimizer('PartialShortcut')
ps.optimizePath (2)
ps.pathLength(ps.numberPaths()-1)

ps.clearPathOptimizers()
ps.addPathOptimizer("GradientBased")
ps.optimizePath (2)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)

pp(ps.numberPaths()-1)

r(q2)

len(ps.getWaypoints (0))

# Add light to scene
lightName = "li"
r.client.gui.addLight (lightName, r.windowId, 0.001, [0.6,0.6,0.6,0.7])
r.client.gui.addToGroup (lightName, r.sceneName)
r.client.gui.applyConfiguration (lightName, [2,0,3,1,0,0,0])
lightName = "li2"
r.client.gui.addLight (lightName, r.windowId, 0.001, [0.6,0.6,0.6,0.7])
r.client.gui.addToGroup (lightName, r.sceneName)
r.client.gui.applyConfiguration (lightName, [-2,0,3,1,0,0,0])
r.client.gui.refresh ()

# Plot or remove frame
from viewer_display_library import plotFrame
plotFrame (r, "framy", [0,0,0], 0.5)
r.client.gui.removeFromGroup ("frame1"+"framy",r.sceneName)
r.client.gui.removeFromGroup ("frame2"+"framy",r.sceneName)
r.client.gui.removeFromGroup ("frame3"+"framy",r.sceneName)

pp.dt = 0.02
r(q1)
r.startCapture ("capture","png")
r(q1)
pp(0)
#pp(ps.numberPaths()-1)
r(q2)
r.stopCapture ()

# Load obstacles in HPP #
cl.obstacle.loadObstacleModel('ur_description','obstacles','') # cylinders
cl.obstacle.loadObstacleModel('ur_description','table','')
cl.obstacle.loadObstacleModel('ur_description','wall','') # wall with hole

## ffmpeg commands
ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4
x=0; for i in *png; do counter=$(printf %03d $x); ln "$i" new"$counter".png; x=$(($x+1)); done
ffmpeg -r 50 -i new%03d.png -r 25 -vcodec libx264 video.mp4


## DEBUG commands
cl.robot.setCurrentConfig(q1)
cl.robot.collisionTest()
cl.robot.distancesToCollision()
from numpy import *
cl.robot.distancesToCollision()[1][argmin(cl.robot.distancesToCollision()[0])]
cl.robot.distancesToCollision()[2][argmin(cl.robot.distancesToCollision()[0])]
r( cl.problem.configAtDistance(0,5) )
cl.problem.optimizePath (0)
cl.problem.clearRoadmap ()
cl.problem.resetGoalConfigs ()
robot.getJointNames ()
robot.getConfigSize ()
cl.obstacle.getObstaclePosition('decor_base')
robot.getJointOuterObjects('shoulder_pan_joint')
robot.getJointNames ()
robot.isConfigValid(q1)

