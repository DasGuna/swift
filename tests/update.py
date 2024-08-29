#!/usr/bin/env python
import roboticstoolbox as rtb 
import spatialmath as sm 
import spatialgeometry as sg
import numpy as np 
import os
import time
from swift import Swift 

env = Swift()
print(f"USER SCRIPT: Starting SWIFT")
# env.launch(realtime=True, open_tab=True, headless=True)
env.launch(realtime=True, open_tab=True)
print(f"USER SCRIPT: SWIFT Started")


# ---[WORKING] TEST ADDITION OF ROBOT (PANDA)
robot = rtb.models.Panda()
robot.q = robot.qr
# Set a desired and effector pose an an offset from the current end-effector pose
Tep = robot.fkine(robot.q) * sm.SE3.Tx(0.2) * sm.SE3.Ty(0.2) * sm.SE3.Tz(0.45)
# env.add(robot)
env.new_add(robot)
# print(f"USER SCRIPT: Waiting for 10 sec...")
# time.sleep(10)

# ---[WORKING] TEST ADDITION OF PRIMATIVE (BOX)
box = sg.Cuboid(scale=[0.3, 0.3, 0.3], pose=sm.SE3(0.5, 0, 0) * sm.UnitQuaternion(1, [0,0,0]).SE3())
env.new_add(box)
# print(f"USER SCRIPT: Waiting for 10 sec...") 
# time.sleep(10)

# ---[WORKING] TEST ADDITION OF AXES
axes = sg.Axes(length=0.2, pose=sm.SE3(0,0,1) * sm.UnitQuaternion(1, [0,0,0]).SE3())
env.new_add(axes)
# print(f"USER SCRIPT: Waiting for 10 sec...")
# time.sleep(10)

# ---[WORKING] TEST ADDITION OF SPLAT
path = os.path.dirname(os.path.realpath(__file__)) + "/data/point_cloud.splat"
splat = {
    'stype': 'splat', 
    'filename': path, 
    't': [0,0,0],
    'scale': [0.05, 0.05, 0.05],
    'euler': [1.5708, 0, 0]
    }
env.new_add(splat)
print(f"USER SCRIPT: Waiting for 10 sec...")
time.sleep(10)

# ---[WORKING] TEST SIMULATION OF ROBOT
# Simulate the robot while it has not arrived at the goal
arrived = False
while not arrived:

    # Work out the required end-effector velocity to go towards the goal
    v, arrived = rtb.p_servo(robot.fkine(robot.q), Tep, 1)
    
    # Set the Panda's joint velocities
    robot.qd = np.linalg.pinv(robot.jacobe(robot.q)) @ v
    
    # Step the simulator by 50 milliseconds
    env.step(0.01)

# END WAIT (DEBUGGING)
# time.sleep(600)
