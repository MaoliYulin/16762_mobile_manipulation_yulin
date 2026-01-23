

"""
Move the arm and gripper back to it’s ‘stow’ position. This can be done with a single 
line of code.
Make sure there is sufficient free space around the robot so it doesn’t collide with 
things as it moves.
Extend the telescoping arm all the way out and raise the lift all the way up at the 
same time. 
Once lifted, move all three of the wrist motors, one at a time (not all at 
once). Any rotation amount is fine as long as it is visible. 

Then open the gripper and close it. 

Then rotate both of the two motors connected to the RealSense (head 
camera). 
Then reset everything back to the ‘stow’ position.
Once in stow, drive the robot forward 0.5 meters, rotate 180 degrees, then drive 0.5 
meters forward (back to the starting position).
"""

#!/usr/bin/env python3
import time
import numpy as np
import hello_helpers.hello_misc as hm


node = hm.HelloNode.quick_create('temp')

node.stow_the_robot()
time.sleep(2)

node.move_to_pose({'joint_arm': 0.52, 'joint_lift': 1.1}, blocking=True)

idx = node.joint_state.name.index('joint_wrist_yaw')
cur = node.joint_state.position[idx]
node.move_to_pose({'joint_wrist_yaw': cur + np.deg2rad(45)}, blocking=True)

idx = node.joint_state.name.index('joint_wrist_pitch')
cur = node.joint_state.position[idx]
node.move_to_pose({'joint_wrist_pitch': cur + np.deg2rad(45)}, blocking=True)

idx = node.joint_state.name.index('joint_wrist_roll')
cur = node.joint_state.position[idx]
node.move_to_pose({'joint_wrist_roll': cur + np.deg2rad(30)}, blocking=True)

node.move_to_pose({'joint_gripper_finger_left': 0.04}, blocking=True)
node.move_to_pose({'joint_gripper_finger_left': 0.0}, blocking=True)

idx = node.joint_state.name.index('joint_head_pan')
cur = node.joint_state.position[idx]
node.move_to_pose({'joint_head_pan': cur + np.deg2rad(30)}, blocking=True)

idx = node.joint_state.name.index('joint_head_tilt')
cur = node.joint_state.position[idx]
node.move_to_pose({'joint_head_tilt': cur + np.deg2rad(20)}, blocking=True)

node.stow_the_robot()
time.sleep(2)

node.drive_straight(0.5)
node.rotate_in_place(np.pi)
node.drive_straight(0.5)
node.rotate_in_place(np.pi)

