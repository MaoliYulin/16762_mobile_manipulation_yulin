#!/usr/bin/env python3

import time
import numpy as np
import rclpy
import hello_helpers.hello_misc as hm

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

class MyNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'my_node', 'my_node', wait_for_first_pointcloud=False)

        self.stow_the_robot()
        time.sleep(3)


        self.move_to_pose({'joint_arm': 0.6, 'joint_lift': 1.1}, blocking=True)


        self.move_to_pose({'joint_wrist_yaw': np.radians(45)}, blocking=True)
        self.move_to_pose({'joint_wrist_pitch': -np.radians(45)}, blocking=True)
        self.move_to_pose({'joint_wrist_roll': np.radians(45)}, blocking=True)


        self.move_to_pose({'joint_gripper': 0.04}, blocking=True) 
        self.move_to_pose({'joint_gripper': 0.0}, blocking=True)

        self.move_to_pose({'joint_head_pan': np.radians(45)}, blocking=True)
        self.move_to_pose({'joint_head_tilt': -np.radians(45)}, blocking=True)

        self.stow_the_robot()
        time.sleep(3)

        self.drive_straight(0.5)
        self.rotate_in_place(np.radians(180))
        self.drive_straight(0.5)

        self.rotate_in_place(np.radians(180))


if __name__ == '__main__':
    rclpy.init()
    node = MyNode()
    node.main()
    rclpy.shutdown()
