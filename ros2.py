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
        self._did_run = False

    def main(self):
        hm.HelloNode.main(self, 'my_node', 'my_node', wait_for_first_pointcloud=False)

        # 关键：注册一个一次性的定时器，让动作在 node 启动后执行
        self.create_timer(0.5, self._run_once)

        # 让节点保持运行（否则 timer 不会触发）
        rclpy.spin(self)

    def _run_once(self):
        if self._did_run:
            return
        self._did_run = True

        # ---- 你的直写动作逻辑（原封不动放这） ----
        self.move_to_pose({'joint_lift': 0.5}, blocking=True)
        self.move_to_pose({'joint_arm': 0.35}, blocking=True)

        self.move_to_pose({'joint_wrist_yaw': np.pi/4}, blocking=True)
        self.move_to_pose({'joint_wrist_pitch': -np.pi/4}, blocking=True)
        self.move_to_pose({'joint_wrist_roll': np.pi/4}, blocking=True)

        self.move_to_pose({'joint_gripper_finger_left': 0.04}, blocking=True)
        self.move_to_pose({'joint_gripper_finger_left': 0.0}, blocking=True)

        self.move_to_pose({'joint_head_pan': np.pi/4}, blocking=True)
        self.move_to_pose({'joint_head_tilt': -np.pi/6}, blocking=True)

        self.stow_the_robot()
        time.sleep(2)

        self.drive_straight(0.5)
        self.rotate_in_place(np.pi)
        self.drive_straight(0.5)

        self.get_logger().info("DONE, shutting down.")
        rclpy.shutdown()



if __name__ == '__main__':
    node = MyNode()
    node.main()

