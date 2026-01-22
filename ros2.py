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

import rclpy, tf2_ros
from rclpy.node import Node
import hello_helpers.hello_misc as hm
import numpy as np
import time


class MoveNode(Node):
    def __init__(self):
        super().__init__('stow_and_move_node')
        self.robot = hm.setup_robot(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def stow_and_move(self):
        self.robot.stow()
        time.sleep(3)

        self.get_logger().info('start arm move and lift up')
        self.robot.arm.move_to(0.5)
        self.robot.lift.move_to(1.1)
        self.robot.push_command()
        self.robot.arm.wait_until_at_setpoint()
        self.robot.lift.wait_until_at_setpoint()

        self.get_logger().info('start end of arm rotations')
        self.robot.end_of_arm.move_to('wrist_yaw', np.radians(45)) 
        self.robot.push_command()
        self.robot.arm.wait_until_at_setpoint()

        self.robot.end_of_arm.move_to('wrist_pitch', np.radians(45))
        self.robot.push_command()
        self.robot.arm.wait_until_at_setpoint()

        self.robot.end_of_arm.move_to('wrist_roll', np.radians(45))
        self.robot.push_command()
        self.robot.arm.wait_until_at_setpoint()

        self.get_logger().info('start gripper open and close')
        self.robot.end_of_arm.move_to('stretch_gripper', 80)
        self.robot.push_command()
        self.robot.gripper.wait_until_at_setpoint()

        self.robot.end_of_arm.move_to('stretch_gripper', 0)
        self.robot.push_command()
        self.robot.gripper.wait_until_at_setpoint()

        self.get_logger().info('start head movements')
        self.robot.head.move_by('head_pan', np.radians(30))
        self.robot.push_command()
        self.robot.head.wait_until_at_setpoint()

        self.robot.head.move_by('head_tilt', np.radians(30)) 
        self.robot.push_command()
        self.robot.head.wait_until_at_setpoint()

        self.get_logger().info('returning to stow position')
        self.robot.stow()
        time.sleep(3)

        self.get_logger().info('starting final motion')
        self.robot.base.go_forward(0.5)
        self.robot.base.rotate_in_place(np.radians(180))
        self.robot.base.go_forward(0.5)
        self.robot.push_command()
        self.robot.base.wait_until_at_setpoint()

        self.get_logger().info('Motion sequence complete')
        self.robot.stop()

def main(args=None):
    rclpy.init(args=args)
    node = MoveNode()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  