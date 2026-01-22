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

import time 
import numpy as np
import stretch_body.robot 
robot = stretch_body.robot.Robot() 
robot.startup()

robot.stow()
time.sleep(3)

robot.arm.move_to(0.5)
robot.lift.move_to(1.1)
robot.push_command()
robot.arm.wait_until_at_setpoint()
robot.lift.wait_until_at_setpoint()

robot.end_of_arm.move_to('wrist_yaw', np.radians(45)) 
robot.push_command()
robot.arm.wait_until_at_setpoint()

robot.end_of_arm.move_to('wrist_pitch', np.radians(45)) 
robot.push_command()
robot.arm.wait_until_at_setpoint()

robot.end_of_arm.move_to('wrist_roll', np.radians(45))
robot.push_command()
robot.arm.wait_until_at_setpoint()

robot.end_of_arm.move_to('stretch_gripper', 80)
robot.push_command()
robot.gripper.wait_until_at_setpoint()

robot.end_of_arm.move_to('stretch_gripper', 0)
robot.push_command()
robot.gripper.wait_until_at_setpoint()


robot.head.move_by('head_pan', np.radians(30))
robot.push_command()
robot.head.wait_until_at_setpoint()

robot.head.move_by('head_tilt', np.radians(30))
robot.push_command()
robot.head.wait_until_at_setpoint()

robot.stow()

robot.base.translate_by(0.5)
robot.base.rotate_by(180)
robot.base.translate_by(0.5)
robot.push_command()

robot.stop()
