import time 
import numpy as np
import stretch_body.robot 
robot = stretch_body.robot.Robot() 
robot.startup()

robot.stow()
time.sleep(3)