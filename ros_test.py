#!/usr/bin/env python3
import time
import numpy as np
import hello_helpers.hello_misc as hm


node = hm.HelloNode.quick_create('temp')

node.stow_the_robot()
time.sleep(2)