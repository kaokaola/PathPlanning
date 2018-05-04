from pymorse import Morse
import sys
import math

try:
    from morse.builder import *
except ImportError:
    pass

morse = Morse()
motion = morse.robot.motion
pose = morse.robot.pose
 
goal_x = 5
goal_y = 5 

print("At first ,the robot is currently at: %s" % pose.get())
morse.sleep(1)

motion.publish({'x':5.0,'y':5.0,'z':0.0,'tolerance':0.1,'speed':2.0})

morse.sleep(10)

print("The robot is currently at: %s" % pose.get())

