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
 
position = pose.get()


print("At first ,the robot is currently at: %s" % pose.get())
