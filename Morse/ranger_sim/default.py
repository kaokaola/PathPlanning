#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <ranger_sim> environment

Feel free to edit this template as you like!
"""

from morse.builder import *
from ranger_sim.builder.robots import Ranger

# Add the MORSE mascott, MORSY.
# Out-the-box available robots are listed here:
# http://www.openrobots.org/morse/doc/stable/components_library.html
#

# Land human
'''human_1 = Human()
human_1.translate(-9.2,14,0)
human_2 = Human()
human_2.translate(-6.7,14,0)
human_3 = Human()
human_3.translate(-4.2,14,0)
human_4 = Human()
human_4.translate(-1.7,14,0)
human_5 = Human()
human_5.translate(0.8,14,0)'''

# 'morse add robot <name> ranger_sim' can help you to build custom robots.
robot = Ranger()

# The list of the main methods to manipulate your components
# is here: http://www.openrobots.org/morse/doc/stable/user/builder_overview.html
robot.translate(-5.0,-5.0,0.0)
robot.rotate(0.0, 0.0, 0.0)

# Add a motion controller
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#actuators
#
# 'morse add actuator <name> ranger_sim' can help you with the creation of a custom
# actuator.
motionvw = MotionVW()

motionvw.translate(0.0,0.0,0.0)

robot.append(motionvw)
motionvw.add_interface('socket')


# Add a keyboard controller to move the robot with arrow keys.
keyboard = Keyboard()
robot.append(keyboard)
keyboard.properties(ControlType = 'Position')

# Add a pose sensor that exports the current location and orientation
# of the robot in the world frame
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#sensors
#
# 'morse add sensor <name> ranger_sim' can help you with the creation of a custom
# sensor.
pose = Pose()
robot.append(pose)

# To ease development and debugging, we add a socket interface to our robot.
#
# Check here: http://www.openrobots.org/morse/doc/stable/user/integration.html 
# the other available interfaces (like ROS, YARP...)
robot.add_default_interface('socket')


# set 'fastmode' to True to switch to wireframe mode
env = Environment('sandbox', fastmode = False)
env.set_camera_location([0,0,20])
env.set_camera_rotation([0,0,0])

