from morse.builder import *
from multi_robots.builder.robots import Ranger1 ,Ranger2 ,Ranger3 ,Ranger4 ,Ranger5

# Land human
human_1 = Human()
human_1.translate(-9.2,14,0)
human_2 = Human()
human_2.translate(-6.7,14,0)
human_3 = Human()
human_3.translate(-4.2,14,0)
human_4 = Human()
human_4.translate(-1.7,14,0)
human_5 = Human()
human_5.translate(0.8,14,0)

#Land robot
robot1 = Ranger1()
robot1.translate(9.4,8.4,0.0)
robot1.rotate(0.0, 0.0, 0.0)

robot2 = Ranger2()
robot2.translate(9.4,9.9,0.0)
robot2.rotate(0.0, 0.0, 0.0)

robot3 = Ranger3()
robot3.translate(9.4,11.4,0.0)
robot3.rotate(0.0, 0.0, 0.0)

robot4 = Ranger4()
robot4.translate(9.4,12.9,0.0)
robot4.rotate(0.0, 0.0, 0.0)

robot5 = Ranger5()
robot5.translate(9.4,14.4,0.0)
robot5.rotate(0.0, 0.0, 0.0)

# Add a motion controller
# actuator.
motionvw = MotionVW()
motionvw.translate(0.0,0.0,0.0)

robot1.append(motionvw)
robot2.append(motionvw)
robot3.append(motionvw)
robot4.append(motionvw)
robot5.append(motionvw)
motionvw.add_interface('socket')


# Add a keyboard controller to move the robot with arrow keys.
keyboard = Keyboard()
robot1.append(keyboard)
keyboard.properties(ControlType = 'Position')

# Add a pose sensor that exports the current location and orientation
pose = Pose()
robot1.append(pose)
robot2.append(pose)
robot3.append(pose)
robot4.append(pose)
robot5.append(pose)

robot1.add_default_interface('socket')
robot2.add_default_interface('socket')
robot3.add_default_interface('socket')
robot4.add_default_interface('socket')
robot5.add_default_interface('socket')

env = Environment('my/map_12', fastmode = False)
env.set_camera_location([0,0,20])
env.set_camera_rotation([0,0,0])
