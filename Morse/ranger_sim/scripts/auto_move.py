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
goal_y = -5

print("At first ,the robot is currently at: %s" % pose.get())
morse.sleep(0.1)

while True:
    position = pose.get()
    current_x = position['x']
    current_y = position['y']
    current_theta = position['yaw']
    
    delta_x = goal_x - current_x
    delta_y = goal_y - current_y
    dis_goal = (delta_x ** 2 + delta_y ** 2) ** 0.5
    
    if delta_y >= 0.0:
        temp_theta = math.acos(delta_x / dis_goal)
    else:
        temp_theta = -math.acos(delta_x / dis_goal)
    #print("the temp_theta is %s" % temp_theta )
    
    goal_theta = current_theta - temp_theta
    if goal_theta < -math.pi:
        goal_theta = 2 * math.pi + goal_theta
    if goal_theta > math.pi:
        goal_theta = -2 * math.pi + goal_theta
    print("the goal_theta is %s" % goal_theta )
    if dis_goal > 0.02:
        if goal_theta < -0.2:
            v = 0.0
            w = 0.5
        elif goal_theta > 0.2:
            v = 0.0
            w = -0.5
        else:
            v = 2.0
            w = 0.0
    else:        
        v = 0.0
        w = 0.0
        motion.publish({"v": v, "w": w})
        morse.sleep(1)
        print("The robot is currently at: %s" % pose.get())
        break
    motion.publish({"v": v, "w": w})



