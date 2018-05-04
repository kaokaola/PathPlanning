from pymorse import Morse
import sys
import math

try:
    from morse.builder import *
except ImportError:
    pass

def compute_distance(A ,B):
    delta_x = A[0] - B[0]
    delta_y = A[1] - B[1]
    distance = (delta_x ** 2 + delta_y ** 2) ** 0.5
    return distance

def goal_theta_and_dis(position , goal):
    current_x ,current_y ,current_theta = position['x'] ,position['y'] ,position['yaw']
    goal_x ,goal_y = goal[0] ,goal[1]
    delta_x = goal_x - current_x
    delta_y = goal_y - current_y
    dis_goal = (delta_x ** 2 + delta_y ** 2) ** 0.5
    if delta_y >= 0.0:
        temp_theta = math.acos(delta_x / dis_goal)
    else:
        temp_theta = -math.acos(delta_x / dis_goal)
    goal_theta = current_theta - temp_theta
    if goal_theta < -math.pi:
        goal_theta = 2 * math.pi + goal_theta
    if goal_theta > math.pi:
        goal_theta = -2 * math.pi + goal_theta
    return dis_goal ,goal_theta

def get_vw(position , goal , dis_goal ,goal_theta):
    if dis_goal > 0.3:
        if goal_theta < -0.1:
            v ,w = 0.0 ,1.0
        elif goal_theta > 0.1:
            v ,w = 0.0 ,-1.0
        else:
            v ,w = 2.5 ,0.0
    else :
        if goal_theta < -0.2:
            v ,w = 0.0 ,0.5
        elif goal_theta > 0.2:
            v ,w = 0.0 ,-0.5
        else:
            v ,w = 1.5 ,0.0
    return v ,w

def get_w(position , goal_yaw_flag ) :
    if goal_yaw_flag == 0:
        w = 0.0
    else:
        if goal_yaw_flag == 1 :
            goal_yaw = math.pi
        elif goal_yaw_flag == 2:
            goal_yaw = math.pi / 2
        else:
            goal_yaw = 0
        current_yaw =  position['yaw']
        turn_theta =  current_yaw - goal_yaw
        if turn_theta < -math.pi:
            turn_theta = 2 * math.pi + turn_theta
        elif turn_theta > math.pi:
            turn_theta = -2 * math.pi + turn_theta
        if turn_theta < -0.1:
            w = 0.5
        elif turn_theta > 0.1:
            w = -0.5
        else:
            w = 0.0
        #print(turn_theta  , goal_yaw, current_yaw ,w)
    return w

def main():
    morse = Morse()
    motion1 ,pose1 = morse.robot1.motion ,morse.robot1.pose
    motion2 ,pose2 = morse.robot2.motion ,morse.robot2.pose
    motion3 ,pose3 = morse.robot3.motion ,morse.robot3.pose
    motion4 ,pose4 = morse.robot4.motion ,morse.robot4.pose
    motion5 ,pose5 = morse.robot5.motion ,morse.robot5.pose

    goal1_tuple = ((3.69,-0.66),(1.5,-1.5),(-1.3,-5.2),(-1.3,-10),(-2.6,4.4),(-8.6,5.0),(-9,11.9),(-9,10.3),(9.4,8.4))
    goal2_tuple = ((8,9.9),(4.05,-5.60),(4.0,-12),(4.05,-5.1),(6,-1.4),(6.8,6.25),(4.3,9.5),(1,11.9),(1.2,11.9),(9.4,9.9))
    goal3_tuple = ((0.64,8.51),(-6.34,-5.05),(-6.58,-8),(-6.6,-5.5),(-4.5,7.8),(-5.4,9.8),(-6.5,11.9),(-6.5,10.3),(9.4,11.4))
    goal4_tuple = ((8.1,12.9),(8.1,7),(9.2,-12.7),(9.2,-5.2),(4,2.2),(1.6,4.2),(-1.5,11.9),(-1.5,10.3),(3.7,10.3),(6.8,12.1),(9.4,12.9))
    goal5_tuple = ((1.3,9.2),(-0.8,-1),(-1.3,-5.5),(-1.3,-11),(-1.3,-5.5),(-1.4,0.1),(-4.6,5.5),(-4,11.9),(-4,10.3),(2.7,10.3),(8.1,14.4),(9.4,14.4))
    
    goal_flag1 ,goal1 = 0 ,goal1_tuple[0]
    goal_flag2 ,goal2 = 0 ,goal2_tuple[0]
    goal_flag3 ,goal3 = 0 ,goal3_tuple[0]
    goal_flag4 ,goal4 = 0 ,goal4_tuple[0]
    goal_flag5 ,goal5 = 0 ,goal5_tuple[0]

    while True:
        if goal_flag1 <= 8 :
            if goal_flag1 == 8:
                goal_yaw_flag1 = 3
            else:
                goal_yaw_flag1 = 0
            position1 = pose1.get()
            dis_goal1 ,goal_theta1 = goal_theta_and_dis(position1 , goal1)
            if dis_goal1 > 0.03 :
                v1 ,w1 = get_vw(position1 , goal1 ,dis_goal1 ,goal_theta1)
            else :
                v1 = 0.0
                w1 = get_w(position1 , goal_yaw_flag1 )
            motion1.publish({"v": v1, "w": w1})
            if v1 == 0.0 and w1 == 0.0 :
                goal_flag1 = goal_flag1 + 1
                if goal_flag1 <= 8:
                    goal1 = goal1_tuple[goal_flag1]

        if goal_flag2 <= 9 :
            if goal_flag2 == 8:
                goal_yaw_flag2 = 3
            else:
                goal_yaw_flag2 = 0
            position2 = pose2.get()
            dis_goal2 ,goal_theta2 = goal_theta_and_dis(position2 , goal2)
            if dis_goal2 > 0.03 :
                v2 ,w2 = get_vw(position2 , goal2 ,dis_goal2 ,goal_theta2)
                distance = compute_distance((position1['x'], position1['y']), (position2['x'], position2['y']))
                if  distance <= 1.35:
                    v2, w2 = 0.01, 0.0
            else :
                v2 = 0.0
                w2 = get_w(position2 , goal_yaw_flag2 )
            motion2.publish({"v": v2, "w": w2})
            if v2 == 0.0 and w2 == 0.0 :
                goal_flag2 = goal_flag2 + 1
                if goal_flag2 <= 9:
                    goal2 = goal2_tuple[goal_flag2]
            
        if goal_flag3 <= 8 and goal_flag1 >= 1:
            if goal_flag3 == 8:
                goal_yaw_flag3 = 3
            else:
                goal_yaw_flag3 = 0
            position3 = pose3.get()
            dis_goal3 ,goal_theta3 = goal_theta_and_dis(position3 , goal3)
            if dis_goal3 > 0.03 :
                v3 ,w3 = get_vw(position3 , goal3 ,dis_goal3 ,goal_theta3)
                distance1 = compute_distance((position1['x'], position1['y']), (position3['x'], position3['y']))
                distance2 = compute_distance((position2['x'], position2['y']), (position3['x'], position3['y']))
                if  distance1 <= 1.35 or distance2 <= 1.35:
                    v3, w3 = 0.01, 0.0
            else :
                v3 = 0.0
                w3 = get_w(position3 , goal_yaw_flag3 )
            motion3.publish({"v": v3, "w": w3})
            if v3 == 0.0 and w3 == 0.0 :
                goal_flag3 = goal_flag3 + 1
                if goal_flag3 <= 8:
                    goal3 = goal3_tuple[goal_flag3]

        if goal_flag4 <= 10 and goal_flag1 >= 3:
            if goal_flag4 == 8:
                goal_yaw_flag4 = 3
            else:
                goal_yaw_flag4 = 0
            position4 = pose4.get()
            dis_goal4 ,goal_theta4 = goal_theta_and_dis(position4 , goal4)
            if dis_goal4 > 0.03 :
                v4 ,w4 = get_vw(position4 , goal4 ,dis_goal4 ,goal_theta4)
                distance1 = compute_distance((position1['x'], position1['y']), (position4['x'], position4['y']))
                distance2 = compute_distance((position2['x'], position2['y']), (position4['x'], position4['y']))
                distance3 = compute_distance((position3['x'], position3['y']), (position4['x'], position4['y']))
                if  distance1 <= 1.35 or distance2 <= 1.35 or distance3 <= 1.35:
                    v4, w4 = 0.01, 0.0
            else :
                v4 = 0.0
                w4 = get_w(position4 , goal_yaw_flag4 )
            motion4.publish({"v": v4, "w": w4})
            if v4 == 0.0 and w4 == 0.0 :
                goal_flag4 = goal_flag4 + 1
                if goal_flag4 <= 10:
                    goal4 = goal4_tuple[goal_flag4]

        if goal_flag5 <= 11 and goal_flag1 >= 5:
            if goal_flag5 == 9:
                goal_yaw_flag5 = 3
            else:
                goal_yaw_flag5 = 0
            position5 = pose5.get()
            dis_goal5 ,goal_theta5 = goal_theta_and_dis(position5 , goal5)
            if dis_goal5 > 0.03 :
                v5 ,w5 = get_vw(position5 , goal5 ,dis_goal5 ,goal_theta5)
                distance1 = compute_distance((position1['x'], position1['y']), (position5['x'], position5['y']))
                distance2 = compute_distance((position2['x'], position2['y']), (position5['x'], position5['y']))
                distance3 = compute_distance((position3['x'], position3['y']), (position5['x'], position5['y']))
                distance4 = compute_distance((position4['x'], position4['y']), (position5['x'], position5['y']))
                if  distance1 <= 1.35 or distance2 <= 1.35 or distance3 <= 1.35 or distance4 <= 1.35:
                    v5, w5 = 0.01, 0.0
            else :
                v5 = 0.0
                w5 = get_w(position5 , goal_yaw_flag5 )
            motion5.publish({"v": v5, "w": w5})
            if v5 == 0.0 and w5 == 0.0 :
                goal_flag5 = goal_flag5 + 1
                if goal_flag5 <= 11:
                    goal5 = goal5_tuple[goal_flag5]

if __name__ =="__main__":
    main()




