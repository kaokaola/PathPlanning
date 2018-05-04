function [Fat_X,Fat_Y] = compute_Attract(cur_position ,goal_position ,ATT ,theta_Goal) 
%计算出x和y方向的引力（当前位置，终点位置，引力计算参数，引力角度）
    delta_goal_X = goal_position(1) - cur_position(1);
    delta_goal_Y = goal_position(2) - cur_position(2);
    rGoal = sqrt ( delta_goal_X ^ 2 + delta_goal_Y ^ 2 ) ;		%路径点与终点的距离 
    Fat_X = ATT * rGoal * cos(theta_Goal);
    Fat_Y = ATT * rGoal * sin(theta_Goal);