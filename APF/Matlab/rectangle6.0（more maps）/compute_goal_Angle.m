function Y = compute_goal_Angle(cur_position ,goal_position)
    %计算引力角度（当前位置，目标位置）
    delta_goal_X = goal_position(1) - cur_position(1);
    delta_goal_Y = goal_position(2) - cur_position(2);
    rGoal = sqrt ( delta_goal_X ^ 2 + delta_goal_Y ^ 2 ) ;		    %路径点与终点的距离 
    Y = sign ( delta_goal_Y ) * acos ( delta_goal_X / rGoal ) ;		%起点到终点连线与X正方向的夹角的角度，即引力角度，逆时针为正方向