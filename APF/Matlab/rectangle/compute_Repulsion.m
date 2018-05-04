function [Frer_X,Frer_Y]=compute_Repulsion(cur_position ,goal_position ,OBJ ,REP,INF_DISTANCE , A)     
    %计算障碍物的斥力(当前点，目标点，障碍物，斥力计算参数，引力角度，斥力角度，障碍物，障碍物数,障碍影响距离，辅助参数)
delta_goal_X = goal_position(1) - cur_position(1);
delta_goal_Y = goal_position(2) - cur_position(2);
rGoal = sqrt ( delta_goal_X ^ 2 + delta_goal_Y ^ 2 ) ;		%路径点与终点的距离

for i = 1:4
    line_s(1) = OBJ(i,1);      %矩形障碍物边界线段起点
    line_s(2) = OBJ(i,2);
    if i < 4 
        line_e(1) = OBJ(i+1,1);    %线段终点
        line_e(2) = OBJ(i+1,2);
    else
        line_e(1) = OBJ(1,1);      %线段终点
        line_e(2) = OBJ(1,2);
    end
    [hline(i),rLine(i)] = distance_to_line(cur_position ,line_s, line_e);       
    if rLine(i) > INF_DISTANCE    %如果每个障碍和路径的距离大于障碍影响距离，斥力为0
          Frerx(i) = 0;
          Frery(i) = 0;
    else
        [theta_Line(i) ,theta1 ,theta2] = complete_theta(cur_position ,line_s, line_e);
        if rLine(i) < ( INF_DISTANCE / 2 )    %如果每个障碍和路径的距离小于障碍影响距离的1/2
            K1 = ( rGoal ^ A ) * REP / ( hline(i) ^ 3) ;
            K2 = ( rGoal ^ A ) * REP / INF_DISTANCE / ( hline(i) ^ 2) ;
        else         %如果每个障碍和路径的距离在障碍影响距离的1/2到1倍之间
            K1 = ( rGoal ^ 2 ) * REP / ( hline(i) ^ 3) ;
            K2 = ( rGoal ^ 2 ) * REP / INF_DISTANCE / ( hline(i) ^ 2) ;
        end
        F_temp_X1(i) = K1 * 1/4 * (sin(theta2)^4 ) - K2 * 1/3 * ( sin(theta2)^3 ) ;
        F_temp_X2(i) = K1 * 1/4 * (sin(theta1)^4 ) - K2 * 1/3 * ( sin(theta1)^3 ) ;
        F_temp_X(i) = F_temp_X2(i) - F_temp_X1(i);
        F_temp_Y1(i) = K1 * ( (-1/4) * (sin(theta2))^3 * cos(theta2) - 3/8 * sin(theta2) * cos(theta2) + 3/8 * theta2) + K2 * 1/3 * (sin(theta2))^2 * cos(theta2) + 2/3 * cos(theta2);
        F_temp_Y2(i) = K1 * ( (-1/4) * (sin(theta1))^3 * cos(theta1) - 3/8 * sin(theta1) * cos(theta1) + 3/8 * theta1) + K2 * 1/3 * (sin(theta1))^2 * cos(theta1) + 2/3 * cos(theta1);
        F_temp_Y(i) = F_temp_Y2(i) - F_temp_Y1(i) ; 
        Frerx(i) = F_temp_X(i) * cos ( theta_Line(i) ) + F_temp_Y(i) * sin ( theta_Line(i) );
        Frery(i) = F_temp_X(i) * sin ( theta_Line(i) ) + F_temp_Y(i) * cos ( theta_Line(i) );
    end

end
   Frer_X = sum(Frerx) ;    %叠加所有障碍物斥力在X方向的分量
   Frer_Y = sum(Frery) ;    %叠加所有障碍物斥力在Y方向的分量