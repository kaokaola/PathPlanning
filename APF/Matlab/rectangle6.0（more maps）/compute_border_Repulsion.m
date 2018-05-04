function [Frer_X,Frer_Y]=compute_border_Repulsion(cur_position ,goal_position ,border ,REP,INF_DISTANCE , A)     
    %计算的斥力(当前点，目标点，边界，斥力计算参数，引力角度，斥力角度，障碍物，障碍物数,障碍影响距离，辅助参数)
    delta_goal_X = goal_position(1) - cur_position(1);
    delta_goal_Y = goal_position(2) - cur_position(2);
    rGoal = sqrt ( delta_goal_X ^ 2 + delta_goal_Y ^ 2 ) ;		%路径点与终点的距离
    rLine = [ cur_position(2)  (100 - cur_position(1)) (100 - cur_position(2))  cur_position(1)];   %当前点与四条边界的距离
    hLine = rLine;
    line_s = border;
    line_e = [(border(2 : 4 , 1 : 2)) ; (border(1 : 1 , 1 : 2))];
    for i = 1:4
        if rLine(i) > INF_DISTANCE    %如果每个障碍和路径的距离大于障碍影响距离，斥力为0
              Frerx(i) = 0;
              Frery(i) = 0;
        else
            [theta_Line(i) ,theta1 ,theta2] = complete_theta(cur_position ,line_s(i,:), line_e(i,:));     %计算出线段与系统x轴的夹角，和积分的上下限角
            if rLine(i) < ( INF_DISTANCE / 2 )    %如果每个障碍和路径的距离小于障碍影响距离的1/2
                K1 = ( rGoal ^ A ) * REP / ( hLine(i) ^ 3) ;
                K2 = ( rGoal ^ A ) * REP / ( INF_DISTANCE * hLine(i)^2 ) ;
            else         %如果每个障碍和路径的距离在障碍影响距离的1/2到1倍之间
                K1 = ( rGoal ^ 2 ) * REP / ( hLine(i) ^ 3) ;
                K2 = ( rGoal ^ 2 ) * REP / ( INF_DISTANCE * hLine(i)^2 ) ;
            end
            F_temp_X1 = K1 /4 * (sin(theta1)^4 ) - K2 / 3 * ( sin(theta1)^3 ) ;      
            F_temp_X2 = K1 /4 * (sin(theta2)^4 ) - K2 / 3 * ( sin(theta2)^3 ) ;
            F_temp_X = F_temp_X2 - F_temp_X1;                          %以线段正方向（s到e）为x正方向时，单条线段斥力在x方向分量
            F_temp_Y1 = K1 * ( (-1/4) * (sin(theta1))^3 * cos(theta1) - 3/8 * sin(theta1) * cos(theta1) + 3/8 * theta1) + K2 /3 * ( (sin(theta1))^2 * cos(theta1) + 2 * cos(theta1) );
            F_temp_Y2 = K1 * ( (-1/4) * (sin(theta2))^3 * cos(theta2) - 3/8 * sin(theta2) * cos(theta2) + 3/8 * theta2) + K2 /3 * ( (sin(theta2))^2 * cos(theta2) + 2 * cos(theta2) );
            F_temp_Y = F_temp_Y2 - F_temp_Y1 ;                         %以线段正方向（s到e）为x正方向时，单条线段斥力在y方向分量
            Frerx(i) =  F_temp_X * cos ( theta_Line(i) ) - F_temp_Y * sin ( theta_Line(i) );       %单条线段斥力在系统坐标系中x方向分量
            Frery(i) =  F_temp_X * sin ( theta_Line(i) ) + F_temp_Y * cos ( theta_Line(i) );       %单条线段斥力在系统坐标系中y方向分量
        end
    end
    Frer_X = sum(Frerx) ;    %叠加所有障碍物斥力在X方向的分量
    Frer_Y = sum(Frery) ;    %叠加所有障碍物斥力在Y方向的分量