function [Frer_X,Frer_Y]=compute_border_Repulsion(cur_position ,goal_position ,border ,REP,INF_DISTANCE , A ,theta_Goal)     
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
                Krer1_1 = ( rGoal ^ A ) * REP / ( hLine(i) ^ 3) ;
                Krer1_2 = ( rGoal ^ A ) * REP / ( INF_DISTANCE * hLine(i)^2 ) ;
                Krer2 = A / 2 * REP * ( rGoal ^ abs(A - 1)) ;
            else         %如果每个障碍和路径的距离在障碍影响距离的1/2到1倍之间
                Krer1_1 = ( rGoal ^ 2 ) * REP / ( hLine(i) ^ 3) ;
                Krer1_2 = ( rGoal ^ 2 ) * REP / ( INF_DISTANCE * hLine(i)^2 ) ;
                Krer2 = REP * rGoal;
            end
            Krer2_1 = Krer2 / (hLine(i)^2);
            Krer2_2 = -2 * Krer2 / ( INF_DISTANCE * hLine(i) ) ;
            Krer2_3 = 1 / ( INF_DISTANCE^2 ) ;
            Frer1_temp_X1 = Krer1_1 /4 * (sin(theta1)^4 ) - Krer1_2 / 3 * ( sin(theta1)^3 ) ;      
            Frer1_temp_X2 = Krer1_1 /4 * (sin(theta2)^4 ) - Krer1_2 / 3 * ( sin(theta2)^3 ) ;
            Frer1_temp_X = Frer1_temp_X2 - Frer1_temp_X1;                          %以线段正方向（s到e）为x正方向时，单条线段斥力在x方向分量
            Frer1_temp_Y1 = Krer1_1 * ( (-1/4) * (sin(theta1))^3 * cos(theta1) - 3/8 * sin(theta1) * cos(theta1) + 3/8 * theta1) + Krer1_2 /3 * ( (sin(theta1))^2 * cos(theta1) + 2 * cos(theta1) );
            Frer1_temp_Y2 = Krer1_1 * ( (-1/4) * (sin(theta2))^3 * cos(theta2) - 3/8 * sin(theta2) * cos(theta2) + 3/8 * theta2) + Krer1_2 /3 * ( (sin(theta2))^2 * cos(theta2) + 2 * cos(theta2) );
            Frer1_temp_Y = Frer1_temp_Y2 - Frer1_temp_Y1 ;                         %以线段正方向（s到e）为x正方向时，单条线段斥力在y方向分量
            Frer1_x(i) =  Frer1_temp_X * cos ( theta_Line(i) ) - Frer1_temp_Y * sin ( theta_Line(i) );       %单条线段斥力Frer1在系统坐标系中x方向分量
            Frer1_y(i) =  Frer1_temp_X * sin ( theta_Line(i) ) + Frer1_temp_Y * cos ( theta_Line(i) );       %单条线段斥力Frer1在系统坐标系中y方向分量
            Frer2_1 = - Krer2_1 / 2 * ( sin(theta1) * cos(theta1) - theta1 ) + Krer2_2 * cos(theta1) + Krer2_3 * theta1;
            Frer2_2 = - Krer2_1 / 2 * ( sin(theta2) * cos(theta2) - theta2 ) + Krer2_2 * cos(theta2) + Krer2_3 * theta2;
            Frer2 = Frer2_2 - Frer2_1;                          %单条线段斥力的分量Frer2
            Frer2_x(i) = Frer2 * cos(theta_Goal);               %单条线段斥力Frer2在系统坐标系中x方向分量
            Frer2_y(i) = Frer2 * sin(theta_Goal);               %单条线段斥力Frer2在系统坐标系中y方向分量
            Frerx(i) = Frer1_x(i) + Frer2_x(i);                       %单条线段总斥力在系统坐标系中x方向分量
            Frery(i) = Frer1_y(i) + Frer2_y(i);                       %单条线段总斥力在系统坐标系中y方向分量
        end
    end
    Frer_X = sum(Frerx) ;    %叠加所有障碍物斥力在X方向的分量
    Frer_Y = sum(Frery) ;    %叠加所有障碍物斥力在Y方向的分量