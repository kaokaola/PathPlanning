function [Frer_X,Frer_Y]=compute_Repulsion(cur_position ,goal_position ,obj ,OBJ_NUM ,REP,INF_DISTANCE , A)     
    %计算障碍物的斥力(当前点，目标点，障碍物，障碍物个数，斥力计算参数，引力角度，斥力角度，障碍物，障碍物数,障碍影响距离，辅助参数)
    
delta_goal_X = goal_position(1) - cur_position(1);
delta_goal_Y = goal_position(2) - cur_position(2);
rGoal = sqrt ( delta_goal_X ^ 2 + delta_goal_Y ^ 2 ) ;		%路径点与终点的距离
for j = 1:OBJ_NUM
    OBJ = obj( (j*4-3) : j*4 , 1 : 2 );
    line_s = OBJ;
    line_e = [(OBJ(2 : 4 , 1 : 2)) ; (OBJ(1 : 1 , 1 : 2))];
    for i = 1:4
        [hLine(i),rLine(i)] = distance_to_line(cur_position ,line_s(i,:), line_e(i,:));       %计算出当前点到障碍物的垂直距离（hLine）和最短距离（rLine）       
        if rLine(i) > INF_DISTANCE    %如果每个障碍和路径的距离大于障碍影响距离，斥力为0
              Frerx(i) = 0;
              Frery(i) = 0;
        else
            [theta_Line(i) ,theta1 ,theta2] = complete_theta(cur_position ,line_s(i,:), line_e(i,:));     %计算出线段与系统x轴的夹角，和积分的上下限角
            if rLine(i) < ( INF_DISTANCE / 2 )    %如果每个障碍和路径的距离小于障碍影响距离的1/2
                Krer1_1 = ( rGoal ^ A ) * REP / ( hLine(i) ^ 3) ;
                Krer1_2 = ( rGoal ^ A ) * REP / ( INF_DISTANCE * hLine(i)^2 ) ;
            else         %如果每个障碍和路径的距离在障碍影响距离的1/2到1倍之间
                Krer1_1 = ( rGoal ^ 2 ) * REP / ( hLine(i) ^ 3) ;
                Krer1_2 = ( rGoal ^ 2 ) * REP / ( INF_DISTANCE * hLine(i)^2 ) ;
            end
            F_temp_X1 = Krer1_1 /4 * (sin(theta1)^4 ) - Krer1_2 / 3 * ( sin(theta1)^3 ) ;      
            F_temp_X2 = Krer1_1 /4 * (sin(theta2)^4 ) - Krer1_2 / 3 * ( sin(theta2)^3 ) ;
            F_temp_X = F_temp_X2 - F_temp_X1;                          %以线段正方向（s到e）为x正方向时，单条线段斥力在x方向分量
            F_temp_Y1 = Krer1_1 * ( (-1/4) * (sin(theta1))^3 * cos(theta1) - 3/8 * sin(theta1) * cos(theta1) + 3/8 * theta1) + Krer1_2 /3 * ( (sin(theta1))^2 * cos(theta1) + 2 * cos(theta1) );
            F_temp_Y2 = Krer1_1 * ( (-1/4) * (sin(theta2))^3 * cos(theta2) - 3/8 * sin(theta2) * cos(theta2) + 3/8 * theta2) + Krer1_2 /3 * ( (sin(theta2))^2 * cos(theta2) + 2 * cos(theta2) );
            F_temp_Y = F_temp_Y2 - F_temp_Y1 ;                         %以线段正方向（s到e）为x正方向时，单条线段斥力在y方向分量
            Frerx1(i) =  F_temp_X * cos ( theta_Line(i) ) - F_temp_Y * sin ( theta_Line(i) );       %单条线段斥力Frer1在系统坐标系中x方向分量
            Frery1(i) =  F_temp_X * sin ( theta_Line(i) ) + F_temp_Y * cos ( theta_Line(i) );       %单条线段斥力Frer1在系统坐标系中y方向分量
        end
    end
    Frer_X_1(j) = sum(Frerx) ;    %叠加单个障碍物所有边界斥力在X方向的分量
    Frer_Y_1(j) = sum(Frery) ;    %叠加单个障碍物所有边界斥力在Y方向的分量
end
Frer_X = sum(Frer_X_1) ;    %叠加所有障碍物斥力在X方向的分量
Frer_Y = sum(Frer_Y_1) ;    %叠加所有障碍物斥力在Y方向的分量