function [Frer_X,Frer_Y] = compute_line_Repulsion(cur_position , line_s ,line_e ,REP,INF_DISTANCE , A ,rGoal ,theta_Goal)
    %计算单条线段产生的斥力,带终点修正（当前点 ，线段起点 ，线段终点 ，斥力计算参数 ，斥力影响距离 ，辅助参数 ，到终点距离 ，引力角度）
    
    [hLine,rLine] = distance_to_line(cur_position ,line_s, line_e);       %计算出当前点到障碍物的垂直距离（hLine）和最短距离（rLine）
    if rLine > INF_DISTANCE    %如果每个障碍和路径的距离大于障碍影响距离，斥力为0
        Frer_X = 0;
        Frer_Y = 0;
    else
        [theta_Line ,theta1 ,theta2] = complete_theta(cur_position ,line_s, line_e);     %计算出线段与系统x轴的夹角，和积分的上下限角
        if (theta1 == 0) | (theta1 > 3.1415926) |  (theta1 < -3.1415926)  %当当前点与线段共线时，只考虑与当前点较近处的线段端点所产生的斥力
            distance_c_s = sqrt((cur_position(1) - line_s(1)) ^ 2 + (cur_position(2) - line_s(2)) ^ 2);     %当前点与线段起点距离
            distance_c_e = sqrt((cur_position(1) - line_e(1)) ^ 2 + (cur_position(2) - line_e(2)) ^ 2);     %当前点与线段终点距离
            if distance_c_s >= distance_c_e         %计算斥力时距离
                distance = distance_c_e; 
                theta_Obj = theta_Line;
            else
                distance = distance_c_s;
                theta_Obj = theta_Line + pi;
            end
            if distance < ( INF_DISTANCE / 2 )           %如果每个障碍线段端点和路径点的距离小于障碍影响距离的1/2
                Frer1_line = REP * ( 1 / distance - 1 / INF_DISTANCE ) * ( 1 / ( distance ^ 2 ) ) * ( rGoal ^ A ) ;
                Frer2_line = A * REP * ( ( 1 / distance - 1 / INF_DISTANCE ) ^ 2 ) * ( rGoal ^ ( 1 - A ) ) / 2 ;
                Frer1_line_x = Frer1_line * cos ( theta_Obj ) ;
                Frer1_line_y = Frer1_line * sin ( theta_Obj ) ;
            else                                         %如果每个障碍线段端点和路径点的距离在障碍影响距离的1/2到1之间
                Frer1_line = REP * ( 1 / distance - 1 / INF_DISTANCE ) * ( 1 / ( distance ^ 2 ) ) * ( rGoal ^ 2 ) ;
                Frer2_line = REP * ( ( 1 / distance - 1 / INF_DISTANCE ) ^ 2 ) * rGoal ;
                Frer1_line_x = Frer1_line * cos ( theta_Obj ) ;
                Frer1_line_y = Frer1_line * sin ( theta_Obj ) ;
            end
        else    %当前点与线段不共线时
            if rLine < ( INF_DISTANCE / 2 )    %如果每个障碍和路径的距离小于障碍影响距离的1/2
                Krer1_1 = ( rGoal ^ A ) * REP / ( hLine ^ 3) ;    %Krer1_1 ... Krer2_3为计算斥力时积分公式化简后的一些系数
                Krer1_2 = ( rGoal ^ A ) * REP / ( INF_DISTANCE * hLine^2 ) ;
                Krer2 = A / 2 * REP * ( rGoal ^ abs(A - 1)) ;
            else         %如果每个障碍和路径的距离在障碍影响距离的1/2到1倍之间
                Krer1_1 = ( rGoal ^ 2 ) * REP / ( hLine ^ 3) ;
                Krer1_2 = ( rGoal ^ 2 ) * REP / ( INF_DISTANCE * hLine^2 ) ;
                Krer2 = REP * rGoal;
            end
            Krer2_1 = Krer2 / (hLine^2);
            Krer2_2 = -2 * Krer2 / ( INF_DISTANCE * hLine ) ;
            Krer2_3 = 1 / ( INF_DISTANCE^2 ) ;
            Frer1_line_temp_X1 = Krer1_1 /4 * (sin(theta1)^4 ) - Krer1_2 / 3 * ( sin(theta1)^3 ) ;      
            Frer1_line_temp_X2 = Krer1_1 /4 * (sin(theta2)^4 ) - Krer1_2 / 3 * ( sin(theta2)^3 ) ;
            Frer1_line_temp_X = Frer1_line_temp_X2 - Frer1_line_temp_X1;                          %以线段正方向（s到e）为x正方向时，单条线段斥力在x方向分量
            Frer1_line_temp_Y1 = Krer1_1 * ( (-1/4) * (sin(theta1))^3 * cos(theta1) - 3/8 * sin(theta1) * cos(theta1) + 3/8 * theta1) + Krer1_2 /3 * ( (sin(theta1))^2 * cos(theta1) + 2 * cos(theta1) );
            Frer1_line_temp_Y2 = Krer1_1 * ( (-1/4) * (sin(theta2))^3 * cos(theta2) - 3/8 * sin(theta2) * cos(theta2) + 3/8 * theta2) + Krer1_2 /3 * ( (sin(theta2))^2 * cos(theta2) + 2 * cos(theta2) );
            Frer1_line_temp_Y = Frer1_line_temp_Y2 - Frer1_line_temp_Y1 ;                         %以线段正方向（s到e）为x正方向时，单条线段斥力在y方向分量
            Frer1_line_x =  Frer1_line_temp_X * cos ( theta_Line ) - Frer1_line_temp_Y * sin ( theta_Line );       %单条线段斥力Frer1_line在系统坐标系中x方向分量
            Frer1_line_y =  Frer1_line_temp_X * sin ( theta_Line ) + Frer1_line_temp_Y * cos ( theta_Line );       %单条线段斥力Frer1_line在系统坐标系中y方向分量
            Frer2_line_1 = - Krer2_1 / 2 * ( sin(theta1) * cos(theta1) - theta1 ) + Krer2_2 * cos(theta1) + Krer2_3 * theta1;
            Frer2_line_2 = - Krer2_1 / 2 * ( sin(theta2) * cos(theta2) - theta2 ) + Krer2_2 * cos(theta2) + Krer2_3 * theta2;
            Frer2_line = Frer2_line_2 - Frer2_line_1;    %单条线段斥力的分量Frer2_line
        end
        Frer2_line_x = Frer2_line * cos(theta_Goal);  %单条线段斥力Frer2_line在系统坐标系中x方向分量
        Frer2_line_y = Frer2_line * sin(theta_Goal);  %单条线段斥力Frer2_line在系统坐标系中y方向分量
        Frer_X = Frer1_line_x + Frer2_line_x;    %单条线段总斥力在系统坐标系中x方向分量
        Frer_Y = Frer1_line_y + Frer2_line_y;    %单条线段总斥力在系统坐标系中y方向分量
    end
