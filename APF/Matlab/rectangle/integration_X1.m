function integration_X1()
delta_goal_X = goal_position(1) - cur_position(1);
delta_goal_Y = goal_position(2) - cur_position(2);
rGoal = sqrt ( delta_goal_X ^ 2 + delta_goal_Y ^ 2 ) ;		%路径点与终点的距离
K1 = ( rGoal ^ A ) * REP / ( hline ^ 3) ;
K2 = ( rGoal ^ A ) * REP * Po / ( hline ^ 2) ;
Frerx1 = (K1 * 1/4 * (sin(theta2)^4 ) - (K2 * 1/3 * ( sin(theta2)^3 ) ;
Frerx1 = (K1 * 1/4 * (sin(theta1)^4 ) - (K2 * 1/3 * ( sin(theta1)^3 ) ;
Frerx = Frerx2 - Frerx1;
Frery2 = K1 * ( (-1/4) * (sin(theta2))^3 * cos(theta2) - 3/8 * sin(theta2) * cos(theta2) + 3/8 * theta2) + K2 * 1/3 * (sin(theta2))^2 * cos(theta2) + 2/3 * cos(thata2);
Frery1 = K1 * ( (-1/4) * (sin(theta1))^3 * cos(theta1) - 3/8 * sin(theta1) * cos(theta1) + 3/8 * theta1) + K2 * 1/3 * (sin(theta1))^2 * cos(theta1) + 2/3 * cos(thata1);
Frery = Frery2 - Frery1 ;     
if rObj(i) < ( INF_DISTANCE / 2 )    %如果每个障碍和路径的距离小于障碍影响距离的1/2
            Frer(i) = REP * ( 1 / rObj(i) - 1 / INF_DISTANCE ) * ( 1 / ( rObj(i) ^ 2 ) ) * ( rGoal ^ A );
            Fata(i) = A * REP * ( ( 1 / rObj (i) - 1 / INF_DISTANCE) ^ 2 ) * ( rGoal ^ ( 1 - A ) ) / 2;
            Frerx(i) = ( 1 + 0.1 ) * Frer(i) * cos ( theta_Obj(i) + pi );       %单个障碍物的斥力在X方向分量
            Frery(i) = ( 1 - 0.1 ) * Frer(i) * sin (theta_Obj(i) + pi );      %单个障碍物的斥力在Y方向分量
            Fatax(i) = Fata(i) * cos ( theta_Goal ) ;       %在X方向上目标点对斥力的修正
            Fatay(i) = Fata(i) * sin ( theta_Goal ) ;       %在Y方向上目标点对斥力的修正
        else         %如果每个障碍和路径的距离在障碍影响距离的1/2到1倍之间
            Frer(i) = REP * ( 1 / rObj(i) - 1 / INF_DISTANCE ) * ( 1 / ( rObj(i) ^ 2 ) ) * (rGoal^2) ;
            Fata(i) = REP * ( ( 1 / rObj(i) - 1 / INF_DISTANCE ) ^ 2 ) * rGoal ;
            Frerx(i) = Frer(i) * cos ( theta_Obj(i) + pi ) ;         %单个障碍物的斥力在X方向分量
            Frery(i) = Frer(i) * sin ( theta_Obj(i) + pi ) ;         %单个障碍物的斥力在Y方向分量
            Fatax(i) = Fata(i) * cos ( theta_Goal ) ;       %在X方向上目标点对斥力的修正
            Fatay(i) = Fata(i) * sin ( theta_Goal )         %在Y方向上目标点对斥力的修正
