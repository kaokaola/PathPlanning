function [Frer_X,Frer_Y]=compute_Repulsion(cur_position ,goal_position ,OBJ ,REP, theta_Goal ,theta_Obj, OBJ_NUM ,INF_DISTANCE , A)     
    %计算障碍物的斥力(当前点，目标点，障碍物，斥力计算参数，引力角度，斥力角度，障碍物，障碍物数,障碍影响距离，辅助参数)
delta_goal_X = goal_position(1) - cur_position(1);
delta_goal_Y = goal_position(2) - cur_position(2);
rGoal = sqrt ( delta_goal_X ^ 2 + delta_goal_Y ^ 2 ) ;		%路径点与终点的距离 
for i=1:OBJ_NUM
    delta_obj_X(i) = OBJ(i,1) - cur_position(1);
    delta_obj_Y(i) = OBJ(i,2) - cur_position(2);
    rObj(i) = sqrt(delta_obj_X(i)^2 + delta_obj_Y(i)^2);     %路径点和障碍的距离保存在数组rre中
    if rObj(i) > INF_DISTANCE    %如果每个障碍和路径的距离大于障碍影响距离，斥力为0
          Frerx(i) = 0;
          Frery(i) = 0;
          Fatax(i) = 0;
          Fatay(i) = 0;
    else
        if rObj(i) < ( INF_DISTANCE / 2 )    %如果每个障碍和路径的距离小于障碍影响距离的1/2
            Frer(i) = REP * ( 1 / rObj(i) - 1 / INF_DISTANCE ) * ( 1 / ( rObj(i) ^ 2 ) ) * ( rGoal ^ A );
            Fata(i) = A * REP * ( ( 1 / rObj (i) - 1 / INF_DISTANCE) ^ 2 ) * ( rGoal ^ ( 1 - A ) ) / 2;
            Frerx(i) = ( 1 + 0 ) * Frer(i) * cos ( theta_Obj(i) + pi );       %单个障碍物的斥力在X方向分量
            Frery(i) = ( 1 - 0 ) * Frer(i) * sin (theta_Obj(i) + pi );      %单个障碍物的斥力在Y方向分量
            Fatax(i) = Fata(i) * cos ( theta_Goal ) ;       %在X方向上目标点对斥力的修正
            Fatay(i) = Fata(i) * sin ( theta_Goal ) ;       %在Y方向上目标点对斥力的修正
        else         %如果每个障碍和路径的距离在障碍影响距离的1/2到1倍之间
            Frer(i) = REP * ( 1 / rObj(i) - 1 / INF_DISTANCE ) * ( 1 / ( rObj(i) ^ 2 ) ) * (rGoal^2) ;
            Fata(i) = REP * ( ( 1 / rObj(i) - 1 / INF_DISTANCE ) ^ 2 ) * rGoal ;
            Frerx(i) = Frer(i) * cos ( theta_Obj(i) + pi ) ;         %单个障碍物的斥力在X方向分量
            Frery(i) = Frer(i) * sin ( theta_Obj(i) + pi ) ;         %单个障碍物的斥力在Y方向分量
            Fatax(i) = Fata(i) * cos ( theta_Goal ) ;       %在X方向上目标点对斥力的修正
            Fatay(i) = Fata(i) * sin ( theta_Goal )         %在Y方向上目标点对斥力的修正
        end
    end
end
   Frer_X = sum(Frerx) + sum(Fatax);    %叠加所有障碍物斥力在X方向的分量
   Frer_Y = sum(Frery) + sum(Fatay);    %叠加所有障碍物斥力在Y方向的分量