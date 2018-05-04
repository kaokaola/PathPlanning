function [Frer_X,Frer_Y] = compute_cir_Repulsion(cur_position ,goal_position ,OBJ ,OBJ_NUM ,REP,INF_DISTANCE , A ,theta_Goal )     
    %计算圆形障碍物的斥力(当前点，目标点，障碍物，障碍物个数，斥力计算参数，障碍影响距离，辅助参数 ，终点角度)
    %对于圆形障碍物，只需仿照单个障碍点计算斥力方法即可
    
    delta_goal_X = goal_position(1) - cur_position(1);
    delta_goal_Y = goal_position(2) - cur_position(2);
    rGoal = sqrt ( delta_goal_X ^ 2 + delta_goal_Y ^ 2 ) ;		%路径点与终点的距离
    theta_Obj = compute_cir_obj_Angle(cur_position, OBJ ,OBJ_NUM);    %计算当前点到圆形障碍物中心连线与系统X轴的夹角
    for i=1:OBJ_NUM
        delta_obj_X(i) = OBJ(i,1) - cur_position(1);
        delta_obj_Y(i) = OBJ(i,2) - cur_position(2);
        rObj(i) = sqrt(delta_obj_X(i)^2 + delta_obj_Y(i)^2) - OBJ(i,3);     %路径点和障碍的距离保存在数组rre中
        if rObj(i) > INF_DISTANCE    %如果每个障碍和当前点的距离大于障碍影响距离，斥力为0
              Frer1_x(i) = 0;
              Frer1_y(i) = 0;
              Frer2_x(i) = 0;
              Frer2_y(i) = 0;
        else
            if rObj(i) < ( INF_DISTANCE / 2 )    %如果每个障碍和路径的距离小于障碍影响距离的1/2
                %单个障碍物的斥力Frer1,即原始斥力公式计算所得
                Frer1(i) = REP * ( 1 / rObj(i) - 1 / INF_DISTANCE ) * ( 1 / ( rObj(i) ^ 2 ) ) * ( rGoal ^ A );
                %单个障碍物的斥力Frer2,为解决终点不可达问题而引入，参考 S. S. Ge and Y. J. Cui, "New potential functions for mobile robot 
                % path planning," in IEEE Transactions on Robotics and Automation, vol. 16, no. 5, pp. 615-620, Oct 2000.
                Frer2(i) = A * REP * ( ( 1 / rObj (i) - 1 / INF_DISTANCE) ^ 2 ) * ( rGoal ^ ( 1 - A ) ) / 2;
                Frer1_x(i) = ( 1 + 0 ) * Frer1(i) * cos ( theta_Obj(i) + pi );      %单个障碍物的斥力Frer1在X方向分量
                Frer1_y(i) = ( 1 - 0 ) * Frer1(i) * sin (theta_Obj(i) + pi );      %单个障碍物的斥力Frer1在Y方向分量
                Frer2_x(i) = Frer2(i) * cos ( theta_Goal ) ;       %斥力Frer2在X方向上目标点对斥力的修正
                Frer2_y(i) = Frer2(i) * sin ( theta_Goal ) ;       %斥力Frer2在Y方向上目标点对斥力的修正
            else         %如果每个障碍和路径的距离在障碍影响距离的1/2到1倍之间
                Frer1(i) = REP * ( 1 / rObj(i) - 1 / INF_DISTANCE ) * ( 1 / ( rObj(i) ^ 2 ) ) * (rGoal^2) ;
                Frer2(i) = REP * ( ( 1 / rObj(i) - 1 / INF_DISTANCE ) ^ 2 ) * rGoal ;
                Frer1_x(i) = Frer1(i) * cos ( theta_Obj(i) + pi ) ;        
                Frer1_y(i) = Frer1(i) * sin ( theta_Obj(i) + pi ) ;        
                Frer2_x(i) = Frer2(i) * cos ( theta_Goal ) ;       
                Frer2_y(i) = Frer2(i) * sin ( theta_Goal ) ;        
            end
        end
    end
    Frer_X = sum(Frer1_x) + sum(Frer2_x);    %叠加所有障碍物斥力在X方向的分量
    Frer_Y = sum(Frer1_y) + sum(Frer2_y);    %叠加所有障碍物斥力在Y方向的分量
