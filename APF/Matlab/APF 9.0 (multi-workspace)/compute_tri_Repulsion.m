function [Frer_X,Frer_Y] = compute_tri_Repulsion(cur_position ,goal_position ,obj ,OBJ_NUM ,REP,INF_DISTANCE , A ,theta_Goal )     
    %计算三角障碍物的斥力(当前点，目标点，三角障碍物，三角障碍物个数，斥力计算参数,障碍影响距离，辅助参数引力角度)
    %计算单个三角障碍物的斥力时先分别计算3条线段斥力，线段的斥力计算可由点的斥力计算公式通过积分推得
    %参考compute_rec_Repulsion
    
    delta_goal_X = goal_position(1) - cur_position(1);
    delta_goal_Y = goal_position(2) - cur_position(2);
    rGoal = sqrt ( delta_goal_X ^ 2 + delta_goal_Y ^ 2 ) ;		%路径点与终点的距离
    for j = 1:OBJ_NUM
        OBJ = obj( (j*3-2) : j*3 , 1 : 2 );     % obj为所有三角障碍物所有点的(3*OBJ_NUM)行2列二维矩阵，从头起每3个点表示一个三角形；OBJ为3*2矩阵
        line_s = OBJ;    %一个三角形中3个起点，左下角为第一个，逆时针为正方向
        line_e = [(OBJ(2 : 3 , 1 : 2)) ; (OBJ(1 : 1 , 1 : 2))];    %一个三角形中3个终点，右角为第一个，逆时针为正方向
        for i = 1:3     %分别计算3条线段斥力
               %计算单条线段产生的斥力在x和y方向上的分量
            [Frer_line_x(i) ,Frer_line_y(i)] = compute_line_Repulsion(cur_position , line_s(i,:), line_e(i,:) ,REP,INF_DISTANCE , A ,rGoal ,theta_Goal);
        end
        Frer_obj_x(j) = sum(Frer_line_x) ;    %叠加单个障碍物所有边界斥力在X方向的分量
        Frer_obj_y(j) = sum(Frer_line_y) ;    %叠加单个障碍物所有边界斥力在Y方向的分量
    end
    Frer_X = sum(Frer_obj_x) ;    %叠加所有障碍物斥力在X方向的分量
    Frer_Y = sum(Frer_obj_y) ;    %叠加所有障碍物斥力在Y方向的分量