function [Frer_X,Frer_Y]=compute_rec_Repulsion(cur_position ,goal_position ,obj ,OBJ_NUM ,REP,INF_DISTANCE , A ,theta_Goal )     
    %计算矩形障碍物的斥力(当前点，目标点，矩形障碍物，矩形障碍物个数，斥力计算参数,障碍影响距离，辅助参数引力角度)
    %计算单个矩形障碍物的斥力时先分别计算4条线段斥力，线段的斥力计算可由点的斥力计算公式通过积分推得
    
    delta_goal_X = goal_position(1) - cur_position(1);
    delta_goal_Y = goal_position(2) - cur_position(2);
    rGoal = sqrt ( delta_goal_X ^ 2 + delta_goal_Y ^ 2 ) ;		%路径点与终点的距离
    for j = 1:OBJ_NUM
        OBJ = obj( (j*4-3) : j*4 , 1 : 2 );    % obj为所有矩形障碍物所有点的(4*OBJ_NUM)行2列二维矩阵，从头起每四个点表示一个矩形；OBJ为4*2矩阵
        line_s = OBJ;   %一个矩形中四个起点，左下角为第一个，逆时针为正方向
        line_e = [(OBJ(2 : 4 , 1 : 2)) ; (OBJ(1 : 1 , 1 : 2))];     %一个矩形中四个终点，又下角为第一个，逆时针为正方向
        for i = 1:4    %分别计算4条线段斥力
            %计算单条线段产生的斥力在x和y方向上的分量
            [Frer_line_x(i) ,Frer_line_y(i)] = compute_line_Repulsion(cur_position , line_s(i,:), line_e(i,:) ,REP,INF_DISTANCE , A ,rGoal ,theta_Goal);
        end
        Frer_obj_x(j) = sum(Frer_line_x) ;    %叠加单个障碍物所有边界斥力在X方向的分量
        Frer_obj_y(j) = sum(Frer_line_y) ;    %叠加单个障碍物所有边界斥力在Y方向的分量
    end
    Frer_X = sum(Frer_obj_x) ;    %叠加所有障碍物斥力在X方向的分量
    Frer_Y = sum(Frer_obj_y) ;    %叠加所有障碍物斥力在Y方向的分量