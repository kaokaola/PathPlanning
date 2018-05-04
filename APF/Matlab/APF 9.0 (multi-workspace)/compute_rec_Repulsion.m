function [Frer_X,Frer_Y]=compute_rec_Repulsion(cur_position ,goal_position ,obj ,OBJ_NUM ,REP,INF_DISTANCE , A ,theta_Goal )     
    %��������ϰ���ĳ���(��ǰ�㣬Ŀ��㣬�����ϰ�������ϰ�������������������,�ϰ�Ӱ����룬�������������Ƕ�)
    %���㵥�������ϰ���ĳ���ʱ�ȷֱ����4���߶γ������߶εĳ���������ɵ�ĳ������㹫ʽͨ�������Ƶ�
    
    delta_goal_X = goal_position(1) - cur_position(1);
    delta_goal_Y = goal_position(2) - cur_position(2);
    rGoal = sqrt ( delta_goal_X ^ 2 + delta_goal_Y ^ 2 ) ;		%·�������յ�ľ���
    for j = 1:OBJ_NUM
        OBJ = obj( (j*4-3) : j*4 , 1 : 2 );    % objΪ���о����ϰ������е��(4*OBJ_NUM)��2�ж�ά���󣬴�ͷ��ÿ�ĸ����ʾһ�����Σ�OBJΪ4*2����
        line_s = OBJ;   %һ���������ĸ���㣬���½�Ϊ��һ������ʱ��Ϊ������
        line_e = [(OBJ(2 : 4 , 1 : 2)) ; (OBJ(1 : 1 , 1 : 2))];     %һ���������ĸ��յ㣬���½�Ϊ��һ������ʱ��Ϊ������
        for i = 1:4    %�ֱ����4���߶γ���
            %���㵥���߶β����ĳ�����x��y�����ϵķ���
            [Frer_line_x(i) ,Frer_line_y(i)] = compute_line_Repulsion(cur_position , line_s(i,:), line_e(i,:) ,REP,INF_DISTANCE , A ,rGoal ,theta_Goal);
        end
        Frer_obj_x(j) = sum(Frer_line_x) ;    %���ӵ����ϰ������б߽������X����ķ���
        Frer_obj_y(j) = sum(Frer_line_y) ;    %���ӵ����ϰ������б߽������Y����ķ���
    end
    Frer_X = sum(Frer_obj_x) ;    %���������ϰ��������X����ķ���
    Frer_Y = sum(Frer_obj_y) ;    %���������ϰ��������Y����ķ���