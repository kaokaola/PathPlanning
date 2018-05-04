function [Frer_X,Frer_Y] = compute_cir_Repulsion(cur_position ,goal_position ,OBJ ,OBJ_NUM ,REP,INF_DISTANCE , A ,theta_Goal )     
    %����Բ���ϰ���ĳ���(��ǰ�㣬Ŀ��㣬�ϰ���ϰ����������������������ϰ�Ӱ����룬�������� ���յ�Ƕ�)
    %����Բ���ϰ��ֻ����յ����ϰ�����������������
    
    delta_goal_X = goal_position(1) - cur_position(1);
    delta_goal_Y = goal_position(2) - cur_position(2);
    rGoal = sqrt ( delta_goal_X ^ 2 + delta_goal_Y ^ 2 ) ;		%·�������յ�ľ���
    theta_Obj = compute_cir_obj_Angle(cur_position, OBJ ,OBJ_NUM);    %���㵱ǰ�㵽Բ���ϰ�������������ϵͳX��ļн�
    for i=1:OBJ_NUM
        delta_obj_X(i) = OBJ(i,1) - cur_position(1);
        delta_obj_Y(i) = OBJ(i,2) - cur_position(2);
        rObj(i) = sqrt(delta_obj_X(i)^2 + delta_obj_Y(i)^2) - OBJ(i,3);     %·������ϰ��ľ��뱣��������rre��
        if rObj(i) > INF_DISTANCE    %���ÿ���ϰ��͵�ǰ��ľ�������ϰ�Ӱ����룬����Ϊ0
              Frer1_x(i) = 0;
              Frer1_y(i) = 0;
              Frer2_x(i) = 0;
              Frer2_y(i) = 0;
        else
            if rObj(i) < ( INF_DISTANCE / 2 )    %���ÿ���ϰ���·���ľ���С���ϰ�Ӱ������1/2
                %�����ϰ���ĳ���Frer1,��ԭʼ������ʽ��������
                Frer1(i) = REP * ( 1 / rObj(i) - 1 / INF_DISTANCE ) * ( 1 / ( rObj(i) ^ 2 ) ) * ( rGoal ^ A );
                %�����ϰ���ĳ���Frer2,Ϊ����յ㲻�ɴ���������룬�ο� S. S. Ge and Y. J. Cui, "New potential functions for mobile robot 
                % path planning," in IEEE Transactions on Robotics and Automation, vol. 16, no. 5, pp. 615-620, Oct 2000.
                Frer2(i) = A * REP * ( ( 1 / rObj (i) - 1 / INF_DISTANCE) ^ 2 ) * ( rGoal ^ ( 1 - A ) ) / 2;
                Frer1_x(i) = ( 1 + 0 ) * Frer1(i) * cos ( theta_Obj(i) + pi );      %�����ϰ���ĳ���Frer1��X�������
                Frer1_y(i) = ( 1 - 0 ) * Frer1(i) * sin (theta_Obj(i) + pi );      %�����ϰ���ĳ���Frer1��Y�������
                Frer2_x(i) = Frer2(i) * cos ( theta_Goal ) ;       %����Frer2��X������Ŀ���Գ���������
                Frer2_y(i) = Frer2(i) * sin ( theta_Goal ) ;       %����Frer2��Y������Ŀ���Գ���������
            else         %���ÿ���ϰ���·���ľ������ϰ�Ӱ������1/2��1��֮��
                Frer1(i) = REP * ( 1 / rObj(i) - 1 / INF_DISTANCE ) * ( 1 / ( rObj(i) ^ 2 ) ) * (rGoal^2) ;
                Frer2(i) = REP * ( ( 1 / rObj(i) - 1 / INF_DISTANCE ) ^ 2 ) * rGoal ;
                Frer1_x(i) = Frer1(i) * cos ( theta_Obj(i) + pi ) ;        
                Frer1_y(i) = Frer1(i) * sin ( theta_Obj(i) + pi ) ;        
                Frer2_x(i) = Frer2(i) * cos ( theta_Goal ) ;       
                Frer2_y(i) = Frer2(i) * sin ( theta_Goal ) ;        
            end
        end
    end
    Frer_X = sum(Frer1_x) + sum(Frer2_x);    %���������ϰ��������X����ķ���
    Frer_Y = sum(Frer1_y) + sum(Frer2_y);    %���������ϰ��������Y����ķ���
