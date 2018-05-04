function [Frer_X,Frer_Y]=compute_Repulsion(cur_position ,goal_position ,OBJ ,REP, theta_Goal ,theta_Obj, OBJ_NUM ,INF_DISTANCE , A)     
    %�����ϰ���ĳ���(��ǰ�㣬Ŀ��㣬�ϰ��������������������Ƕȣ������Ƕȣ��ϰ���ϰ�����,�ϰ�Ӱ����룬��������)
delta_goal_X = goal_position(1) - cur_position(1);
delta_goal_Y = goal_position(2) - cur_position(2);
rGoal = sqrt ( delta_goal_X ^ 2 + delta_goal_Y ^ 2 ) ;		%·�������յ�ľ��� 
for i=1:OBJ_NUM
    delta_obj_X(i) = OBJ(i,1) - cur_position(1);
    delta_obj_Y(i) = OBJ(i,2) - cur_position(2);
    rObj(i) = sqrt(delta_obj_X(i)^2 + delta_obj_Y(i)^2);     %·������ϰ��ľ��뱣��������rre��
    if rObj(i) > INF_DISTANCE    %���ÿ���ϰ���·���ľ�������ϰ�Ӱ����룬����Ϊ0
          Frerx(i) = 0;
          Frery(i) = 0;
          Fatax(i) = 0;
          Fatay(i) = 0;
    else
        if rObj(i) < ( INF_DISTANCE / 2 )    %���ÿ���ϰ���·���ľ���С���ϰ�Ӱ������1/2
            Frer(i) = REP * ( 1 / rObj(i) - 1 / INF_DISTANCE ) * ( 1 / ( rObj(i) ^ 2 ) ) * ( rGoal ^ A );
            Fata(i) = A * REP * ( ( 1 / rObj (i) - 1 / INF_DISTANCE) ^ 2 ) * ( rGoal ^ ( 1 - A ) ) / 2;
            Frerx(i) = ( 1 + 0 ) * Frer(i) * cos ( theta_Obj(i) + pi );       %�����ϰ���ĳ�����X�������
            Frery(i) = ( 1 - 0 ) * Frer(i) * sin (theta_Obj(i) + pi );      %�����ϰ���ĳ�����Y�������
            Fatax(i) = Fata(i) * cos ( theta_Goal ) ;       %��X������Ŀ���Գ���������
            Fatay(i) = Fata(i) * sin ( theta_Goal ) ;       %��Y������Ŀ���Գ���������
        else         %���ÿ���ϰ���·���ľ������ϰ�Ӱ������1/2��1��֮��
            Frer(i) = REP * ( 1 / rObj(i) - 1 / INF_DISTANCE ) * ( 1 / ( rObj(i) ^ 2 ) ) * (rGoal^2) ;
            Fata(i) = REP * ( ( 1 / rObj(i) - 1 / INF_DISTANCE ) ^ 2 ) * rGoal ;
            Frerx(i) = Frer(i) * cos ( theta_Obj(i) + pi ) ;         %�����ϰ���ĳ�����X�������
            Frery(i) = Frer(i) * sin ( theta_Obj(i) + pi ) ;         %�����ϰ���ĳ�����Y�������
            Fatax(i) = Fata(i) * cos ( theta_Goal ) ;       %��X������Ŀ���Գ���������
            Fatay(i) = Fata(i) * sin ( theta_Goal )         %��Y������Ŀ���Գ���������
        end
    end
end
   Frer_X = sum(Frerx) + sum(Fatax);    %���������ϰ��������X����ķ���
   Frer_Y = sum(Frery) + sum(Fatay);    %���������ϰ��������Y����ķ���