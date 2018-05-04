function [Frer_X,Frer_Y]=compute_Repulsion(cur_position ,goal_position ,obj ,OBJ_NUM ,REP,INF_DISTANCE , A ,theta_Goal )     
    %�����ϰ���ĳ���(��ǰ�㣬Ŀ��㣬�ϰ���ϰ��������������������������Ƕȣ������Ƕȣ��ϰ���ϰ�����,�ϰ�Ӱ����룬��������)
    
delta_goal_X = goal_position(1) - cur_position(1);
delta_goal_Y = goal_position(2) - cur_position(2);
rGoal = sqrt ( delta_goal_X ^ 2 + delta_goal_Y ^ 2 ) ;		%·�������յ�ľ���
for j = 1:OBJ_NUM
    OBJ = obj( (j*4-3) : j*4 , 1 : 2 );
    line_s = OBJ;
    line_e = [(OBJ(2 : 4 , 1 : 2)) ; (OBJ(1 : 1 , 1 : 2))];
    for i = 1:4
        [hLine(i),rLine(i)] = distance_to_line(cur_position ,line_s(i,:), line_e(i,:));       %�������ǰ�㵽�ϰ���Ĵ�ֱ���루hLine������̾��루rLine��       
        if rLine(i) > INF_DISTANCE    %���ÿ���ϰ���·���ľ�������ϰ�Ӱ����룬����Ϊ0
              Frerx(i) = 0;
              Frery(i) = 0;
        else
            [theta_Line(i) ,theta1 ,theta2] = complete_theta(cur_position ,line_s(i,:), line_e(i,:));     %������߶���ϵͳx��ļнǣ��ͻ��ֵ������޽�
            if rLine(i) < ( INF_DISTANCE / 2 )    %���ÿ���ϰ���·���ľ���С���ϰ�Ӱ������1/2
                Krer1_1 = ( rGoal ^ A ) * REP / ( hLine(i) ^ 3) ;
                Krer1_2 = ( rGoal ^ A ) * REP / ( INF_DISTANCE * hLine(i)^2 ) ;
                Krer2 = A / 2 * REP * ( rGoal ^ abs(A - 1)) ;
            else         %���ÿ���ϰ���·���ľ������ϰ�Ӱ������1/2��1��֮��
                Krer1_1 = ( rGoal ^ 2 ) * REP / ( hLine(i) ^ 3) ;
                Krer1_2 = ( rGoal ^ 2 ) * REP / ( INF_DISTANCE * hLine(i)^2 ) ;
                Krer2 = REP * rGoal;
            end
            Krer2_1 = Krer2 / (hLine(i)^2);
            Krer2_2 = -2 * Krer2 / ( INF_DISTANCE * hLine(i) ) ;
            Krer2_3 = 1 / ( INF_DISTANCE^2 ) ;
            Frer1_temp_X1 = Krer1_1 /4 * (sin(theta1)^4 ) - Krer1_2 / 3 * ( sin(theta1)^3 ) ;      
            Frer1_temp_X2 = Krer1_1 /4 * (sin(theta2)^4 ) - Krer1_2 / 3 * ( sin(theta2)^3 ) ;
            Frer1_temp_X = Frer1_temp_X2 - Frer1_temp_X1;                          %���߶�������s��e��Ϊx������ʱ�������߶γ�����x�������
            Frer1_temp_Y1 = Krer1_1 * ( (-1/4) * (sin(theta1))^3 * cos(theta1) - 3/8 * sin(theta1) * cos(theta1) + 3/8 * theta1) + Krer1_2 /3 * ( (sin(theta1))^2 * cos(theta1) + 2 * cos(theta1) );
            Frer1_temp_Y2 = Krer1_1 * ( (-1/4) * (sin(theta2))^3 * cos(theta2) - 3/8 * sin(theta2) * cos(theta2) + 3/8 * theta2) + Krer1_2 /3 * ( (sin(theta2))^2 * cos(theta2) + 2 * cos(theta2) );
            Frer1_temp_Y = Frer1_temp_Y2 - Frer1_temp_Y1 ;                         %���߶�������s��e��Ϊx������ʱ�������߶γ�����y�������
            Frer1_x(i) =  Frer1_temp_X * cos ( theta_Line(i) ) - Frer1_temp_Y * sin ( theta_Line(i) );       %�����߶γ���Frer1��ϵͳ����ϵ��x�������
            Frer1_y(i) =  Frer1_temp_X * sin ( theta_Line(i) ) + Frer1_temp_Y * cos ( theta_Line(i) );       %�����߶γ���Frer1��ϵͳ����ϵ��y�������
            Frer2_1 = - Krer2_1 / 2 * ( sin(theta1) * cos(theta1) - theta1 ) + Krer2_2 * cos(theta1) + Krer2_3 * theta1;
            Frer2_2 = - Krer2_1 / 2 * ( sin(theta2) * cos(theta2) - theta2 ) + Krer2_2 * cos(theta2) + Krer2_3 * theta2;
            Frer2 = Frer2_2 - Frer2_1;                          %�����߶γ����ķ���Frer2
            Frer2_x(i) = Frer2 * cos(theta_Goal);               %�����߶γ���Frer2��ϵͳ����ϵ��x�������
            Frer2_y(i) = Frer2 * sin(theta_Goal);               %�����߶γ���Frer2��ϵͳ����ϵ��y�������
            Frerx(i) = Frer1_x(i) + Frer2_x(i);                       %�����߶��ܳ�����ϵͳ����ϵ��x�������
            Frery(i) = Frer1_y(i) + Frer2_y(i);                       %�����߶��ܳ�����ϵͳ����ϵ��y�������
        end
    end
    Frer_X_one(j) = sum(Frerx) ;    %���ӵ����ϰ������б߽������X����ķ���
    Frer_Y_one(j) = sum(Frery) ;    %���ӵ����ϰ������б߽������Y����ķ���
end
Frer_X = sum(Frer_X_one) ;    %���������ϰ��������X����ķ���
Frer_Y = sum(Frer_Y_one) ;    %���������ϰ��������Y����ķ���