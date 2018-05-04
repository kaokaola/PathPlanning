function [Frer_X,Frer_Y]=compute_Repulsion(cur_position ,goal_position ,obj ,OBJ_NUM ,REP,INF_DISTANCE , A)     
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
            else         %���ÿ���ϰ���·���ľ������ϰ�Ӱ������1/2��1��֮��
                Krer1_1 = ( rGoal ^ 2 ) * REP / ( hLine(i) ^ 3) ;
                Krer1_2 = ( rGoal ^ 2 ) * REP / ( INF_DISTANCE * hLine(i)^2 ) ;
            end
            F_temp_X1 = Krer1_1 /4 * (sin(theta1)^4 ) - Krer1_2 / 3 * ( sin(theta1)^3 ) ;      
            F_temp_X2 = Krer1_1 /4 * (sin(theta2)^4 ) - Krer1_2 / 3 * ( sin(theta2)^3 ) ;
            F_temp_X = F_temp_X2 - F_temp_X1;                          %���߶�������s��e��Ϊx������ʱ�������߶γ�����x�������
            F_temp_Y1 = Krer1_1 * ( (-1/4) * (sin(theta1))^3 * cos(theta1) - 3/8 * sin(theta1) * cos(theta1) + 3/8 * theta1) + Krer1_2 /3 * ( (sin(theta1))^2 * cos(theta1) + 2 * cos(theta1) );
            F_temp_Y2 = Krer1_1 * ( (-1/4) * (sin(theta2))^3 * cos(theta2) - 3/8 * sin(theta2) * cos(theta2) + 3/8 * theta2) + Krer1_2 /3 * ( (sin(theta2))^2 * cos(theta2) + 2 * cos(theta2) );
            F_temp_Y = F_temp_Y2 - F_temp_Y1 ;                         %���߶�������s��e��Ϊx������ʱ�������߶γ�����y�������
            Frerx1(i) =  F_temp_X * cos ( theta_Line(i) ) - F_temp_Y * sin ( theta_Line(i) );       %�����߶γ���Frer1��ϵͳ����ϵ��x�������
            Frery1(i) =  F_temp_X * sin ( theta_Line(i) ) + F_temp_Y * cos ( theta_Line(i) );       %�����߶γ���Frer1��ϵͳ����ϵ��y�������
        end
    end
    Frer_X_1(j) = sum(Frerx) ;    %���ӵ����ϰ������б߽������X����ķ���
    Frer_Y_1(j) = sum(Frery) ;    %���ӵ����ϰ������б߽������Y����ķ���
end
Frer_X = sum(Frer_X_1) ;    %���������ϰ��������X����ķ���
Frer_Y = sum(Frer_Y_1) ;    %���������ϰ��������Y����ķ���