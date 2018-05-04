function [Frer_X,Frer_Y]=compute_border_Repulsion(cur_position ,goal_position ,border ,REP,INF_DISTANCE , A)     
    %����ĳ���(��ǰ�㣬Ŀ��㣬�߽磬������������������Ƕȣ������Ƕȣ��ϰ���ϰ�����,�ϰ�Ӱ����룬��������)
    delta_goal_X = goal_position(1) - cur_position(1);
    delta_goal_Y = goal_position(2) - cur_position(2);
    rGoal = sqrt ( delta_goal_X ^ 2 + delta_goal_Y ^ 2 ) ;		%·�������յ�ľ���
    rLine = [ cur_position(2)  (100 - cur_position(1)) (100 - cur_position(2))  cur_position(1)];   %��ǰ���������߽�ľ���
    hLine = rLine;
    line_s = border;
    line_e = [(border(2 : 4 , 1 : 2)) ; (border(1 : 1 , 1 : 2))];
    for i = 1:4
        if rLine(i) > INF_DISTANCE    %���ÿ���ϰ���·���ľ�������ϰ�Ӱ����룬����Ϊ0
              Frerx(i) = 0;
              Frery(i) = 0;
        else
            [theta_Line(i) ,theta1 ,theta2] = complete_theta(cur_position ,line_s(i,:), line_e(i,:));     %������߶���ϵͳx��ļнǣ��ͻ��ֵ������޽�
            if rLine(i) < ( INF_DISTANCE / 2 )    %���ÿ���ϰ���·���ľ���С���ϰ�Ӱ������1/2
                K1 = ( rGoal ^ A ) * REP / ( hLine(i) ^ 3) ;
                K2 = ( rGoal ^ A ) * REP / ( INF_DISTANCE * hLine(i)^2 ) ;
            else         %���ÿ���ϰ���·���ľ������ϰ�Ӱ������1/2��1��֮��
                K1 = ( rGoal ^ 2 ) * REP / ( hLine(i) ^ 3) ;
                K2 = ( rGoal ^ 2 ) * REP / ( INF_DISTANCE * hLine(i)^2 ) ;
            end
            F_temp_X1 = K1 /4 * (sin(theta1)^4 ) - K2 / 3 * ( sin(theta1)^3 ) ;      
            F_temp_X2 = K1 /4 * (sin(theta2)^4 ) - K2 / 3 * ( sin(theta2)^3 ) ;
            F_temp_X = F_temp_X2 - F_temp_X1;                          %���߶�������s��e��Ϊx������ʱ�������߶γ�����x�������
            F_temp_Y1 = K1 * ( (-1/4) * (sin(theta1))^3 * cos(theta1) - 3/8 * sin(theta1) * cos(theta1) + 3/8 * theta1) + K2 /3 * ( (sin(theta1))^2 * cos(theta1) + 2 * cos(theta1) );
            F_temp_Y2 = K1 * ( (-1/4) * (sin(theta2))^3 * cos(theta2) - 3/8 * sin(theta2) * cos(theta2) + 3/8 * theta2) + K2 /3 * ( (sin(theta2))^2 * cos(theta2) + 2 * cos(theta2) );
            F_temp_Y = F_temp_Y2 - F_temp_Y1 ;                         %���߶�������s��e��Ϊx������ʱ�������߶γ�����y�������
            Frerx(i) =  F_temp_X * cos ( theta_Line(i) ) - F_temp_Y * sin ( theta_Line(i) );       %�����߶γ�����ϵͳ����ϵ��x�������
            Frery(i) =  F_temp_X * sin ( theta_Line(i) ) + F_temp_Y * cos ( theta_Line(i) );       %�����߶γ�����ϵͳ����ϵ��y�������
        end
    end
    Frer_X = sum(Frerx) ;    %���������ϰ��������X����ķ���
    Frer_Y = sum(Frery) ;    %���������ϰ��������Y����ķ���