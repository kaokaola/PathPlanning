function [Frer_X,Frer_Y] = compute_line_Repulsion(cur_position , line_s ,line_e ,REP,INF_DISTANCE , A ,rGoal ,theta_Goal)
    %���㵥���߶β����ĳ���,���յ���������ǰ�� ���߶���� ���߶��յ� ������������� ������Ӱ����� ���������� �����յ���� �������Ƕȣ�
    
    [hLine,rLine] = distance_to_line(cur_position ,line_s, line_e);       %�������ǰ�㵽�ϰ���Ĵ�ֱ���루hLine������̾��루rLine��
    if rLine > INF_DISTANCE    %���ÿ���ϰ���·���ľ�������ϰ�Ӱ����룬����Ϊ0
        Frer_X = 0;
        Frer_Y = 0;
    else
        [theta_Line ,theta1 ,theta2] = complete_theta(cur_position ,line_s, line_e);     %������߶���ϵͳx��ļнǣ��ͻ��ֵ������޽�
        if (theta1 == 0) | (theta1 > 3.1415926) |  (theta1 < -3.1415926)  %����ǰ�����߶ι���ʱ��ֻ�����뵱ǰ��Ͻ������߶ζ˵��������ĳ���
            distance_c_s = sqrt((cur_position(1) - line_s(1)) ^ 2 + (cur_position(2) - line_s(2)) ^ 2);     %��ǰ�����߶�������
            distance_c_e = sqrt((cur_position(1) - line_e(1)) ^ 2 + (cur_position(2) - line_e(2)) ^ 2);     %��ǰ�����߶��յ����
            if distance_c_s >= distance_c_e         %�������ʱ����
                distance = distance_c_e; 
                theta_Obj = theta_Line;
            else
                distance = distance_c_s;
                theta_Obj = theta_Line + pi;
            end
            if distance < ( INF_DISTANCE / 2 )           %���ÿ���ϰ��߶ζ˵��·����ľ���С���ϰ�Ӱ������1/2
                Frer1_line = REP * ( 1 / distance - 1 / INF_DISTANCE ) * ( 1 / ( distance ^ 2 ) ) * ( rGoal ^ A ) ;
                Frer2_line = A * REP * ( ( 1 / distance - 1 / INF_DISTANCE ) ^ 2 ) * ( rGoal ^ ( 1 - A ) ) / 2 ;
                Frer1_line_x = Frer1_line * cos ( theta_Obj ) ;
                Frer1_line_y = Frer1_line * sin ( theta_Obj ) ;
            else                                         %���ÿ���ϰ��߶ζ˵��·����ľ������ϰ�Ӱ������1/2��1֮��
                Frer1_line = REP * ( 1 / distance - 1 / INF_DISTANCE ) * ( 1 / ( distance ^ 2 ) ) * ( rGoal ^ 2 ) ;
                Frer2_line = REP * ( ( 1 / distance - 1 / INF_DISTANCE ) ^ 2 ) * rGoal ;
                Frer1_line_x = Frer1_line * cos ( theta_Obj ) ;
                Frer1_line_y = Frer1_line * sin ( theta_Obj ) ;
            end
        else    %��ǰ�����߶β�����ʱ
            if rLine < ( INF_DISTANCE / 2 )    %���ÿ���ϰ���·���ľ���С���ϰ�Ӱ������1/2
                Krer1_1 = ( rGoal ^ A ) * REP / ( hLine ^ 3) ;    %Krer1_1 ... Krer2_3Ϊ�������ʱ���ֹ�ʽ������һЩϵ��
                Krer1_2 = ( rGoal ^ A ) * REP / ( INF_DISTANCE * hLine^2 ) ;
                Krer2 = A / 2 * REP * ( rGoal ^ abs(A - 1)) ;
            else         %���ÿ���ϰ���·���ľ������ϰ�Ӱ������1/2��1��֮��
                Krer1_1 = ( rGoal ^ 2 ) * REP / ( hLine ^ 3) ;
                Krer1_2 = ( rGoal ^ 2 ) * REP / ( INF_DISTANCE * hLine^2 ) ;
                Krer2 = REP * rGoal;
            end
            Krer2_1 = Krer2 / (hLine^2);
            Krer2_2 = -2 * Krer2 / ( INF_DISTANCE * hLine ) ;
            Krer2_3 = 1 / ( INF_DISTANCE^2 ) ;
            Frer1_line_temp_X1 = Krer1_1 /4 * (sin(theta1)^4 ) - Krer1_2 / 3 * ( sin(theta1)^3 ) ;      
            Frer1_line_temp_X2 = Krer1_1 /4 * (sin(theta2)^4 ) - Krer1_2 / 3 * ( sin(theta2)^3 ) ;
            Frer1_line_temp_X = Frer1_line_temp_X2 - Frer1_line_temp_X1;                          %���߶�������s��e��Ϊx������ʱ�������߶γ�����x�������
            Frer1_line_temp_Y1 = Krer1_1 * ( (-1/4) * (sin(theta1))^3 * cos(theta1) - 3/8 * sin(theta1) * cos(theta1) + 3/8 * theta1) + Krer1_2 /3 * ( (sin(theta1))^2 * cos(theta1) + 2 * cos(theta1) );
            Frer1_line_temp_Y2 = Krer1_1 * ( (-1/4) * (sin(theta2))^3 * cos(theta2) - 3/8 * sin(theta2) * cos(theta2) + 3/8 * theta2) + Krer1_2 /3 * ( (sin(theta2))^2 * cos(theta2) + 2 * cos(theta2) );
            Frer1_line_temp_Y = Frer1_line_temp_Y2 - Frer1_line_temp_Y1 ;                         %���߶�������s��e��Ϊx������ʱ�������߶γ�����y�������
            Frer1_line_x =  Frer1_line_temp_X * cos ( theta_Line ) - Frer1_line_temp_Y * sin ( theta_Line );       %�����߶γ���Frer1_line��ϵͳ����ϵ��x�������
            Frer1_line_y =  Frer1_line_temp_X * sin ( theta_Line ) + Frer1_line_temp_Y * cos ( theta_Line );       %�����߶γ���Frer1_line��ϵͳ����ϵ��y�������
            Frer2_line_1 = - Krer2_1 / 2 * ( sin(theta1) * cos(theta1) - theta1 ) + Krer2_2 * cos(theta1) + Krer2_3 * theta1;
            Frer2_line_2 = - Krer2_1 / 2 * ( sin(theta2) * cos(theta2) - theta2 ) + Krer2_2 * cos(theta2) + Krer2_3 * theta2;
            Frer2_line = Frer2_line_2 - Frer2_line_1;    %�����߶γ����ķ���Frer2_line
        end
        Frer2_line_x = Frer2_line * cos(theta_Goal);  %�����߶γ���Frer2_line��ϵͳ����ϵ��x�������
        Frer2_line_y = Frer2_line * sin(theta_Goal);  %�����߶γ���Frer2_line��ϵͳ����ϵ��y�������
        Frer_X = Frer1_line_x + Frer2_line_x;    %�����߶��ܳ�����ϵͳ����ϵ��x�������
        Frer_Y = Frer1_line_y + Frer2_line_y;    %�����߶��ܳ�����ϵͳ����ϵ��y�������
    end