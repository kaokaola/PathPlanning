function [theta_Line ,theta1 ,theta2] = complete_theta(cur_position ,line_s, line_e)
    %�����߶���ϵͳX��ĽǶȣ��Լ����ֵ������߽Ƕ� ����ǰ�� �� �߶���� �� �߶��յ㣩
    opp_cur = sqrt ( ( line_e(1) - line_s(1) ) ^ 2 + ( line_e(2) - line_s(2) ) ^ 2 ) ;   %��ǰ��Ա߳��ȣ����ϰ����߶γ���
    if ( line_e(2) - line_s(2) ) == 0           %������������
        if ( line_e(1) - line_s(1) ) >= 0
            theta_Line = 0;                     %�յ�������ұ�ʱ
        else
            theta_Line = pi;                    %�յ���������ʱ
        end
    else
        theta_Line = sign ( line_e(2) - line_s(2) ) * acos ( ( line_e(1) - line_s(1) ) / opp_cur ) ;	 %�߶���ϵͳX��ĽǶ�
    end
    direction = det([line_s(1), line_e(1),cur_position(1);line_s(2), line_e(2),cur_position(2);1 1 1]);     %�жϣ�line_s��line_e��cur_position������ķ�������Ϊ��ʱ�룬����Ϊ˳ʱ��
    opp_s = sqrt ( ( line_e(1) - cur_position(1) ) ^ 2 + ( line_e(2) - cur_position(2) ) ^ 2 ) ;     %�߶����Ա߳��ȣ�����ǰ�����߶��յ����߳���
    opp_e = sqrt ( ( cur_position(1) - line_s(1) ) ^ 2 + ( cur_position(2) - line_s(2) ) ^ 2 ) ;     %�߶��յ�Ա߳��ȣ�����ǰ�����߶�������߳���
    theta1 = sign (  direction ) * acos ( ( opp_cur^2 + opp_e^2 - opp_s^2 ) / ( 2 * opp_e * opp_cur ) );   %���Ƕȣ����������޽� 
    theta2 = sign (  direction ) * ( pi - acos ( ( opp_cur^2  + opp_s^2 - opp_e^2) / ( 2 * opp_cur * opp_s ) ) );   %(pi-�յ�Ƕ�)�����������޽�
