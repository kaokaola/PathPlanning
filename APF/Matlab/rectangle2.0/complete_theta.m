function [theta_Line ,theta1 ,theta2] = complete_theta(cur_position ,line_s, line_e)
    %�����߶���ϵͳX��ĽǶȣ��Լ����ֵ������߽Ƕ� ����ǰ�� �� �߶���� �� �߶��յ㣩
    opp_cur = sqrt ( ( line_e(1) - line_s(1) ) ^ 2 + ( line_e(2) - line_s(2) ) ^ 2 ) ;  %��ǰ��Ա߳��ȣ����ϰ����߶γ���		
    theta_Line = sign ( line_e(2) - line_s(2) ) * acos ( ( line_e(1) - line_s(1) ) / opp_cur ) ;	 %�߶���ϵͳX��ĽǶ�
    opp_s = sqrt ( ( line_e(1) - cur_position(1) ) ^ 2 + ( line_e(2) - cur_position(2) ) ^ 2 ) ;    %�߶����Ա߳��ȣ�����ǰ�����߶��յ����߳���
    opp_e = sqrt ( ( cur_position(1) - line_s(1) ) ^ 2 + ( cur_position(2) - line_s(2) ) ^ 2 ) ;    %�߶��յ�Ա߳��ȣ�����ǰ�����߶�������߳���
    theta1 = acos ( ( opp_cur * opp_cur - opp_s * opp_s + opp_e * opp_e ) / ( 2 * opp_e * opp_cur ) );
    theta2 = acos ( ( opp_cur * opp_cur + opp_s * opp_s - opp_e * opp_e ) / ( 2 * opp_cur * opp_s ) );