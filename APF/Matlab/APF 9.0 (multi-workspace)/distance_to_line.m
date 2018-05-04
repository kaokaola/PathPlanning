function [hline,rline] = distance_to_line(cur_position ,line_s, line_e)
    %����㵽�߶ε���̾������ ����ǰ�� ���߶���� ���߶��յ㣩
    
    opp_cur = sqrt ( ( line_e(1) - line_s(1) ) ^ 2 + ( line_e(2) - line_s(2) ) ^ 2 ) ;   %��ǰ��Ա߳��ȣ����ϰ����߶γ���		
    opp_s = sqrt ( ( line_e(1) - cur_position(1) ) ^ 2 + ( line_e(2) - cur_position(2) ) ^ 2 ) ;     %�߶����Ա߳��ȣ�����ǰ�����߶��յ����߳���
    opp_e = sqrt ( ( cur_position(1) - line_s(1) ) ^ 2 + ( cur_position(2) - line_s(2) ) ^ 2 ) ;     %�߶��յ�Ա߳��ȣ�����ǰ�����߶�������߳���
    face = abs((cur_position(1) - line_e(1)) * (line_s(2) - line_e(2)) - (cur_position(2) - line_e(2)) * (line_s(1) - line_e(1)));   
    hline = face / opp_cur;     %�Ե�ǰ��Ϊ���㣬�ϰ����߶�Ϊ�ױߵ������εĸ�
	if ( ( ( opp_cur^2 + opp_e^2 - opp_s^2 ) / ( 2 * opp_e * opp_cur  ) ) <= 0)       %����line_sΪ�۽ǣ�����̾���Ϊ��ǰ�����߶�������߳���
        rline = opp_e ;		
	else if ( ( ( opp_cur^2 + opp_s^2 - opp_e^2 ) / ( 2 * opp_s * opp_cur ) ) <= 0)   %����line_eΪ�۽ǣ�����̾���Ϊ��ǰ�����߶��յ����߳���
        rline = opp_s ;
        else                    %�����׽Ǿ�Ϊ��ǣ���̾��뼴Ϊ��ǰ��Ϊ����ĸ�
        rline = hline;
        end
    end
        
