function [hline,rline] = distance_to_line(cur_position ,line_s, line_e)
    %计算点到线段的最短距离距离 （当前点 ，线段起点 ，线段终点）
    
    opp_cur = sqrt ( ( line_e(1) - line_s(1) ) ^ 2 + ( line_e(2) - line_s(2) ) ^ 2 ) ;   %当前点对边长度，即障碍物线段长度		
    opp_s = sqrt ( ( line_e(1) - cur_position(1) ) ^ 2 + ( line_e(2) - cur_position(2) ) ^ 2 ) ;     %线段起点对边长度，即当前点与线段终点连线长度
    opp_e = sqrt ( ( cur_position(1) - line_s(1) ) ^ 2 + ( cur_position(2) - line_s(2) ) ^ 2 ) ;     %线段终点对边长度，即当前点与线段起点连线长度
    face = abs((cur_position(1) - line_e(1)) * (line_s(2) - line_e(2)) - (cur_position(2) - line_e(2)) * (line_s(1) - line_e(1)));   
    hline = face / opp_cur;     %以当前点为顶点，障碍物线段为底边的三角形的高
	if ( ( ( opp_cur^2 + opp_e^2 - opp_s^2 ) / ( 2 * opp_e * opp_cur  ) ) <= 0)       %若角line_s为钝角，则最短距离为当前点与线段起点连线长度
        rline = opp_e ;		
	else if ( ( ( opp_cur^2 + opp_s^2 - opp_e^2 ) / ( 2 * opp_s * opp_cur ) ) <= 0)   %若角line_e为钝角，则最短距离为当前点与线段终点连线长度
        rline = opp_s ;
        else                    %两个底角均为锐角，最短距离即为当前点为顶点的高
        rline = hline;
        end
    end
        
