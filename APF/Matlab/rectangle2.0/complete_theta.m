function [theta_Line ,theta1 ,theta2] = complete_theta(cur_position ,line_s, line_e)
    %计算线段与系统X轴的角度，以及积分的上下线角度 （当前点 ， 线段起点 ， 线段终点）
    opp_cur = sqrt ( ( line_e(1) - line_s(1) ) ^ 2 + ( line_e(2) - line_s(2) ) ^ 2 ) ;  %当前点对边长度，即障碍物线段长度		
    theta_Line = sign ( line_e(2) - line_s(2) ) * acos ( ( line_e(1) - line_s(1) ) / opp_cur ) ;	 %线段与系统X轴的角度
    opp_s = sqrt ( ( line_e(1) - cur_position(1) ) ^ 2 + ( line_e(2) - cur_position(2) ) ^ 2 ) ;    %线段起点对边长度，即当前点与线段终点连线长度
    opp_e = sqrt ( ( cur_position(1) - line_s(1) ) ^ 2 + ( cur_position(2) - line_s(2) ) ^ 2 ) ;    %线段终点对边长度，即当前点与线段起点连线长度
    theta1 = acos ( ( opp_cur * opp_cur - opp_s * opp_s + opp_e * opp_e ) / ( 2 * opp_e * opp_cur ) );
    theta2 = acos ( ( opp_cur * opp_cur + opp_s * opp_s - opp_e * opp_e ) / ( 2 * opp_cur * opp_s ) );