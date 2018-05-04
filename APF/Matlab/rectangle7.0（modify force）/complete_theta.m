function [theta_Line ,theta1 ,theta2] = complete_theta(cur_position ,line_s, line_e)
    %计算线段与系统X轴的角度，以及积分的上下线角度 （当前点 ， 线段起点 ， 线段终点）
    opp_cur = sqrt ( ( line_e(1) - line_s(1) ) ^ 2 + ( line_e(2) - line_s(2) ) ^ 2 ) ;   %当前点对边长度，即障碍物线段长度
    if ( line_e(2) - line_s(2) ) == 0           %如果纵坐标相等
        if ( line_e(1) - line_s(1) ) >= 0
            theta_Line = 0;                     %终点在起点右边时
        else
            theta_Line = pi;                    %终点在起点左边时
        end
    else
        theta_Line = sign ( line_e(2) - line_s(2) ) * acos ( ( line_e(1) - line_s(1) ) / opp_cur ) ;	 %线段与系统X轴的角度
    end
    direction = det([line_s(1), line_e(1),cur_position(1);line_s(2), line_e(2),cur_position(2);1 1 1]);     %判断（line_s，line_e，cur_position）三点的方向，正数为逆时针，负数为顺时针
    opp_s = sqrt ( ( line_e(1) - cur_position(1) ) ^ 2 + ( line_e(2) - cur_position(2) ) ^ 2 ) ;     %线段起点对边长度，即当前点与线段终点连线长度
    opp_e = sqrt ( ( cur_position(1) - line_s(1) ) ^ 2 + ( cur_position(2) - line_s(2) ) ^ 2 ) ;     %线段终点对边长度，即当前点与线段起点连线长度
    theta1 = sign (  direction ) * acos ( ( opp_cur^2 + opp_e^2 - opp_s^2 ) / ( 2 * opp_e * opp_cur ) );   %起点角度，即积分下限角 
    theta2 = sign (  direction ) * ( pi - acos ( ( opp_cur^2  + opp_s^2 - opp_e^2) / ( 2 * opp_cur * opp_s ) ) );   %(pi-终点角度)，即积分上限角
