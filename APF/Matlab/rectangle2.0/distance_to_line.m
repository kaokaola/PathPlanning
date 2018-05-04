function [hline,rline] = distance_to_line(cur_position ,line_s, line_e)
    face = abs((cur_position(1) - line_e(1)) * (line_s(2) - line_e(2)) - (cur_position(2) - line_e(2)) * (line_s(1) - line_e(1)));
    delta_X = line_e(1) - line_s(1);
    delta_Y = line_e(2) - line_s(2);
    line_length = sqrt ( delta_X ^ 2 + delta_Y ^ 2 ) ;    
    hline = face / line_length;
	if ((cur_position(1) - line_s(1)) * (line_e(2) - line_s(2)) - (cur_position(2) - line_s(2)) * (line_e(1) - line_s(1)) <= 0)
		delta_line_X = cur_position(1) - line_s(1);
        delta_line_Y = cur_position(2) - line_s(2);
        rline = sqrt ( delta_line_X ^ 2 + delta_line_Y ^ 2 ) ;		%路径点与障碍物边线的距离
	else if ((cur_position(1) - line_e(1)) * (line_s(1) - line_e(1)) + (cur_position(2) - line_e(2)) * (line_s(2) - line_e(2)) <= 0)
		delta_line_X = cur_position(1) - line_e(1);
        delta_line_Y = cur_position(2) - line_e(2);
        rline = sqrt ( delta_line_X ^ 2 + delta_line_Y ^ 2 ) ;
    else
        rline = hline;
        end
    end
        
