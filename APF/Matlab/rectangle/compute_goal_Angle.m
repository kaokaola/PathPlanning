function Y = compute_obj_Angle(cur_position ,goal_position)
%���������Ƕȣ���ǰλ�ã�Ŀ��λ�ã�
delta_goal_X = goal_position(1) - cur_position(1);
delta_goal_Y = goal_position(2) - cur_position(2);
rGoal = sqrt ( delta_goal_X ^ 2 + delta_goal_Y ^ 2 ) ;		%·�������յ�ľ��� 
Y = sign ( delta_goal_Y ) * acos ( delta_goal_X / rGoal ) ;		%�����Ƕ�