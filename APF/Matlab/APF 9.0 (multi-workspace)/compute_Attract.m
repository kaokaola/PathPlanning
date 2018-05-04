function [Fat_X,Fat_Y] = compute_Attract(cur_position ,goal_position ,ATT ,theta_Goal) 
    %�����x��y�������������ǰλ�ã��յ�λ�ã�������������������Ƕȣ�
    
    delta_goal_X = goal_position(1) - cur_position(1);
    delta_goal_Y = goal_position(2) - cur_position(2);
    rGoal = sqrt ( delta_goal_X ^ 2 + delta_goal_Y ^ 2 );   %·�������յ�ľ��� 
    Fat_X = ATT * rGoal * cos(theta_Goal);	%�յ�����������X�᷽�����
    Fat_Y = ATT * rGoal * sin(theta_Goal);	%�յ�����������Y�᷽�����