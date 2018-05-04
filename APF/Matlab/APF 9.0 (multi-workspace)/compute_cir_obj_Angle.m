function Y = compute_cir_obj_Angle(cur_Position ,OBJ,OBJ_NUM)
    %���㵱ǰ�㵽Բ���ϰ�������������ϵͳX��ļн�
    
    for i=1:OBJ_NUM
        delta_obj_X = OBJ(i,1) - cur_Position(1);
        delta_obj_Y = OBJ(i,2) - cur_Position(2);
        rObj = sqrt(delta_obj_X^2 + delta_obj_Y^2);    %ǰ�㵽Բ���ϰ������ĵľ���
        if delta_obj_Y == 0          
            if delta_goal_X >= 0
                Y(i) = 0;                    
            else
                Y(i) = pi;                   
            end
        else
            Y(i) = sign ( delta_obj_Y ) * acos ( delta_obj_X / rObj );
        end
    end