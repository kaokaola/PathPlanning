function Y = whether_Cross(cur_position ,goal_position ,obj ,OBJ_NUM)
    %�жϵ�ǰ�����յ������Ƿ����ϰ����ཻ��(��ǰ�㣬Ŀ��㣬�ϰ���ϰ������)
for j = 1:OBJ_NUM
    OBJ = obj( (j*4-3) : j*4 , 1 : 2 );                         %��j���ϰ���
    line_s = OBJ;                                               %��j���ϰ�������
    line_e = [(OBJ(2 : 4 , 1 : 2)) ; (OBJ(1 : 1 , 1 : 2))];     %��j���ϰ�����յ�
    for i = 1:4
        delta = det([(goal_position(1)-cur_position(1)) (line_s(1)-line_e(1));(goal_position(2)-cur_position(2)) (line_s(2)-line_e(2))]);  
        if ( delta <= (1e-6) & delta >= -(1e-6) )  % delta=0����ʾ���߶��غϻ�ƽ��   
            cross_Flag1(i) =  0; 
        else
            namenda = det([(line_s(1)-cur_position(1)) (line_s(1)-line_e(1)) ; (line_s(2)-cur_position(2)) (line_s(2)-line_e(2))]) / delta;  
            if ( namenda>1 | namenda<0 )           
                cross_Flag1(i) =  0;
            else
                miu = det([(goal_position(1)-cur_position(1)) (line_s(1)-cur_position(1)) ; (goal_position(2)-cur_position(2)) (line_s(2)-cur_position(2))]) / delta;  
                if ( miu>1 | miu<0 )   
                    cross_Flag1(i) =  0;
                else
                    cross_Flag1(i) =  1;
                end
            end
        end
    end
    cross_Flag2(j) = prob(cross_Flag1);
end
Y  = prob(cross_Flag2);

