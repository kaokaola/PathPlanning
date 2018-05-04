function Y = whether_Cross(cur_position ,goal_position ,obj ,OBJ_NUM)
    %�жϵ�ǰ�����յ������Ƿ����ϰ����ཻ��(��ǰ�㣬Ŀ��㣬�ϰ���ϰ������)
    
    for j = 1:OBJ_NUM
        OBJ = obj( (j*4-3) : j*4 , 1 : 2 );                         %��j���ϰ���
        line_s = OBJ;                                               %��j���ϰ�������
        line_e = [(OBJ(2 : 4 , 1 : 2)) ; (OBJ(1 : 1 , 1 : 2))];     %��j���ϰ�����յ�
        for i = 1:4
            A = cur_position;
            B = goal_position;
            C = line_s(i,:);
            D = line_e(i,:);
            AC = C - A;
            CB = B - C;
            BD = D - B;
            DA = A - D;
            FLAG = [det([AC ; CB]) , det([CB ; BD]) , det([BD ; DA]) , det([DA ; AC])];
            count = [0 0];
            for k = 1:4
                if FLAG(k) > 0
                    count(1) = count(1) + 1;
                end
                if FLAG(k) < 0
                    count(2) = count(2) + 1;
                end
            end
            if (count(1) > 0) & (count(2) > 0)
                Y = 1;
                break
            else
                Y = 0;
            end
        end
        if Y == 1
            break
        end
    end

