function Y = whether_Cross(cur_position ,goal_position ,obj ,OBJ_NUM)
    %判断当前点与终点连线是否与障碍物相交（(当前点，目标点，障碍物，障碍物个数)
for j = 1:OBJ_NUM
    OBJ = obj( (j*4-3) : j*4 , 1 : 2 );                         %第j个障碍物
    line_s = OBJ;                                               %第j个障碍物的起点
    line_e = [(OBJ(2 : 4 , 1 : 2)) ; (OBJ(1 : 1 , 1 : 2))];     %第j个障碍物的终点
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
            if (count(1) > 0) & (count(2) > 0)
                cross_line_Flag(i) = 1;
            else
                cross_line_Flag(i) = 0;
            end
        end
        cross_obj_Flag(j) = prod(cross_line_Flag);
    end
end
Y  = ~prod(cross_obj_Flag);

