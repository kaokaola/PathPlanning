function y = whether_inside_OBJ(point ,cir_obj ,tri_obj ,rec_obj ,cir_OBJ_NUM ,tri_OBJ_NUM ,rec_OBJ_NUM)
    %判断一个点是否在一些区域内（点坐标 ，圆形区域 ，三角区域 ，矩形区域 ，圆形区域数量 ，三角区域数量 ，矩形区域数量）
    %只要在某一个区域内就返回1
    
    y = 0;
    if cir_OBJ_NUM > 0
        for i =1:cir_OBJ_NUM
            dis = sqrt((point(1) - cir_obj(i, 1))^2 + (point(2) - cir_obj(i, 2))^2);    %点到圆形障碍物圆心距离
            if dis <= cir_obj(i ,3)    % cir_obj(i ,3)为圆形障碍物半径
                y = 1;
                break
            else
                y = 0;
            end
        end
    end
    if (tri_OBJ_NUM > 0) & (y == 0) 
        for i =1:tri_OBJ_NUM
            xv = [tri_obj((3*i-2) ,1) ,tri_obj((3*i-1) ,1) ,tri_obj((3*i) ,1) ,tri_obj((3*i-2) ,1)];
            yv = [tri_obj((3*i-2) ,2) ,tri_obj((3*i-1) ,2) ,tri_obj((3*i) ,2) ,tri_obj((3*i-2) ,2)];
            y = inpolygon(point(1) ,point(2) ,xv ,yv);    %inpolygon函数判断一个点是否在多边形区域内
            if y == 1
                break
            end
        end
    end
    if(rec_OBJ_NUM > 0) & (y == 0)
        for i =1:tri_OBJ_NUM
            xv = [rec_obj((4*i-3) ,1) ,rec_obj((4*i-2) ,1) ,rec_obj((4*i-1) ,1) ,rec_obj((4*i) ,1) ,rec_obj((4*i-3) ,1)];
            yv = [rec_obj((4*i-3) ,2) ,rec_obj((4*i-2) ,2) ,rec_obj((4*i-1) ,2) ,rec_obj((4*i) ,2) ,rec_obj((4*i-3) ,2)];
            y = inpolygon(point(1) ,point(2) ,xv ,yv);
            if y == 1
                break
            end
        end
    end