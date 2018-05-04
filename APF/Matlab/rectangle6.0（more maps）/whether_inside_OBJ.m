function y = whether_inside_OBJ(point , obj )
    w = size(obj);
    h = w(1);
    for i =1:(h/4)
        if point(1) >= obj((i-1)*4+1,1) & point(1) <= obj((i-1)*4+3,1) & point(2) >= obj((i-1)*4+1,2) & point(2) <= obj((i-1)*4+3,2)
            flag(i) = 0;
        else
            flag(i) = 1;
        end
    end
    y = ~prod(flag);