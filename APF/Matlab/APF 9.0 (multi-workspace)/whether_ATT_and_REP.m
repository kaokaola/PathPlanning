function Y = whether_ATT_and_REP(ATT , REP , A , Po)
    %�ж���������ϵ���ͳ�������ϵ���Ƿ����Ҫ����������ϵ������������ϵ�� , ������������(A>0) �� �ϰ�Ӱ����룩
    
    r = 0.5;
    if A == 2
        Kn = ( 2 / (9 * Po^2) + 2 * r / (27 * Po^3) ) * sqrt (1 + 3 * Po / r) - 2 / (3 * Po^2) + 2 * r / (27 * Po^3) ;
    else
        Pm = 2 * r / (1 - A / 2 + sqrt( (1 - A / 2)^2 + 2 * A * r / Po ) ) ;
        Kn = ( 1 / Pm - 1 / Po ) * ((Po - r)^(A - 1)) / Po^2 ;
    end
    if (ATT / REP) > Kn
        Y = 1;
    else
        Y = 0;
    end

