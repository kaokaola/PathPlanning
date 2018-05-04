%������ʼ��
SG_NUM = 500;           %������ɵ������յ��������Ϊ1�����ͼ
MAX_CYCLE = 500;        %����·������ѭ������
ATT = 1.5;              %��������ʱ����ϵ��
REP = 0.5;              %�������ʱ����ϵ��
A = 2;                  %�������ʱ��������
INF_DISTANCE = 5;       %�ϰ�Ӱ����룬���ϰ��ͳ��ľ�������������ʱ������Ϊ0�������ܸ��ϰ���Ӱ��
L = 1;                  %����
STOP_RANGE = L;         %��ǰ�����յ���STOP_FLAG��Χ��ʱֹͣ����
RANDOM = 1;             %�Ƿ�Ҫ������������յ㣬1(��)��0�����ǣ�
START = [74  36];       %���   
GOAL = [15  37];        %�յ�
Model = 3;              %ѡ������·��ģʽ��1-���򣺴���㵽�յ㣻2-���򣺴��յ㵽��㣺3-˫�򣺴���㿪ʼ��һ���㣻4-˫�򣺴��յ㿪ʼ��һ���㣩
MAP = 7;                %ѡ���ͼ

%���õ�ͼ
if MAP == 0
    cir_OBJ_NUM = 0;        %Բ���ϰ������
    tri_OBJ_NUM = 0;        %�������ϰ������
    rec_OBJ_NUM = 1;        %�����ϰ������
    cir_OBJ = [0 0];        %Բ���ϰ���(û��ʱ�������һ������ֹ�������������)
    tri_OBJ = [0 0];        %�����ϰ���
    rec_OBJ = [30 30;50 30;50 60;30 60];        %�����ϰ���
elseif MAP == 1
    cir_OBJ_NUM = 14;        
    tri_OBJ_NUM = 0;
    rec_OBJ_NUM = 0;
    cir_OBJ = [15 ,90, 5 ;40, 85, 4 ;70 ,85 ,6 ;85 ,75, 5 ;30 ,65, 6 ;50 ,70, 6 ;70, 60, 5 ;...
               20, 40, 7 ;35, 45, 4 ;60 , 40, 6 ;85 ,40 ,4 ;30, 20, 5 ;55 ,25, 5 ;80, 20, 7 ];
    tri_OBJ = [0 ,0];
    rec_OBJ = [0 ,0];
elseif MAP == 2
    cir_OBJ_NUM = 0;         
    tri_OBJ_NUM = 0;
    rec_OBJ_NUM = 8;  
    cir_OBJ = [0 ,0];
    tri_OBJ = [0 ,0];
    rec_OBJ = [18 0 ; 20.5 0 ; 20.5 20 ;18 20 ; 18 50 ; 20.5 50 ; 20.5 100 ; 18 100 ; ... 
           38.5 0 ; 41 0 ; 41 50 ; 38.5 50 ; 38.5 80 ; 41 80 ; 41 100 ; 38.5 100 ; ...
           59 0 ; 61.5 0 ; 61.5 20 ; 59 20 ; 59 50 ; 61.5 50 ; 61.5 100 ; 59 100 ; ...
           79.5 0 ; 82 0 ; 82 50 ; 79.5 50 ; 79.5 80 ; 82 80 ; 82 100 ; 79.5 100 ];
elseif MAP == 3
    cir_OBJ_NUM = 0;         
    tri_OBJ_NUM = 0;
    rec_OBJ_NUM = 30; 
    OBJ_length = 20;      %�ϰ��ﳤ��
    OBJ_width = 2;        %�ϰ�����
    theta = [zeros(24,1) , 22 * ones(24,1)];
    Rectangle = [0,2 ; OBJ_length,2 ; OBJ_length,OBJ_width + 2; 0,OBJ_width + 2];
    OBJ1 = [ Rectangle ; Rectangle + 30 * [ones(4,1) , zeros(4,1)] ; Rectangle + 60 * [ones(4,1) , zeros(4,1)] ];
    OBJ2 = [ ( OBJ1 + [20 * ones(12,1) , 9 * ones(12,1)] ) ; ( OBJ1 + [zeros(12,1) , 20 * ones(12,1)] ) ] ;
    OBJ3 = OBJ1 + [20 * ones(12,1) , 95 * ones(12,1)];
    rec_OBJ = [OBJ1 ; OBJ2 ; (OBJ2 + theta) ; (OBJ2 + 2 * theta) ; (OBJ2 + 3 * theta) ; OBJ3];
    cir_OBJ = [0 ,0];
    tri_OBJ = [0 ,0];
elseif MAP == 4
    cir_OBJ_NUM = 9;         
    tri_OBJ_NUM = 4;
    rec_OBJ_NUM = 0;
    cir_OBJ = [15 ,90, 6 ;40, 85, 5 ;85 ,75, 5 ;25 ,65, 6 ;60 ,40, 6 ;80 ,50 ,5 ;25, 15, 5 ;80, 20, 7 ;35 ,40 ,4];
    tri_OBJ = [42 ,10 ;50 ,11 ;56 ,22 ;10 ,38 ;22 ,44 ;13 ,50 ; 52 ,62 ;60 ,78 ;46 ,70 ;76 ,82 ;86 ,94 ;70 ,90];
    rec_OBJ = [0 ,0];
elseif MAP == 5
    cir_OBJ_NUM = 18;        
    tri_OBJ_NUM = 0;
    rec_OBJ_NUM = 0;
    cir_OBJ = [10, 90 ,7 ;10 ,65 ,7 ;10 ,40 ,7 ;10 ,15 ,7 ;30, 80 ,7 ;30 ,55 ,7 ;30 ,30 ,7 ;...
               50, 90 ,7 ;50 ,65 ,7 ;50 ,40 ,7 ;50 ,15 ,7 ;70 ,80 ,7 ;70 ,55 ,7 ;70 ,30 ,7 ;...
               90, 90 ,7 ;90 ,65 ,7 ;90 ,40 ,7 ;90 ,15 ,7 ];
    tri_OBJ = [0 ,0];
    rec_OBJ = [0 ,0];
elseif MAP == 6
    cir_OBJ_NUM = 0;
    tri_OBJ_NUM = 15;
    rec_OBJ_NUM = 0;
    cir_OBJ = [0 ,0];
    tri_OBJ = [6, 96; 16, 96; 11, 88 ;38, 96; 48, 96; 43, 88 ;70, 96; 80, 96; 75, 88;...
               6, 76; 16, 76; 11, 68 ;38, 76; 48, 76; 43, 68 ;70, 76; 80, 76; 75, 68;...
               6, 56; 16, 56; 11, 48 ;38, 56; 48, 56; 43, 48 ;70, 56; 80, 56; 75, 48;...
               6, 36; 16, 36; 11, 28 ;38, 36; 48, 36; 43, 28 ;70, 36; 80, 36; 75, 28;...
               6, 16; 16, 16; 11, 8 ; 38, 16; 48, 16; 43, 8 ; 70, 16; 80, 16; 75, 8 ];
    rec_OBJ = [0 ,0];
elseif MAP == 7
    cir_OBJ_NUM = 2;
    tri_OBJ_NUM = 4;
    rec_OBJ_NUM = 6;
    cir_OBJ = [40 ,60 ,5 ;60 ,60 ,5];
    tri_OBJ = [16, 85; 25, 65; 36, 70;  84, 85; 75, 65; 66, 70;  50, 45; 50, 55; 45, 48; 35, 40; 50, 35; 65, 40 ];
    rectangle = [10 ,50 ;30 ,50 ;30 ,51 ;10 ,51];
    rec_OBJ = [rectangle ;rectangle + [60*ones(4 ,1) ,0*ones(4 ,1)]; rectangle + [-5*ones(4 ,1) ,-10*ones(4 ,1)];...
               rectangle + [65*ones(4 ,1) ,-10*ones(4 ,1)];rectangle + [0*ones(4 ,1) ,-20*ones(4 ,1)];rectangle + [60*ones(4 ,1) ,-20*ones(4 ,1)]];
elseif MAP == 8
    cir_OBJ_NUM = 2;
    tri_OBJ_NUM = 1;
    rec_OBJ_NUM = 3;
    cir_OBJ = [15 ,40 ,7 ;85 ,40 ,7];
    tri_OBJ = [16, 90; 26 ,66; 11 ,80];
    rec_OBJ = [20 ,11 ;80 ,11 ;80 ,13 ;20 ,13 ;50 ,40 ;60 ,40 ;60 ,60 ;50 ,60 ;40 ,75 ;60 ,75 ;60 ,77 ;40 ,77];
elseif MAP == 9
    cir_OBJ_NUM = 0;         
    tri_OBJ_NUM = 0;
    rec_OBJ_NUM = 2;
    cir_OBJ = [0 ,0];
    tri_OBJ = [0 ,0];
    rec_OBJ = [50 ,30 ;51 ,30 ;51 ,60 ;50 ,60 ;30 ,60 ;70 ,60 ;70 ,61 ;30 ,61];
elseif MAP == 10
    cir_OBJ_NUM = 0;         
    tri_OBJ_NUM = 0;
    rec_OBJ_NUM = 4;
    cir_OBJ = [0 ,0];
    tri_OBJ = [0 ,0];
    rec_OBJ = [50 ,30 ;52 ,30 ;52 ,60 ;50 ,60 ;30 ,40 ;32 ,40 ;32 ,60 ;30 ,60 ;...
               70 ,40 ;72 ,40 ;72 ,62 ;70 ,62 ;30 ,60 ;70 ,60 ;70 ,62 ;30 ,62];
elseif MAP == 11
    cir_OBJ_NUM = 0;         
    tri_OBJ_NUM = 0;
    rec_OBJ_NUM = 5;
    cir_OBJ = [0 ,0];
    tri_OBJ = [0 ,0];
    rec_OBJ = [40 ,20 ;45 ,20 ;45 ,50 ;40 ,50 ;80 ,20 ;85 ,20 ;85 ,65 ;80 ,65 ;20 ,20 ;25 ,20 ;...
               25 ,60 ;20 ,60 ;60 ,20 ;65 ,20 ;65 ,65 ;60 ,65 ;20 ,60 ;60 ,60 ;60 ,65 ;20 ,65];
elseif MAP == 12
    cir_OBJ_NUM = 4;         
    tri_OBJ_NUM = 0;
    rec_OBJ_NUM = 5;
    r = 5;
    cir_OBJ = [25 ,30 ,r ;25 ,70 ,r ;50,30 ,r ;50 ,70 ,r];
    tri_OBJ = [0 ,0];
    rectangle = [70 ,10 ;90 ,10 ;90 ,15 ;70 ,15];
    rec_OBJ = [rectangle ;rectangle + 20 * [zeros(4 ,1) ,ones(4 ,1)] ;rectangle + 40 * [zeros(4 ,1) ,ones(4 ,1)] ;...
               rectangle + 60 * [zeros(4 ,1) ,ones(4 ,1)] ;rectangle + 80 * [zeros(4 ,1) ,ones(4 ,1)]];
end

BORDER = [0 0 ; 100 0 ; 100 100 ; 0 100];    %��ͼ��СΪ100*100
PATH = zeros(MAX_CYCLE,2);                   %��ʼ��·������

%��ʼ����·��
if SG_NUM == 1
    if RANDOM == 1
        START = unifrnd(0 , 100 , 1 , 2);   %��������ϰ�����������
        while ( whether_inside_OBJ(START ,cir_OBJ ,tri_OBJ ,rec_OBJ ,cir_OBJ_NUM ,tri_OBJ_NUM ,rec_OBJ_NUM))    %�ж����ɵ�����Ƿ����ϰ�����
            START = unifrnd (0 , 100 , 1 , 2);               
        end
        GOAL = unifrnd(0 , 100 , 1 , 2);    %��������ϰ���������յ�
        while ( whether_inside_OBJ(GOAL ,cir_OBJ ,tri_OBJ ,rec_OBJ ,cir_OBJ_NUM ,tri_OBJ_NUM ,rec_OBJ_NUM) )    %�ж����ɵ��յ��Ƿ����ϰ�����
            GOAL = unifrnd (0 , 100 , 1 , 2);                
        end
    end
    if (Model == 1) | (Model ==3) 
        cur_Position = START;
        goal_Position = GOAL;
    else
        cur_Position = GOAL;
        goal_Position = START;
    end
    success_Flag = 0;                            %·���ɹ���־
    for i = 1:MAX_CYCLE
        theta_Goal = compute_goal_Angle(cur_Position ,goal_Position);   %�������㵽�յ�������X������ļнǵĽǶȣ��������Ƕȣ���ʱ��Ϊ������
        %�������нǶȾ�Ϊ��ϵͳX������������ΪX����������ΪY�����򣩵ļнǣ���ʱ��Ϊ�����򣬷�ΧΪ ��-180 , +180��
        [Fata_X ,Fata_Y] = compute_Attract(cur_Position ,goal_Position ,ATT ,theta_Goal);   %�����x��y���������
        [Frer_X ,Frer_Y] = compute_Repulsion(cur_Position ,goal_Position ,cir_OBJ ,tri_OBJ ,rec_OBJ ,cir_OBJ_NUM ,tri_OBJ_NUM ,rec_OBJ_NUM ,REP , INF_DISTANCE ,A ,theta_Goal);     %����������ϰ�����x����ĳ�������y����ĳ���
        [Frer_X_B ,Frer_Y_B] = compute_border_Repulsion(cur_Position ,goal_Position ,BORDER ,REP , INF_DISTANCE ,A ,theta_Goal);    %�������ͼ�߽���x����ĳ�������y����ĳ���
        Fsum_X = Fata_X +Frer_X+ Frer_X_B;           %X�������
        Fsum_Y = Fata_Y +Frer_Y+ Frer_Y_B;           %Y�������
        if( Fsum_X > 0)
            Fsum_Angle(i) = atan ( Fsum_Y / Fsum_X );     %�����ĽǶ�
        else
            Fsum_Angle(i) = atan ( Fsum_Y / Fsum_X ) + pi;
        end
        next_Position(1) = cur_Position(1) + L * cos ( Fsum_Angle(i) ) ;    %��һ��X����		
        next_Position(2) = cur_Position(2) + L * sin ( Fsum_Angle(i) ) ;    %��һ��Y����
        REP_ASS = REP;
        while(whether_inside_OBJ(next_Position ,cir_OBJ ,tri_OBJ ,rec_OBJ ,cir_OBJ_NUM ,tri_OBJ_NUM ,rec_OBJ_NUM))  %�����������һ���������ϰ����ڣ�����������ظ�����ֱ����һ�㲻���ϰ�����
            REP_ASS = REP_ASS * 2;
            [Frer_X ,Frer_Y] = compute_Repulsion(cur_Position ,goal_Position ,cir_OBJ ,tri_OBJ ,rec_OBJ ,cir_OBJ_NUM ,tri_OBJ_NUM ,rec_OBJ_NUM ,REP_ASS , INF_DISTANCE ,A ,theta_Goal);     %����������ϰ�����x����ĳ�������y����ĳ���
            [Frer_X_B ,Frer_Y_B] = compute_border_Repulsion(cur_Position ,goal_Position ,BORDER ,REP_ASS , INF_DISTANCE ,A ,theta_Goal);    %�������ͼ�߽���x����ĳ�������y����ĳ���
            Fsum_X = Fata_X +Frer_X+ Frer_X_B;           %X�������
            Fsum_Y = Fata_Y +Frer_Y+ Frer_Y_B;           %Y�������
            if( Fsum_X > 0)
                Fsum_Angle(i) = atan ( Fsum_Y / Fsum_X );     %�����ĽǶ�
            else
                Fsum_Angle(i) = atan ( Fsum_Y / Fsum_X ) + pi;
            end
            next_Position(1) = cur_Position(1) + L * cos ( Fsum_Angle(i) ) ;    %��һ��X����		
            next_Position(2) = cur_Position(2) + L * sin ( Fsum_Angle(i) ) ;    %��һ��Y����
        end
        if (Model == 1) | (Model == 2)
            cur_Position = next_Position;        %����Ѱ��·����ֻ����µ�ǰ��
        else (Model == 3) | (Model == 4)
            cur_Position = goal_Position ;       %���յ���Ϊ���
            goal_Position = next_Position;       %�����յ�
        end
        PATH(i,1) = next_Position(1);            %��һ��д��·��������
        PATH(i,2) = next_Position(2);
        distance_c_g = sqrt(( cur_Position(1) - goal_Position(1))^2 + ( cur_Position(2) - goal_Position(2))^2);
        if ( distance_c_g < STOP_RANGE)      %�����յ㸽��or��ǰ�㵽�յ������������ϰ����޽���
            i = i + 1;
            PATH(i,1) = next_Position(1);        %���һ��д��·��������
            PATH(i,2) = next_Position(2);
            success_Flag = 1;
           break;
        end
    end
    PATH_NUM = i;   %·����������ѭ����

    %�������·�����Ϊ�������
    if Model == 3 | Model == 4
        path_Temp = zeros(PATH_NUM,2);
        i = 1;
        j = 1;
        while(1)
             path_Temp(j,1) = PATH(i,1);
             path_Temp(j,2) = PATH(i,2);
             i = i + 2;
             j = j + 1;
             if i > PATH_NUM;
                 break;
             end
        end
        if mod(PATH_NUM,2) == 0
            i = PATH_NUM;
        else
            i = PATH_NUM - 1;
        end
        while(1)
             path_Temp(j,1) = PATH(i,1);
             path_Temp(j,2) = PATH(i,2);
             j = j + 1;
             i = i - 2;
             if i < 2;
                 break;
             end
         end
        PATH = path_Temp;
    end

    %����Ϊ��ͼ����
    if cir_OBJ_NUM > 0       %��Բ���ϰ���
        for i = 1:cir_OBJ_NUM 
            theta = 0 : 0.1 : 2 * pi;  
            x = cir_OBJ(i ,1) + cir_OBJ(i ,3) * cos(theta);  
            y = cir_OBJ(i ,2) + cir_OBJ(i ,3) * sin(theta);  
            color = [255 255 0];  
            plot(x ,y ,'c' ,'linewidth' ,1);
            hold on
        end
    end
    if tri_OBJ_NUM > 0       %���������ϰ���
        for i = 1:tri_OBJ_NUM
            x = [tri_OBJ(3*i-2,1) tri_OBJ(3*i-1,1) tri_OBJ(3*i,1)];
            y = [tri_OBJ(3*i-2,2) tri_OBJ(3*i-1,2) tri_OBJ(3*i,2)];
            fill(x, y, 'y')      
            hold on
        end
    end
    if rec_OBJ_NUM > 0       %�������ϰ���
        for i = 1:rec_OBJ_NUM 
            x = [rec_OBJ(4*i-3,1) rec_OBJ(4*i-2,1) rec_OBJ(4*i-1,1) rec_OBJ(4*i,1)];
            y = [rec_OBJ(4*i-3,2) rec_OBJ(4*i-2,2) rec_OBJ(4*i-1,2) rec_OBJ(4*i,2)];
            fill(x, y, 'y')      
            hold on
        end
    end
    X = PATH(:,1);          %·��
    Y = PATH(:,2);
    plot( START(1), START(2) ,'v', GOAL(1),GOAL(2), 'ms' ,  X, Y, '.r');    %���Ϊ�����Σ��յ�Ϊ�����Σ�·����Ϊ���
    if success_Flag == 1
       plot( PATH(PATH_NUM,1),PATH(PATH_NUM,2),'o');    %o��ʾ���һ����
    end
    axis([0 100 0 100])     %��ͼ��С
    axis square
else
    SUCCESS_NUM = [0 0];                         %[����ʱ  ˫��ʱ]�ɹ�Ѱ�ҵ�·���ܴ���
    SUCCESS_RATE = [0.0 0.0];                    %[����ʱ  ˫��ʱ]�ɹ���
    SUM_LENGTH = [0 0];                          %[����ʱ  ˫��ʱ]�ɹ�Ѱ�ҵ�·��ʱ·�����ܸ���
    AVE_LENGTH = [0 0];                          %[����ʱ  ˫��ʱ]�ɹ�Ѱ�ҵ�·��ʱ·��ƽ������
    SUM_TIME = [0.0 0.0];                        %[����ʱ  ˫��ʱ]��ʱ��
    AVE_TIME = [0.0 0.0];                        %[����ʱ  ˫��ʱ]ƽ��ʱ��
    for k = 1:SG_NUM;
        START = unifrnd(0 , 100 , 1 , 2);
        while (whether_inside_OBJ(START ,cir_OBJ ,tri_OBJ ,rec_OBJ ,cir_OBJ_NUM ,tri_OBJ_NUM ,rec_OBJ_NUM) )
            START = unifrnd (0 , 100 , 1 , 2);               %��������ϰ�����������
        end
        GOAL = unifrnd(0 , 100 , 1 , 2);
        while (whether_inside_OBJ(START ,cir_OBJ ,tri_OBJ ,rec_OBJ ,cir_OBJ_NUM ,tri_OBJ_NUM ,rec_OBJ_NUM))
            GOAL = unifrnd (0 , 100 , 1 , 2);                %��������ϰ���������յ�
        end
        for Model = 1:2
            cur_Position = START;
            goal_Position = GOAL;
            t1 = clock;
            success_Flag = 0;
            success_PATH_NUM = 0;
            for i = 1:MAX_CYCLE
                theta_Goal = compute_goal_Angle(cur_Position ,goal_Position);   %�������㵽�յ�������X������ļнǵĽǶȣ��������Ƕȣ���ʱ��Ϊ������
                [Fata_X ,Fata_Y] = compute_Attract(cur_Position ,goal_Position ,ATT ,theta_Goal);       %�����x��y���������
                [Frer_X ,Frer_Y] = compute_Repulsion(cur_Position ,goal_Position ,cir_OBJ ,tri_OBJ ,rec_OBJ ,cir_OBJ_NUM ,tri_OBJ_NUM ,rec_OBJ_NUM ,REP , INF_DISTANCE ,A ,theta_Goal);      %����������ϰ�����x����ĳ�������y����ĳ���
                [Frer_X_B ,Frer_Y_B] = compute_border_Repulsion(cur_Position ,goal_Position ,BORDER ,REP , INF_DISTANCE ,A ,theta_Goal);    %�������ͼ�߽���x����ĳ�������y����ĳ���
                Fsum_X = Fata_X + Frer_X + Frer_X_B;           %X�������
                Fsum_Y = Fata_Y + Frer_Y + Frer_Y_B;           %Y�������
                if( Fsum_X > 0)
                    Fsum_Angle(i) = atan ( Fsum_Y / Fsum_X );     %�����ĽǶ�
                else
                    Fsum_Angle(i) = atan ( Fsum_Y / Fsum_X ) + pi;
                end
                next_Position(1) = cur_Position(1) + L * cos ( Fsum_Angle(i) ) ;    %��һ��X����		
                next_Position(2) = cur_Position(2) + L * sin ( Fsum_Angle(i) ) ;    %��һ��Y����
                REP_ASS = REP;
                while(whether_inside_OBJ(START ,cir_OBJ ,tri_OBJ ,rec_OBJ ,cir_OBJ_NUM ,tri_OBJ_NUM ,rec_OBJ_NUM))                   %�����������һ���������ϰ����ڣ�����������ظ�����ֱ����һ�㲻���ϰ�����
                    REP_ASS = REP_ASS * 2;                         %����������ʱ�ĳ�������ϵ��
                    [Frer_X ,Frer_Y] = compute_Repulsion(cur_Position ,goal_Position ,cir_OBJ ,tri_OBJ ,rec_OBJ ,cir_OBJ_NUM ,tri_OBJ_NUM ,rec_OBJ_NUM ,REP_ASS , INF_DISTANCE ,A ,theta_Goal); 
                    [Frer_X_B ,Frer_Y_B] = compute_border_Repulsion(cur_Position ,goal_Position ,BORDER ,REP_ASS , INF_DISTANCE ,A ,theta_Goal);    %�������ͼ�߽���x����ĳ�������y����ĳ���
                    Fsum_X = Fata_X + Frer_X + Frer_X_B;           %X�������
                    Fsum_Y = Fata_Y + Frer_Y + Frer_Y_B;           %Y�������
                    if( Fsum_X > 0)
                        Fsum_Angle(i) = atan ( Fsum_Y / Fsum_X );     %�����ĽǶ�
                    else
                        Fsum_Angle(i) = atan ( Fsum_Y / Fsum_X ) + pi;
                    end
                    next_Position(1) = cur_Position(1) + L * cos ( Fsum_Angle(i) ) ;    %��һ��X����		
                    next_Position(2) = cur_Position(2) + L * sin ( Fsum_Angle(i) ) ;    %��һ��Y����
                end
                if (Model == 1)
                    cur_Position = next_Position;       %����Ѱ��·����ֻ����µ�ǰ��
                else 
                    cur_Position = goal_Position ;       %���յ���Ϊ���
                    goal_Position = next_Position;       %�����յ�
                end
                distance_c_g = sqrt(( cur_Position(1) - goal_Position(1))^2 + ( cur_Position(2) - goal_Position(2))^2);
                if ( distance_c_g < STOP_RANGE) %| (whether_Cross(cur_Position ,goal_Position ,OBJ ,OBJ_NUM) == 0)       %�����յ㸽��or��ǰ�㵽�յ������������ϰ����޽���
                    success_Flag = 1;
                    success_PATH_NUM = i;
                    break;
                end
            end
            if (success_Flag == 1)
                SUM_LENGTH(Model) = SUM_LENGTH(Model) + success_PATH_NUM ;
                SUCCESS_NUM(Model) = SUCCESS_NUM(Model) + 1;
            end
            t2 = clock;
            SUM_TIME(Model) = SUM_TIME(Model) + etime(t2,t1);
        end
    end
    AVE_LENGTH = SUM_LENGTH ./ SUCCESS_NUM;
    SUCCESS_RATE = SUCCESS_NUM / SG_NUM;
    AVE_TIME = SUM_TIME / SG_NUM;  
    %[SUCCESS_NUM ; SUCCESS_RATE ; SUM_LENGTH ; AVE_LENGTH ; SUM_TIME ; AVE_TIME]
    %[SUCCESS_RATE ; AVE_LENGTH ; AVE_TIME]
    %[SUCCESS_NUM ; SUM_LENGTH ]
end