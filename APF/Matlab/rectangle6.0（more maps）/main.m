%变量初始化
MAX_CYCLE = 500;        %单次路径搜索循环次数
ATT = 1.0;              %计算引力时增益系数
REP = 1.0;              %计算斥力时增益系数
A = 2;                 %计算斥力时辅助参数
INF_DISTANCE = 8;      %障碍影响距离，当障碍和车的距离大于这个距离时，斥力为0，即不受该障碍的影响
L = 1;                 %步长
STOP_RANGE = L;         %当前点与终点在STOP_FLAG范围内时停止搜索,
%START = [1 10];        %起点   
%GOAL = [2 15];   %终点
Model = 3 ;            %选择搜索路径模式（1-单向：从起点到终点；2-单向：从终点到起点：3-双向：从起点开始第一个点；3-双向：从终点开始第一个点）
MAP = 4;              %选择地图
if MAP == 1
    OBJ_NUM = 8;       %障碍物个数
    OBJ = [18 0 ; 20.5 0 ; 20.5,20 ;18 20 ; 18 50 ; 20.5 50 ; 20.5 100 ; 18 100 ; ... 
           38.5 0 ; 41 0 ; 41 50 ; 38.5 50 ; 38.5 80 ; 41 80 ; 41 100 ; 38.5 100 ; ...
           59 0 ; 61.5 0 ; 61.5 20 ; 59 20 ; 59 50 ; 61.5 50 ; 61.5 100 ; 59 100 ; ...
           79.5 0 ; 82 0 ; 82 50 ; 79.5 50 ; 79.5 80 ; 82 80 ; 82 100 ; 79.5 100 ];
elseif MAP == 2
    OBJ_NUM = 8;        %障碍物个数
    OBJ = [17,0 ; 22,0 ; 22,20 ; 17,20 ; 17,50 ; 22,50 ; 22,100 ; 17,100 ; ...
           39,0 ; 44,0 ; 44,50 ; 39,50 ; 39,80 ; 44,80 ; 44,100 ; 39,100 ; ...
           61,0 ; 66,0 ; 66,20 ; 61,20 ; 61,50 ; 66,50 ; 66,100 ; 61,100 ; ...
           83,0 ; 88,0 ; 88,50 ; 83,50 ; 83,80 ; 88,80 ; 88,100 ; 83,100 ];
elseif MAP == 3
    OBJ_NUM = 8;        %障碍物个数
    OBJ = [12,0 ; 22,0 ; 22,20 ; 12,20 ; 12,50 ; 22,50 ; 22,100 ; 12,100 ; ...
           34,0 ; 44,0 ; 44,50 ; 34,50 ; 34,80 ; 44,80 ; 44,100 ; 34,100 ; ...
           56,0 ; 66,0 ; 66,20 ; 56,20 ; 56,50 ; 66,50 ; 66,100 ; 56,100 ; ...
           78,0 ; 88,0 ; 88,50 ; 78,50 ; 78,80 ; 88,80 ; 88,100 ; 78,100 ];  
    else
        OBJ_NUM = 30;         %障碍物个数
        OBJ_length = 20;      %障碍物长度
        OBJ_width = 2;        %障碍物宽度
        theta = [zeros(24,1) , 22 * ones(24,1)];
        Rectangle = [0,2 ; OBJ_length,2 ; OBJ_length,OBJ_width + 2; 0,OBJ_width + 2];
        OBJ1 = [ Rectangle ; Rectangle + 30 * [ones(4,1) , zeros(4,1)] ; Rectangle + 60 * [ones(4,1) , zeros(4,1)] ];
        OBJ2 = [ ( OBJ1 + [20 * ones(12,1) , 9 * ones(12,1)] ) ; ( OBJ1 + [zeros(12,1) , 20 * ones(12,1)] ) ] ;
        OBJ3 = OBJ1 + [20 * ones(12,1) , 95 * ones(12,1)];
        OBJ = [OBJ1 ; OBJ2 ; (OBJ2 + theta) ; (OBJ2 + 2 * theta) ; (OBJ2 + 3 * theta) ; OBJ3];
            
end
BORDER = [0 0 ; 100 0 ; 100 100 ; 0 100];    %地图大小为100*100
PATH = zeros(MAX_CYCLE,2);                   %初始化路径矩阵
success_Flag = 0;                            %路径成功标志

%开始计算路径
    START = unifrnd(0 , 100 , 1 , 2);
    while ( whether_inside_OBJ(START , OBJ) )
        START = unifrnd (0 , 100 , 1 , 2);               %随机生成障碍物以外的起点
    end
    GOAL = unifrnd(0 , 100 , 1 , 2);
    while ( whether_inside_OBJ(GOAL , OBJ) )
        GOAL = unifrnd (0 , 100 , 1 , 2);                %随机生成障碍物以外的终点
    end
if (Model == 1) | (Model ==3)
    cur_Position = START;
    goal_Position = GOAL;
else
    cur_Position = GOAL;
    goal_Position = START;
end
for i = 1:MAX_CYCLE
    theta_Goal = compute_goal_Angle(cur_Position ,goal_Position);   %计算出起点到终点连线与X正方向的夹角的角度，即引力角度，逆时针为正方向
    [Fata_X ,Fata_Y] = compute_Attract(cur_Position ,goal_Position ,ATT ,theta_Goal);       %计算出x和y方向的引力
    [Frer_X ,Frer_Y] = compute_Repulsion(cur_Position ,goal_Position ,OBJ ,OBJ_NUM ,REP , INF_DISTANCE ,A );     %计算出各个障碍物在x方向的斥力与在y方向的斥力
    [Frer_X_B ,Frer_Y_B] = compute_border_Repulsion(cur_Position ,goal_Position ,BORDER ,REP , INF_DISTANCE ,A );    %计算出地图边界在x方向的斥力与在y方向的斥力
    Fsum_X = Fata_X + Frer_X + Frer_X_B;           %X方向合力
	Fsum_Y = Fata_Y + Frer_Y + Frer_Y_B;           %Y方向合力
    if( Fsum_X > 0)
        Fsum_Angle = atan ( Fsum_Y / Fsum_X );     %合力的角度
    else
        Fsum_Angle = atan ( Fsum_Y / Fsum_X ) + pi;
    end
  	next_Position(1) = cur_Position(1) + L * cos ( Fsum_Angle ) ;    %下一点X坐标		
   	next_Position(2) = cur_Position(2) + L * sin ( Fsum_Angle ) ;    %下一点Y坐标
    REP_ASS = REP;
    while(whether_inside_OBJ(next_Position , OBJ))                   %若计算出的下一点坐标在障碍物内，则增大斥力重复计算直到下一点不在障碍物内
        REP_ASS = REP_ASS * 2;
        [Frer_X ,Frer_Y] = compute_Repulsion(cur_Position ,goal_Position ,OBJ ,OBJ_NUM ,REP_ASS , INF_DISTANCE ,A );
        [Frer_X_B ,Frer_Y_B] = compute_border_Repulsion(cur_Position ,goal_Position ,BORDER ,REP_ASS , INF_DISTANCE ,A );    %计算出地图边界在x方向的斥力与在y方向的斥力
        Fsum_X = Fata_X + Frer_X + Frer_X_B;           %X方向合力
        Fsum_Y = Fata_Y + Frer_Y + Frer_Y_B;           %Y方向合力
        if( Fsum_X > 0)
            Fsum_Angle = atan ( Fsum_Y / Fsum_X );     %合力的角度
        else
            Fsum_Angle = atan ( Fsum_Y / Fsum_X ) + pi;
        end
        next_Position(1) = cur_Position(1) + L * cos ( Fsum_Angle ) ;    %下一点X坐标		
        next_Position(2) = cur_Position(2) + L * sin ( Fsum_Angle ) ;    %下一点Y坐标
    end
    if (Model == 1) | (Model == 2)
        cur_Position = next_Position;        %单向寻找路径，只需更新当前点
    else 
        cur_Position = goal_Position ;       %将终点设为起点
        goal_Position = next_Position;       %更新终点
    end
    PATH(i,1) = next_Position(1);            %下一点写到路径矩阵中
    PATH(i,2) = next_Position(2);
    distance_c_g = sqrt(( cur_Position(1) - goal_Position(1))^2 + ( cur_Position(2) - goal_Position(2))^2);
    if ( distance_c_g < STOP_RANGE)       %到达终点附近
        i = i + 1;
        PATH(i,1) = next_Position(1);        %下一点写到路径矩阵中
        PATH(i,2) = next_Position(2);
        success_Flag = 1;
       break;
    end
end
PATH_NUM = i;
%将无序的路径点变为有序
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

%以下为画图部分
for i = 1:OBJ_NUM 
    x = [OBJ((i-1)*4+1,1) OBJ((i-1)*4+3,1) OBJ((i-1)*4+3,1) OBJ((i-1)*4+1,1)];
    y = [OBJ((i-1)*4+1,2) OBJ((i-1)*4+1,2) OBJ((i-1)*4+3,2) OBJ((i-1)*4+3,2)];
    fill(x, y, 'y')       %画矩形i
    hold on
end
X = PATH(:,1);          %路径
Y = PATH(:,2);
plot( START(1), START(2) ,'v', GOAL(1),GOAL(2), 'ms' ,  X, Y, '.r');    %起点为三角形，终点为正方形，路径点为红点
if success_Flag == 1
   plot( PATH(PATH_NUM,1),PATH(PATH_NUM,2),'o');
end
axis([0 100 0 100])     %地图大小
axis square