 %变量初始化
OBJ_NUM = 1;    %障碍物个数
MAX_CYCLE = 200;    %循环次数
ATT = 15.0;     %计算引力时增益系数
REP = 5.0;      %计算斥力时增益系数
A = 0.5;    %计算斥力时辅助参数
INF_DISTANCE = 2.5;     %障碍影响距离，当障碍和车的距离大于这个距离时，斥力为0，即不受该障碍的影响
L = 0.1;    %步长
OBJ = [4 4; 6 4 ;6 6 ; 4 6];   %障碍物
%OBJ_2 = [4 4; 6 4 ;6 6 ; 4 6];
%OBJ = [OBJ_1;OBJ_2];
START_AND_GOAL = [0 0; 7 7];   %起终点
PATH = zeros(MAX_CYCLE,2);      %初始化路径矩阵
cur_Position(1) = START_AND_GOAL(1,1);
cur_Position(2) = START_AND_GOAL(1,2);
goal_Position(1) = START_AND_GOAL(2,1);
goal_Position(2) = START_AND_GOAL(2,2);
%开始计算路径
for i = 1:MAX_CYCLE
    PATH(i,1) = cur_Position(1);        %当前点写到路径中
    PATH(i,2) = cur_Position(2);
    theta_Goal = compute_goal_Angle(cur_Position ,goal_Position);   %计算出起点到终点连线与X正方向的角度，即引力角度，逆时针为正方向
    [Fata_X ,Fata_Y] = compute_Attract(cur_Position ,goal_Position ,ATT ,theta_Goal);       %计算出x和y方向的引力
    [Frer_X ,Frer_Y] = compute_Repulsion(cur_Position ,goal_Position ,OBJ ,REP , INF_DISTANCE ,A);  %计算出各个障碍物在x方向的引力和与在y方向的引力和
   	Fsum_X = Fata_X + Frer_X ;		%X方向合力
	Fsum_Y = Fata_Y + Frer_Y ;       %Y方向合力
    if( Fsum_X > 0)
        Fsum_Angle = atan ( Fsum_Y / Fsum_X );     %合力的角度
    else
        Fsum_Angle = atan ( Fsum_Y / Fsum_X ) + pi;
    end
  	next_Position(1) = cur_Position(1) + L * cos ( Fsum_Angle ) ;    %下一点X坐标		
   	next_Position(2) = cur_Position(2) + L * sin ( Fsum_Angle ) ;    %下一点Y坐标	
    cur_Position = next_Position;       %更新当前点
    if ( abs ( cur_Position(1) - goal_Position(1) ) < 0.1) & ( abs ( cur_Position(2) - goal_Position(2) ) < 0.1)      %到达终点附近
        i = i + 1;
        PATH(i,1) = cur_Position(1);
        PATH(i,2) = cur_Position(2);
       break;
    end
end
PATH_NUM = i + 1;
PATH(PATH_NUM,1) = goal_Position(1);        %将终点加入路径
PATH(PATH_NUM,2) = goal_Position(2);

%以下为画图部分
% 矩形障碍物
left = 4;
right = left + 2;
bottom = 4;
top = bottom + 2;
x = [left left right right];
y = [bottom top top bottom];
fill(x, y, 'y')
hold on

X = PATH(:,1);      %路径
Y = PATH(:,2);
plot( goal_Position(1) , goal_Position(2) ,'v',START_AND_GOAL(1,1),START_AND_GOAL(1,2), 'ms' , X, Y, '.r');