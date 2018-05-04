%变量初始化
OBJ_NUM = 2;    %障碍物个数
MAX_CYCLE = 500;    %循环次数
ATT = 15.0;     %计算引力时增益系数
REP = 5.0;      %计算斥力时增益系数
A = 0.5;    %计算斥力时辅助参数
INF_DISTANCE = 2.5;     %障碍影响距离，当障碍和车的距离大于这个距离时，斥力为0，即不受该障碍的影响
L = 1;    %步长
START_AND_GOAL =[4.5 8; 90 90];   %起终点
left1 = 40;              %障碍物1左边界
bottom1 = 0;            %障碍物1下边界
left2 = 40;              %障碍物2左边界
bottom2 = 60;            %障碍物2下边界
right1 = left1 + 20;      %障碍物1右边界
top1 = bottom1 + 40;    %障碍物1上边界
right2 = left2 + 20;      %障碍物2右边界
top2 = bottom2 + 40;    %障碍物2上边界
OBJ1 = [left1 bottom1 ; right1 bottom1 ; right1 top1 ; left1 top1];   %障碍物1
OBJ2 = [left2 bottom2 ; right2 bottom2 ; right2 top2 ; left2 top2];   %障碍物2
OBJ = [OBJ1 ; OBJ2];
PATH = zeros(MAX_CYCLE,2);      %初始化路径矩阵
cur_Position(1) = START_AND_GOAL(1,1);
cur_Position(2) = START_AND_GOAL(1,2);
goal_Position(1) = START_AND_GOAL(2,1);
goal_Position(2) = START_AND_GOAL(2,2);

%开始计算路径
for i = 1:MAX_CYCLE
    PATH(i,1) = cur_Position(1);        %当前点写到路径矩阵中
    PATH(i,2) = cur_Position(2);
    theta_Goal = compute_goal_Angle(cur_Position ,goal_Position);   %计算出起点到终点连线与X正方向的夹角的角度，即引力角度，逆时针为正方向
    [Fata_X ,Fata_Y] = compute_Attract(cur_Position ,goal_Position ,ATT ,theta_Goal);       %计算出x和y方向的引力
    [Frer_X ,Frer_Y] = compute_Repulsion(cur_Position ,goal_Position ,OBJ ,OBJ_NUM ,REP , INF_DISTANCE ,A);  %计算出各个障碍物在x方向的斥力与在y方向的斥力
    Fsum_X = Fata_X + Frer_X ;		 %X方向合力
	Fsum_Y = Fata_Y + Frer_Y ;       %Y方向合力
    if( Fsum_X > 0)
        Fsum_Angle = atan ( Fsum_Y / Fsum_X );     %合力的角度
    else
        Fsum_Angle = atan ( Fsum_Y / Fsum_X ) + pi;
    end
  	next_Position(1) = cur_Position(1) + L * cos ( Fsum_Angle ) ;    %下一点X坐标		
   	next_Position(2) = cur_Position(2) + L * sin ( Fsum_Angle ) ;    %下一点Y坐标
    cur_Position = goal_Position ;       %将终点设为起点
    goal_Position = next_Position;       %更新终点
    %cur_Position = next_Position;      %单向寻找路径，只需更新当前点
    if ( abs ( cur_Position(1) - goal_Position(1) ) < 0.5) & ( abs ( cur_Position(2) - goal_Position(2) ) < 0.5)      %到达终点附近
        i = i + 1;
        PATH(i,1) = cur_Position(1);        %当前点写到路径矩阵中
        PATH(i,2) = cur_Position(2);
       break;
    end
end
PATH_NUM = i + 1;
PATH(PATH_NUM,1) = goal_Position(1);        %将终点加入路径
PATH(PATH_NUM,2) = goal_Position(2);

%以下为画图部分
axis([0 100 0 100])     %地图大小
axis square
x1 = [left1 left1 right1 right1];
y1 = [bottom1 top1 top1 bottom1];
fill(x1, y1, 'y')       %画矩形1
hold on
x2 = [left2 left2 right2 right2];
y2 = [bottom2 top2 top2 bottom2];
fill(x2, y2, 'y')       %画矩形2
hold on
X = PATH(:,1);          %路径
Y = PATH(:,2);
plot( START_AND_GOAL(1,1), START_AND_GOAL(1,2) ,'v', START_AND_GOAL(2,1),START_AND_GOAL(2,2), 'ms' , X, Y, '.r');    %起点为三角形，终点为正方形，路径点为红点