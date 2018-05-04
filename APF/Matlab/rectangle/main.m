 %������ʼ��
OBJ_NUM = 1;    %�ϰ������
MAX_CYCLE = 200;    %ѭ������
ATT = 15.0;     %��������ʱ����ϵ��
REP = 5.0;      %�������ʱ����ϵ��
A = 0.5;    %�������ʱ��������
INF_DISTANCE = 2.5;     %�ϰ�Ӱ����룬���ϰ��ͳ��ľ�������������ʱ������Ϊ0�������ܸ��ϰ���Ӱ��
L = 0.1;    %����
OBJ = [4 4; 6 4 ;6 6 ; 4 6];   %�ϰ���
%OBJ_2 = [4 4; 6 4 ;6 6 ; 4 6];
%OBJ = [OBJ_1;OBJ_2];
START_AND_GOAL = [0 0; 7 7];   %���յ�
PATH = zeros(MAX_CYCLE,2);      %��ʼ��·������
cur_Position(1) = START_AND_GOAL(1,1);
cur_Position(2) = START_AND_GOAL(1,2);
goal_Position(1) = START_AND_GOAL(2,1);
goal_Position(2) = START_AND_GOAL(2,2);
%��ʼ����·��
for i = 1:MAX_CYCLE
    PATH(i,1) = cur_Position(1);        %��ǰ��д��·����
    PATH(i,2) = cur_Position(2);
    theta_Goal = compute_goal_Angle(cur_Position ,goal_Position);   %�������㵽�յ�������X������ĽǶȣ��������Ƕȣ���ʱ��Ϊ������
    [Fata_X ,Fata_Y] = compute_Attract(cur_Position ,goal_Position ,ATT ,theta_Goal);       %�����x��y���������
    [Frer_X ,Frer_Y] = compute_Repulsion(cur_Position ,goal_Position ,OBJ ,REP , INF_DISTANCE ,A);  %����������ϰ�����x���������������y�����������
   	Fsum_X = Fata_X + Frer_X ;		%X�������
	Fsum_Y = Fata_Y + Frer_Y ;       %Y�������
    if( Fsum_X > 0)
        Fsum_Angle = atan ( Fsum_Y / Fsum_X );     %�����ĽǶ�
    else
        Fsum_Angle = atan ( Fsum_Y / Fsum_X ) + pi;
    end
  	next_Position(1) = cur_Position(1) + L * cos ( Fsum_Angle ) ;    %��һ��X����		
   	next_Position(2) = cur_Position(2) + L * sin ( Fsum_Angle ) ;    %��һ��Y����	
    cur_Position = next_Position;       %���µ�ǰ��
    if ( abs ( cur_Position(1) - goal_Position(1) ) < 0.1) & ( abs ( cur_Position(2) - goal_Position(2) ) < 0.1)      %�����յ㸽��
        i = i + 1;
        PATH(i,1) = cur_Position(1);
        PATH(i,2) = cur_Position(2);
       break;
    end
end
PATH_NUM = i + 1;
PATH(PATH_NUM,1) = goal_Position(1);        %���յ����·��
PATH(PATH_NUM,2) = goal_Position(2);

%����Ϊ��ͼ����
% �����ϰ���
left = 4;
right = left + 2;
bottom = 4;
top = bottom + 2;
x = [left left right right];
y = [bottom top top bottom];
fill(x, y, 'y')
hold on

X = PATH(:,1);      %·��
Y = PATH(:,2);
plot( goal_Position(1) , goal_Position(2) ,'v',START_AND_GOAL(1,1),START_AND_GOAL(1,2), 'ms' , X, Y, '.r');