%������ʼ��
OBJ_NUM = 2;    %�ϰ������
MAX_CYCLE = 500;    %ѭ������
ATT = 15.0;     %��������ʱ����ϵ��
REP = 5.0;      %�������ʱ����ϵ��
A = 0.5;    %�������ʱ��������
INF_DISTANCE = 2.5;     %�ϰ�Ӱ����룬���ϰ��ͳ��ľ�������������ʱ������Ϊ0�������ܸ��ϰ���Ӱ��
L = 1;    %����
START_AND_GOAL =[4.5 8; 90 90];   %���յ�
left1 = 40;              %�ϰ���1��߽�
bottom1 = 0;            %�ϰ���1�±߽�
left2 = 40;              %�ϰ���2��߽�
bottom2 = 60;            %�ϰ���2�±߽�
right1 = left1 + 20;      %�ϰ���1�ұ߽�
top1 = bottom1 + 40;    %�ϰ���1�ϱ߽�
right2 = left2 + 20;      %�ϰ���2�ұ߽�
top2 = bottom2 + 40;    %�ϰ���2�ϱ߽�
OBJ1 = [left1 bottom1 ; right1 bottom1 ; right1 top1 ; left1 top1];   %�ϰ���1
OBJ2 = [left2 bottom2 ; right2 bottom2 ; right2 top2 ; left2 top2];   %�ϰ���2
OBJ = [OBJ1 ; OBJ2];
PATH = zeros(MAX_CYCLE,2);      %��ʼ��·������
cur_Position(1) = START_AND_GOAL(1,1);
cur_Position(2) = START_AND_GOAL(1,2);
goal_Position(1) = START_AND_GOAL(2,1);
goal_Position(2) = START_AND_GOAL(2,2);

%��ʼ����·��
for i = 1:MAX_CYCLE
    PATH(i,1) = cur_Position(1);        %��ǰ��д��·��������
    PATH(i,2) = cur_Position(2);
    theta_Goal = compute_goal_Angle(cur_Position ,goal_Position);   %�������㵽�յ�������X������ļнǵĽǶȣ��������Ƕȣ���ʱ��Ϊ������
    [Fata_X ,Fata_Y] = compute_Attract(cur_Position ,goal_Position ,ATT ,theta_Goal);       %�����x��y���������
    [Frer_X ,Frer_Y] = compute_Repulsion(cur_Position ,goal_Position ,OBJ ,OBJ_NUM ,REP , INF_DISTANCE ,A);  %����������ϰ�����x����ĳ�������y����ĳ���
    Fsum_X = Fata_X + Frer_X ;		 %X�������
	Fsum_Y = Fata_Y + Frer_Y ;       %Y�������
    if( Fsum_X > 0)
        Fsum_Angle = atan ( Fsum_Y / Fsum_X );     %�����ĽǶ�
    else
        Fsum_Angle = atan ( Fsum_Y / Fsum_X ) + pi;
    end
  	next_Position(1) = cur_Position(1) + L * cos ( Fsum_Angle ) ;    %��һ��X����		
   	next_Position(2) = cur_Position(2) + L * sin ( Fsum_Angle ) ;    %��һ��Y����
    cur_Position = goal_Position ;       %���յ���Ϊ���
    goal_Position = next_Position;       %�����յ�
    %cur_Position = next_Position;      %����Ѱ��·����ֻ����µ�ǰ��
    if ( abs ( cur_Position(1) - goal_Position(1) ) < 0.5) & ( abs ( cur_Position(2) - goal_Position(2) ) < 0.5)      %�����յ㸽��
        i = i + 1;
        PATH(i,1) = cur_Position(1);        %��ǰ��д��·��������
        PATH(i,2) = cur_Position(2);
       break;
    end
end
PATH_NUM = i + 1;
PATH(PATH_NUM,1) = goal_Position(1);        %���յ����·��
PATH(PATH_NUM,2) = goal_Position(2);

%����Ϊ��ͼ����
axis([0 100 0 100])     %��ͼ��С
axis square
x1 = [left1 left1 right1 right1];
y1 = [bottom1 top1 top1 bottom1];
fill(x1, y1, 'y')       %������1
hold on
x2 = [left2 left2 right2 right2];
y2 = [bottom2 top2 top2 bottom2];
fill(x2, y2, 'y')       %������2
hold on
X = PATH(:,1);          %·��
Y = PATH(:,2);
plot( START_AND_GOAL(1,1), START_AND_GOAL(1,2) ,'v', START_AND_GOAL(2,1),START_AND_GOAL(2,2), 'ms' , X, Y, '.r');    %���Ϊ�����Σ��յ�Ϊ�����Σ�·����Ϊ���