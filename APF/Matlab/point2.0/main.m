%������ʼ��
OBJ_NUM = 7;    %�ϰ������
MAX_CYCLE =500;    %ѭ������
ATT = 1;     %��������ʱ����ϵ��
REP = 0.1;      %�������ʱ����ϵ��
A = 2;    %�������ʱ��������
INF_DISTANCE = 0.8;     %�ϰ�Ӱ����룬���ϰ��ͳ��ľ�������������ʱ������Ϊ0�������ܸ��ϰ���Ӱ��
L = 0.1;    %����
OBJ = [1 1.2;3 2.5;4 4.5;3 6;6 2;5.5 5.5;8 8.5];   %�ϰ���
START_AND_GOAL = [3 3; 6 7];   %���յ�
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
    theta_Obj = compute_obj_Angle(cur_Position, OBJ ,OBJ_NUM);      %�������㵽�ϰ���������X������ĽǶȣ��������Ƕȣ���ʱ��Ϊ������
    [Fata_X ,Fata_Y] = compute_Attract(cur_Position ,goal_Position ,ATT ,theta_Goal);       %�����x��y���������
    [Frer_X ,Frer_Y] = compute_Repulsion(cur_Position ,goal_Position ,OBJ ,REP ,theta_Goal ,theta_Obj ,OBJ_NUM , INF_DISTANCE ,A);  %����������ϰ�����x���������������y�����������
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
X = PATH(:,1);      %·��
Y = PATH(:,2);
x=[1 3 4 3 6 5.5 8];
y=[1.2 2.5 4.5 6 2 5.5 8.5];
plot(x , y, 'o' , goal_Position(1) , goal_Position(2) ,'v',START_AND_GOAL(1,1),START_AND_GOAL(1,2), 'ms' , X, Y, '.r');