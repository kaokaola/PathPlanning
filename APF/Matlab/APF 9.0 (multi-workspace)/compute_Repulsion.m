function [Frer_X,Frer_Y] = compute_Repulsion(cur_Position ,goal_Position ,cir_OBJ ,tri_OBJ ,rec_OBJ ,cir_OBJ_NUM ,tri_OBJ_NUM ,rec_OBJ_NUM ,REP , INF_DISTANCE ,A ,theta_Goal)
     %����������ϰ�����x����ĳ�������y����ĳ�������ǰ�� ��Ŀ��� ��Բ���ϰ��� �������ϰ�������ϰ��Բ���ϰ������ �������ϰ�������������ϰ����������������������ϰ�Ӱ����룬�������� ,�����Ƕȣ�
    
     if cir_OBJ_NUM >= 1    %����Բ���ϰ����������
        [Frer_cir_X ,Frer_cir_Y] = compute_cir_Repulsion(cur_Position ,goal_Position ,cir_OBJ ,cir_OBJ_NUM ,REP , INF_DISTANCE ,A ,theta_Goal);
    else
        Frer_cir_X = 0.0 ;    
        Frer_cir_Y = 0.0 ; 
    end
    if tri_OBJ_NUM >= 1     %���������ϰ����������
        [Frer_tri_X ,Frer_tri_Y] = compute_tri_Repulsion(cur_Position ,goal_Position ,tri_OBJ ,tri_OBJ_NUM ,REP , INF_DISTANCE ,A ,theta_Goal);
    else
        Frer_tri_X = 0.0 ;    
        Frer_tri_Y = 0.0 ; 
    end
    if rec_OBJ_NUM >= 1     %��������ϰ����������
        [Frer_rec_X ,Frer_rec_Y] = compute_rec_Repulsion(cur_Position ,goal_Position ,rec_OBJ ,rec_OBJ_NUM ,REP , INF_DISTANCE ,A ,theta_Goal);
    else
        Frer_rec_X = 0.0 ;    
        Frer_rec_Y = 0.0 ; 
    end
    Frer_X = Frer_cir_X + Frer_tri_X + Frer_rec_X ;           %�����ϰ���X�������
    Frer_Y = Frer_cir_Y + Frer_tri_Y + Frer_rec_Y ;           %�����ϰ���Y�������