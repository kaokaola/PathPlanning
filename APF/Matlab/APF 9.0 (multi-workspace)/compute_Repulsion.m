function [Frer_X,Frer_Y] = compute_Repulsion(cur_Position ,goal_Position ,cir_OBJ ,tri_OBJ ,rec_OBJ ,cir_OBJ_NUM ,tri_OBJ_NUM ,rec_OBJ_NUM ,REP , INF_DISTANCE ,A ,theta_Goal)
     %计算出各个障碍物在x方向的斥力与在y方向的斥力（当前点 ，目标点 ，圆形障碍物 ，三角障碍物，矩形障碍物，圆形障碍物个数 ，三角障碍物个数，矩形障碍物个数，斥力计算参数，障碍影响距离，辅助参数 ,引力角度）
    
     if cir_OBJ_NUM >= 1    %计算圆形障碍物产生斥力
        [Frer_cir_X ,Frer_cir_Y] = compute_cir_Repulsion(cur_Position ,goal_Position ,cir_OBJ ,cir_OBJ_NUM ,REP , INF_DISTANCE ,A ,theta_Goal);
    else
        Frer_cir_X = 0.0 ;    
        Frer_cir_Y = 0.0 ; 
    end
    if tri_OBJ_NUM >= 1     %计算三角障碍物产生斥力
        [Frer_tri_X ,Frer_tri_Y] = compute_tri_Repulsion(cur_Position ,goal_Position ,tri_OBJ ,tri_OBJ_NUM ,REP , INF_DISTANCE ,A ,theta_Goal);
    else
        Frer_tri_X = 0.0 ;    
        Frer_tri_Y = 0.0 ; 
    end
    if rec_OBJ_NUM >= 1     %计算矩形障碍物产生斥力
        [Frer_rec_X ,Frer_rec_Y] = compute_rec_Repulsion(cur_Position ,goal_Position ,rec_OBJ ,rec_OBJ_NUM ,REP , INF_DISTANCE ,A ,theta_Goal);
    else
        Frer_rec_X = 0.0 ;    
        Frer_rec_Y = 0.0 ; 
    end
    Frer_X = Frer_cir_X + Frer_tri_X + Frer_rec_X ;           %所有障碍物X方向合力
    Frer_Y = Frer_cir_Y + Frer_tri_Y + Frer_rec_Y ;           %所有障碍物Y方向合力