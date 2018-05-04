function Y = compute_obj_Angle(cur_Position ,OBJ,OBJ_NUM)
  for i=1:OBJ_NUM
      delta_obj_X(i) = OBJ(i,1) - cur_Position(1);
      delta_obj_Y(i) = OBJ(i,2) - cur_Position(2);
      rObj(i) = sqrt(delta_obj_X(i)^2 + delta_obj_Y(i)^2);
      Y(i) = sign ( delta_obj_Y(i) ) * acos ( delta_obj_X(i) / rObj(i) );
  end