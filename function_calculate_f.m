function f = function_calculate_f(time, state, PARAMETERS)
  M =  PARAMETERS.PLANT.POLE_MASS + PARAMETERS.PLANT.CART_MASS;
  num = PARAMETERS.PLANT.GRAVITY*sin(state(1,1)) - (PARAMETERS.PLANT.POLE_MASS*PARAMETERS.PLANT.POLE_HALF_LENGTH*((state(2,1))^2)*cos(state(1,1))*sin(state(1,1)))/M; 
  den = PARAMETERS.PLANT.POLE_HALF_LENGTH*((4/3) - (PARAMETERS.PLANT.POLE_MASS*((cos(state(1,1)))^2)/M));
  f = num / den; 
end