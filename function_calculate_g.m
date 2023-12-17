function g = function_calculate_g(time, state, PARAMETERS)

      M =  PARAMETERS.PLANT.POLE_MASS + PARAMETERS.PLANT.CART_MASS;
      num = cos(state(1,1))/M; 
      den = PARAMETERS.PLANT.POLE_HALF_LENGTH*((4/3) - (PARAMETERS.PLANT.POLE_MASS*((cos(state(1,1)))^2)/M));
      g = num / den; 

end