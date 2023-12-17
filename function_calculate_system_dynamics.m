function [state_new] = function_calculate_system_dynamics(time, state, control, disturbance, PARAMETERS)
   
    f = function_calculate_f(time, state, PARAMETERS);
    g = function_calculate_g(time, state, PARAMETERS);            
    dot_x1 = state(2,1);
    dot_x2 = f + g*control + disturbance;
    x2_new = state(2,1) + dot_x2*PARAMETERS.SIMULATION.SAMPLING_TIME;
    x1_new = state(1,1) + dot_x1*PARAMETERS.SIMULATION.SAMPLING_TIME;
    state_new = [x1_new; x2_new];

end