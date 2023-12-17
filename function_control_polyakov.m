function [control, control_state_new] = function_control_polyakov(time, reference, state, control_state, PARAMETERS)
    % Parameters

    alfa1 = 2.0;
    beta1 = 2.0;
    alfa2 = 1.0;    
    beta2 = 1.0;
    gamma = 2.0;

    % Read states and create errors
    e1 = state(1,1) - reference(1,1);
    e2 = state(2,1) - reference(2,1);


    % Sliding surface
    s = e2 + function_fractional_power(function_fractional_power(e2,2) + alfa1*e1 + beta1*function_fractional_power(e1,3), 0.5);
    f = function_calculate_f(time, state, PARAMETERS);
    g = function_calculate_g(time, state, PARAMETERS);
    control = (-1/g)*(f - reference(3,1) + gamma*sign(s) + ((alfa1*+3*beta1*e1^2)/2.0)*sign(s) + 1*function_fractional_power(alfa2*s+beta2*function_fractional_power(s, 3), 0.5));
    control_state_new = control_state;
end

