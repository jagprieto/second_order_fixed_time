function [control, control_state_new] = function_control_he(time, reference, state, control_state, PARAMETERS)
%     disp('.....................................................')    
    % Read states 
    x_1 = state(1,1);
    x_2 = state(2,1);
    x_1_r = reference(1,1);
    x_2_r = reference(2,1);
    dot_x_2_r = reference(3,1);

    %%%%%%%%%%%%%%%%%%%%%%%% DYNAMICS FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%
    % Known functions
    f = function_calculate_f(time, state, PARAMETERS);
    g = function_calculate_g(time, state, PARAMETERS);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    alfa1 = 13/15;
    alfa2 = 5/9;
    k1 = 2;
    k2 = 1;
    o1 = 2;
    o2 = 1;
    gamma = 2;
    zeta = 2;
    e1t = 1.1;
    st = 1.1;

    e_1 = x_1 - x_1_r;
    e_2 = x_2 - x_2_r;

    if abs(e_1) > e1t
        s = e_2 + k1*(abs(e_1)^alfa1)*sign(e_1) + k2*(abs(e_1)^(2-alfa1))*sign(e_1);
    else
        s = e_2 + k1*e_1 + k2*(abs(e_1)^alfa1)*sign(e_1);
    end
    e_1_e2 = (abs(e_1)^(alfa1-1))*sign(e_1)*e_2;
    e_1_e2_zeta = min(zeta, abs(e_1_e2))*sign(e_1_e2);
    sign_e1_1t = ((k1+k2)/2) + ((k2-k1)/2)*sign(abs(e_1) - e1t);
    sign_e1_1t = sign_e1_1t * ( ((3-alfa1)/2) + ((1-alfa1)/2)*sign(abs(e_1) - e1t) );
    sign_e1_1t = sign_e1_1t * ( abs(e_1)^( ((1-alfa1)/2) + ((1-alfa1)/2)*sign(abs(e_1) - e1t) )*sign(e_1)*e_2 );
    o_1_s = ((o1+o2)/2) + ((o2-o1)/2)*sign(abs(s) - st);
    o_1_s = o_1_s * ( abs(s)^( ((3-alfa2)/2) + ((1-alfa2)/2)*sign(abs(s) - st) )*sign(s)  );
    control = (-1/g)*(f - dot_x_2_r + gamma*sign(s) + k1*alfa1*e_1_e2_zeta + sign_e1_1t + o1*(abs(s)^alfa2)*sign(s) + o_1_s);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SAVE CONTROL STATE DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    control_state_new = control_state; 
    control_state_new(5) = s;
    control_state_new(6) = e_1; 
    control_state_new(7) = e_2; 
end

