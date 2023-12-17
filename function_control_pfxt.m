function [control, control_state_new] = function_control_pfxt(time, reference, state, control_state, PARAMETERS, z_min, e_min)
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

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PFxT virtual tracking error %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    DOT_D = PARAMETERS.DISTURBANCE.DOT_FREQUENCY_MAX;
    t_z = PARAMETERS.SIMULATION.SETTLING_TIME;
    delta_ = 0.2768;
    mu = PARAMETERS.CONTROL.MU;
    p = 1 - (1/mu);
    q = 1 + (1/mu);    
    bar_p = 2*p - 1;
    bar_q = 2*q - 1;    
    a_z = (pi*mu)/(2*t_z*sqrt(2^(q-p)));
    b_z = a_z*2^(2/mu);
    alfa_z = a_z / 2^p;
    beta_z = b_z / 2^q;  
    if time < PARAMETERS.SIMULATION.SAMPLING_TIME
        x_1_c = x_1;
    else
        x_1_c = control_state(10);
    end
    z_1 = x_1_c - x_1_r;
    kappa_z = 1/mu;
    gamma_z = tanh(1.0/kappa_z) / z_min;
    Lambda_z_1 = alfa_z*function_fraction_power_approximation(z_1, bar_p, kappa_z, gamma_z)  + (beta_z*((abs(z_1))^bar_q))*sign(z_1); 
    dot_x_1_c = x_2_r - Lambda_z_1;
    z_2 = dot_x_1_c - x_2_r;
    dot_Lambda_z_1 = (alfa_z*function_dot_fraction_power_approximation(z_1, bar_p, kappa_z, gamma_z) + beta_z*bar_q*((abs(z_1))^(bar_q-1))*sign(z_1))*z_2;
    ddot_x_1_c = dot_x_2_r - dot_Lambda_z_1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TRACKING ERRORS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    e_1 = x_1 - x_1_c;
    e_2 = x_2 - dot_x_1_c;  
    alfa = PARAMETERS.CONTROL.GAIN*alfa_z;
    c_e = 1/delta_;
    tau_inv = 1/PARAMETERS.SIMULATION.SAMPLING_TIME;
    c_w = 1 / (2*tau_inv - alfa - 1);
    alfa_s = (alfa+c_e+1)/2; 
    alfa_e = (alfa+1)/2;
    alfa_w = (c_w*(alfa+1)+1)/(2*c_w);
    kappa_s = DOT_D;
    kappa_e = (c_w*DOT_D^2) / (2*c_e*z_min);  
    gamma_s = ((1/PARAMETERS.SIMULATION.SAMPLING_TIME)-alfa_s)/kappa_s; 
    gamma_e = c_e*kappa_e*gamma_s / kappa_s;
    Lambda_e_1 = alfa_e*e_1 + kappa_e*tanh(gamma_e*e_1);
    dot_Lambda_e_1 = alfa_e*e_2 + kappa_e*gamma_e*((sech(gamma_e*e_1))^2)*e_2;
    s = e_2 + Lambda_e_1;
    if time < PARAMETERS.SIMULATION.SAMPLING_TIME
        phi_s = -alfa_w*s;
    else
        phi_s = control_state(12);
    end
    Lambda_s = alfa_s*s + kappa_s*tanh(gamma_s*s);
    dot_phi_s = alfa_w*Lambda_s;
    hat_d = alfa_w*s + phi_s;
    control = (1/g)*(ddot_x_1_c - f - dot_Lambda_e_1 - Lambda_s - hat_d);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SAVE CONTROL STATE DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    control_state_new = control_state; 
    control_state_new(4) = hat_d;
    control_state_new(5) = s;
    control_state_new(6) = e_1; 
    control_state_new(7) = e_2; 
    control_state_new(8) = abs(Lambda_s);
    control_state_new(9) = abs(Lambda_e_1);
    control_state_new(10) = x_1_c + dot_x_1_c*PARAMETERS.SIMULATION.SAMPLING_TIME;
    control_state_new(11) = alfa_s;
    control_state_new(12) = phi_s + dot_phi_s*PARAMETERS.SIMULATION.SAMPLING_TIME;
    control_state_new(13) = z_1;
    control_state_new(14) = kappa_s;
    control_state_new(15) = c_e;
end

