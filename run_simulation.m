function [SIMULATION_DATA, PARAMETERS] = run_simulation(PARAMETERS)
     
    % Prepare simulation data
    SIMULATION_DATA = {};
    SIMULATION_DATA.REFERENCE = zeros(3, PARAMETERS.SIMULATION.TOTAL_STEPS); % x1d,x2d, dot_x2d
    SIMULATION_DATA.TIME = zeros(1, PARAMETERS.SIMULATION.TOTAL_STEPS);
    
    % HUIJIE
    SIMULATION_DATA.HUIJIE.DISTURBANCE = zeros(1, PARAMETERS.SIMULATION.TOTAL_STEPS); 
    SIMULATION_DATA.HUIJIE.STATE = zeros(2, PARAMETERS.SIMULATION.TOTAL_STEPS); 
    huijie_state = PARAMETERS.PLANT.INITIAL_STATE; 
    SIMULATION_DATA.HUIJIE.CONTROL_STATE = zeros(20, PARAMETERS.SIMULATION.TOTAL_STEPS);
    huijie_control_state = zeros(20,1);
    SIMULATION_DATA.HUIJIE.CONTROL = zeros(1, PARAMETERS.SIMULATION.TOTAL_STEPS);

    % PFXT
    SIMULATION_DATA.PFXT.DISTURBANCE = zeros(1, PARAMETERS.SIMULATION.TOTAL_STEPS); 
    SIMULATION_DATA.PFXT.STATE = zeros(2, PARAMETERS.SIMULATION.TOTAL_STEPS); 
    pfxt_state = PARAMETERS.PLANT.INITIAL_STATE; 
    SIMULATION_DATA.PFXT.CONTROL_STATE = zeros(20, PARAMETERS.SIMULATION.TOTAL_STEPS);
    pfxt_control_state = zeros(20,1);
    SIMULATION_DATA.PFXT.CONTROL = zeros(1, PARAMETERS.SIMULATION.TOTAL_STEPS);

    % HE
    SIMULATION_DATA.HE.DISTURBANCE = zeros(1, PARAMETERS.SIMULATION.TOTAL_STEPS); 
    SIMULATION_DATA.HE.STATE = zeros(2, PARAMETERS.SIMULATION.TOTAL_STEPS); 
    he_state = PARAMETERS.PLANT.INITIAL_STATE; 
    SIMULATION_DATA.HE.CONTROL_STATE = zeros(20, PARAMETERS.SIMULATION.TOTAL_STEPS);
    he_control_state = zeros(20,1);
    SIMULATION_DATA.HE.CONTROL = zeros(1, PARAMETERS.SIMULATION.TOTAL_STEPS);

    % Run simulation
    simulation_time = 0.0;
   
    for simulation_step = 1:PARAMETERS.SIMULATION.TOTAL_STEPS
        [reference] = function_calculate_reference(simulation_time, PARAMETERS);

        % Save data
        SIMULATION_DATA.TIME (:, simulation_step) = simulation_time;
        SIMULATION_DATA.REFERENCE(:, simulation_step) = reference;
        SIMULATION_DATA.HUIJIE.STATE(:, simulation_step) = huijie_state;
        SIMULATION_DATA.PFXT.STATE(:, simulation_step) = pfxt_state;
        SIMULATION_DATA.HE.STATE(:, simulation_step) = he_state;

        % PFXT
        [pfxt_disturbance, PARAMETERS] = function_calculate_disturbance(simulation_time, pfxt_state, PARAMETERS);    
        z_min_1 = PARAMETERS.CONTROL.Z_MIN_1;
        [pfxt_control, pfxt_control_state] = function_control_pfxt(simulation_time, reference, pfxt_state, pfxt_control_state, PARAMETERS, z_min_1, e_min_1);
        [pfxt_state] = function_calculate_system_dynamics(simulation_time, pfxt_state, pfxt_control, pfxt_disturbance, PARAMETERS);
        SIMULATION_DATA.PFXT.CONTROL(:, simulation_step) = pfxt_control;
        SIMULATION_DATA.PFXT.CONTROL_STATE(:, simulation_step) = pfxt_control_state;
        SIMULATION_DATA.PFXT.DISTURBANCE(:, simulation_step) = pfxt_disturbance;

        % HE
        [he_disturbance, PARAMETERS] = function_calculate_disturbance(simulation_time, he_state, PARAMETERS); 
        [he_control, he_control_state] = function_control_he(simulation_time, reference, he_state, he_control_state, PARAMETERS);
        [he_state] = function_calculate_system_dynamics(simulation_time, he_state, he_control, he_disturbance, PARAMETERS);
        SIMULATION_DATA.HE.CONTROL(:, simulation_step) = he_control;
        SIMULATION_DATA.HE.CONTROL_STATE(:, simulation_step) = he_control_state;
        SIMULATION_DATA.HE.DISTURBANCE(:, simulation_step) = he_disturbance;

        % HUIJIE
        [huijie_disturbance, PARAMETERS] = function_calculate_disturbance(simulation_time, huijie_state, PARAMETERS);    
        [huijie_control, huijie_control_state] = function_control_huijie(simulation_time, reference, huijie_state, huijie_control_state, PARAMETERS);
        [huijie_state] = function_calculate_system_dynamics(simulation_time, huijie_state, huijie_control, huijie_disturbance, PARAMETERS);
        SIMULATION_DATA.HUIJIE.CONTROL(:, simulation_step) = huijie_control;
        SIMULATION_DATA.HUIJIE.CONTROL_STATE(:, simulation_step) = huijie_control_state;
        SIMULATION_DATA.HUIJIE.DISTURBANCE(:, simulation_step) = huijie_disturbance;


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Update time
        simulation_time = simulation_time + PARAMETERS.SIMULATION.SAMPLING_TIME;
    end
end
        