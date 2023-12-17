% Configure
configuration;

PARAMETERS.DISTURBANCE.D_0 = 5;
PARAMETERS.DISTURBANCE.D_0_EST = PARAMETERS.DISTURBANCE.D_0 + 1;
PARAMETERS.PLOT.FONT_SIZE = 14;
PARAMETERS.PLOT.CREATE_PDF = 1;
PARAMETERS.SIMULATION.NUM_SIMULATIONS = 300;
PARAMETERS.SIMULATION.SCENARIO = 1;
if PARAMETERS.SIMULATION.SCENARIO == 1
    PARAMETERS.SIMULATION.SAMPLING_TIME = 1e-3;
    PARAMETERS.DISTURBANCE.TYPE = 1;
elseif PARAMETERS.SIMULATION.SCENARIO == 2
    PARAMETERS.SIMULATION.SAMPLING_TIME = 1e-3;
    PARAMETERS.DISTURBANCE.TYPE = 2;
elseif PARAMETERS.SIMULATION.SCENARIO == 3
    PARAMETERS.SIMULATION.SAMPLING_TIME = 1e-3;
    PARAMETERS.DISTURBANCE.TYPE = 3;
end
PARAMETERS.SIMULATION.TIME = 0:PARAMETERS.SIMULATION.SAMPLING_TIME:PARAMETERS.SIMULATION.TOTAL_TIME;
PARAMETERS.SIMULATION.TOTAL_STEPS = size(PARAMETERS.SIMULATION.TIME, 2);

t_s_bound = 0.005;
sum_steady_2_error_huijie = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
var_steady_error_huijie = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
max_control_huijie = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
ts_huijie = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
overhoot_huijie = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
sum_steady_2_error_pfxt = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
var_steady_error_pfxt = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
max_control_pfxt = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
overhoot_pfxt = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
ts_pfxt = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
sum_steady_2_error_he = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
var_steady_error_he = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
max_control_he = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
overhoot_he = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
ts_he = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
simulations = 1:PARAMETERS.SIMULATION.NUM_SIMULATIONS;

from_step = ceil(2*PARAMETERS.SIMULATION.TOTAL_STEPS/3);
MAX_D = PARAMETERS.DISTURBANCE.D_0;
for sim_number = 1:PARAMETERS.SIMULATION.NUM_SIMULATIONS

    sim_number
    D_0 = MAX_D*(2*rand(1)-1);
    PARAMETERS.DISTURBANCE.D_0 = D_0;
    rand_val = 2*rand(1)-1;
    if abs(rand_val) < 0.25
        rand_val = sign(rand_val)*(abs(rand_val) + 0.25);
    end
    x_1_0 = PARAMETERS.PLANT.X_1_MAX*(rand_val);
    rand_val = 2*rand(1)-1;
    if abs(rand_val) < 0.25
        rand_val = sign(rand_val)*(abs(rand_val) + 0.25);
    end
    x_2_0 = PARAMETERS.PLANT.X_2_MAX*(rand_val);
    PARAMETERS.PLANT.INITIAL_STATE = [x_1_0 x_2_0]';
    SIMULATION_DATA = run_simulation(PARAMETERS);

    error_pfxt = SIMULATION_DATA.REFERENCE(1,:) - SIMULATION_DATA.PFXT.STATE(1,:);
    error_he = SIMULATION_DATA.REFERENCE(1,:) - SIMULATION_DATA.HE.STATE(1,:);
    error_huijie = SIMULATION_DATA.REFERENCE(1,:) - SIMULATION_DATA.HUIJIE.STATE(1,:);
    
    huijie_steady_error = error_huijie(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS);
    sum_steady_2_error_huijie(1,sim_number) = sum(abs(huijie_steady_error));%.*huijie_steady_error);
    max_control_huijie(1,sim_number) = max(abs(SIMULATION_DATA.HUIJIE.CONTROL(1,:)));
    ts_huijie_index = find(abs(error_huijie) < t_s_bound);
    overhoot_huijie(1,sim_number) = max(abs(error_huijie(1,ts_huijie_index(1):PARAMETERS.SIMULATION.TOTAL_STEPS)));
    ts_huijie_index_diff = diff(ts_huijie_index);
    max_ts_huijie_index_diff = find(abs(ts_huijie_index_diff) > 1);
    if length(max_ts_huijie_index_diff) > 0
        ts_huijie_index = max_ts_huijie_index_diff(length(max_ts_huijie_index_diff));
    else
        ts_huijie_index = ts_huijie_index(1);
    end
    ts_huijie(1,sim_number) = PARAMETERS.SIMULATION.TIME(ts_huijie_index);
    

    pfxt_steady_error = error_pfxt(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS);
    sum_steady_2_error_pfxt(1,sim_number) = sum(abs(pfxt_steady_error));%sum(pfxt_steady_error.*pfxt_steady_error);
    max_control_pfxt(1,sim_number) = max(abs(SIMULATION_DATA.PFXT.CONTROL(1,:)));
    ts_pfxt_index = find(abs(error_pfxt) < t_s_bound);
    overhoot_pfxt(1,sim_number) = max(abs(error_pfxt(1,ts_pfxt_index(1):PARAMETERS.SIMULATION.TOTAL_STEPS)));
    ts_pfxt_index_diff = diff(ts_pfxt_index);
    max_ts_pfxt_index_diff = find(abs(ts_pfxt_index_diff) > 1);
    if length(max_ts_pfxt_index_diff) > 0
        ts_pfxt_index = max_ts_pfxt_index_diff(length(max_ts_pfxt_index_diff));   
    else
        ts_pfxt_index = ts_pfxt_index(1);
    end
    ts_pfxt(1,sim_number) = PARAMETERS.SIMULATION.TIME(ts_pfxt_index);
    

    he_steady_error = error_he(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS);
    sum_steady_2_error_he(1,sim_number) = sum(abs(he_steady_error));%sum(he_steady_error.*he_steady_error);
    max_control_he(1,sim_number) = max(abs(SIMULATION_DATA.HE.CONTROL(1,:)));
    ts_he_index = find(abs(error_he) < t_s_bound);
    overhoot_he(1,sim_number) = max(abs(error_he(1,ts_he_index(1):PARAMETERS.SIMULATION.TOTAL_STEPS)));
    ts_he_index_diff = diff(ts_he_index);
    max_ts_he_index_diff = find(abs(ts_he_index_diff) > 1);
    if length(max_ts_he_index_diff) > 0
        ts_he_index = max_ts_he_index_diff(length(max_ts_he_index_diff));   
    else
        ts_he_index = ts_he_index(1);
    end  
    ts_he(1,sim_number) = PARAMETERS.SIMULATION.TIME(ts_he_index);    
    
    
end
legends = {'Gonzalez-Prieto', 'He','Huijie'};
sum_steady_2_error_pfxt = sum_steady_2_error_pfxt / PARAMETERS.SIMULATION.NUM_SIMULATIONS;
sum_steady_2_error_he = sum_steady_2_error_he / PARAMETERS.SIMULATION.NUM_SIMULATIONS;
sum_steady_2_error_huijie = sum_steady_2_error_huijie / PARAMETERS.SIMULATION.NUM_SIMULATIONS;

num_cols = 20;
fig30 = figure(30);
clf(fig30);
subplot(3,3,1);
histogram(sum_steady_2_error_pfxt, num_cols, 'FaceColor', 'r', 'Normalization','probability');%, 'FaceAlpha', 0.5);
grid on;
hold on;
xlabel ('$(\Sigma |e|)/N$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
ylabel('Frequency');
title(legends{1}, 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
subplot(3,3,2);
histogram(sum_steady_2_error_he, num_cols, 'FaceColor', 'k', 'Normalization','probability');%, 'FaceAlpha', 0.5);
grid on;
hold on;
xlabel ('$(\Sigma |e|)/N$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
ylabel('Frequency');
title(legends{2}, 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
subplot(3,3,3);
histogram(sum_steady_2_error_huijie, num_cols, 'FaceColor', 'b', 'Normalization','probability');%, 'FaceAlpha', 0.5);
grid on;
hold on;
xlabel ('$(\Sigma |e|)/N$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
ylabel('Frequency');
title(legends{3}, 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
subplot(3,3,4);
histogram(ts_pfxt, num_cols, 'FaceColor', 'r', 'Normalization','probability');%, 'FaceAlpha', 0.5);
grid on;
hold on;
xlabel ('Settling time', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
ylabel('Frequency');
title(legends{1}, 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
subplot(3,3,5);
histogram(ts_he, num_cols, 'FaceColor', 'k', 'Normalization','probability');%, 'FaceAlpha', 0.5);
grid on;
hold on;
xlabel ('Settling time', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
ylabel('Frequency');
title(legends{2}, 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
subplot(3,3,6);
histogram(ts_huijie, num_cols, 'FaceColor', 'b', 'Normalization','probability');%, 'FaceAlpha', 0.5);
grid on;
hold on;
xlabel ('Settling time', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
ylabel('Frequency');
title(legends{3}, 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
subplot(3,3,7);
histogram(overhoot_pfxt, num_cols, 'FaceColor', 'r', 'Normalization','probability');%, 'FaceAlpha', 0.5);
grid on;
hold on;
xlabel ('Overshoot', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
ylabel('Frequency');
title(legends{1}, 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
subplot(3,3,8);
histogram(overhoot_he, num_cols, 'FaceColor', 'k', 'Normalization','probability');%, 'FaceAlpha', 0.5);
grid on;
hold on;
xlabel ('Overshoot', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
ylabel('Frequency');
title(legends{2}, 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
subplot(3,3,9);
histogram(overhoot_huijie, num_cols, 'FaceColor', 'b', 'Normalization','probability');%, 'FaceAlpha', 0.5);
grid on;
hold on;
xlabel ('Overshoot', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
ylabel('Frequency');
title(legends{3}, 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');

if PARAMETERS.PLOT.CREATE_PDF
    set(gcf, 'color', 'white') 
    set(gca, 'color', 'white');
    graph_file_path = strcat('../../MANUSCRIPT/GRAPHICS/scenario_', num2str(PARAMETERS.SIMULATION.SCENARIO), '_multiple_simulation_histogram.pdf')
    export_fig(graph_file_path, '-nocrop'); %, '-transparent'
end


fig20 = figure(20);
clf(fig20);
subplot(4,1,1);
plot(simulations, sum_steady_2_error_pfxt, '--.r', 'LineWidth', 0.5,'MarkerSize',20);
grid on;
hold on;
plot(simulations, sum_steady_2_error_he, '--.k', 'LineWidth', 0.5,'MarkerSize',20);
plot(simulations, sum_steady_2_error_huijie, '--.b', 'LineWidth', 0.5,'MarkerSize',20);
ylabel ('$(\Sigma e^2)/N$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
% xlabel('Simulations', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
title('Steady state squared error mean', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
xlim([1, PARAMETERS.SIMULATION.NUM_SIMULATIONS]); 
legend(legends);
subplot(4,1,2);
plot(simulations, ts_pfxt, '--.r', 'LineWidth', 0.5,'MarkerSize',20);
grid on;
hold on;
plot(simulations, ts_he, '--.k', 'LineWidth', 0.5,'MarkerSize',20);
plot(simulations, ts_huijie, '--.b', 'LineWidth', 0.5,'MarkerSize',20);
ylabel ('$t_s$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
% xlabel('Simulations', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
title('Settling time', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
xlim([1, PARAMETERS.SIMULATION.NUM_SIMULATIONS]);
subplot(4,1,3);
plot(simulations, overhoot_pfxt, '--.r', 'LineWidth', 0.5,'MarkerSize',20);
grid on;
hold on;
plot(simulations, overhoot_he, '--.k', 'LineWidth', 0.5,'MarkerSize',20);
plot(simulations, overhoot_huijie, '--.b', 'LineWidth', 0.5,'MarkerSize',20);
ylabel ('Overshoot', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
title('Overshoot', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
xlim([1, PARAMETERS.SIMULATION.NUM_SIMULATIONS]);
legend(legends);
subplot(4,1,4);
plot(simulations, max_control_pfxt, '--.r', 'LineWidth', 0.5,'MarkerSize',20);
grid on;
hold on;
plot(simulations, max_control_he, '--.k', 'LineWidth', 0.5,'MarkerSize',20);
plot(simulations, max_control_huijie, '--.b', 'LineWidth', 0.5,'MarkerSize',20);
ylabel ('$max(|u|)$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
title('Maximum control value', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
xlim([1, PARAMETERS.SIMULATION.NUM_SIMULATIONS]);

% if PARAMETERS.PLOT.CREATE_PDF
%     graph_file_path = strcat('../../MANUSCRIPT/GRAPHICS/scenario_', num2str(PARAMETERS.SIMULATION.SCENARIO), '_multiple_simulation.pdf')
%     export_fig(graph_file_path, '-transparent', '-nocrop');
% end