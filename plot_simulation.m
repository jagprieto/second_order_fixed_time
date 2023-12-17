
% Plot simulation 
function plot_simulation(SIMULATION_DATA, PARAMETERS)
    legends = {'Gonzalez-Prieto', 'He','Huijie'};

    % Plot errors
    fig1 = figure(1);
    clf(fig1);
    subplot(2,1,1);
    plot(SIMULATION_DATA.TIME(1,:), SIMULATION_DATA.REFERENCE(1,:) - SIMULATION_DATA.PFXT.STATE(1,:),'-', 'Color', 'r', 'LineWidth', 2.0);   
    grid on;
    hold on;
    plot(SIMULATION_DATA.TIME(1,:), SIMULATION_DATA.REFERENCE(1,:) - SIMULATION_DATA.HE.STATE(1,:),'-', 'Color', 'k', 'LineWidth', 2.0);   
    plot(SIMULATION_DATA.TIME(1,:), SIMULATION_DATA.REFERENCE(1,:) - SIMULATION_DATA.HUIJIE.STATE(1,:) ,'-', 'Color', 'b', 'LineWidth', 2.0);
    xlabel ('Time (s)', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    ylabel ('$e_x(t)$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    title('Position error: $e_x(t) = x^d_1(t)-x_1(t)$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    xlim([0.0, PARAMETERS.SIMULATION.TOTAL_TIME]);  
    legend(legends); 
    subplot(2,1,2);
    plot(SIMULATION_DATA.TIME(1,:), SIMULATION_DATA.REFERENCE(2,:) - SIMULATION_DATA.PFXT.STATE(2,:),'-', 'Color', 'r', 'LineWidth', 2.0);
    grid on;
    hold on;
    plot(SIMULATION_DATA.TIME(1,:), SIMULATION_DATA.REFERENCE(2,:) - SIMULATION_DATA.HE.STATE(2,:),'-', 'Color', 'k', 'LineWidth', 2.0);
    plot(SIMULATION_DATA.TIME(1,:),SIMULATION_DATA.REFERENCE(2,:) - SIMULATION_DATA.HUIJIE.STATE(2,:) ,'-', 'Color', 'b', 'LineWidth', 2.0);
    ylabel ('$\dot{e}_{x}(t)$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    title('Velocity error: $\dot{e}_{x} = x^d_2(t)-x_2(t)$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    xlim([0.0, PARAMETERS.SIMULATION.TOTAL_TIME]);  
   
    if PARAMETERS.PLOT.CREATE_PDF
        graph_file_path = strcat('../../MANUSCRIPT/GRAPHICS/scenario_', num2str(PARAMETERS.SIMULATION.SCENARIO), '_errors.pdf')
        export_fig(graph_file_path, '-transparent', '-nocrop');
    end
    
  
    % Plot control and steady state error
    fig2 = figure(2);
    clf(fig2);
    subplot(2,1,1);
    plot(SIMULATION_DATA.TIME, SIMULATION_DATA.PFXT.CONTROL(1,:),'-', 'Color', 'r', 'LineWidth', 2.0);   
    grid on;
    hold on; 
    plot(SIMULATION_DATA.TIME, SIMULATION_DATA.HE.CONTROL(1,:),'-', 'Color', 'k', 'LineWidth', 2.0);   
    plot(SIMULATION_DATA.TIME, SIMULATION_DATA.HUIJIE.CONTROL(1,:),'-', 'Color', 'b', 'LineWidth', 2.0); 
    xlabel('Time (s)', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    ylabel ('u(t)', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    title('Control effort: u(t)', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    xlim([0.0, PARAMETERS.SIMULATION.TOTAL_TIME]);  
    legend(legends);  
    from_step = ceil(PARAMETERS.SIMULATION.TOTAL_STEPS/2);
    huijie_steady_error = SIMULATION_DATA.REFERENCE(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS) - SIMULATION_DATA.HUIJIE.STATE(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS);
    pfxt_steady_error = SIMULATION_DATA.REFERENCE(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS) - SIMULATION_DATA.PFXT.STATE(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS);
    he_steady_error = SIMULATION_DATA.REFERENCE(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS) - SIMULATION_DATA.HE.STATE(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS);
    subplot(2,3,4);
    histogram(pfxt_steady_error, 50, 'FaceColor', 'r', 'Normalization','probability');%, 'FaceAlpha', 0.5);
    grid on;
    hold on;
    xlabel ('Steady state error', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    ylabel ('Frequency', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    title('Gonzalez-Prieto', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    subplot(2,3,5);

    histogram(he_steady_error, 50, 'FaceColor', 'k', 'Normalization','probability');%, 'FaceAlpha', 0.5);
    grid on;
    hold on;
    xlabel ('Steady state error', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    ylabel ('Frequency', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    title('He', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    subplot(2,3,6);
    histogram(huijie_steady_error, 50, 'FaceColor', 'b', 'Normalization','probability');
    grid on;
    hold on;
    xlabel ('Steady state error', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    ylabel ('Frequency', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    title('Huijie', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    if PARAMETERS.PLOT.CREATE_PDF
        set(gcf, 'color', 'white') 
        set(gca, 'color','white');
        graph_file_path = strcat('../../MANUSCRIPT/GRAPHICS/scenario_', num2str(PARAMETERS.SIMULATION.SCENARIO), '_control_steady_state_error.pdf')
        export_fig(graph_file_path, '-nocrop'); %, '-transparent'
    end

    % Plot error variables
    fig3 = figure(3);
    clf(fig3);
    subplot(2,1,1);
    plot(SIMULATION_DATA.TIME, (SIMULATION_DATA.PFXT.CONTROL_STATE(5,:)),'-', 'Color', 'r', 'LineWidth', 2.0);  
    grid on;
    hold on;
    xlabel ('Time (s)', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    ylabel ('$s(t)$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    title('Evolution of s(t)', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    xlim([0.0, 1]);  
    legend(legends); 
    subplot(2,1,2);
    plot(SIMULATION_DATA.TIME, (SIMULATION_DATA.PFXT.CONTROL_STATE(6,:)),'-', 'Color', 'r', 'LineWidth', 2.0);  
    grid on;
    hold on;
    ylabel ('$e_1(t)$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    xlabel ('Time (s)', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    title('Position tracking error $e_1(t)$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    xlim([0.0, 1]);
    if PARAMETERS.PLOT.CREATE_PDF
        graph_file_path = strcat('../../MANUSCRIPT/GRAPHICS/scenario_', num2str(PARAMETERS.SIMULATION.SCENARIO), '_s_e.pdf')
        export_fig(graph_file_path, '-transparent', '-nocrop');
    end
end

