function [reference] = function_calculate_reference(time, PARAMETERS)
    reference = zeros(3,1);
    omega = 0.5;
    reference(1,1) = cos(omega*pi*time);
    reference(2,1) = -omega*pi*sin(omega*pi*time);
    reference(3,1) = -omega*pi*omega*pi*cos(omega*pi*time);
end