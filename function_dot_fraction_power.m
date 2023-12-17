function x_fraction_power = function_dot_fraction_power(x, power)
   x_fraction_power = power*((abs(x))^(power-1))*sign(x);
end