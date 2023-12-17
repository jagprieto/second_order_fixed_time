function power_value = function_dot_fraction_power_approximation(value, delta, kappa, gamma)
   power_value = (1-delta)*kappa*gamma*((sech(gamma*value))^2) + delta;
end