function power_value = function_fraction_power_approximation(value, delta, kappa, gamma)
   power_value = (1-delta)*kappa*tanh(gamma*value) + delta*value;
end