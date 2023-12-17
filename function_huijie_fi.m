function [fi1, fi2] = function_huijie_fi(x, alfa, beta)
    fi1 = function_fraction_power(x, alfa) + function_fraction_power(x, beta);
    fi2 = function_fraction_power(x, 2*alfa-1) + function_fraction_power(x, 2*beta-1);
end