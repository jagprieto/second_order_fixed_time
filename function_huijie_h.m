function [h_x1, doth_x1] = function_huijie_h(x1, x2, gamma2, epsilon)
   beta1 = ((gamma2-2)*(gamma2-3)/2.0)*epsilon^(gamma2-1.0);
   beta2 = -(gamma2-1)*(gamma2-3)*epsilon^(gamma2-2.0);
   beta3 = ((gamma2-1)*(gamma2-2)/2.0)*epsilon^(gamma2-3.0);
   if abs(x1) < epsilon
       h_x1 = beta1*x1 + beta2*function_fraction_power(x1, 2) + beta3*x1^3;
        doth_x1 = (beta1 + 2.0*beta2*abs(x1) + 3*beta3*x1^2)*x2;
   else
       h_x1 = function_fraction_power(x1, gamma2);
        doth_x1 = gamma2*abs(x1)^(gamma2-1)*x2;
   end
end