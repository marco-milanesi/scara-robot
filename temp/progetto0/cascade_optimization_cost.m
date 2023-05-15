function J = cascade_optimization_cost(x,P,umax)

% Controller 1 (outer loop)
Kp1 = x(1);
Ti1 = x(2);

% Controller 2 (inner loop)
Kp2 = x(3); 

s=tf('s');
C1 = Kp1*(1+1/(s*Ti1));
C2 = Kp2;



inner_loop = C2*P;




end

