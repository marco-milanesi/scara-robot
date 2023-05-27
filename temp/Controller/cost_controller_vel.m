function cost = cost_controller_vel(X,system,id_data)
kp=X(1);
Ti=X(2);
s=tf('s');
PI=kp*(1+1/(Ti*s));

end