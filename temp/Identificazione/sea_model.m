function [A,B,C,D] = sea_model(Jm,Jl,k,c,cm,Ts)
% ODE function for computing state-space matrices as functions of parameters
%
% Jm motor inertia 
% Jl link inertia
% k spring stiffness
% c spring damping
% cm motor viscuous friction
%
% x=[motor_pos, link_pos, motor_vel, link_vel]
%
% u=motor_eff
%
% Jm*Derivative(motor_vel) = u - cm*motor_vel -k*(motor_pos-link_pos) - c*(motor_vel-link_vel)
%
% Jl*Derivative(link_vel)  = k*(motor_pos-link_pos) + c*(motor_vel-link_vel)

A=[0 0 1 0;
    0 0 0 1;
    [-k k -cm-c c]/Jm;
    [k -k c -c]/Jl];
B=[0;
   0;
   1/Jm;
   0];
C=[1 0 0 0;
    0 1 0 0];
D=[0;0];