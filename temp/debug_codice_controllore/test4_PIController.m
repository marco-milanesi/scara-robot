clear all;clc;close all
s=tf("s");
st=1e-3;
Kp=5; % valori taratura
Ki=5; % valori taratura
umax=150;

PI=(Kp+Ki/s);
ctrl=Controller(st,PI,1);
ctrl.setUMax(umax);


%% TEST CLOSE LOOP
s=tf('s');

ctrl_continuo=Kp+Ki/s;
ctrl_discreto=c2d(ctrl_continuo,st);

P_continuo=1/(s+1); % modello identificato

P_discreto=c2d(P_continuo,st);
[A,B,C,D]=ssdata(P_discreto);

Y_over_R_discreto=feedback(P_discreto*ctrl_discreto,1);
U_over_R_discreto=feedback(ctrl_discreto,P_discreto);

Y_over_R_continuo=feedback(P_continuo*ctrl_continuo,1);
U_over_R_continuo=feedback(ctrl_continuo,P_continuo);

time=(0:st:30)';
reference=time>2; % step
noise=0.01*randn(length(time),1);

y_close_loop_matlab_discreto=lsim(Y_over_R_discreto,reference,time);
u_close_loop_matlab_discreto=lsim(U_over_R_discreto,reference,time);

y_close_loop_matlab_continuo=lsim(Y_over_R_continuo,reference,time);
u_close_loop_matlab_continuo=lsim(U_over_R_continuo,reference,time);


x_processo=zeros(order(P_discreto),1);
ctrl.initialize;

y_close_loop_class=nan(length(time),1);
u_close_loop_class=nan(length(time),1);
for idx=1:length(time)
    y_close_loop_class(idx,1)=C*x_processo+noise(idx,1);
    u_close_loop_class(idx,1)=ctrl.computeControlAction(reference(idx),y_close_loop_class(idx,1));
    x_processo=A*x_processo+B*u_close_loop_class(idx,1);
end


figure(3)
subplot(2,1,1)
stairs(time,y_close_loop_matlab_discreto)
hold on
stairs(time,y_close_loop_matlab_continuo)
stairs(time,y_close_loop_class)
hold off
grid on
xlabel('time')
ylabel('process output')
legend('matlab discreto','matlab continuo','class')

subplot(2,1,2)
stairs(time,u_close_loop_matlab_discreto)
hold on
stairs(time,u_close_loop_matlab_continuo)
stairs(time,u_close_loop_class)
hold off
grid on
xlabel('time')
ylabel('control action')
legend('matlab discreto','matlab continuo','class')

drawnow
