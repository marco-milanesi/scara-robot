function [J, Y] = cascade_optimization_cost(x,P,umax,st,reference)

% Controller 1 (outer loop)
Kp1 = x(1);
Ti1 = x(2);

% Controller 2 (inner loop)
Kp2 = x(3); 

s=tf('s');
C1 = Controller(st,Kp1*(1+1/(s*Ti1)),1);
C1.setUMax(umax);
C2 = Controller(st,tf(Kp2),1);
C2.setUMax(umax);

ss_vel=ss(P);
ss_vel_states=ss(ss_vel.A,ss_vel.B,eye(size(ss_vel.A)),zeros(size(ss_vel.A,1),size(ss_vel.B,2)));

ss_pos=ss(P/s);
ss_pos_states=ss(ss_pos.A,ss_pos.B,eye(size(ss_pos.A)),zeros(size(ss_pos.A,1),size(ss_pos.B,2)));

y0=[0 0];
y=y0;
error=zeros(1,length(reference));
Y=zeros(1,length(reference));
x0_vel=zeros(1,size(ss_vel.A,1));
x0_pos=zeros(1,size(ss_pos.A,1));
for i = 1 : length(reference)
    set_vel=C1.computeControlAction(reference(i),y(1));
    torque = C2.computeControlAction(set_vel,y(2));
    vel=lsim(ss_vel,torque*ones(1,2),[0 st],x0_vel);
    x0_vel=lsim(ss_vel_states,torque*ones(1,2),[0 st],x0_vel);
    pos=lsim(ss_pos,torque*ones(1,2),[0 st],x0_pos);
    x0_pos=lsim(ss_pos_states,torque*ones(1,2),[0 st],x0_pos);
    y=[pos(end) vel(end)];
    x0_pos = x0_pos(end,:);
    x0_vel = x0_vel(end,:);
    error(i)=reference(i)-y(1);
    Y(i)=y(1);
end
J=sum(abs(error*st));
end

