function [J, Y, Wc] = cascade_optimization_cost(x,P,umax,st,reference,wc_des)

% Controller 1 (outer loop)
Kp1 = x(1);
Ti1 = x(2);

% Controller 2 (inner loop)
Kp2 = x(3); 

s=tf('s');
C1 = Kp1*(1+1/(s*Ti1));
C2 = tf(Kp2);

ss_vel=ss(P);
ss_vel_states=ss(ss_vel.A,ss_vel.B,eye(size(ss_vel.A)),zeros(size(ss_vel.A,1),size(ss_vel.B,2)));

ss_pos=ss(P/s);
ss_pos_states=ss(ss_pos.A,ss_pos.B,eye(size(ss_pos.A)),zeros(size(ss_pos.A,1),size(ss_pos.B,2)));

L2=C2*ss_vel;
[gpeak,fpeak] = getPeakGain(L2,[],[100 1000]); % constraint of the frequencies personalize for our system
if(gpeak > 1/10)
    filt=notch(fpeak,1/10/gpeak);
else
    filt=1;
end
C2=C2*filt;
L2_filt=C2*ss_vel;
F2=L2_filt/(1+L2_filt);
S2=stepinfo(F2);

L1=C1*F2;
F1=L1/(1+L1);
S1=stepinfo(F1);

C1 = Controller(st,C1,S1.SettlingTime/log(50));
C1.setUMax(umax);
C2 = Controller(st,C2,S2.SettlingTime/log(50));
C2.setUMax(umax);

y0=[0 0];
y=y0;
error=zeros(1,length(reference));
Y=zeros(1,length(reference));
x0_vel=zeros(1,size(ss_vel.A,1));
x0_pos=zeros(1,size(ss_pos.A,1));
% for i = 1 : length(reference)
%     set_vel=C1.computeControlAction(reference(i),y(1));
%     torque = C2.computeControlAction(set_vel,y(2));
%     vel=lsim(ss_vel,torque*ones(1,2),[0 st],x0_vel);
%     x0_vel=lsim(ss_vel_states,torque*ones(1,2),[0 st],x0_vel);
%     pos=lsim(ss_pos,torque*ones(1,2),[0 st],x0_pos);
%     x0_pos=lsim(ss_pos_states,torque*ones(1,2),[0 st],x0_pos);
%     y=[pos(end) vel(end)];
%     x0_pos = x0_pos(end,:);
%     x0_vel = x0_vel(end,:);
%     error(i)=reference(i)-y(1);
%     Y(i)=y(1);
% end
margins=allmargin(L1);
if isempty(margins.PMFrequency)
    cost=inf;
    Wc=-1;
else
    cost=(wc_des-margins.PMFrequency(end))^2;
    Wc=margins.PMFrequency(end);
end
% J=sum(abs(error*st))+cost;
J=cost;
end

