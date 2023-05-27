function [cost] = untitled(X)
kp=X(1);
Ti=X(2);
s=tf('s');
PI=kp*(1+1/(Ti*s));
PI_d=c2d(PI,st);

st=0.1;

load("modello.mat")
sys=System(modello_continuo_tf/s);

omega_portante=1;
ampiezza_portante=150;
T_portante=2*pi/omega_portante;

t=(0:st:(1.1*T_portante))';
portante=ampiezza_portante*sin(omega_portante*t);

w0=10; %rad/s
w1=pi/st; %rad/s

ampliezza_identificazione=400;

control_action=portante + control_action_identificazione*ampliezza_identificazione;

cs.initialize
ref_j0=0;
x0=zeros(1,4);

controller1=Controller(st,PI_d,[-600 600]);
torque=zeros(length(control_action),2);
process_output=[];
for idx=1:length(t)
    still_ca = controller1.computeControlAction(ref_j0,x(1));
    [x,t(idx,1)]=sys.openloop([control_action(idx) still_ca]);
    process_output(idx,:) = x;
    torque(idx,:) = [control_action(idx) still_ca];
end
end