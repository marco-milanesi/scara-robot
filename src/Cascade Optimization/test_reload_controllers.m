clear all;
clc;
close all;

system=ElasticRoboticSystem();

st=system.getSamplingPeriod;

cs=ControlledSystemScara(system,'Alfa');

max_acc=1;

[file_name, file_path] = uigetfile("controller_*.mat", "Controller");
load([file_path file_name])

load modello_j1.mat
load modello_j2.mat
%%
load parameters_controller_ext_joint2.mat
Kp = x1(1);
Ki = x1(2);
Tfu = x1(5);
Cp_joint2 = (Kp+Ki/s)*1/(Tfu*s+1);

%%

s = tf('s');

%L1 = C1_in*modello_continuo_tf_j1;

L2_in = C2_in*modello_continuo_tf_j2;
figure
margin(L2_in);

F2_in = minreal(feedback(L2_in,1)*1/s);

L2_ext = C2_out*F2_in
figure
margin(L2_ext)

 
