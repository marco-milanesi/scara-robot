clear all;
clc;
close all;


system=ElasticRoboticSystem();

st=system.getSamplingPeriod;

cs=ControlledSystemScara(system,'Alfa');

max_acc=15;

x1 = load("data\parameters_controller_ext_joint1.mat");
x2 = load("data\parameters_controller_ext_joint2.mat");

x1 = cell2mat(struct2cell(x1));
x2 = cell2mat(struct2cell(x2));


%%
s=tf('s');

Kp = x1(1);
Ki = x1(2);
Tfu = x1(5);
PI = (Kp+Ki/s)*1/(Tfu*s+1);
ctrl1=Controller(st,PI,[-600 600]);

Kp = x2(1);
Ki = x2(2);
Kd = x2(3);
Tfd = x2(4);
Tfu = x2(5);
PID = (Kp+Ki/s+Kd*s/(Tfd*s+1))*1/(Tfu*s+1);
ctrl2=Controller(st,PID,[-600 600]);

cs.setController(ctrl1);
cs.setMaxAcceleration(max_acc);
cs.setRestTime(0.1);

% simulo il sistema per la valutazione
% "fast" fa 5 esecuzioni
% "complete" fa 5 x 5 esecuzioni (modalità esame)
tic
[score,results]=cs.evalution("fast");
computation_time=toc;
simulated_time=sum([results.Ttot]);
fprintf('Real-time factor = %f\n',simulated_time/computation_time)


% grafico i risultati
fprintf('lo score è %f\n',score);
cs.showResults(results)