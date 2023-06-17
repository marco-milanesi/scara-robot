clear all;
clc;
close all;
warning off

% BEST CONTROLLER FILE TO LOAD IS: 
% - MV saturated: controller_wf_400_600_PM_70_80_MS_14_14.mat_ff.mat max_acc=15;
% - MV not saturated: controller_wf_400_600_PM_70_80_MS_14_14.mat_ff.mat max_acc=7;

system=ElasticRoboticSystem();

st=system.getSamplingPeriod;

cs=ControlledSystemScara(system,'Alfa');

max_acc=7; % rad/s^2
rest_time=0.22; % s
is_ff = true;

[file_name, file_path] = uigetfile("controller_*.mat", "Controller");
load([file_path file_name])
disp(file_name)

%%
disp(is_ff)
ctrl=ScaraController(st, C1_in, C1_out, tau1_in, tau1_out, M1, C2_in, C2_out, tau2_in, tau2_out, M2, is_ff);
cs.setController(ctrl);
cs.setMaxAcceleration(max_acc);
cs.setRestTime(rest_time);

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