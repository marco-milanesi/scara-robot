clear all;
clc;
close all;

% BEST CONTROLLER FILE TO LOAD IS: controller_wf_50_100.mat

system=ElasticRoboticSystem();

st=system.getSamplingPeriod;

cs=ControlledSystemScara(system,'Alfa');

max_acc=10;

[file_name, file_path] = uigetfile("controller_*.mat", "Controller");
load([file_path file_name])


%%

ctrl=ScaraController(st, C1_in, C1_out, 2, 2, C2_in, C2_out, 2, 2)
cs.setController(ctrl);
cs.setMaxAcceleration(max_acc);

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