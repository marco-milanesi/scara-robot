clear all;
clc;
close all;

system=ElasticRoboticSystem();

st=system.getSamplingPeriod;

cs=ControlledSystemScara(system,'Alfa');

max_acc=1;

ctrl=SimpleScaraController(cs.getSamplingPeriod);
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