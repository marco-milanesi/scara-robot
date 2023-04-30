clear all;
clc;
close all;

st=1e-3; % sample period
system=ElasticTransmission(st);
system.show;

% manager della simulazione
cs=ControlledSystem(system);

% coppia massima attuatore
umax=system.getUMax;

t=(0:st:10)';

% massima coppia per 10 secondi
tau=umax*ones(length(t),1);

output=zeros(length(t),system.getOutputNumber);

% simulo anello aperto
cs.initialize
tic
for it=1:length(t)
    output(it,:)=cs.openloop(tau(it,:)');
end
toc

figure(1)
set(gcf,'Name','Open-loop output')

for io=1:system.getOutputNumber
    subplot(1,system.getOutputNumber,io)
    plot(t,output(:,io))
    xlabel('t')
    ylabel(system.getOutputName{io})
    grid on
end


figure(2)
set(gcf,'Name','Open-loop input')
for ii=1:system.getInputNumber
    subplot(1,system.getInputNumber,ii)
    plot(t,tau(:,ii))
    xlabel('t')
    ylabel(system.getInputName{ii})
    grid on
end

%% close loop
st=1e-3;
Kp=10; % valori taratura
Ki=.05; % valori taratura
ctrl=PIController(st,Kp,Ki);

cs.setController(ctrl,1); % il controllore usa solo la prima uscita
reference=ones(length(tau),1);


cs.initialize
tic
for it=1:length(t)
    [output(it,:),control_action(it,:)]=cs.step(reference(it,:));
end
toc

figure(3)
set(gcf,'Name','Close-loop output')

for io=1:system.getOutputNumber
    subplot(1,system.getOutputNumber,io)
    plot(t,output(:,io))
    xlabel('t')
    ylabel(system.getOutputName{io})
    grid on
end

figure(4)
for ii=1:system.getInputNumber
    subplot(1,system.getInputNumber,ii)
    plot(t,control_action(:,ii))
    xlabel('t')
    ylabel(system.getInputName{ii})
    grid on
end
set(gcf,'Name','Close-loop input')
