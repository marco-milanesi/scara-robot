clear all;clc;close all
s=tf('s');
% questo script testa le funzioni della classe

%% TEST starting conditions
for itest=1:100
    st=1e-3;
    Kp=5*rand; % setto dei valori random
    Ki=5*rand; % setto dei valori random
    umax=10*rand;
    
    PI=Kp+Ki/s;
    PI_D=c2d(PI,st);
    ctrl=Controller(st,PI_D,[-umax umax]);

    ctrl.initialize; % inizializzo

    setpoint=randn;
    y=randn;

    uinitial=rand*umax;

    u=ctrl.computeControlAction(setpoint,y);
    % the first u should be equal to uinitial
    assert(abs(u-uinitial)<1e-6)
end

%% TEST 2

%% TEST computeControlAction
parfor itest=1:100
    st=1e-3;
    Kp=5*rand; % setto dei valori random
    Ki=5*rand; % setto dei valori random
    umax=10*rand;

    PI=Kp+Ki/s;
    PI_D=c2d(PI,st);
    ctrl=Controller(st,PI_D,[-umax umax]);

    ctrl.initialize; % inizializzo


    uinitial=rand*umax;
%     ctrl.starting(setpoint,y,uinitial); % inizializzo lo stato

    for istep=1:10000
        setpoint=randn;
        y=randn;
        u=ctrl.computeControlAction(setpoint,y);
        % check if u is limited
        assert((u<=umax) && (u>=-umax));
    end
end

