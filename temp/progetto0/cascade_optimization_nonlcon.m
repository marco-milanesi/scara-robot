function [Cin, Ceq] = cascade_optimization_nonlcon(x, P, min_PM, MS, w_vector)
Ceq = []; 

% Controller 1 (outer loop)
Kp1 = x(1);
Ti1 = x(2);

% Controller 2 (inner loop)
Kp2 = x(3); 

s=tf('s');
C1 = Kp1*(1+1/(s*Ti1));
C2 = tf(Kp2);

L2=P*C2;
[gpeak,fpeak] = getPeakGain(L2,[],[100 1000]); % constraint of the frequencies personalize for our system
if(gpeak > 1/10)
    filt=notch(fpeak,1/10/gpeak);
else
    filt=1;
end
C2=C2*filt;

C1_fr=freqresp(C1,w_vector); % frequency response
C2_fr=freqresp(C2,w_vector);
P_fr=freqresp(P,w_vector);
C1_fr=C1_fr(:);  % reshaping 1x1xn -> nx1
C2_fr=C2_fr(:); 
P_fr=P_fr(:);

L2_fr=C2_fr.*P_fr;
S2_fr=1./(1+L2_fr); % S=1/(1+L)
F2_fr=L2_fr.*S2_fr; % F = L/(1+L)=L*S
DY2_fr=P_fr.*S2_fr; %DY=Y/D=P/(1+L)



Cin(1)=max(abs(L2_fr(w_vector>fpeak)))-0.1;
Cin(2)=max(abs(S2_fr))-MS;

L2=P*C2;
margin=allmargin(L2);

if isempty(margin.PMFrequency)
    Cin(3)=1;
else
    Cin(3)=min_PM-min(margin.PhaseMargin); %min(PM)>=PM_min -> PM_min-PM<=0
end

L1_fr=C1_fr.*F2_fr;
S1_fr=1./(1+L1_fr); % S=1/(1+L)
F1_fr=L1_fr.*S1_fr; % F = L/(1+L)=L*S
DY1_fr=F2_fr.*S1_fr; %DY=Y/D=P/(1+L)

Cin(4)=max(abs(S1_fr))-MS;

L1=C2*(L2/(1+L2));
margin=allmargin(L1);

if isempty(margin.PMFrequency)
    Cin(5)=1;
else
    Cin(5)=min_PM-min(margin.PhaseMargin); %min(PM)>=PM_min -> PM_min-PM<=0
end
end

