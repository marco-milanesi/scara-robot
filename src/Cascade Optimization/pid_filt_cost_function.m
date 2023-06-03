function J = pid_filt_cost_function(x,P,wc_des,insertNotch)
% PID_FILT_COST_FUNCTION Calcola il valore di una cifra di merito J.
%   J: cifra di merito
%   x = [Kp,Ki,Kd,Tfd,Tfu]: vettore dei parametri.
%   P: modello continuo del processo
%   wc_des: pulsazione di taglio desiderata

Kp=x(1); Ki=x(2); Kd=x(3); Tfd=x(4); Tfu=x(5);

s=tf('s');

C=(Kp+Ki/s+Kd*s/(Tfd*s+1))*1/(Tfu*s+1);     % controllore PID filtrato
L=P*C;    % funzione d'anello aperto



if insertNotch    
    [gpeak,fpeak] = getPeakGain(L,[],[100 1000]); % constraint of the frequencies personalize for our system
    if(gpeak > 1/10)
        filt=notch(fpeak,1/10/gpeak);
    else
        filt=1;
    end
    C=C*filt;
    L=P*C;
end

margini=allmargin(L);    % calcolo margini di L

if isempty(margini.PMFrequency)    % se L ha margine di fase infinito
    J=100;    % pongo J a un valore alto
else
    J=(wc_des-margini.PMFrequency(end))^2;    % altrimenti calcolo J
end


end
