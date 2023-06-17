function joint2_optimization_par(PM_int, PM_ext, MS_int, MS_ext, w_f)

load modello_j2.mat

process = modello_continuo_tf_j2;



s=tf('s');
[gpeak,fpeak] = getPeakGain(process*10^(70/20),[],[300 1000]); % constraint of the frequencies personalize for our system
if(gpeak > 1/10)
    notch_filter=notch(fpeak,1/10/gpeak);
else
    notch_filter=1;
end
process_notch = process * notch_filter;


filter = 1/(s/w_f+1)^1;


% ylim([-120 -40])
total_process = minreal(process_notch * filter);

tot_filter = filter * notch_filter;

w_inf = 1;
w_sup = 10^4;


internal_joint1.wc_des = 250;    % pulsazione di taglio desiderata

% funzione che restituisce il valore di J in funzione di x
J = @(x)pid_filt_cost_function([x, 0, 0, 0, 0],total_process,internal_joint1.wc_des,0);

% Massima sensitività
internal_joint1.MS = MS_int;

% Reiezione disturbo di misura
internal_joint1.wh = 2000;
internal_joint1.Fh_max = 0.1;

% Reiezione disturbo sul carico
internal_joint1.wl = 10;
internal_joint1.Dl_max = 0.1;

% Minimo margine di fase tollerabile
internal_joint1.PM_min = PM_int;
w_vector = logspace(log10(w_inf),log10(w_sup))';
w_vector = sort(unique([w_vector;internal_joint1.wl;internal_joint1.wh]));

nlcon = @(x)pid_filt_constraints([x, 0, 0, 0, 0], total_process, w_vector, ...
                                 internal_joint1.MS, internal_joint1.wh, internal_joint1.Fh_max, ...
                                 internal_joint1.wl, internal_joint1.Dl_max, internal_joint1.PM_min,0);


timeout = 60*2;
internal_joint1.aKp = [5800, 100, 10000];

x0 = [internal_joint1.aKp(1)];


problem = createOptimProblem('fmincon','x0',x0,  'objective',J, 'lb',[internal_joint1.aKp(2)],  'ub', [internal_joint1.aKp(3)],  'nonlcon',nlcon);

solver = GlobalSearch('MaxTime',timeout);
x = run(solver,problem);



s=tf('s');
Kp = x(1);
Cv_joint1 = tf(Kp);
Cv_joint1.InputName = 'sp-vel';
Cv_joint1.OutputName = 'torque';

Loop_int_joint1 = total_process*Cv_joint1;






%% Extern loop
process_ext = minreal(feedback(Loop_int_joint1,1)*1/s);


external_joint1.wc_des = 200;    % pulsazione di taglio desiderata

% funzione che restituisce il valore di J in funzione di x
J = @(x)pid_filt_cost_function([x(1), x(2), 0, 0, 0],process_ext,external_joint1.wc_des,0);
% Massima sensitività
external_joint1.MS = MS_ext;

% Reiezione disturbo di misura
external_joint1.wh = 2000;
external_joint1.Fh_max = 0.1;

% Reiezione disturbo sul carico
external_joint1.wl = 10;
external_joint1.Dl_max = 0.1;

% Minimo margine di fase tollerabile
external_joint1.PM_min = PM_ext;

w_vector = logspace(log10(w_inf),log10(w_sup))';
w_vector = sort(unique([w_vector;external_joint1.wl;external_joint1.wh]));

nlcon = @(x)pid_filt_constraints([x(1), x(2), 0, 0, 0], process_ext, w_vector, ...
                                 external_joint1.MS, external_joint1.wh, external_joint1.Fh_max, ...
                                 external_joint1.wl, external_joint1.Dl_max, external_joint1.PM_min,0);


timeout = 60*2;
% [valore iniziale, limite inferiore, limite superiore]
external_joint1.aKp = [130, 0.01, 2000];
external_joint1.aKi = [1/200, 1/1000, 100];


x0 = [external_joint1.aKp(1), external_joint1.aKi(1)];

opt1=optimoptions("ga","MaxTime",3600*0.1,'InitialPopulation',x0);

x1=ga(J,2,[],[],[],[],[external_joint1.aKp(2), external_joint1.aKi(2)] ...
    ,[external_joint1.aKp(3), external_joint1.aKi(3)] ...
    ,nlcon,opt1);



s=tf('s');
Kp=x1(1); Ki=x1(2);
Cp_joint1=(Kp+Ki/s);

Loop_ext_joint1 = process_ext*Cp_joint1;





x = [x, 0, 0, 0, 0];
x1 = [x1, 0, 0, 0];
save("parameters_controller_int_joint2_" +num2str(w_f)+"_"+num2str(internal_joint1.PM_min)+"_"+num2str(MS_int)+".mat", "x")
save("parameters_controller_ext_joint2_"+num2str(w_f)+"_"+num2str(external_joint1.PM_min)+"_"+num2str(MS_ext)+".mat","x1")
save("tot_filter_joint2_"+num2str(w_f)+"_"+num2str(internal_joint1.PM_min)+"_"+num2str(external_joint1.PM_min)+"_"+num2str(MS_int)+"_"+num2str(MS_ext)+".mat","tot_filter");
