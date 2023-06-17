clc
clear all
close all

% .\scara-robot\src\Cascade Optimization\controllers_parameters

[file_joint1_in, path_joint1_in] = uigetfile("parameters_controller_int_joint1*.mat")
load([path_joint1_in, file_joint1_in], "x"); parms1_in = x;

[file_joint1_ext, path_joint1_ext] = uigetfile("parameters_controller_ext_joint1*.mat")
load([path_joint1_ext, file_joint1_ext], "x1"); parms1_out = x1;

[file_joint2_in, path_joint2_in] = uigetfile("parameters_controller_int_joint2*.mat")
load([path_joint2_in, file_joint2_in], "x"); parms2_in = x;

[file_joint2_ext, path_joint2_ext] = uigetfile("parameters_controller_ext_joint2*.mat")
load([path_joint2_ext, file_joint2_ext], "x1"); parms2_out = x1;

[file_joint1_filter, path_joint1_filter] = uigetfile("tot_filter_joint1*.mat");
load([path_joint1_filter, file_joint1_filter], "tot_filter"); tot_filter1 = tot_filter;  

[file_joint2_filter, path_joint2_filter] = uigetfile("tot_filter_joint2*.mat");
load([path_joint2_filter, file_joint2_filter], "tot_filter"); tot_filter2 = tot_filter;

C1_in = generate_controller(parms1_in) * tot_filter1;
C1_out = generate_controller(parms1_out);
C2_in = generate_controller(parms2_in) * tot_filter2;
C2_out = generate_controller(parms2_out);

file_name = input("Insert name for this set of controllers: ", "s");
save("./controllers/controller_"+file_name+".mat", "C1_in", "C1_out", "C2_in", "C2_out");


function C = generate_controller(parms)
    
    Kp=parms(1); Ki=parms(2); Kd=parms(3); Tfd=parms(4); Tfu=parms(5);
    
    s=tf('s');
    
    C=(Kp+Ki/s+Kd*s/(Tfd*s+1))*1/(Tfu*s+1);

end
