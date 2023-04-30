function [t,qsp,qsp_d,qsp_dd]=computeTrajectory(st,max_acc,rest_time)

max_vel=4;
wpts = [0 pi/6;0 pi/3];

[t1,qsp1,qsp1_d,qsp1_dd]=motionlaw(wpts,max_vel,max_acc,st,rest_time);

wpts_return = fliplr(wpts);

[t2,qsp2,qsp2_d,qsp2_dd]=motionlaw(wpts_return,max_vel,max_acc,st,rest_time);

t=[t1 t2+t1(end)+st];
qsp=[qsp1 qsp2];
qsp_d=[qsp1_d qsp2_d];
qsp_dd=[qsp1_dd qsp2_dd];


