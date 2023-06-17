function plot_with_charachteristics(sys_cl, t)

% Step with Characteristics of a step response
stinf = stepinfo(sys_cl);
stinft = struct2table(stinf);
VarNm = stinft.Properties.VariableNames;
Vals = stinft{1,:};


outliers = [1,3,4,6,7,8];
VarNm(outliers+1) = [];
Vals(outliers+1) = [];

figure

SP = 1;
[y,~]=step(SP*sys_cl, t);
sserror=abs(SP-y(end))*100; %get the steady state error
step(SP*sys_cl, t);
text(0.3, min(ylim)+mean(ylim)*0.05, compose('SS error = %2.2f %%', sserror))
text(0.3, min(ylim)+mean(ylim)*0.1, compose('%s = %7.3f', [string(VarNm); Vals].'), 'Horiz','left','Vert','bottom', 'Interpreter','latex' )
grid


end

