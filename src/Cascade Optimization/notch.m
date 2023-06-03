function sys = notch(wn,peak_gain,xci_p)
% wn : frequency of the filter
% peak_gain : is direct proportionally to the depht of the valley
% xci_p : change the width of the valley
if nargin < 2 || isempty(peak_gain)
    peak_gain=1/10;
end
if nargin < 3
    xci_p=0.9;
end
xci_z=0.1;
s=tf('s');
sys=(s^2+2*xci_z*wn*s+wn^2)/(s^2+2*xci_p*wn*s+wn^2);
gpeak = getPeakGain(1/sys,[],[wn*0.9 wn*1.1]);
xci_z=xci_z*peak_gain*gpeak;
sys=(s^2+2*xci_z*wn*s+wn^2)/(s^2+2*xci_p*wn*s+wn^2);
end