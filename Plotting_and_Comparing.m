% Code to plot and compare SITL data and any other model data. Here,
% Shayak's model
clc
close all
Trace = readtable("/home/kiryuzerodev/Downloads/Shayak_Model/Zip V2/Trace.csv","VariableNamingRule","preserve");
tS = Trace.("Time (s)");            % time of the Digital Twin trace

% My Model from Simulink outblock (run Simulink model first)
X = squeeze(out.SITL_Log);
t = out.tout;

pitch = rad2deg(asin(2*(X(7,:).*X(9,:) - X(10,:).*X(8,:))));
alt   = -X(3,:)/0.3048;
Va    = sqrt(X(4,:).^2 + X(5,:).^2 + X(6,:).^2)*3.6;

figure;
plot(tS, Trace.("Pitch (degrees)"), t, pitch,"LineWidth",2);
title("Pitch (deg)");
legend("Digital Twin","SITL"); xlabel("Time (s)");
grid on

figure;
plot(tS, Trace.("Altitude (ft)"), t, alt,"LineWidth",2);
title("Altitude (ft)"); legend("Digital Twin","SITL"); xlabel("Time (s)");
grid on

figure;
plot(tS, Trace.("True airspeed (km/hr)"), t, Va,"LineWidth",2);
title("Airspeed (km/h)"); legend("Digital Twin","SITL"); xlabel("Time (s)");
grid on