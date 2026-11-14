%% This code needs to be inserted into the Model Properties> Callbacks> InitFcn location
fprintf("~~~~ Ardupilot SITL Time ~~~~~");
params = SITL_RascalParams();
params.DT = 1/1200;
X0 = [0;0;0;  0;0;0;  1;0;0;0;  0;0;0];   % home, still, level, north
PWM0 = 1500*ones(16,1); % This is the systematic delay or else 
%simulink thinks everything is happening at once! 
clear SITL_Link_Step
fprintf("~~~~ INITIALIZED ~~~~~");
