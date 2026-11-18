%% This code needs to be inserted into the Model Properties> Callbacks> InitFcn location
fprintf("~~~~ Ardupilot SITL Time ~~~~~");
clear SITL_Link_Step
HOLD = 0;
params = SITL_RascalParams();
params.DT = 1/1200;
X0 = [0;0;-100;  20.7;0;0;  1;0;0;0;  0;0;0];   % home, still, level, north
PWM0 = 1500*ones(16,1); % This is the systematic delay or else 
%simulink thinks everything is happening at once! 
delta0 = [0;0;0;0]; % For the aileron, elevator, rudder and throttle
% We are not considering a special one for the throttle as well
% This should also become the trim states if you are trying to 
% start the SITL from a point in the Air

fprintf("~~~~ INITIALIZED ~~~~~");
