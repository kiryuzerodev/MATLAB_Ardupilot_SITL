% Place this in SITL_SimulinkModel > Model Properties > Call Backs >
% InitFcn to initialize the states and stuff
fprintf("~~~~ Ardupilot SITL Time ~~~~~");
clear SITL_Link_Step
HOLD = 0;
params = SITL_RascalParams();
params.DT = 1/1200;
theta0 = deg2rad(14.3);     % taildragger rest attitude
X0 = [0;0;-200;  15.9201;0;1.5973;  0.99875;0;0.049979;0;  0;0;0];    % home, still, level, north
PWM0 = 1500*ones(16,1); % This is the systematic delay or else 
%simulink thinks everything is happening at once! 
PWM0(3) = 1100 + 0.10*800;
delta0 = [0;0;0;0.10]; % For the aileron, elevator, rudder and throttle
% We are not considering a special one for the throttle as well
% This should also become the trim states if you are trying to 
% start the SITL from a point in the Air
omega0 = 331.8;

fprintf("~~~~ INITIALIZED ~~~~~");