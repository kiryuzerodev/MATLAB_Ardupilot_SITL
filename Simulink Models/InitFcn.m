%% This code needs to be inserted into the Model Properties> Callbacks> InitFcn location
fprintf("~~~~ Ardupilot SITL Time ~~~~~");
clear SITL_Link_Step
HOLD = 0;
params = SITL_RascalParams();
params.DT = 1/1200;
theta0 = deg2rad(14.3);     % taildragger rest attitude
X0 = [0;0;-0.40;  0;0;0;  cos(theta0/2);0;sin(theta0/2);0;  0;0;0];  % home, still, level, north
PWM0 = 1500*ones(16,1); % This is the systematic delay or else 
%simulink thinks everything is happening at once! 
delta0 = [0;0;0;0]; % For the aileron, elevator, rudder and throttle
% We are not considering a special one for the throttle as well
% This should also become the trim states if you are trying to 
% start the SITL from a point in the Air
omega0 = 0;

fprintf("~~~~ INITIALIZED ~~~~~");
