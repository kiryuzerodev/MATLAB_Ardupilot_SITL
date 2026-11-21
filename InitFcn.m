% Place this in SITL_SimulinkModel > Model Properties > Call Backs >
% InitFcn to initialize the states and stuff
fprintf("~~~~ Ardupilot SITL Time ~~~~~");
clear SITL_Link_Step
params = SITL_RascalParams();
params.DT = 1/1200;
theta0 = deg2rad(14.3);     % taildragger rest attitude

% Enter the required euler angles here
p = deg2rad(0); % Phi - Roll angle
t = deg2rad(6); % Theta - Pitch angle
s = deg2rad(0); % Psi - Yaw angle

q= eul2quat([s t p]);

% Currently for pitch axis only no AoA
V_airspeed = 16;
u = V_airspeed*cos(t);
w = V_airspeed*sin(t);

X0 = [0;0;-200;  16;0;0;  q';  0;0;0];    % home, still, level, north
PWM0 = 1500*ones(16,1); % This is the systematic delay or else 
%simulink thinks everything is happening at once! 
delta0 = [0;0;0;0.10]; % For the aileron, elevator, rudder and throttle
% We are not considering a special one for the throttle as well
% This should also become the trim states if you are trying to 
% start the SITL from a point in the Air
omega0 = 0;


PWM_TEST = 1500*ones(16,1);
PWM_TEST(2) = 1601;              % trim elevator (-0.0888 rad)
PWM_TEST(3) = 1169;              % trim throttle (8.7%)

fprintf("~~~~ INITIALIZED ~~~~~");