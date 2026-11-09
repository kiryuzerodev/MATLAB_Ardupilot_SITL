function XDOT = SITL_6DoFEqns(X,Forces,Moments)
% This function takes care of the Rascal Equations of Motion only. Meaning
% no other dynamics are involved over here for now and we will just use the
% classical 6 DoF model here to evaluate

%% Extracting the infromation from the input vectors
% States 
x = X(1);       % Position in X (NED Convention)
y = X(2);       % Position in Y 
z = X(3);       % Position in Z (-ve is Ascending!)
u = X(4);       % Body velocity in X
v = X(5);       % Body velocity in Y
w = X(6);       % Body velocity in Z
phi = X(7);     % Body angle along X (Roll angle)
theta = X(8);   % Body angle along y (Pitch angle)
psi = X(9);     % Body angle along z (Yaw angle/Heading angle)
p = X(10);      % Body angular rate along x - roll rate
q = X(11);      % Body angular rate along y - pitch rate
r = X(12);      % Body angular rate along z - yaw rate

% % Control inputs
% deltaA = U(1);  % Aileron angle input
% deltaE = U(2);  % Elevator angle input
% deltaR = U(3);  % Rudder angle input
% deltaT = U(4);  % Throttle % input
%% Extracting the Forces and Moments
Fx = Forces(1); % Total force in X Direction (N)
Fy = Forces(2); %   
Fz = Forces(3);
L = Moments(1);
M = Moments(2);
N = Moments(3);

%% Equations of Motion to build the States
end