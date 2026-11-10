function XDOT = SITL_6DoFEqns(X,Forces,Moments,params)
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
q0 = X(7);      % Quaternion (scalar part)
q1 = X(8);      % Quaternion 1
q2 = X(9);      % Quaternion 2
q3 = X(10);     % Quaternion 3
p = X(11);      % Body angular rate along x - roll rate
q = X(12);      % Body angular rate along y - pitch rate
r = X(13);      % Body angular rate along z - yaw rate
%% Extracting the Forces and Moments
Fx = Forces(1); % Total force in X Direction (N)
Fy = Forces(2); %
Fz = Forces(3);
L = Moments(1);
M = Moments(2);
N = Moments(3);
%% Get the constants
mass = params.MASS;
Itensor = params.INERTIA_TENSOR;
Iinv = params.INERTIA_INVERSE;

%% Quaternion DCM for body to NED
q_Body2NED= [1-2*(q2^2+q3^2), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
    2*(q1*q2+q0*q3), 1-2*(q1^2+q3^2), 2*(q2*q3-q0*q1);
    2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1^2+q2^2)];
% Source: Quaternion to DCM and Back Again - Kurt A. Motekew

%% Translational Dynamics
udot = r*v - q*w + Fx/mass;
vdot = p*w - r*u + Fy/mass;
wdot = q*u - p*v + Fz/mass;

%% Rotational Dynamics
omega = [p;q;r];
tempROT = Iinv*([L;M;N]-cross(omega,Itensor*omega));
pdot = tempROT(1);
qdot = tempROT(2);
rdot = tempROT(3);

%% Quaternion Kinematics
qnorm = 1 - (q0^2 + q1^2 + q2^2 + q3^2);    % norm error correction
q0dot = 0.5*(-p*q1 - q*q2 - r*q3) + qnorm*q0;
q1dot = 0.5*( p*q0 + r*q2 - q*q3) + qnorm*q1;
q2dot = 0.5*( q*q0 - r*q1 + p*q3) + qnorm*q2;
q3dot = 0.5*( r*q0 + q*q1 - p*q2) + qnorm*q3;

%% Position Kinematics (Body to NED)
posdot = q_Body2NED*[u;v;w];
xdot = posdot(1);
ydot = posdot(2);
zdot = posdot(3);

%% State Derivatives
XDOT = [xdot; ydot; zdot; udot; vdot; wdot; q0dot; q1dot; q2dot; q3dot; pdot; qdot; rdot];
end