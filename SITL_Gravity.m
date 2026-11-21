function F_gravity = SITL_Gravity(X,params)
% This function will later be replaced in SITL_Environment
%% Extract the 4 quaternions as that is all we need here
q0 = X(7);
q1 = X(8);
q2 = X(9);
q3 = X(10);

%% Get the constants
mass = params.MASS;  % Mass in kg
grav = params.g;     % Gravity in m/s^2

% This DCM will be repeated everywhere, consider making it a function soon!
q_Body2NED = [1-2*(q2^2+q3^2), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
    2*(q1*q2+q0*q3), 1-2*(q1^2+q3^2), 2*(q2*q3-q0*q1);
    2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1^2+q2^2)];

gravityNED = [0; 0; mass*grav];
%% Return the Force due to Gravity
% Transpose! as we are going NED to Body!
F_gravity = q_Body2NED' * gravityNED;

end