function [gyro, accel, pos, att, vel] = SITL_Sensors(X, Force_GRAVONLY, params)
% Converts states to SITL JSON outputs
%% Extract the States
% For GPS
x = X(1);
y = X(2);
z = X(3);

% For Accelrometer
mass = params.MASS;
% For velocities
u = X(4);
v = X(5);
w = X(6);

% Quaternions
q0 = X(7);
q1 = X(8);
q2 = X(9);
q3 = X(10);
% For Gyroscope
p = X(11);
q = X(12);
r = X(13);
%% Quaternion normalization for more accuracy (because all should be = 1) or some shi
qn = sqrt(q0^2 + q1^2 + q2^2 + q3^2);
q0 = q0/qn; q1 = q1/qn; q2 = q2/qn; q3 = q3/qn;

% Copy pasted DCM ----
q_Body2NED = [1-2*(q2^2+q3^2), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
    2*(q1*q2+q0*q3), 1-2*(q1^2+q3^2), 2*(q2*q3-q0*q1);
    2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1^2+q2^2)];
% Source: AI Generated - mostly from exsisting literature

%% Generate the Sensor Output - these are TRUE VALUES
gyro  = [p q r]';                    % body rates (rad/s)
accel = Force_GRAVONLY/mass;     % specific force, NO gravity (m/s^2)
% Source: Newton's 2nd Law of Motion bro
pos   = [x y z]';                      % NED from home (m)
vel   = q_Body2NED*[u v w]';         % NED velocity (m/s)

% Convert the quaternions into Euler Angles
phi   = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
theta = asin(max(-1, min(1, 2*(q0*q2 - q3*q1))));
psi   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
att   = [phi; theta; psi];           % rad
% Source: AI Generated - mostly from exsisting literature

end