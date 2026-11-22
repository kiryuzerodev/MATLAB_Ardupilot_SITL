function rng = SITL_Rangefinder(X, params)
% My logic, AI assisted at the end
% Extract the needed states
z = X(3);
q0 = X(7);
q1 = X(8);
q2 = X(9);
q3 = X(10);

% Quaternion DCM matrix. Again.....
q_Body2NED = [1-2*(q2^2+q3^2), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
    2*(q1*q2+q0*q3), 1-2*(q1^2+q3^2), 2*(q2*q3-q0*q1);
    2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1^2+q2^2)];

% Usual, get in NED from body
sensorNED = q_Body2NED*params.RNG_POS;

h_sensor = -(z + sensorNED(3));
% How much the beam is tilted from vertical: 1 because we want z dist only
cosComp = q_Body2NED*[0;0;1];
% Multiplication with the 3rd element in quaternion DCM matrix. Because the
% euler angle DCM matrix, this corresponds to cosphicostheta. So by
% default, this is the same
if cosComp > 0.5 && h_sensor > 0     % within ~60 deg of vertical and above ground
    % to check if the plane was underground (not possible now)
    rng = h_sensor/cosComp;          % slant range along the beam
else
    % AI Assisted from here on
    rng = params.RNG_MAX + 1;        % nothing useful, report beyond max
end
if rng > params.RNG_MAX
    rng = params.RNG_MAX + 1;        % ArduPilot treats this as "out of range high"
end
end