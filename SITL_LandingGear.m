function [F_gear,M_gear]=SITL_LandingGear(X,delta,params)
% Based on: https://kirilboychev.com/aircraft-ground-interaction-simulation
% and my logic
% The landing gears on Rascal 110 are:
% 1 - Left Main 
% 2 - Right Main
% 3 - Tail

% Extract necessary states:
z = X(3);
V = [X(4);X(5);X(6)];
AngRate = [X(11);X(12);X(13)];
q0 = X(7);
q1 = X(8);
q2 = X(9);
q3 = X(10);

% Hello again
q_Body2NED = [1-2*(q2^2+q3^2), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
    2*(q1*q2+q0*q3), 1-2*(q1^2+q3^2), 2*(q2*q3-q0*q1);
    2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1^2+q2^2)];
% Aircraft heading from the DCM (valid at any pitch)
psi = atan2(q_Body2NED(2,1), q_Body2NED(1,1));

% Compute for every landing gear (here 3)
for i = 1:3
    % Wheel position
    wheel_loc = params.R_GEAR(i,:);
    wheel_loc_NED = q_Body2NED*wheel_loc;
    z_wheel = z + wheel_loc_NED;

    if z_wheel > 0 % Positive means we are already underground
        % Wheel velocity
        V_wheel = q_Body2NED*(V + cross(AngRate,wheel_loc));
        vN = V_wheel(1);
        vE = V_wheel(2);
        vD = V_wheel(3);

        F_normal = params.GEAR_K*z_wheel + params.GEAR_C*vD;
        if F_normal < 0
            F_normal = 0;
        end

        % Plane heading = wheel heading always 
        psi_w = psi;
        if i == 3
            % This is the actual steering logic
            psi_w = psi + (delta(3)/params.DR_MAX)*params.TAIL_MAX_STEER;
        end

        % Check the rolling velocity and 
        V_wh_roll = sqrt(vN^2+vE^2);
        if V_wh_roll > 1e-3
            beta_w = atan2(vE, vN) - psi_w;
            v_fwd = V_wh_roll*cos(beta_w);
            v_lat = V_wh_roll*sin(beta_w);
        else
            v_fwd = 0;                      % parked: slip angle undefined
            v_lat = 0;
        end


    end

end

end