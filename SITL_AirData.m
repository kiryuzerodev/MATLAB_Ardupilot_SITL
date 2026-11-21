function [Va, alpha, beta, qbar, Mach, h_agl, alphadot] = SITL_AirData(X, XDOT_prev, wind_NED, rho, a)
% Air data from states: airspeed, angle of attack, sideslip, dynamic pressure
% XDOT_prev comes through a Memory block (avoids an algebraic loop) without
% needing an integrator 
% Source for AI provided by me: Stevens and Lewis: Aircraft Control and Simulation Dynamics Chapter 1 and 2
%% Extract States
z = X(3);
u = X(4);
v = X(5);
w = X(6);
q0 = X(7);
q1 = X(8);
q2 = X(9);
q3 = X(10);
udot = XDOT_prev(4);
wdot = XDOT_prev(6);

% We meet again! 
q_Body2NED = [1-2*(q2^2+q3^2), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
    2*(q1*q2+q0*q3), 1-2*(q1^2+q3^2), 2*(q2*q3-q0*q1);
    2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1^2+q2^2)];

%% Velocity relative to the air (body)
V_rel = [u; v; w] - q_Body2NED'*wind_NED;
ur = V_rel(1);
vr = V_rel(2);
wr = V_rel(3);
Va = sqrt(ur^2 + vr^2 + wr^2);

%% Angles - guard for Va ~ 0 (parked on the runway)
if Va > 0.1
    alpha = atan2(wr, ur);
    beta  = asin(vr/Va);
else
    alpha = 0;
    beta  = 0;
end

if (u^2 + w^2) > 0.01
    alphadot = (u*wdot - w*udot)/(u^2 + w^2);
else
    alphadot = 0;
end

%% Dynamic pressure, Mach, height above ground
qbar  = 0.5*rho*Va^2;
Mach  = Va/a;
h_agl = -z;                        % flat runway at home height
% Later you can also add the height of wing MAC over ground here or in the
% ground effect block directly
end