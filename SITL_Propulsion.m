function [F_prop, M_prop, omegadot] = SITL_Propulsion(Va, rho, delta,omega,params)
% Source:
% J.B. Brandt, R.W. Deters, G.K. Ananda, O.D. Dantsker, and M.S. Selig (insert date downloaded), UIUC Propeller Database,
% Vols 1-4, University of Illinois at Urbana-Champaign, Department of Aerospace Engineering, retrieved from https://m-selig.ae.illinois.edu/props/propDB.html.

D = params.PROP_DIA;
deltaT = delta(4);

% To protect against Advance Ratio going to infinity
n = max(omega,1)/(2*pi); % This in revolutions per second
J = Va/(n*D);
% Saturation to use least or max value
% Change: Got rid of min max as it was misleading and hard to read
if J < params.PROP_J(1)
    J = params.PROP_J(1);
elseif J > params.PROP_J(end)
    J = params.PROP_J(end);
end
% Propeller - Thrust and Torque 
CT = interp1(params.PROP_J, params.PROP_CT, J);
CP = interp1(params.PROP_J, params.PROP_CP, J);
T = CT*rho*n^2*D^4; % Thrust
P = CP*rho*n^3*D^5; % Power

% Having power, we can get torque as torque*speed
Q_prop = P/(2*pi*n);

% Engine spooling dynamics
Q_eng = (deltaT*params.P_MAX)/(2*pi*n);
omegadot = (Q_eng - Q_prop)/params.I_PROP;

% AI Added: To prevent the propeller from spinning backwards
if omegadot < 0 && omega <= 0 
    omegadot = 0;                        
end

% Make the output vectors of Force and Moments
F_prop = [T; 0 ; 0]; 
M_prop = [-params.PROP_DIRECTION*Q_prop; 0; 0]; % Because torque is opp dir of spin
M_prop = M_prop + cross(params.R_PROP,F_prop);
end