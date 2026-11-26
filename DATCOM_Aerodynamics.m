function [F_Aero,M_Aero] = DATCOM_Aerodynamics(X, Va, alpha, beta, qbar, alphadot, params,delta)
% This function just computes the force and moments for the plane. For
% Rascal 110, the thesis values and JSBSim values are a close match so we
% will be going with that itself

%% States and inputs
p = X(11);
q = X(12);
r = X(13);
deltaA = delta(1);
deltaE = delta(2);
deltaR = delta(3);

S = params.S;
b = params.b;
c = params.c;

% Switch off the rate non dimensionalization
if Va > 1
    bhat = b/(2*Va);
    chat = c/(2*Va);
else
    % Means the plane is on the ground and it is stationary
    bhat = 0;
    chat = 0;
end

%% Calculating the forces and moments now
% This is in the form of a Clamped look up table
% Clampped values
aL  = min(max(alpha, params.CL_ALPHA(1)),  params.CL_ALPHA(end));
aD  = min(max(alpha, params.CD0_ALPHA(1)), params.CD0_ALPHA(end));
bD  = min(max(beta,  params.CDB_BETA(1)),  params.CDB_BETA(end));
% Using interp1 for linear interpolation
CL_a  = interp1(params.CL_ALPHA,  params.CL_TABLE,  aL);
CD0_a = interp1(params.CD0_ALPHA, params.CD0_TABLE, aD);
CDb_b = interp1(params.CDB_BETA,  params.CDB_TABLE, bD);

% Coefficients
CL = CL_a + params.CLde*deltaE;
CD = CD0_a + params.CDi*CL^2 + CDb_b + params.CDde*(deltaE/0.3);
CY = params.CYb*beta;
Cl = params.Clb*beta + params.Clp*bhat*p + params.Clr*bhat*r ...
    + params.Clda*deltaA + params.Cldr*deltaR;
Cm = params.Cma*alpha + params.Cmde*deltaE + params.Cmq*chat*q ...
    + params.Cmadot*chat*alphadot;
Cn = params.Cnb*beta + params.Cnr*bhat*r + params.Cndr*deltaR ...
    + params.Cnda*deltaA + params.Cn0;

% Forces: wind -> body
F_wind = qbar*S*[-CD; CY; -CL];
ca = cos(alpha); sa = sin(alpha);
cb = cos(beta);  sb = sin(beta);
% Directly from Brian and Lewis textbook
C_wind2body = [ca*cb,-ca*sb,-sa;sb,cb, 0;sa*cb, -sa*sb,  ca];
F_Aero = C_wind2body*F_wind;

% Moments: about AERORP -> about CG
M_ref  = qbar*S*[b*Cl; c*Cm; b*Cn];
% To make it go about CoG, we need a moment transfer given as
% M + (posn X F) where X is the cross operation. Basic in force and moment
% transfer
M_Aero = M_ref + cross(params.R_AERO, F_Aero);
end