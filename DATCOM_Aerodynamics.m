function [F_Aero,M_Aero] = DATCOM_Aerodynamics(X, Va, alpha, beta, qbar, alphadot, params,delta)
% This function just computes the force and moments for the plane. All
% coefficients come from Digital Datcom (Jodeh 2006, Appendix G), loaded
% by DATCOM_RascalParams.m. Tables are in alpha (rad) and deflection (rad)
%% States and inputs
p = X(11);
q = X(12);
r = X(13);
deltaA = delta(1);
deltaE = delta(2);
deltaR = delta(3); % Datcom has no rudder case -> old JSBSim Cldr, Cndr
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
% Clampped values - every table has its own alpha range
% Lift table
aL = alpha;
if aL < params.AOA_CL(1)
    aL = params.AOA_CL(1);
elseif aL > params.AOA_CL(end)
    aL = params.AOA_CL(end);
end
% Drag table
aD = alpha;
if aD < params.AOA_CD(1)
    aD = params.AOA_CD(1);
elseif aD > params.AOA_CD(end)
    aD = params.AOA_CD(end);
end
% Pitching moment table (only up to 16 deg)
aM = alpha;
if aM < params.AOA_CM(1)
    aM = params.AOA_CM(1);
elseif aM > params.AOA_CM(end)
    aM = params.AOA_CM(end);
end
% Clb table
aB = alpha;
if aB < params.AOA_CLB(1)
    aB = params.AOA_CLB(1);
elseif aB > params.AOA_CLB(end)
    aB = params.AOA_CLB(end);
end
% Dynamic derivative tables
aDyn = alpha;
if aDyn < params.AOA_DYN(1)
    aDyn = params.AOA_DYN(1);
elseif aDyn > params.AOA_DYN(end)
    aDyn = params.AOA_DYN(end);
end
% Elevator induced drag table
aDI = alpha;
if aDI < params.AOA_DCDI(1)
    aDI = params.AOA_DCDI(1);
elseif aDI > params.AOA_DCDI(end)
    aDI = params.AOA_DCDI(end);
end
% Aileron yaw table
aDA = alpha;
if aDA < params.AOA_DA(1)
    aDA = params.AOA_DA(1);
elseif aDA > params.AOA_DA(end)
    aDA = params.AOA_DA(end);
end
% Clamped deflections - Datcom only ran -20 to +20 deg
dE = deltaE;
if dE < params.DE(1)
    dE = params.DE(1);
elseif dE > params.DE(end)
    dE = params.DE(end);
end
dA = deltaA;
if dA < params.DA(1)
    dA = params.DA(1);
elseif dA > params.DA(end)
    dA = params.DA(end);
end

% Static (alpha) tables
CL_a   = interp1(params.AOA_CL,  params.CL,       aL);
CD_a   = interp1(params.AOA_CD,  params.CD,       aD);
CM_a   = interp1(params.AOA_CM,  params.CM_TABLE, aM);
Clb_a  = interp1(params.AOA_CLB, params.Clb,      aB);
% Dynamic (alpha) tables
CLad_a = interp1(params.AOA_DYN, params.CLadot, aDyn);
Cmad_a = interp1(params.AOA_DYN, params.Cmadot, aDyn);
CYp_a  = interp1(params.AOA_DYN, params.CYp,    aDyn);
Clp_a  = interp1(params.AOA_DYN, params.Clp,    aDyn);
Cnp_a  = interp1(params.AOA_DYN, params.Cnp,    aDyn);
Cnr_a  = interp1(params.AOA_DYN, params.Cnr,    aDyn);
% Elevator increments
dCL_e    = interp1(params.DE, params.dCL_DE,    dE);
dCM_e    = interp1(params.DE, params.dCM_DE,    dE);
dCDmin_e = interp1(params.DE, params.dCDMIN_DE, dE);
dCDi_e   = interp2(params.DE, params.AOA_DCDI, params.dCDI_DE, dE, aDI);  % (alpha, deltaE)
% Aileron increments
dCl_a  = interp1(params.DA, params.dCl_DA, dA);
dCn_a  = interp2(params.DA, params.AOA_DA, params.dCn_DA, dA, aDA);       % (alpha, deltaA)

% Coefficients
% Lift: static + pitch rate + alpha rate + elevator
CL = CL_a + params.CLq*chat*q + CLad_a*chat*alphadot + dCL_e;
% Drag: Datcom CD is total (profile + induced) + elevator profile + elevator induced
CD = CD_a + dCDmin_e + dCDi_e;
% Side force: sideslip + roll rate
CY = params.CYb*beta + CYp_a*bhat*p;
% Rolling moment: sideslip + roll rate + yaw rate + aileron + rudder
Cl = Clb_a*beta + Clp_a*bhat*p + params.Clr*bhat*r + dCl_a + params.Cldr*deltaR;
% Pitching moment: static + pitch rate + alpha rate + elevator
Cm = CM_a + params.Cmq*chat*q + Cmad_a*chat*alphadot + dCM_e;
% Yawing moment: sideslip + roll rate + yaw rate + aileron (adverse yaw) + rudder
Cn = params.Cnb*beta + Cnp_a*bhat*p + Cnr_a*bhat*r + dCn_a + params.Cndr*deltaR + params.Cn0;

% Forces: wind -> body
F_wind = qbar*S*[-CD; CY; -CL];
ca = cos(alpha); sa = sin(alpha);
cb = cos(beta);  sb = sin(beta);
% Directly from Brian and Lewis textbook
C_wind2body = [ca*cb,-ca*sb,-sa;sb,cb, 0;sa*cb, -sa*sb,  ca];
F_Aero = C_wind2body*F_wind;
% Moments: Datcom gives them about the CG, R_AERO = [0;0;0] in the params
M_ref  = qbar*S*[b*Cl; c*Cm; b*Cn];
% To make it go about CoG, we need a moment transfer given as
% M + (posn X F) where X is the cross operation. Basic in force and moment
% transfer
M_Aero = M_ref + cross(params.R_AERO, F_Aero);
end
