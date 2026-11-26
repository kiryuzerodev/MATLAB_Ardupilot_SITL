function p = DATCOM_RascalParams()
% This function defines all the constants to be used when simulating the
% Rascal aircraft. The parameters are derived from the DATCOM results of the
% Rascal 110
% Body frame: x forward, y right, z down, origin at CG.
% JSBSim structural frame: x aft, y right, z up, origin at VRP (nose).

%% Environment
disp("Loading the Environment data....")
p.g = 9.80665;                 % m/s^2

%% Mass and Inertia
disp("Loading the Mass data....")
p.MASS = 17*0.45359237;     % kg, Datcom WT = 17 lb (7.711 kg)
Ixx = 2.6438; Iyy = 2.1015; Izz = 2.5896;   % kg*m^2
Ixy = 0;      Ixz = 0;      Iyz = 0;
p.INERTIA_TENSOR  = [ Ixx -Ixy -Ixz;
                     -Ixy  Iyy -Iyz;
                     -Ixz -Iyz  Izz];
p.INERTIA_INVERSE = inv(p.INERTIA_TENSOR);

%% Geometry
disp("Loading the Geometry data....")
p.S = 0.9811;               % Wing area (m^2), Datcom SREF = 10.56 ft^2
p.b = 2.7920;               % Wingspan (m), Datcom BLREF = 9.16 ft
p.c = 0.3810;               % Mean chord (m), Datcom CBARR = 1.25 ft

% Structural frame -> body frame (about CG)
cg = [0.9246 0 0.1016];     % CG in structural frame (m)
s2b = @(r) [-(r(1)-cg(1)); r(2)-cg(2); -(r(3)-cg(3))];

p.R_AERO = [0; 0; 0];                   % Datcom moments are already about the CG (XCG = 2.0 ft)
p.R_PROP = s2b([0.0254 0 0]);           % Propeller (thrust point)

%% Landing Gear
disp("Loading the Landing Gear data....")
p.R_GEAR = [(s2b([0.8407 -0.3277 -0.3327]))'; ...   % Left main
            (s2b([0.8407  0.3277 -0.3327]))'; ...   % Right main
            (s2b([1.7501  0      -0.1016]))'];      % Tail wheel
p.GEAR_K = 7005.1;          % Spring coeff (N/m)
p.GEAR_C = 1459.4;          % Damping coeff (N*s/m)
p.MU_STATIC  = 0.8;
p.MU_DYNAMIC = 0.5;
p.MU_ROLLING = 0.1;
p.TAIL_MAX_STEER = deg2rad(30);

%% Contact points - these are crucial points we want to know the exact location of
 
p.CONTACT_POINTS = [(s2b([0.9246  0 0.2540]))'; ... % Wing 1/4th MAC Location (any side)
                    (s2b([2.0426  0 0.1516]))'; ... % Tail 1/4th MAC Location (any side)
                    (s2b([0.9246  1.3970 0.2921]))'; ... % Wing Tip Right
                    (s2b([0.9246 -1.3970 0.2921]))'; ... % Wing Tip Left
                    (s2b([2.0426 -0.4636 0.1516]))'; ... % Tail Horizontal Stabilizer Left tip
                    (s2b([2.0426  0.4636 0.1516]))'; ... % Tail Horizontal Stabilizer Right tip
                    (s2b([2.0426  0 0.4374]))'; ... % Tail Vertical Stabilizer Tip
                    (s2b([0.3786  0 -0.1270]))' ... % Propeller Tip (nose)
                    ];
p.CONTACT_COUNT = size(p.CONTACT_POINTS,1);
p.CONTACT_STIFFNESS = 70000;
p.CONTACT_DAMPING = 1500;
%% Control Surfaces (limits in rad)
disp("Loading the Control Surface data....")
p.DE_MIN = -0.35; 
p.DE_MAX = 0.30;      % Elevator

p.DA_MAX = 0.35;
p.DA_MIN = -0.35;      % Aileron

p.DR_MAX = 0.35;    
p.DR_MIN = -0.35;    % Rudder

%% Propulsion (JSBSim: Zenoah G-26A as 2207 W source + 18x8 prop)
disp("Loading the Propulsion data....")
% New: Switch off engines
p.P_PROP_ON = 0; % 0 or 1 based on if we want to switch off engines. 0: Datcom tables are POWER OFF
p.P_MAX    = 2207.27;            % W, engine power at full throttle
p.PROP_DIA = 18*0.0254;          % m
p.I_PROP   = 0.000085*1.35582;       % kg*m^2 (JSBSim value; ~0.003 for a real prop)
p.PROP_DIRECTION = 1;                % 1 = clockwise viewed from behind (JSBSim default)

p.PROP_J  = [0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 1.0 1.4];
p.PROP_CT = [0.0776 0.0744 0.0712 0.0655 0.0588 0.0518 0.0419 0.0318 0.0172 -0.0058 -0.0549];
p.PROP_CP = [0.0902 0.0893 0.0880 0.0860 0.0810 0.0742 0.0681 0.0572 0.0467 0.0167 -0.0803];

%% Aerodynamics (coefficients are per rad, from Digital Datcom - Jodeh 2006 Appendix G, WBVH, power off)
disp("Loading the Aero data....")
% Lift related terms
p.AOA_CL = (-10:2:28)*pi/180;
p.CL = [-0.660 -0.456 -0.262 -0.072 0.114 0.310 0.516 0.730 0.951 1.177 1.399 1.580 1.736 1.859 1.916 1.730 0.715 -0.186 -0.989 -1.488];
% Dynamic lift
p.CLq = 5.3377;
p.AOA_DYN = (-10:2:28)*pi/180;
p.CLadot = [2.2575 2.2116 2.2569 2.2185 2.2059 2.2248 2.2208 2.1944 2.1154 2.0065 1.7796 1.4267 1.1184 0.6096 -0.8062 -3.6222 -5.3892 -5.5153 -4.2840 -2.9095];

% Drag related terms
p.AOA_CD = (-10:2:28)*pi/180;
p.CD = [0.043 0.033 0.026 0.023 0.022 0.026 0.033 0.044 0.060 0.080 0.105 0.129 0.153 0.176 0.191 0.174 0.098 0.079 0.121 0.184];
p.CDi = 0; % because induced also comes from Datcom directly

% Side force related terms
p.CYb = -0.3221;
% Dynamic side force
p.CYp = [-0.0910 -0.0949 -0.0985 -0.1019 -0.1052 -0.1091 -0.1137 -0.1192 -0.1263 -0.1370 -0.1956 -0.1132 -0.1261 -0.1324 -0.1376 -0.1336 -0.0888 -0.0507 -0.0193 0.0013];

% Rolling moment related terms
p.AOA_CLB = (-10:2:28)*pi/180;
p.Clb = [-0.0828 -0.0859 -0.0890 -0.0918 -0.0944 -0.0973 -0.1004 -0.1037 -0.1071 -0.1107 -0.1141 -0.1165 -0.1183 -0.1192 -0.1185 -0.1121 -0.0878 -0.0689 -0.0525 -0.0414];
% Dynamic rolling moment
% Datcom prints CLP positive, sign flipped so roll damping is negative
p.Clp = -[0.9374 0.9036 0.8657 0.8337 0.8445 0.8898 0.9322 0.9660 0.9912 0.9884 0.8640 0.6812 0.5214 0.2588 -0.5315 -2.9662 -4.2387 -3.4418 -2.6448 -1.5533];
p.Clr = 0.15; % NOT Datcom (NDM in every row), JSBSim value

% Pitching moment related terms
p.AOA_CM = (-10:2:16)*pi/180;
p.CM_TABLE = [-0.0330 -0.0296 -0.0232 -0.0302 -0.0418 -0.0536 -0.0665 -0.0821 -0.1013 -0.1257 -0.1558 -0.1996 -0.2542 -0.3180];
% Dynamic pitching moment
p.Cmq = -12.2326;
p.Cmadot = [-6.1593 -6.0332 -6.1593 -6.0562 -6.0218 -6.0734 -6.0619 -5.9874 -5.7754 -5.4752 -4.8575 -3.8938 -3.0516 -1.6639 2.1996 9.8835 14.7078 15.0516 11.6883 7.9412];

% Yawing moment related terms
p.Cnb = 0.0131;
p.Cn0 = 0;
% Dynamic yawing moment
p.Cnp = [0.0694 0.0488 0.0275 0.0069 -0.0141 -0.0377 -0.0647 -0.0966 -0.1364 -0.1947 -0.5155 -0.0521 -0.1238 -0.1571 -0.1788 -0.1661 -0.0854 0.0102 0.0756 0.1290];
p.Cnr = [-0.0367 -0.0347 -0.0338 -0.0336 -0.0342 -0.0356 -0.0379 -0.0413 -0.0458 -0.0515 -0.0582 -0.0640 -0.0692 -0.0730 -0.0736 -0.0634 -0.0353 -0.0343 -0.0480 -0.0639];

%% Control Surface data
disp("Loading the Control Surface Aero data....")
%% Control surfaces (Digital Datcom, Jodeh 2006 App. G) - increments, deflections in rad
% Elevator: p.164 "TAIL PLAIN TRAILING-EDGE FLAP" (+ = trailing edge down)
p.DE = [-20 -10 0 10 20]*pi/180;
% Lift increment
p.dCL_DE = [-0.097 -0.062 0.000 0.062 0.097];
% Pitching moment increment
p.dCM_DE = [0.2218 0.1402 -0.0001 -0.1402 -0.2218];
% Drag increment: profile part
p.dCDMIN_DE = [0.00770 0.00245 0 0.00245 0.00770];
% Drag increment: induced part, rows = alpha (-10:2:28 deg), cols = p.DE
p.AOA_DCDI = (-10:2:28)*pi/180;
p.dCDI_DE = [
    3.19E-03  1.53E-03 -6.20E-07  2.86E-04  1.25E-03
    2.49E-03  1.08E-03 -1.70E-07  7.35E-04  1.95E-03
    1.75E-03  6.05E-04  3.02E-07  1.21E-03  2.69E-03
    9.74E-04  1.09E-04  7.98E-07  1.70E-03  3.47E-03
    1.77E-04 -4.00E-04  1.31E-06  2.21E-03  4.27E-03
    -6.07E-04 -9.00E-04  1.81E-06  2.71E-03  5.05E-03
    -1.39E-03 -1.40E-03  2.31E-06  3.21E-03  5.84E-03
    -2.18E-03 -1.90E-03  2.81E-06  3.72E-03  6.62E-03
    -2.98E-03 -2.42E-03  3.32E-06  4.23E-03  7.42E-03
    -3.81E-03 -2.95E-03  3.86E-06  4.76E-03  8.26E-03
    -4.68E-03 -3.50E-03  4.41E-06  5.31E-03  9.12E-03
    -5.65E-03 -4.12E-03  5.02E-06  5.93E-03  1.01E-02
    -6.72E-03 -4.81E-03  5.71E-06  6.62E-03  1.12E-02
    -7.87E-03 -5.54E-03  6.45E-06  7.35E-03  1.23E-02
    -9.25E-03 -6.42E-03  7.33E-06  8.23E-03  1.37E-02
    -1.12E-02 -7.69E-03  8.59E-06  9.50E-03  1.57E-02
    -1.43E-02 -9.63E-03  1.06E-05  1.14E-02  1.87E-02
    -1.73E-02 -1.16E-02  1.25E-05  1.34E-02  2.18E-02
    -2.04E-02 -1.36E-02  1.45E-05  1.54E-02  2.49E-02
    -2.27E-02 -1.50E-02  1.59E-05  1.68E-02  2.72E-02];

% Aileron: p.171 "WING PLAIN TRAILING-EDGE FLAP", differential
% dA = (deltaL - deltaR)/2, i.e. deflection of ONE aileron
p.DA = [-20 -10 0 10 20]*pi/180;
% Rolling moment increment
p.dCl_DA = [-0.074618 -0.042649 0 0.042649 0.074618];
% Yawing moment increment (adverse yaw), rows = alpha (-10:2:28 deg), cols = p.DA
p.AOA_DA = (-10:2:28)*pi/180;
p.dCn_DA = [
    -4.623e-3 -2.639e-3 0  2.639e-3  4.623e-3
    -3.131e-3 -1.788e-3 0  1.788e-3  3.131e-3
    -1.694e-3 -9.674e-4 0  9.674e-4  1.694e-3
    -3.264e-4 -1.864e-4 0  1.864e-4  3.264e-4
    1.011e-3  5.774e-4 0 -5.774e-4 -1.011e-3
    2.430e-3  1.388e-3 0 -1.388e-3 -2.430e-3
    3.924e-3  2.241e-3 0 -2.241e-3 -3.924e-3
    5.480e-3  3.129e-3 0 -3.129e-3 -5.480e-3
    7.085e-3  4.045e-3 0 -4.045e-3 -7.085e-3
    8.723e-3  4.981e-3 0 -4.981e-3 -8.723e-3
    1.032e-2  5.891e-3 0 -5.891e-3 -1.032e-2
    1.156e-2  6.599e-3 0 -6.599e-3 -1.156e-2
    1.257e-2  7.174e-3 0 -7.174e-3 -1.257e-2
    1.329e-2  7.591e-3 0 -7.591e-3 -1.329e-2
    1.346e-2  7.685e-3 0 -7.685e-3 -1.346e-2
    1.168e-2  6.672e-3 0 -6.672e-3 -1.168e-2
    3.387e-3  1.934e-3 0 -1.934e-3 -3.387e-3
    -3.026e-3 -1.728e-3 0  1.728e-3  3.026e-3
    -8.643e-3 -4.935e-3 0  4.935e-3  8.643e-3
    -1.226e-2 -6.998e-3 0  6.998e-3  1.226e-2];

% Rudder: Datcom was NOT run for a rudder -> old JSBSim values (per rad)
p.Cldr =  0.01;   % Rolling moment due to rudder
p.Cndr = -0.05;   % Yawing moment due to rudder

%% Servos
disp("Loading Servo data....")
% Changed: Removed bullshit AI logic of all same PWM Trim
% Servo PWM ranges (per channel, same as ArduPilot SERVOn_MIN/TRIM/MAX)
% aileron elevator throttle rudder (because Channels are assigned this way)
p.PWM_MIN  = [1100; 1100; 1100; 1100];
p.PWM_TRIM = [1500; 1500; 1100; 1500];   % note: for Throttle trim = min always
p.PWM_MAX  = [1900; 1900; 1900; 1900]; 

% Arduplane default values
p.SERVO_TAU  = [0.05 0.05 0.05 0.05]';        % s, first-order lag for every channel. Thrust is the slowest
p.SERVO_RATE = [10 10 10 7]';          % rad/s -> rate at which servo can move -> faster than this and gets saturated

%% Rangefinder 
disp("Loading Rangefinder/LiDAR/RADAlt data....")
p.RNG_POS = s2b([0.9246 0 -0.0191]);
p.RNG_MAX = 50;          % m, match RNGFND1_MAX in the param file
%% Home (Waterfly Warehouse) - must match the SITL launch location
disp("Loading the Home data....")
p.HOME_LAT = 12.886097191929261;   % deg
p.HOME_LNG = 79.8657674964945;     % deg
p.H_HOME   = 0.0;                  % m (MSL)
p.HOME_HDG = deg2rad(0.0);         % rad, takeoff heading (north)
%% Simulation
disp("Loading the Simulation data....")
p.DT = 1/400;                   % fixed step (s), match SITL frame rate
p.SITL_PORT = 9002;

%% Control select - which channels YOUR controller owns
% []    = ArduPilot flies everything (option 1)
% [2]   = you own the elevator (option 4a / A)
% [2 3] = elevator + throttle (option 4b)
p.CTRL_MASK = [];
disp("Rascal parameters loaded!")
end