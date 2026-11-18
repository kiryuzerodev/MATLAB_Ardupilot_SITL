function p = SITL_RascalParams()
% This function defines all the constants to be used when simulating the
% Rascal aircraft. The parameters are derived from the XML file of the
% Rascal 110
% Body frame: x forward, y right, z down, origin at CG.
% JSBSim structural frame: x aft, y right, z up, origin at VRP (nose).

%% Environment
disp("Loading the Environment data....")
p.g = 9.80665;                 % m/s^2

%% Mass and Inertia
disp("Loading the Mass data....")
p.MASS = 5.8967 + 0.6804;   % kg (empty + full fuel)
Ixx = 2.6438; Iyy = 2.1015; Izz = 2.5896;   % kg*m^2
Ixy = 0;      Ixz = 0;      Iyz = 0;
p.INERTIA_TENSOR  = [ Ixx -Ixy -Ixz;
                     -Ixy  Iyy -Iyz;
                     -Ixz -Iyz  Izz];
p.INERTIA_INVERSE = inv(p.INERTIA_TENSOR);

%% Geometry
disp("Loading the Geometry data....")
p.S = 0.9820;               % Wing area (m^2)
p.b = 2.7950;               % Wingspan (m)
p.c = 0.3505;               % Mean chord (m)

% Structural frame -> body frame (about CG)
cg = [0.9246 0 0.1016];     % CG in structural frame (m)
s2b = @(r) [-(r(1)-cg(1)); r(2)-cg(2); -(r(3)-cg(3))];

p.R_AERO = s2b([0.9500 0 0]);           % Aero reference point (AERORP)
p.R_PROP = s2b([0.0254 0 0]);           % Propeller (thrust point)

%% Landing Gear
disp("Loading the Landing Gear data....")
p.R_GEAR = [s2b([0.8407 -0.3277 -0.3327]), ...   % Left main
            s2b([0.8407  0.3277 -0.3327]), ...   % Right main
            s2b([1.7501  0      -0.1016])];      % Tail wheel
p.GEAR_K = 7005.1;          % Spring coeff (N/m)
p.GEAR_C = 1459.4;          % Damping coeff (N*s/m)
p.MU_STATIC  = 0.8;
p.MU_DYNAMIC = 0.5;
p.MU_ROLLING = 0.1;
p.TAIL_MAX_STEER = deg2rad(360);

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
p.P_MAX    = 2207.27;            % W, engine power at full throttle
p.PROP_DIA = 18*0.0254;          % m
p.I_PROP   = 0.05*1.35582;       % kg*m^2 (JSBSim value; ~0.003 for a real prop)
p.PROP_DIRECTION = 1;                % 1 = clockwise viewed from behind (JSBSim default)

p.PROP_J  = [0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 1.0 1.4];
p.PROP_CT = [0.0776 0.0744 0.0712 0.0655 0.0588 0.0518 0.0419 0.0318 0.0172 -0.0058 -0.0549];
p.PROP_CP = [0.0902 0.0893 0.0880 0.0860 0.0810 0.0742 0.0681 0.0572 0.0467 0.0167 -0.0803];

%% Aerodynamics (coefficients are per rad, from JSBSim)
disp("Loading the Aero data....")
% Drag
p.CD0_ALPHA = [-1.57 -0.26 0 0.26 1.57];
p.CD0_TABLE = [ 1.50  0.056 0.028 0.056 1.50];
p.CDi       = 0.04;             % times CL^2
p.CDB_BETA  = [-1.57 -0.26 0 0.26 1.57];
p.CDB_TABLE = [ 1.23  0.05 0 0.05 1.23];
p.CDde      = 0.03;             % times NORMALIZED elevator (-1 to 1)
% Side force
p.CYb = -1.0;
% Lift
p.CL_ALPHA = [-0.20 0 0.23 0.60];
p.CL_TABLE = [-0.75 0.25 1.40 0.71];
p.CLde = 0.2;
% Roll
p.Clb  = -0.10;
p.Clp  = -0.40;
p.Clr  =  0.15;
p.Clda =  0.13;                 % Mach 0 value (Mach effect negligible here)
p.Cldr =  0.01;
% Pitch
p.Cma   = -0.5;
p.Cmde  = -0.5;                 % Mach 0 value
p.Cmq   = -12.0;
p.Cmadot = -7.0;
% Yaw
p.Cnb  =  0.12;
p.Cnr  = -0.15;
p.Cndr = -0.05;
p.Cnda = -0.03;
p.Cn0  =  0.0007;               % tail incidence

%% Servos
disp("Loading Servo data....")
p.PWM_MIN  = 1100; 
p.PWM_TRIM = 1500;
p.PWM_MAX = 1900;   % Arduplane default values
p.SERVO_TAU  = [0.05 0.05 0.05 0.05]';        % s, first-order lag for every channel. Thrust is the slowest
p.SERVO_RATE = [10 10 10 7]';          % rad/s -> rate at which servo can move -> faster than this and gets saturated

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
disp("Rascal parameters loaded!")
end