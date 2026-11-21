function [deltaCMD] = SITL_ServoMap(PWM,params)
% This is a function that takes a PWM value, like 1700 and transalates that
% into an actual command surface deflection angle. The bridge between PWM
% signal and actual surface movement

% Throttle should not go to trim value - map from 0 to 1
% Changed: Removed bullshit AI logic of all same PWM Trim
PWM_Thr = PWM(3);
% Normalize
deltaThr = (PWM_Thr - params.PWM_TRIM(3))/(params.PWM_MAX(3) - params.PWM_MIN(3));
% Clamp using saturation logic
if deltaThr > 1
    deltaThr = 1;
elseif deltaThr < 0
    deltaThr = 0;
end


% If no value in the signal, then goes to trim value. But throttle gets
% mapped to 0 directly
for k = 1:4
    if PWM(k) == 0
        PWM(k) = params.PWM_TRIM(k);
    end
end
% Extract the PWM values
% Since there are 16 channels, by default the assignment of these will be
% as follows: ch1 aileron, ch2 elevator, ch3 throttle, ch4 rudder
PWM_Ail = PWM(1);
PWM_Ele = PWM(2);
PWM_Rud = PWM(4);
% Changed: Removed bullshit AI logic of all same PWM Trim
% Logic is that, above trim is +1 and below is -1. Hence the first step is
% to normalize it
% Fix: Only half is taken in old, this gives negative values as well
% Fix: Removed else if and ensured exact trim value is handled  

% Ailerons
if PWM_Ail >= params.PWM_TRIM(1)
    deltaAil = (PWM_Ail-params.PWM_TRIM(1))/(params.PWM_MAX(1)-params.PWM_TRIM(1));
else
    deltaAil = (PWM_Ail-params.PWM_TRIM(1))/(params.PWM_TRIM(1)-params.PWM_MIN(1));
end
if deltaAil > 1
    deltaAil = 1;
elseif deltaAil < -1
    deltaAil = -1;
end

% Elevators
if PWM_Ele >= params.PWM_TRIM(2)
    deltaEle = (PWM_Ele-params.PWM_TRIM(2))/(params.PWM_MAX(2)-params.PWM_TRIM(2));
else 
    deltaEle = (PWM_Ele-params.PWM_TRIM(2))/(params.PWM_TRIM(2)-params.PWM_MIN(2));
end
if deltaEle > 1
    deltaEle = 1;
elseif deltaEle < -1
    deltaEle = -1;
end

% Rudder
if PWM_Rud >= params.PWM_TRIM(4)
    deltaRud = (PWM_Rud-params.PWM_TRIM(4))/(params.PWM_MAX(4)-params.PWM_TRIM(4));
else
    deltaRud = (PWM_Rud-params.PWM_TRIM(4))/(params.PWM_TRIM(4)-params.PWM_MIN(4));
end
if deltaRud > 1
    deltaRud = 1;
elseif deltaRud < -1
    deltaRud = -1;
end

% Fix: Change signs for elevator and rudder
Ail = deltaAil;
Ele = -deltaEle;
Rud = -deltaRud;


% Now to map these to actual angles, you need to just multiply
if Ail <= 0
    deltaAil = -Ail*params.DA_MIN;
else
    deltaAil = Ail*params.DA_MAX;
end

if Ele <= 0
    deltaEle = -Ele*params.DE_MIN;
else
    deltaEle = Ele*params.DE_MAX;
end

if Rud <= 0
    deltaRud = -Rud*params.DR_MIN;
else
    deltaRud = Rud*params.DR_MAX;
end

% Form the final output
deltaCMD = [deltaAil;deltaEle;deltaRud;deltaThr];


end