function [deltaCMD] = SITL_ServoMap(PWM,params)
% This is a function that takes a PWM value, like 1700 and transalates that
% into an actual command surface deflection angle. The bridge between PWM
% signal and actual surface movement

% Throttle should not go to trim value - map from 0 to 1
PWM_Thr = PWM(3);
deltaThr = max(0,min(1,(PWM_Thr - params.PWM_MIN)/(params.PWM_MAX - params.PWM_MIN)));

% If no value in the signal, then goes to trim value. But throttle gets
% mapped to 0 directly
for k = 1:length(PWM)
    if PWM(k) == 0
        PWM(k) = params.PWM_TRIM;
    end
end
% Extract the PWM values
% Since there are 16 channels, by default the assignment of these will be
% as follows: ch1 aileron, ch2 elevator, ch3 throttle, ch4 rudder
PWM_Ail = PWM(1);
PWM_Ele = PWM(2);
PWM_Rud = PWM(4);

% Same logic but mapping from -1 to 1
half = params.PWM_MAX - params.PWM_TRIM;
Ail = max(-1,min(1,(PWM_Ail - params.PWM_TRIM) / half));
Ele = max(-1,min(1,(PWM_Ele - params.PWM_TRIM) / half));
Rud = max(-1,min(1,(PWM_Rud - params.PWM_TRIM) / half));

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