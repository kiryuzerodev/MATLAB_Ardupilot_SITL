function deltaDOT = SITL_ServoLag (deltaCMD,delta,params)
% First order lag in discrete form. In laplace, it is 1/tau*s+1
deltaDOT = (deltaCMD-delta)./params.SERVO_TAU;
% This is the maximum rate it can be allowed to go at. In rad/s. Backwards
% and forwards
deltaDOT = max(-params.SERVO_RATE, min(params.SERVO_RATE, deltaDOT));
end