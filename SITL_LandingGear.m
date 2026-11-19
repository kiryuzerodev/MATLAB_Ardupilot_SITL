function [F_gear,M_gear]=SITL_LandingGear(X,delta,params)
% Based on: https://kirilboychev.com/aircraft-ground-interaction-simulation
% and my logic
% The landing gears on Rascal 110 are:
% 1 - Left Main 
% 2 - Right Main
% 3 - Tail

% Extract necessary states:
z = X(3);
q0 = X(7);
q1 = X(8);
q2 = X(9);
q3 = X(10);

% Hello again
q_Body2NED = [1-2*(q2^2+q3^2), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
    2*(q1*q2+q0*q3), 1-2*(q1^2+q3^2), 2*(q2*q3-q0*q1);
    2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1^2+q2^2)];

% Compute for every landing gear (here 3)
for i = 1:3
    wheel_loc = params.R_PROP(i,:);

end

end