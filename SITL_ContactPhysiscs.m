function [h_ref_wing,h_ref_tail,F_cont,M_cont] = SITL_ContactPhysiscs(X,params)
% This is a function that adds a basic spring damper for every significant
% part of the aircraft. This helps defines the geometry of single part of
% the aircraft, and takes care of cases such as clipping into the ground.
% This is also important for obstacle avoidance, if decided in the future.
% It also gives us the chance to read the position of the wing and tail and
% use ground effect models on the same

% Extract the necessary states
z_ac = X(3); % NED so -ve is above ground
V= X(4:6);
AngRate= X(10:13);
q0 = X(7);
q1 = X(8);
q2 = X(9);
q3 = X(10);
F_cont = [0;0;0];
M_cont = [0;0;0];
% Hello once more!
q_Body2NED = [1-2*(q2^2+q3^2), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
    2*(q1*q2+q0*q3), 1-2*(q1^2+q3^2), 2*(q2*q3-q0*q1);
    2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1^2+q2^2)];
% The goal: get everyone in NED to see height
    for i = 1:params.CONTACT_COUNT
        cpoint = params.CONTACT_POINTS(i,:)';
        cpointNED = q_Body2NED*cpoint;
        z_point = z_ac + cpointNED(3);

        % Check if it is a crucial point
        if i == 1
            h_ref_wing = z_point;
        end
        if i==2
            h_ref_tail = z_point;
        end
        if z_point > 0 % This is again cause NED
            penetration = z_point;
            contactVelocity = V + cross(AngRate(1:3), cpointNED);
            dampingForce = params.CONTACT_DAMPING * max(contactVelocity(3), 0);
            F_contact = [0; 0; -params.CONTACT_STIFFNESS * penetration - dampingForce];
            F_cont = F_cont + F_contact;
            M_cont = M_cont + cross(cpointNED, F_contact);
        end

    end
    
end
