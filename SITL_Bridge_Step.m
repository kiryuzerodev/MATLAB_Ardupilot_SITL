function [theta_cmd, phi_cmd, armed, mode, mnvr, fresh] = SITL_Bridge_Step()
% Reads whatever the bridge sent last. NEVER waits - if it waited, ArduPilot
% would wait, and then everyone waits, forever. Lock-step is a fragile friendship.
% This code is AI generated: Working of UDP and IPV4 is not needed so I am
% adding only comments to make it easier to understand.
persistent u last
if isempty(u)
    u = udpport("datagram","IPV4","LocalPort",9003);
    last = zeros(6,1);
end
fresh = 0;
n = u.NumDatagramsAvailable;
if n > 0
    d = read(u, n, "uint8");          % grab everything, only the newest matters
    b = uint8(d(end).Data);
    last = double(typecast(b, "double"))';
    fresh = 1;                        % new data this step, yay
end
armed     = last(2);
mode      = last(3);
theta_cmd = last(4);                  % TECS pitch demand (rad)
phi_cmd   = last(5);                  % L1 roll demand (rad), free bonus for later
mnvr      = last(6);                  % Lua maneuver value, 0 if Lua is quiet
end