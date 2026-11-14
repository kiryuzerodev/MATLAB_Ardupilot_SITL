function pwm = SITL_Link_Step(gyro, accel, pos, att, vel)
% One exchange with ArduPilot per Simulink step
% This is AI generated! Derive once later - 15th Nov, 2026
% XXXX You also need to add a UNIT DELAY 
persistent u t          % kept alive between calls

% Open the port only on the first call
if isempty(u)
    u = udpport("datagram","IPV4","LocalPort",9002);
    t = 0;
end

% 1. Receive: wait for ArduPilot's PWM
while u.NumDatagramsAvailable == 0
    pause(1e-4);
end
d = read(u,1,"uint8");
b = uint8(d.Data);
rate = double(typecast(b(3:4),"uint16"));
pwm  = double(typecast(b(9:40),"uint16"))';   % first 16 channels, as a column

% 2. Send: reply with this step's sensors
t = t + 1/rate;
msg = struct('timestamp', t, ...
    'imu', struct('gyro', gyro', 'accel_body', accel'), ...
    'position', pos', 'attitude', att', 'velocity', vel');
write(u, uint8([jsonencode(msg) newline]), d.SenderAddress, d.SenderPort);
end