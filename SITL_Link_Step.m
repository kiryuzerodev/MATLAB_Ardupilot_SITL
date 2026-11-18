function pwm = SITL_Link_Step(gyro, accel, pos, att, vel)
% One exchange with ArduPilot per Simulink step
% This is AI generated! Derive once later - 15th Nov, 2026
% V2: Adds a timeout to stop as soon as SITL is stopped
persistent u t last_pwm connected   % kept alive between calls
% Open the port only on the first call
if isempty(u)
    u = udpport("datagram","IPV4","LocalPort",9002);
    t = 0;
    last_pwm = 1500*ones(16,1);
    connected = false;
end
if connected
    timeout = 2;      % s, ArduPilot stopped mid-run
else
    timeout = 120;    % s, time to start SITL
end
% 1. Receive: wait for ArduPilot's PWM
tStart = tic;
while u.NumDatagramsAvailable == 0
    if toc(tStart) > timeout
        fprintf("SITL_Link: no data from ArduPilot for %d s, stopping simulation\n", timeout);
        set_param(bdroot,'SimulationCommand','stop');
        pwm = last_pwm;
        return
    end
    pause(1e-4);
end
connected = true;
d = read(u,1,"uint8");
b = uint8(d.Data);
rate = double(typecast(b(3:4),"uint16"));
pwm  = double(typecast(b(9:40),"uint16"))';   % first 16 channels, as a column
last_pwm = pwm;
% 2. Send: reply with this step's sensors
t = t + 1/rate;
msg = struct('timestamp', t, ...
    'imu', struct('gyro', gyro', 'accel_body', accel'), ...
    'position', pos', 'attitude', att', 'velocity', vel');
write(u, uint8([jsonencode(msg) newline]), d.SenderAddress, d.SenderPort);
end