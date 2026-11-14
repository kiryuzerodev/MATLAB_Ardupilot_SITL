%% Almost fully written by me
% Source for information is at:
% https://github.com/ArduPilot/ardupilot/blob/98f6b8395adba40c6d04415a26251a1ca49a2318/libraries/SITL/SIM_JSON.h

% Command to start SITL:
% sim_vehicle.py -v ArduPlane -f JSON --model json:127.0.0.1 --console --map
function SITL_Link_Test
% Code to just see if the UDP port 9002 - revision UDP is without handshake
UDP_Port_Object = udpport("datagram","IPV4","LocalPort",9002);
t = 0; % time variable to check time between calls

GreetingMessage = true;
while true
    if UDP_Port_Object.NumDatagramsAvailable == 0
        pause(5e-4);
        continue
    end
    Datagram = read(UDP_Port_Object,1,"uint8");

    % Once we have read the data, we will check if it is the file signature
    % or also known as Magic Number
    % these Magic Numbers are hard coded by Ardupilot's dev team!
    Byte = uint8(Datagram.Data);
    Magic_Number = typecast(Byte(1:2),uint16);
    if Magic_Number ~= 18458 && Magic_Number ~= 29569
        fprintf("Nigga what the fuck are you trying to run?? Wrong Magic!");
    end

    % Now we can actually use the data
    rate  = typecast(b(3:4),"uint16");
    count = typecast(b(5:8),"uint32");
    pwm   = typecast(b(9:end),"uint16");

    if GreetingMessage == true
        fprintf("******in a thick Chineese accent******")
        fprintf("Your SITL is connected successfullay!")
        GreetingMessage = false;
    end

    %% Dummy Plant
    t = t + 1/double(rate);
    msg = struct('timestamp', t, ... % Time stamp
        'imu', struct('gyro',[0 0 0], 'accel_body',[0 0 -9.81]), ... % IMU Data
        'position',[0 0 0], ... % Position - these are all Vector3f. For x,y,z
        'attitude',[0 0 0], ... % Euler Angles - can pass quaternions as well but that shi is confusing
        'velocity',[0 0 0]);    % Body velocity
    % NOTE: Ardupilot does not care about convention, so you need to be
    % aware of how you are sending the data

    % Send the message
    write(u, uint8([jsonencode(msg) newline]), Datagram.SenderAddress, Datagram.SenderPort);

    if mod(count,400)==0
        fprintf("frame %d  PWM1-4: %s\n", count, mat2str(pwm(1:4)));
    end
end
end