%This code incrementally increase the belt speeds to allow the participant to find out their preferred walking speed;
%Orginal idea in Kinematic variability and local dynamic stability of upper body motions when walking at different speeds
%I cited this from Habitual exercise evokes fast and persistent adaptation during split-belt walking

ip = '127.0.0.1:22222';
initial_tread_speed_min = 0.8;
initial_tread_speed_max = 1.8;
direction = 1; %1 == gradually increasing treadmill speed; otherwise, gradually decrease treadmill speed

%%%%%Remote connection to Bertec Treadmill control panel%%%%%%%%%%%
% Open the connection (do it only once at start of the control session)
remote = tcpip('localhost', 4000);
fclose(remote); %In case TCP/IP hasn't properly closed last time
fopen(remote);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if direction == 1
    tread_speed = initial_tread_speed_min;
    tm_set(remote, [tread_speed,tread_speed], [0.5,0.5]);
    while tread_speed <= initial_tread_speed_max
        pause(4);
        tread_speed = tread_speed + 0.05;
        tm_set(remote, [tread_speed,tread_speed], [0.5,0.5]);
        disp(['Current treadmill speed is: ',num2str(tread_speed)])
    end
else
    tread_speed = initial_tread_speed_max;
    tm_set(remote, [tread_speed,tread_speed], [0.25,0.25]);
    while tread_speed >= initial_tread_speed_min
        pause(4);
        tread_speed = tread_speed - 0.05;
        tm_set(remote, [tread_speed,tread_speed], [0.25,0.25]);
        disp(['Current treadmill speed is: ',num2str(tread_speed)])
    end
end

tm_set(remote, [0,0], [0.8,0.8]);
fclose(remote);