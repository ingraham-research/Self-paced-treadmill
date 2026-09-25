%This code is based on, and modified from the following paper:
% S Song, H Choi and SH Collins, Using force data to self-pace an instrumentedtreadmill and measure self-selected walking speed
% Original code github link: https://github.com/smsong/self-paced-treadmill/tree/JNER2020


pause(5) %For testing on myself

%Note, the force plates are in their own reference frame, and the x-direction in lab frame is the y for treadmill
%In this code, the naming of coordinates are those in the lab frame

%%%%%%%%%%%%%%Settings that change depending on experiment%%%%%%%%%%%%%%%%%
error_max = 10; %Max amount of errors we allow before disconnecting. For now, practically ignore all errors and go ahead
%Fz_thres = 50; %Threshold for registering a step as foot strike. For now setting this as a static variable. 
Fz_thres = 100; %Increase this for now (2026 Feb) to deal with treadmill plate issues

%Adjust these per participant:
time = 370; % Total time of experiment, in seconds
mass = 65; %In kg, including cosmed/exo/etc.
V_treadmill_target = 1.3; %V_tm_tgt in paper, this sets the initial speed
steps_wait_time = 5; %How many qualified steps to wait before SPT engages
%mass = 64;

send_to_exo = 0; %If flagged, we create also a UDP portal to send the treadmill speed commands to the exo

%Parameters for the controller for treadmill feedback
G_p = 0.13;
G_v = 0.15;
pos_offset = -0.1; %Position offset of human on treadmill, empirically set at -0.1 which is about near the center of treadmill
step_length_adjustment = 0; %Adjust this if needed, I set it at 0.25 normally - this is the "high" setting that sometimes treadmill just WANTS
del_t_target = 0.5; %How fast do we want the treadmill speed to achieve target? In seconds
max_treadmill_V = 2.3;
min_treadmill_V = 0.2;
a_treadmill_target_min = 0.01;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Define variables
fps = 100;
force_per_frame = 10;
num_frames = fps*time;
num_forces = num_frames*force_per_frame;
ip = '127.0.0.1:22222';
exo_ip = "10.18.54.152";


Fx = zeros(num_frames,2); 
Fy = zeros(num_frames,2); 
Fz = zeros(num_frames,2); 
My = zeros(num_frames,2); 
COP_x = zeros(num_frames,2); %In our QTM frame, x is the direction of treadmill motion
frame_elapsed_count = zeros(num_frames,1);

step_count = 0; %This counts all steps, including the ones rejected
tracked_steps = 0; %This counts only those steps that aren't excluded

step_flag = 0; %Here we define 1 == left foot strike, -1 == right foot strike, 0 == no strike
prev_valid_step = 0;
step_ok = 0; %Flag to store if the step should be discarded

%We will only update the treadmill speed once per step
V_treadmill = 0; %Vbar_tm in paper
V_treadmill_target_prev = V_treadmill_target;

a_treadmill_target = 0.5; %a_tm_tgt in paper
a_treadmill_target_prev = 0.5;

t_treadmill_prev_update = 0; %The last time treadmill speed was updated

%Create the butterworth filter
fc= 25;
fs = 100;
[filter_b,filter_a] = butter(3,fc/(fs/2));

%Use MATLAB's filter state feature to store initial states of the 
COP_x_initial_state = zeros(max(length(filter_a),length(filter_b))-1, 2);
Fx_initial_state = zeros(max(length(filter_a),length(filter_b))-1, 2);
Fz_initial_state= zeros(max(length(filter_a),length(filter_b))-1, 2);
My_initial_state = zeros(max(length(filter_a),length(filter_b))-1, 2);

%%%%%%%%%%%%%%%%%%%Define Kalman Filter variables and constants%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Most constants directly come from the paper
%R = 10^(-3)*[3.5,1.5;1.5,1.6];
R = 10^(-2)*[1.5,1.5;1.5,1.6];
P0 = 10^(-3)*[0.6,0;0,7.2];
g = 9.81;

X = nan(num_frames,2); %X = [pos_KF,V_KF]
acc_mea = 0; %a_mes in paper
V_human_mea = 0; %Vbar_mes in paper
pos_mea = 0; %Pmea in paper

%Initialize the states
X(1,:) = [0,0];

pos_current_step = 0; %yf1 in paper
pos_prev_step = 0; %yf0 in paper

pos_current_step_test = 0;
pos_prev_step_test = 0;

t_current_step = 0;%t1 in paper
t_prev_step = 0;%t0 in paper

KF_initialized = 0; %Determines whether we started tracking dt
filter_initialized = 0; %Whether we had collected 10 data points (3rd order butterworth filter requires at least 3 points)
n_ts = 0; %How many time steps did we have since the previous heel strike
pos_average = 0; %Average human position during a step
u_average = 0; %Average human velocity during a step
V_treadmill_average = 0; %Average treadmill velocity during a step

%For debugging how many loops we lost due to "continue"
loop_count = 0; %Always increments at the begining of a loop
loop_success_count = 0; %Only increments if a loop reached foot position update part

KF_non_initialized_continue_count = 0; %The continue which should trigger exactly once
Fx_isnan_continue_count = 0; %Triggeres whenever Fx sends a NaN frame
u_current_isnan_continue_count = 0; %Triggers if u_current is computed to be NaN

%On top of tracking how many times "continue" loss triggered, let's also track how much actual loop time is gone due to continue, as this gives a better idea of whether or not we actually have problem
t_total = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%Define data logging variables%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pos_estimate_average = nan(num_frames,1);
pos_inst_log = nan(num_frames,1);
V_human_estimate_average = nan(num_frames,1);
V_human_inst_log = nan(num_frames,1);
V_treadmill_average_log = nan(num_frames,1);
pos_mea_log = nan(num_frames,1);
V_human_mea_log = nan(num_frames,1);
V_treadmill_inst_log = nan(num_frames,1);
t_lapsed_treadmill_log = nan(num_frames,1);
t_lapsed_step_log = nan(num_frames,1);
step_length_log = nan(num_frames,1);
step_length_test_log = nan(num_frames,1);
pos_prev_step_log = nan(num_frames,1);
pos_current_step_log = nan(num_frames,1);
dt_log = nan(num_frames,1);


%%%%%Remote connection to Bertec Treadmill control panel%%%%%%%%%%%
% Open the connection (do it only once at start of the control session)
%First clear off any potentially leftover TCPIP clients
if exist('remote','var')
    clear remote;
end
remote = tcpclient('localhost', 4000);
tm_set(remote, [V_treadmill_target,V_treadmill_target], [a_treadmill_target,a_treadmill_target]);
t_treadmill_prev_update = datetime;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if send_to_exo
    udp_exo = udpport("byte"); %Add a udp port to talk to the exoskeleton. Change the port if needed, but we should probably leave it at 5004
    exo_port = 5004;
end

QCM('connect', ip,'force'); % Connects to QTM and keeps the connection alive.
error_count = 0; %Track how many exceptions we run into
dt = 0; %Initialize dt


label_last_step = NaN;   % QTM frame number of the last detected foot strike
t_current_step = 0;
i = 1;
frame_init = 1;
first_frame = 1; %Tracks whether we are in the 1st frame. If so, don't estimate dt from this frame.

while i <= num_frames 
    %Note that with this code, I don't have a fixed dt. Instead, dt changes depending on how fast the code can run
    %minimum dt is 1/fps of QTM

    % disp(['Frame ', num2str(i)]); %This shows the actual amount of frames we captured
    loop_count = loop_count + 1;
    t_frame_start = datetime; 

    %Update dt and the constants for Kalman filter
    if ~KF_initialized
        KF_initialized = 1;
        [Fx_frame, Fz_frame,My_frame,COP_x_frame,dataRT,frames,label] = get_frame_self_paced(QCM);
        if isnan(Fx_frame)
            Fx_frame = [0,0];
            COP_x_frame = [0;0];
        end
        X_current = [mean(COP_x_frame);0];
        P = P0;
        KF_non_initialized_continue_count = KF_non_initialized_continue_count + 1;
        continue;
    end

    %%%%%%%%%%%%%%%%%Try reading the data from QTM%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    try
        [Fx_frame, Fz_frame,My_frame,COP_x_frame,dataRT,frames,label] = get_frame_self_paced(QCM);

        if isnan(Fx_frame(1)) || isnan(Fx_frame(2))
            Fx_isnan_continue_count = Fx_isnan_continue_count + 1;
            continue;
        end

        %Filter the values using MATLAB's filter initial state feature. COPs should NOT be filtered because they are garbage below Fz threshold
        %[Fx_frame(1), Fx_initial_state(:,1)] = filter(filter_b, filter_a, Fx_frame(1), Fx_initial_state(:,1));
        %[Fx_frame(2), Fx_initial_state(:,2)] = filter(filter_b, filter_a, Fx_frame(2), Fx_initial_state(:,2));
        
        [Fz_frame(1), Fz_initial_state(:,1)] = filter(filter_b, filter_a, Fz_frame(1), Fz_initial_state(:,1));
        [Fz_frame(2), Fz_initial_state(:,2)] = filter(filter_b, filter_a, Fz_frame(2), Fz_initial_state(:,2));
        [My_frame(1), My_initial_state(:,1)] = filter(filter_b, filter_a, My_frame(1), My_initial_state(:,1));
        [My_frame(2), My_initial_state(:,2)] = filter(filter_b, filter_a, My_frame(2), My_initial_state(:,2));
        
        label = floor(double(label)/force_per_frame);   % QCM returns an integer type; convert before dividing
        if i<0 || label <frame_init 
            %I.e. we failed to read initial frame
            %Not sure what causes the 2nd case, but I have this happen a few times..?
            frame_init = 0;
            label = label - frame_init;
            i = label;
            continue;
        end

        if label <= 0
            continue;
        end

        if i == 0
            i = label;
            continue;
        end

        frame_elapsed = label - i;
        % if (i>frame_init && frame_elapsed <= 0 && frame_elapsed_count(i-1) <=0 && frame_elapsed_count(i-2) <=0) 
        %     %That means we have hit the end
        %     disp('Max time reached, ending experiment, the last frame captured is: ')
        %     disp(label);
        %     break;
        % end

        if frame_elapsed >1 %If we do get more than 1 frame, assume all the frames equal to the average value we computed
            if frame_elapsed >1500
                %In this case it would be more than 15 seconds of data...
                %might be a fluke,ignore the data and move on
                continue;
            end
            for j = i:i+frame_elapsed-1
                if j>num_frames + frame_init %Don't extrapolate to beyond end point
                    disp('Max time for computation reached,ending experiment, the last frame captured is: ')
                    disp(j);
                    break;
                end
                Fx(j,:) = Fx_frame;
                Fz(j,:) = Fz_frame;
                COP_x(j,:) = COP_x_frame;
                My(j,:) = My_frame;
            end
        end

        %Log the data if they are properly received
        Fx(i,:) = Fx_frame;
        Fz(i,:) = Fz_frame;
        My(i,:) = My_frame;
        COP_x(i,:) = COP_x_frame;
        frame_elapsed_count(i) = frame_elapsed;

    catch
        warning('Error reading QTM package, trying again');
        error_count = error_count + 1;
        if error_count <= error_max
            continue;
        else
            disp('Reached maximum error count due to QTM reading');
            break;
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %Update estimated current speed using measurements
    u_current = Fx_frame(1) + Fx_frame(2);
    u_current = u_current/mass; %The GRF values reported are in the lab reference frame, a positive X-force means acceleration

    if isnan(u_current)
        u_current_isnan_continue_count = u_current_isnan_continue_count + 1;
        continue;
    end

    %%%%% Update treadmill speed from time base from QTM frame numbers, not wall clock %%%%%%%%%%%%%%%
    if frame_elapsed <= 0
        continue;   % no new frame since last time
    end
    dt = frame_elapsed/fps;          % real elapsed time, however many polls returned NaN
    if first_frame % don't count the frames QTM ran before MATLAB joined
        dt = 1/fps;      
        first_frame = 0;
        t_run = tic; %Track total run time
    end
    A = [1, dt; 0, 1];
    B = [dt^2/2; dt];
    noise_q2 = 2.9;
    Q = B*B'*noise_q2;

    % Treadmill speed estimate, advanced by the same dt
    del_v = V_treadmill_target - V_treadmill;
    if del_v > 0
        del_v = min(del_v,  a_treadmill_target*dt);
    else
        del_v = max(del_v, -a_treadmill_target*dt);
    end
    V_treadmill = min(max(V_treadmill + del_v, min_treadmill_V), max_treadmill_V);
    V_treadmill_inst_log(i) = V_treadmill;
    t_lapsed_treadmill_log(i) = dt;
    dt_log(i) = dt;
    t_total = t_total + dt;

    %%%%%%%%%%%%%%%%%Update the Kalman estimate%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %Update the state predictions

    %To troubleshoot any potential NaNs:
    X_prev = X_current;
    X_current = A*X_current + B*u_current;
    X(i,:) = X_current;
    pos_inst_log(i) = X_current(1);
    V_human_inst_log(i) = X_current(2);
    
    %Update the Kalman variables
    P = A*P*A' + Q;

    %Update the average during a step:
    pos_average = (pos_average*n_ts + X_current(1))/(n_ts + 1);
    u_average = (u_average*n_ts + X_current(2))/(n_ts + 1);
    V_treadmill_average = (V_treadmill_average*n_ts + V_treadmill)/(n_ts + 1);
    n_ts = n_ts +1;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%Check if there is a heel strike%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    step_flag = 0; 

    if i <3
        i = i + frame_elapsed;
        continue; %Don't do step computation at 1st frame
    end

    %Step on right belt?
    if (Fz(i-1,1) - Fz_thres < 0) && (Fz(i-2,1) - Fz_thres < 0) && (Fz(i,1) - Fz_thres > 0)  
        step_flag = -1;
        pos_prev_step = pos_current_step;
        pos_current_step = COP_x_frame(1);  

        %Test calculating COP using torques and forces directly
        pos_prev_step_test = pos_current_step_test;
        temp = My_frame./Fz_frame;
        pos_current_step_test = temp(1);
    end

    %Step on left belt?
    if (Fz(i-1,2) - Fz_thres < 0) && (Fz(i-2,2) - Fz_thres < 0) && (Fz(i,2) - Fz_thres > 0) 
        step_flag = 1;
        pos_prev_step = pos_current_step;
        pos_current_step = COP_x_frame(2);

        %Test calculating COP using torques and forces directly
        pos_prev_step_test = pos_current_step_test;
        temp = My_frame./Fz_frame;
        pos_current_step_test = temp(2);
    end


    %Note, we update the position of "current" step at each frame to reflect the leg moving with the belt 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%Update measurements and predictions if there is a heel strike%%%%%
    if step_flag ~= 0
        step_length = pos_current_step - pos_prev_step;
        step_length = step_length + step_length_adjustment; %For some reason, our estimated step lengths using force plate COP is consistently smaller than what we expect
        step_length_test = pos_current_step_test - pos_prev_step_test;
        t_lapsed_step = (label - label_last_step)/fps;   % NaN on the first strike, so it gets rejected
        label_last_step = label;
        step_count = step_count + 1;
        n_ts = 0; %Reset the time_since_step count
    end

    pos_current_step = pos_current_step - dt*V_treadmill; %This is a minus sign, because treadmill velocity is in -x direction
    pos_prev_step = pos_prev_step - dt*V_treadmill; 

    pos_current_step_test = pos_current_step_test - dt*V_treadmill;
    pos_prev_step_test = pos_prev_step_test - dt*V_treadmill;

    loop_success_count = loop_success_count + 1; %We reached this part of the code, increment the count

    %Check if the step is strange
    if (step_flag) ~= 0 && (prev_valid_step ~= step_flag) && (t_lapsed_step>0.3) && (t_lapsed_step < 1.2) && (step_length>0.1 +step_length_adjustment)
        %So we didn't have someone stepping on the same belt with both legs
        step_ok = 1;
        prev_valid_step = step_flag; %If we accept a step, then this is the step that next step check needs to compare to
    else
        step_ok = 0;
    end

    %Update the Kalman filter if step is ok
    if step_ok
        %Update Kalman gain
        K = P/(P+R);

        %Update measured output
        pos_mea = (pos_prev_step + pos_current_step)/2;
        V_human_mea = step_length/t_lapsed_step - V_treadmill_average; %Plus sign here, because human speed is in opposite direction of treadmill speed

        y_mea = [pos_mea;V_human_mea];

        %Check if the measurement makes no sense
        if ((abs(pos_mea)<0.8) && (abs(V_human_mea)<1.5)) || ((i>10*fps)&&(abs(pos_mea)<0.8)&&(abs(V_human_mea)<0.8)) 
            mea_ok = 1;
        else
            mea_ok = 0;
        end

        %And some sanity checks for measurements being ok
        if step_length < 0.1 + step_length_adjustment  %So the original measured step length is <0.1 m
            mea_ok = 0;
        end

        if step_length > 1.2 + step_length_adjustment  %If the original measured step length is >1...thats either a super tall person, or a fluke
            mea_ok = 0;
        end

        %If the step happened extremely quickly or extremely slowly, discard it
        if t_lapsed_step < 0.3 || t_lapsed_step > 1.2
            mea_ok = 0;
        end
        
        if mea_ok
            %Update state with measurement:
            y_est = [pos_average; u_average]; 
            correction = K*(y_mea - y_est); 
            X_current = X_current + correction;            

            %We should already update the pos_average and u_average here, as opposed to waiting until the beginning of next loop
            %This way, the good measurement helps with state estimation immediately, and we can use mea_ok as a check for whether we should update treadmill speed
            %This is ok because at this point, n_ts = 0. So even if we set those two variables, they will be re-initialized to X_current at the beginining of next loop:
            pos_average = y_est(1) + correction(1); 
            u_average = y_est(2) + correction(2); 

            if isnan(X_current)
                disp(['Warning, NaN encountered. The previous value that wasnt Nan is: ',num2str(y_est)]);
                break;
            end
            P = (eye(2) - K)*P;

            tracked_steps = tracked_steps + 1;
    
            %log the averages
            pos_estimate_average(step_count) = pos_average;
            V_human_estimate_average(step_count) = u_average;
            pos_mea_log(step_count) = pos_mea;
            V_human_mea_log(step_count) = V_human_mea;
            step_length_log(step_count) = step_length;
            step_length_test_log(step_count) = step_length_test;
            pos_prev_step_log(step_count) = pos_prev_step;
            pos_current_step_log(step_count) = pos_current_step;
            t_lapsed_step_log(step_count) =  t_lapsed_step;
        end
    else
        y_mea = [-100,-100]; %Basically placeholder
    end


    %Update the speed of the treadmill if step is ok and we can trust the measurement
    if step_ok && mea_ok && tracked_steps > steps_wait_time %Give a few steps for the participant to move normally
        % del_v_target = G_p*(X_current(1)-pos_offset) + G_v*X_current(2);
        del_v_target = G_p*(pos_average-pos_offset) + G_v*u_average;
        a_treadmill_target = abs(del_v_target/del_t_target);

        %To prevent any potential issues locking treadmill at low acceleration:
        if isnan(del_v_target)
            a_treadmill_target = 0.25;
        end
        
        V_treadmill_target = V_treadmill + del_v_target;
        
        %Clip the speed if it exceeds min/max speed
        V_treadmill_target = max(V_treadmill_target, min_treadmill_V);
        V_treadmill_target = min(V_treadmill_target, max_treadmill_V);
        a_treadmill_target = max(a_treadmill_target, a_treadmill_target_min);

        %Send the command to treadmill
        tm_set(remote, [V_treadmill_target,V_treadmill_target], [a_treadmill_target,a_treadmill_target]);

        %Also echo the speed setting to exoskeleton if needed
        if send_to_exo
            write(udp_exo, typecast(single(V_treadmill_target), "uint8"), "uint8", exo_ip, exo_port); 
        end

        %Log the time stamp
        t_treadmill_prev_update = datetime;
        V_treadmill_average_log(step_count) = V_treadmill_target; %For now, we record not the average, rather just the target speed

        %Keep track of the previous target
        V_treadmill_target_prev = V_treadmill_target;
        a_treadmill_target_prev = a_treadmill_target;
        
    else
        V_treadmill_target = V_treadmill_target_prev;
        a_treadmill_target = a_treadmill_target_prev;
        del_v_target = 0;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    i = i + frame_elapsed;

    %Log the data, display them as needed
    if step_ok && mea_ok
        disp(['Frame ', num2str(i)]); %This shows the actual amount of frames we captured 
        disp(['Current estimated human position is ',num2str(pos_average), ' with speed ', num2str(u_average)])
        disp(['Treadmill speed before update is: ',num2str(V_treadmill)])
        disp(['Computed del_v_target is: ', num2str(del_v_target)]);
        disp(['Sending speed command: ',num2str(V_treadmill_target)]);
        disp(['Measured position is ',num2str(pos_mea), ' with velocity ', num2str(V_human_mea)])
    elseif step_ok
        disp(['Errorneous step with step length ',num2str(step_length),' and step time ', num2str(t_lapsed_step)])
    end
    
end

%Disconnect from QTM and treadmill
QCM('disconnect');
tm_set(remote, [0,0], [0.4,0.4]);
clear remote % Disconnection for new tcpipclient is just this
if send_to_exo
     clear udp_exo; %Clean up the UDP port if it has been created
end

disp(['Total run time is ', num2str(toc(t_run)), '. Total time calculated from t_total is ',num2str(t_total)])

%Clean up the data but store them in separate variables, so original variables can be used to debug
V_treadmill_average_log_cleared = V_treadmill_average_log(~isnan(V_treadmill_average_log));
pos_estimate_average_cleared = pos_estimate_average(~isnan(pos_estimate_average));
V_human_estimate_average_cleared = V_human_estimate_average(~isnan(V_human_estimate_average));
pos_mea_log_cleared = pos_mea_log(~isnan(pos_mea_log));
V_human_mea_log_cleared = V_human_mea_log(~isnan(V_human_mea_log));
step_length_log_cleared = step_length_log(~isnan(step_length_log));
step_length_test_log_cleared = step_length_test_log(~isnan(step_length_test_log));
t_lapsed_step_log_cleared = t_lapsed_step_log(~isnan(t_lapsed_step_log));
V_human_inst_log_cleared = V_human_inst_log(~isnan(V_human_inst_log));

V_treadmill_target_cleared = V_treadmill_target(~isnan(V_treadmill_target));
a_treadmill_target_cleared = a_treadmill_target(~isnan(a_treadmill_target));

%Plot the results
mean_tread_spd = mean(V_treadmill_average_log_cleared(end-20:end));
plot(V_treadmill_average_log_cleared./(mean_tread_spd))
yline(1)
disp(['The average speed of last 20 steps is ',num2str(mean_tread_spd),' m/s'])
