%This code is based on, and modified from the following paper:
% S Song, H Choi and SH Collins, Using force data to self-pace an instrumentedtreadmill and measure self-selected walking speed
% Original code github link: https://github.com/smsong/self-paced-treadmill/tree/JNER2020


%pause(5) %For testing on myself

%%%%%%%%%%%%%%Settings that change depending on experiment%%%%%%%%%%%%%%%%%
error_max = 5; %Max amount of errors we allow before disconnecting. For now, pratically ignore all errors and go ahead
Fz_thres = 30; %Threshold for registering a step as foot strike. For nowc setting this as a static variable. 
time = 45; % Total time of experiment, in seconds
mass = 70; %In kg

%Parameters for the controller for treadmill feedback
G_p = 0.15;
G_v = 0.3;
pos_offset = 0.1; %Position offset of human on treadmill, probably needs to be experimentally updated
del_t_target = 0.5; %How fast do we want the treadmill speed to achieve target? In seconds
max_treadmill_V = 3;
min_treadmill_V = 0.7;
a_treadmill_target_min = 0.01;
QTM_filter_max = 10; %How many frames of GRF we receive and filter in one go (to filt out spike noises)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Define variables
fps = 100;
force_per_frame = 10;
num_frames = fps*time;
num_forces = num_frames*force_per_frame;
ip = '127.0.0.1:22222';

Fx = zeros(num_frames,2); 
Fy = zeros(num_frames,2); 
Fz = zeros(num_frames,2); 
My = zeros(num_frames,2); 
COP_x = zeros(num_frames,2); %In our QTM frame, x is the direction of treadmill motion
frame_elapsed_count = zeros(num_frames,1);

COP_x_queue = zeros(QTM_filter_max,2);
Fx_queue = zeros(QTM_filter_max,2);
Fz_queue = zeros(QTM_filter_max,2);
My_queue = zeros(QTM_filter_max,2);

QTM_filter_idx = 0;
filter_padded = 0; %Whether we already have a full queue to be filtered

step_count = 0;

step_flag = 0; %Here we define 1 == left foot strike, -1 == right foot strike, 0 == no strike
step_prev = 0;
step_ok = 0; %Flag to store if the step should be discarded

%We will only update the treadmill speed once per step
V_treadmill = 0; %Vbar_tm in paper
V_treadmill_target = 1; %V_tm_tgt in paper
V_treadmill_target_prev = 1;

a_treadmill_target = 0.25; %a_tm_tgt in paper
a_treadmill_target_prev = 0.25;

t_treadmill_prev_update = 0; %The last time treadmill speed was updated

%Create the butterworth filter
fc= 25;
fs = 100;
[filter_b,filter_a] = butter(3,fc/(fs/2));

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%Define data logging variables%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pos_estimate_average = nan(num_frames,1);
pos_inst_log = nan(num_frames,1);
V_human_estimate_average = nan(num_frames,1);
V_human_inst_log = nan(num_frames,1);
pos_estimate_KF = nan(num_frames,1);
V_human_estimate_KF = nan(num_frames,1);
V_treadmill_average_log = nan(num_frames,1);
V_treadmill_KF_log = nan(num_frames,1);
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

QCM('connect', ip,'force'); % Connects to QTM and keeps the connection alive.
error_count = 0; %Track how many exceptions we run into
dt = 0; %Initialize dt


t_step = tic; %Initialize the step time
t_current_step = 0;
i = 1;
frame_init = 1;

while i <= num_frames 
    %Note that with this code, I don't have a fixed dt. Instead, dt changes depending on how fast the code can run
    %minimum dt is 1/fps of QTM

    % disp(['Frame ', num2str(i)]); %This shows the actual amount of frames we captured
    t_frame_start = datetime; 

    %Update dt and the constants for Kalman filter
    if KF_initialized
        %Track dt, then re-initialize the timer
        dt = toc(dt_start);
        A = [1, dt;0,1];
        B = [dt^2/2;dt];
        noise_q2 = 2.9; % square of q: from experiment (mocap as reference data)
        Q = B*B'*noise_q2;
        dt_start = tic;
        
    else %We initialize the timer for dt and skip to the next loop
        dt_start = tic;
        KF_initialized = 1;

        [Fx_frame, Fz_frame,My_frame,COP_x_frame,dataRT,frames,label] = get_frame_self_paced(QCM);
        if isnan(Fx_frame)
            Fx_frame = [0,0];
            COP_x_frame = [0;0];
        end
        X_current = [mean(COP_x_frame);0];
        u_current = Fx_frame(1) + Fx_frame(2);
        P = P0;
        continue;
    end

    %%%%%%%%%%%Try updating estimation for current treadmill speed%%%%%%%%%%%%%%%%%%%%%%%%
    % t_lapsed_treadmill = second(t_frame_start) - second(t_treadmill_prev_update);   
    % if t_lapsed_treadmill <0 
    %     t_lapsed_treadmill = 0;
    % end

    t_lapsed_treadmill = dt;
    del_v = V_treadmill_target - V_treadmill;
    %How much has the treadmill speed change since we last gave it a command?
    if del_v > 0
        del_v = min(del_v, a_treadmill_target*t_lapsed_treadmill);
    else
        del_v = max(del_v, -a_treadmill_target*t_lapsed_treadmill);
    end
    V_treadmill = V_treadmill + del_v;

    %Because we are just guessing the speed, clip it in case something went wrong
    if V_treadmill > max_treadmill_V
        V_treadmill = max_treadmill_V;
    end

    if V_treadmill < min_treadmill_V
        V_treadmill = min_treadmill_V;
    end

    %Debugging
    if i >0
        V_treadmill_inst_log(i) = V_treadmill;
        t_lapsed_treadmill_log(i) = t_lapsed_treadmill;
    end

    V_treadmill = 1; %For monitoring purpose, we don't need to real time adjust treadmill speed
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%Try reading the data from QTM%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    try
        [Fx_frame, Fz_frame,My_frame,COP_x_frame,dataRT,frames,label] = get_frame_self_paced(QCM);

        if isnan(Fx_frame)
            continue;
        end

        %Store the data in buffer and filter them out
        if filter_padded
            %If the queue if full, throw out the oldest value and add in the latest
            COP_x_queue(1:end-1,:) = COP_x_queue(2:end,:);
            COP_x_queue(end,:) = COP_x_frame;
            Fx_queue(1:end-1,:) = Fx_queue(2:end,:);
            Fx_queue(end,:) = Fx_frame;
            Fz_queue(1:end-1,:) = Fz_queue(2:end,:);
            Fz_queue(end,:) = Fz_frame;
            My_queue(1:end-1,:) = My_queue(2:end,:);
            My_queue(end,:) = My_frame;

            %Filter the force data
            temp = filter(filter_b,filter_a,Fx_queue(:,1));
            Fx_frame(1) = temp(end);
            temp = filter(filter_b,filter_a,Fx_queue(:,2));
            Fx_frame(2) = temp(end);

            temp = filter(filter_b,filter_a,Fz_queue(:,1));
            Fz_frame(1) = temp(end);
            temp = filter(filter_b,filter_a,Fz_queue(:,2));
            Fz_frame(2) = temp(end);

            temp = filter(filter_b,filter_a,My_queue(:,1));
            My_frame(1) = temp(end);
            temp = filter(filter_b,filter_a,My_queue(:,2));
            My_frame(2) = temp(end);

            %I think applying the filter to COP is prob not necessary, so not doing that for now

        else
            QTM_filter_idx = QTM_filter_idx +1;
            COP_x_queue(QTM_filter_idx,:) = COP_x_frame;
            Fx_queue(QTM_filter_idx,:) = Fx_frame;
            Fz_queue(QTM_filter_idx,:) = Fz_frame;
            My_queue(QTM_filter_idx,:) = My_frame;
            if QTM_filter_idx >= QTM_filter_max
                filter_padded = 1;
            end
        end


        %disp(dataRT);
        label = floor(label/force_per_frame);
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
        if (i>frame_init && frame_elapsed <= 0 && frame_elaspsed_count(i-1) <=0) 
            %That means we have hit the end
            disp('Max time reached, ending experiment, the last frame captured is: ')
            disp(label);
            break;
        end

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

        %The following section is legacy and not currently in use
        Fx(i,:) = Fx_frame;
        Fz(i,:) = Fz_frame;
        My(i,:) = My_frame;
        COP_x(i,:) = COP_x_frame;
        frame_elapsed_count(i) = frame_elapsed;
        dt_log(i) = dt;

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


    %%%%%%%%%%%%%%%%%Update the Kalman estimate%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %Update the state predictions
    u_current = Fx_frame(1) + Fx_frame(2);
    u_current = u_current/mass/g; %The GRF values reported are in the lab reference frame, a positive X-force means acceleration
    
    %To troubleshoot any potential NaNs:
    X_prev = X_current;
    X_current = A*X_current + B*u_current;
    if isnan(X_current)
        disp(['Warning, NaN encountered. The previous value that wasnt Nan is: ',num2str(X_prev)]);
        break;
    end
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
        i = i+1;
        continue; %Don't do step computation at 1st frame and 2nd
    end

    %Step on right belt?
    if (Fz(i-1,1) - Fz_thres < 0) && (Fz(i-2,1) - Fz_thres < 0) && (Fz(i,1) - Fz_thres > 0)  
        step_prev = step_flag; 
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
        step_prev = step_flag; 
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
        step_length_test = pos_current_step_test - pos_prev_step_test;
        t_lapsed_step = toc(t_step);
        t_step = tic;
        step_count = step_count + 1;
        n_ts = 0; %Reset the time_since_step count
    end

    pos_current_step = pos_current_step - dt*V_treadmill; %This is a minus sign, because treadmill velocity is in -x direction
    pos_prev_step = pos_prev_step - dt*V_treadmill;

    pos_current_step_test = pos_current_step_test - dt*V_treadmill;
    pos_prev_step_test = pos_prev_step_test - dt*V_treadmill;

    %Check if the step is strange
    if (step_flag) ~= 0 && (step_prev ~= step_flag) && (t_lapsed_step>0.3) && (t_lapsed_step < 1)
        %So we didn't have someone stepping on the same belt with both legs
        step_ok = 1;
    else
        step_ok = 0;
    end

    %Update the Kalman filter if step is ok
    if step_ok
        %Update Kalman gain
        K = P/(P+R);

        %Update measured output

        step_length = step_length + 0.3; %For some reason, our estimated step lengths using force plate COP is consistently smaller than what we expect


        pos_mea = (pos_prev_step + pos_current_step)/2;
        V_human_mea = step_length/t_lapsed_step - V_treadmill_average; %Plus sign here, because human speed is in opposite direction of treadmill speed

        y_mea = [pos_mea;V_human_mea];

        %Check if the measurement makes no sense
        if ((abs(pos_mea)<0.6) && (abs(V_human_mea)<1.5)) || ((i>10*fps)&&(abs(pos_mea)<0.6)&&(abs(V_human_mea)<0.8)) 
            mea_ok = 1;
        else
            mea_ok = 0;
        end

        
        if mea_ok
            %Update state with measurement:
            y_est = [pos_average; u_average]; 
            X_current = X_current + K*(y_mea - y_est);
            if isnan(X_current)
                disp(['Warning, NaN encountered. The previous value that wasnt Nan is: ',num2str(y_est)]);
                break;
            end
            P = (eye(2) - K)*P;
    
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


    %Update the speed of the treadmill if step is ok
    if step_ok
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

        %Log the time stamp
        t_treadmill_prev_update = datetime;
        V_treadmill_average_log(step_count) = V_treadmill_target; %For now, we record not the average, rather just the target speed

        %Keep track of the previous target
        V_treadmill_target_prev = V_treadmill_target;
        a_treadmill_target_prev = a_treadmill_target;
        
    else
        V_treadmill_target = V_treadmill_target_prev;
        a_treadmill_target = a_treadmill_target_prev;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    i = i + frame_elapsed;

    %Log the data, display them as needed
    if step_ok
        disp(['Frame ', num2str(i)]); %This shows the actual amount of frames we captured 
        disp(['Current estimated human position is ',num2str(X_current(1)), ' with speed ', num2str(X_current(2))])
        disp(['Treadmill speed before update is: ',num2str(V_treadmill)])
        disp(['Computed del_v_target is: ', num2str(del_v_target)]);
        disp(['Sending speed command: ',num2str(V_treadmill_target)]);
        disp(['Measured position is ',num2str(pos_mea), ' with velocity ', num2str(V_human_mea)])
    end
    
end

%Disconnect from QTM and treadmill
QCM('disconnect');

%Clean up the data
V_treadmill_average_log = V_treadmill_average_log(~isnan(V_treadmill_average_log));
pos_estimate_average = pos_estimate_average(~isnan(pos_estimate_average));
V_human_estimate_average = V_human_estimate_average(~isnan(V_human_estimate_average));
pos_mea_log = pos_mea_log(~isnan(pos_mea_log));
V_human_mea_log = V_human_mea_log(~isnan(V_human_mea_log));
step_length_log = step_length_log(~isnan(step_length_log));
step_length_test_log = step_length_test_log(~isnan(step_length_test_log));
t_lapsed_step_log = t_lapsed_step_log(~isnan(t_lapsed_step_log));
% V_human_inst_log = V_human_inst_log(~isnan(V_human_inst_log));

% V_treadmill_target = V_treadmill_target(~isnan(V_treadmill_target));
% a_treadmill_target = a_treadmill_target(~isnan(a_treadmill_target));

%Plot the results
mean_tread_spd = mean(V_treadmill_average_log(end-20:end));
plot(V_treadmill_average_log./(mean_tread_spd))
yline(1)
disp(['The average speed of last 20 steps is ',num2str(mean_tread_spd),' m/s'])
