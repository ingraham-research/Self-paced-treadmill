%This code is to prescribe several fixed velocities to a participant and record their response step frequencies, to create a V-f curve


%pause(5) %For testing on myself

%%%%%%%%%%%%%%Settings that change depending on experiment%%%%%%%%%%%%%%%%%
error_max = 5; %Max amount of errors we allow before disconnecting. For now, pratically ignore all errors and go ahead
Fz_thres = 30; %Threshold for registering a step as foot strike. For nowc setting this as a static variable. 
time = 30; % Total time of each trial, in seconds
trials_total = 10;
target_v = linspace(0.8,1.8,trials_total);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Define variables
fps = 100;
force_per_frame = 10;
num_frames = fps*time;
num_forces = num_frames*force_per_frame;
ip = '127.0.0.1:22222';

%Create the butterworth filter
fc= 25;
fs = 100;
[filter_b,filter_a] = butter(3,fc/(fs/2));

%%%%%%%%%%%%%%%%%%%Define additional variables and constants%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = 9.81;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%Remote connection to Bertec Treadmill control panel%%%%%%%%%%%
% Open the connection (do it only once at start of the control session)
remote = tcpip('localhost', 4000);
fclose(remote); %In case TCP/IP hasn't properly closed last time
fopen(remote);
tm_set(remote, [0,0], [0.5,0.5]);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

QCM('connect', ip,'force'); % Connects to QTM and keeps the connection alive.
error_count = 0; %Track how many exceptions we run into


for trial = 1:trials_total

    %%%%%%%%%%%%%%%%%%%Define data logging variables%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    t_lapsed_step_log = nan(num_frames,1);
    step_length_log = nan(num_frames,1);
    pos_prev_step_log = nan(num_frames,1);
    pos_current_step_log = nan(num_frames,1);
    dt_log = nan(num_frames,1);

    Fx = zeros(num_frames,2); 
    Fy = zeros(num_frames,2); 
    Fz = zeros(num_frames,2); 
    My = zeros(num_frames,2); 
    COP_x = zeros(num_frames,2); %In our QTM frame, x is the direction of treadmill motion
    frame_elapsed_count = zeros(num_frames,1);
    
    QTM_filter_max = 10;
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

    pos_current_step = 0; 
    pos_prev_step = 0; 
    
    pos_current_step_test = 0;
    pos_prev_step_test = 0;
    
    t_current_step = 0;
    t_prev_step = 0;

    dt = 0; %Initialize dt
    t_step = tic; %Initialize the step time
    t_current_step = 0;
    i = 1;
    frame_init = 1;
    initialized = 0;

    V_treadmill = target_v(trial);
    disp(['Executing trial ',num2str(trial),' with target speed: ',num2str(V_treadmill), ' m/s'])
    tm_set(remote, [V_treadmill,V_treadmill], [0.5,0.5]);


    while i <= num_frames 
        %Note that with this code, I don't have a fixed dt. Instead, dt changes depending on how fast the code can run
        %minimum dt is 1/fps of QTM
    
        % disp(['Frame ', num2str(i)]); %This shows the actual amount of frames we captured
        t_frame_start = datetime; 
    
        %Update dt 
        if initialized
            %Track dt, then re-initialize the timer
            dt = toc(dt_start);
            dt_start = tic;
            
        else %We initialize the timer for dt and skip to the next loop
            dt_start = tic;
            initialized = 1;
    
            [Fx_frame, Fz_frame,My_frame,COP_x_frame,dataRT,frames,label] = get_frame_self_paced(QCM);
            if isnan(Fx_frame)
                Fx_frame = [0,0];
                COP_x_frame = [0;0];
            end
            continue;
        end
    
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
            label = label - (trial-1)*fps*time; %In this code we run multiple trials so we need to reset the labels
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
            temp = -My_frame./Fz_frame;
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
            temp = -My_frame./Fz_frame;
            pos_current_step_test = temp(2);
        end
    
    
        %Note, we update the position of "current" step at each frame to reflect the leg moving with the belt 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
        %%%%%%Update measurements and predictions if there is a heel strike%%%%%
        if step_flag ~= 0
            step_length = pos_current_step - pos_prev_step;
            step_length_test = pos_current_step_test - pos_prev_step_test;
            t_lapsed_step = toc(t_step);
            t_step = tic; %Even if we don't think the step was ok, we still need to reset the timer
            step_count = step_count + 1;
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
    
        if step_ok
            disp(['Step detected at time ',num2str(double(i)/fps,3), ' s'])
            %log the averages
            step_length_log(step_count) = step_length;
            pos_prev_step_log(step_count) = pos_prev_step;
            pos_current_step_log(step_count) = pos_current_step;
            t_lapsed_step_log(step_count) =  t_lapsed_step;
        end
    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
        i = i + frame_elapsed;
    
        
    end
    
    %Clean up the data
    step_length_log = step_length_log(~isnan(step_length_log));
    t_lapsed_step_log = t_lapsed_step_log(~isnan(t_lapsed_step_log));

    %Write the data into a structure
    results.step_length_log = step_length_log;
    results.t_lapsed_step_log = t_lapsed_step_log;
    results.treadmill_speed = V_treadmill;
    results.steady_state_cadence = 1/mean(t_lapsed_step_log(end-20:end)); %Convert the result from time lapsed to cadence (frequency)

    all_results{trial} = results;
end


%Disconnect from QTM and treadmill
QCM('disconnect');
tm_set(remote, [0,0], [0.8,0.8]);
fclose(remote);

SS_cadence = zeros(trials_total,1);
for trial = 1:trials_total
    SS_cadence(trial) =  all_results{trial}.steady_state_cadence;
end
SS_cadence = SS_cadence';
