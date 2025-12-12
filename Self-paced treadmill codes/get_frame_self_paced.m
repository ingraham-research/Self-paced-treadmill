function [Fx_frame, Fz_frame,My_frame,COP_x_frame,dataRT,frames,label] = get_frame_self_paced(QCM)
    %Note, the force plates are in their own reference frame, and the x-direction in lab frame is the y for treadmill
    %In this code, the naming of coordinates are those in the lab frame
    try    
        dataRT = QCM;
        Fx_frame = [0 0];
        Fz_frame = [0 0];
        COP_x_frame = [0 0];
        
        %Sometimes, because of computation overhead we might get more than one frames of data in 1 query
        frames = size(dataRT{2,5}(:,8));
        frames = floor(frames(1)/10); %The amount of frames in this packet, usually 1-2
        label = dataRT{1}; %The internal label of this data group
        if frames <1
            frames = 1;
        end
        %QTM is connected to 6 plates but only the last 2 are Bertec connected to treadmill

        %The X and Y directions for force plate and lab reference frame are flipped - we want the X in lab frame which is Y for force plates
        %Right belt
        Fx_frame(1) = mean(dataRT{2,5}(:,2));
        Fz_frame(1) = mean(dataRT{2,5}(:,3));
        My_frame(1) = mean(dataRT{2,5}(:,4));
        COP_x_frame(1) = mean(dataRT{2,5}(:,8));

        %Left belt
        Fx_frame(2) = mean(dataRT{2,6}(:,2));
        Fz_frame(2) = mean(dataRT{2,6}(:,3));
        My_frame(2) = mean(dataRT{2,6}(:,4));
        COP_x_frame(2) = mean(dataRT{2,6}(:,8));
        COP_x_frame = COP_x_frame/1000; %Convert to meters

        %Because of coordinate system change, we need to flip the sign of x (but not Z - see QTM treadmill reference frame setting)
        Fx_frame = -Fx_frame;

        %I'm not sure if we need to do this for My, but we aren't using My as of 20240917, so I won't change it for now
    catch
    end