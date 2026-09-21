function [V_treadmill_target] = ComputeMachineAction(current_cadence)
%This function computes the machine action (treadmill speed) given the human action (cadence)

% Placeholder equation at the moment
%V_treadmill_target = 0.112083*exp(1.26518*current_cadence); %Pilot subject 1
%V_treadmill_target = 0.123517*exp(1.30907*current_cadence); %Pilot subject 2
%V_treadmill_target = 0.171413*exp(1.17689*current_cadence); %Pilot subject 3
V_treadmill_target = 0.12598*exp(1.27681*current_cadence); %Pilot subject 4

if V_treadmill_target < 0.4
    V_treadmill_target = 0.4;
end

if V_treadmill_target > 1.8
    V_treadmill_target = 1.8;
end

end

