%% Creates parameters used to define the scenario

speedLimit = 120/3.6;           % Speed limit on road in m/s

horizon = 150;                  % Prediction horizon in m
numSamples = 181;               % Number of samples
numSteps = 1;                   % Number of steps in MPC to perform

egoVehiclePosition = 1;         % Position of ego vehicle wrt adjacent vehicles

posLeading = 75;                % Position of leading vehicle wrt ego in m
speedLeading = 100/3.6;         % Speed of leading vehicle in m/s

isAdjVeh1 = false;
posAdjVeh1 = -70;               % Position of 1st adjacent vehicle wrt ego in m
speedAdjVeh1 = 75/3.6;          % Speed of 1st adjacent vehicle in m/s

isAdjVeh2 = false;
posAdjVeh2 = -50;               % Position of 2nd adjacent vehicle wrt ego in m
speedAdjVeh2 = 75/3.6;          % Speed of 2nd adjacent vehicle in m/s