%% Creates parameters used to define the scenario

speedLimit = 120/3.6;           % Speed limit on road in m/s

horizon = 150;                  % Prediction horizon in m
numSamples = 181;               % Number of samples
numSteps = 1;                   % Number of steps in MPC to perform

%% Ego vehicle

egoVehiclePosition = 1;         % Position of ego vehicle wrt adjacent vehicles

%% Leading vehicle

posLeading = 075;               % Position of leading vehicle wrt ego in m
speedLeading = 100/3.6;         % Speed of leading vehicle in m/s

%% Adjacent Vehicle 1

isAdjVeh1 = true;
posAdjVeh1 = 0;                 % Position of 1st adjacent vehicle wrt ego in m
speedAdjVeh1 = speedLimit;      % Speed of 1st adjacent vehicle in m/s

%% Adjacent Vehicle 2

isAdjVeh2 = false;
posAdjVeh2 = 50;               % Position of 2nd adjacent vehicle wrt ego in m
speedAdjVeh2 = 75/3.6;          % Speed of 2nd adjacent vehicle in m/s

%% Weights for decision comparison cost
w = ones(1,6);
w(3) = 0; w(5) = 0;