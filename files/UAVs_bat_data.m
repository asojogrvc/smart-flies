%% _______Don't touch___________
bat ={};UAVs_model={}; 
%% ________________________________

%% Battery template
% bat1.name = "battery 1";
% bat1.capacity = 2200; %capacity [mAh]
% bat1.cells = 4; 
% bat1.volt = 3.9; % voltage/cell [V/cell]
% bat=[bat; bat1];  % Not eliminate

%% UAV template
% %% --- MODEL 1 UAV ---
% UAV1.name = "model 1"; 
% % Mass and geometry:
% UAV1.m      = 7.2;     % total mass                  [kg]
% UAV1.nr     = 4;       % number of rotors            [-]
% UAV1.b      = 2;       % number of rotor blades      [-]
% UAV1.R      = 0.2667;  % rotor radius                [m]
% UAV1.c      = 0.0247;  % blade chord (x=0.7*R)       [m]
% % Aerodynamics:
% UAV1.Cl     = 1.01;    % section lift coefficient    [-]
% UAV1.Cd     = 0.01;    % section drag coefficient    [-]
% UAV1.kappa  = 1.15;    % induced power factor        [-]
% UAV1.eta    = 0.72;    % energy efficiency           [-]
% UAV1.K_mu   = 4.65;    % P0 numerical constant       [-]
% UAV1.f      = 0.35;    % equivalent flat-plate area  [m^2]
% UAV1.camera = 1;       % If need camera actions. 1 == yes, 0 == no 
%UAV=[UAV; UAV1];


%% ---BATTERY 1---

bat1.name = "battery";
bat1.capacity = 5935; %capacity [mAh]
bat1.cells = 1; 
bat1.volt = 52.8; % voltage/cell [V/cell]
bat=[bat; bat1];

%% ---BATTERY 2---
bat2.name = "double battery";
bat2.capacity = 5935*2; %capacity [mAh]
bat2.cells = 1; 
bat2.volt = 52.8; % voltage/cell [V/cell]
bat=[bat;bat2];

%% ---BATTERY 3---
bat3.name = "battery 3";
bat3.capacity = 2200; %capacity [mAh]
bat3.cells = 4; 
bat3.volt = 3.9; % voltage/cell [V/cell]
bat=[bat;bat3];

%% --- MODEL 1 UAV ---
UAV1.name = "M300 aprox"; 
% Mass and geometry:
UAV1.m      = 7.2;     % total mass                  [kg]
UAV1.nr     = 4;       % number of rotors            [-]
UAV1.b      = 2;       % number of rotor blades      [-]
UAV1.R      = 0.2667;  % rotor radius                [m]
UAV1.c      = 0.0247;  % blade chord (x=0.7*R)       [m]

% Aerodynamics:
UAV1.Cl     = 1.01;    % section lift coefficient    [-]
UAV1.Cd     = 0.01;    % section drag coefficient    [-]
UAV1.kappa  = 1.15;    % induced power factor        [-]
UAV1.eta    = 0.72;    % energy efficiency           [-]

UAV1.K_mu   = 4.65;    % P0 numerical constant       [-]
UAV1.f      = 0.35;    % equivalent flat-plate area  [m^2]
UAV1.camera = 1;       % If need camera actions. 1 == yes, 0 == no 
UAVs_model=[UAVs_model; UAV1];

%% --- MODEL 2 UAV ---
UAV2.name = "model 2"; 
% Mass and geometry:
UAV2.m      = 7.2;     % total mass                  [kg]
UAV2.nr     = 4;       % number of rotors            [-]
UAV2.b      = 2;       % number of rotor blades      [-]
UAV2.R      = 0.2667;  % rotor radius                [m]
UAV2.c      = 0.0247;  % blade chord (x=0.7*R)       [m]

% Aerodynamics:
UAV2.Cl     = 1.01;    % section lift coefficient    [-]
UAV2.Cd     = 0.01;    % section drag coefficient    [-]
UAV2.kappa  = 1.15;    % induced power factor        [-]
UAV2.eta    = 0.72;    % energy eelciency           [-]

UAV2.K_mu   = 4.65;    % P0 numerical constant       [-]
UAV2.f      = 0.35;    % equivalent flat-plate area  [m^2]
UAV2.camera = 1;       % If need camera actions. 1 == yes, 0 == no 
UAVs_model=[UAVs_model; UAV2];