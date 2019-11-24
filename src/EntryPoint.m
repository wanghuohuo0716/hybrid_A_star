ObstList = []; % Obstacle point list
for i = -25:25
    ObstList(end+1,:) = [i,15];
end
for i = -10:10
    ObstList(end+1,:) = [i, 0];
end
for i = -25:-10
    ObstList(end+1,:) = [i, 5];
end
for i = 10:25
    ObstList(end+1,:) = [i, 5];
end
for i = 0:5
    ObstList(end+1,:) = [10, i];
end
for i = 0:5
    ObstList(end+1,:) = [-10, i];
end

ObstLine = []; % Park lot line for collision check
tLine = [-25, 15 , 25, 15];
ObstLine(end+1,:) = tLine;
tLine = [-25, 5, -10, 5];
ObstLine(end+1,:) = tLine;
tLine = [-10, 5, -10, 0];
ObstLine(end+1,:) = tLine;
tLine = [-10, 0, 10, 0];
ObstLine(end+1,:) = tLine;
tLine = [10, 0, 10, 5];
ObstLine(end+1,:) = tLine;
tLine = [10, 5, 25, 5];
ObstLine(end+1,:) = tLine;
tLine = [-25, 5, -25, 15];
ObstLine(end+1,:) = tLine;
tLine = [25, 5, 25, 15];
ObstLine(end+1,:) = tLine;

Vehicle.WB = 3.7;  % [m] wheel base: rear to front steer
Vehicle.W = 2.6; % [m] width of vehicle
Vehicle.LF = 4.5; % [m] distance from rear to vehicle front end of vehicle
Vehicle.LB = 1.0; % [m] distance from rear to vehicle back end of vehicle
Vehicle.MAX_STEER = 0.6; % [rad] maximum steering angle 
Vehicle.MIN_CIRCLE = Vehicle.WB/tan(Vehicle.MAX_STEER); % [m] mininum steering circle radius

% ObstList and ObstLine
Configure.ObstList = ObstList;
Configure.ObstLine = ObstLine;

% Motion resolution define
Configure.MOTION_RESOLUTION = 0.1; % [m] path interporate resolution
Configure.N_STEER = 20.0; % number of steer command
Configure.EXTEND_AREA = 0; % [m] map extend length
Configure.XY_GRID_RESOLUTION = 2.0; % [m]
Configure.YAW_GRID_RESOLUTION = deg2rad(15.0); % [rad]
% Grid bound
Configure.MINX = min(ObstList(:,1))-Configure.EXTEND_AREA;
Configure.MAXX = max(ObstList(:,1))+Configure.EXTEND_AREA;
Configure.MINY = min(ObstList(:,2))-Configure.EXTEND_AREA;
Configure.MAXY = max(ObstList(:,2))+Configure.EXTEND_AREA;
Configure.MINYAW = -pi;
Configure.MAXYAW = pi;
% Cost related define
Configure.SB_COST = 0; % switch back penalty cost
Configure.BACK_COST = 1.5; % backward penalty cost
Configure.STEER_CHANGE_COST = 1.5; % steer angle change penalty cost
Configure.STEER_COST = 1.5; % steer angle change penalty cost
Configure.H_COST = 10; % Heuristic cost

% Start = [-15, 10, 0];
% End = [-4, 3, pi];

Start = [22, 13, pi];
End = [7, 2, pi/2];

ObstMap = GridAStar(Configure.ObstList,End,Configure.XY_GRID_RESOLUTION);
Configure.ObstMap = ObstMap;
cla
[x,y,th,D,delta] = HybridAStar(Start,End,Vehicle,Configure);
% GridAStar(ObstList,End,2);
if isempty(x)
    disp("Failed to find path!")
else
    VehicleAnimation(x,y,th,Configure,Vehicle)
end