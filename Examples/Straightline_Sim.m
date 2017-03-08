%% User Entry For Simulation
% Enter parameters for the sim here.  Note that EVERYTHING IS IN SI UNITS!
% I know, it's terrible.  But it makes the math so much easier.

%Tires:
%   Just care about effective and loaded radii for now.  I know they
%   change with speed and load, but haven't implemented yet.  Also, for now
%   please include all outboard mass in the wheel mass.

FrontEffectiveRadius = 0.325; %m
FrontLoadedRadius    = 0.318; %m
FrontTireMass        = 4.5; %kg
FrontTireInertia     = 0.0; %kg*m^2
FrontWheelMass       = 5.0; %kg
FrontWheelInertia    = 0.0; %kg*m^2

RearEffectiveRadius  = 0.351; %m
RearLoadedRadius     = 0.345; %m
RearTireMass         = 5.0; %kg
RearTireInertia      = 0.0; %kg*m^2
RearWheelMass        = 5.5; %kg
RearWheelInertia     = 0.0; %kg*m^2

%Engine Parameters:
%   This is where you enter the basics about the engine.  I haven't
%   implemented correction factors yet.

ThrottleVector = [0;1]; %ratio
SpeedVector =    [0,16.6667,33.3334,50.0001,66.6668,83.3335,100.0002,116.6669,133.3336;]; %cycles/sec
TorqueMatrix =   [0,0,0,0,0,0,0,0,0;...
                  0,73.7562149,162.2636728,258.1467522,331.9029671,368.7810745,368.7810745,331.9029671,221.2686447;]; %N*m
EngineInertia =  0; %kg*m^2

%Limiter:
%   2-stage type or single stage can be entered.  Just delete or comment
%   out the soft limit lines to do a single stage.  This models the control
%   as proportional gain only, which is not 100% accurate, but the engine
%   guys can give you an approximation of a prop gain term.

SoftLimit     = [117,117,117,117,117,117]; %cycles/sec
SoftLimitGain = [10,10,10,10,10,10]; %cycles/sec
HardLimit     = [125,125,125,125,125,125]; %cycles/sec
HardLimitGain = [40,40,40,40,40,40]; %cycles/sec

%Gearbox Parameters:
%   This should be familiar and straightforward.

GearRatios = [2.43 1.94 1.65 1.47 1.33 1.21]; %out:in
FinalDrive = 2.5; %out:in
DrivelineEfficiency = 0.9; %ratio
InertiaGearbox = 0; %kg*m^2
InertiaFinal =   0; %kg*m^2

%Aero:
%   Only SCx matters for now

SCx = 1.2;
SCz = 0;
Abal = 0.4;

%Chassis:
%   Ez pz

SprungMass = 1134; %kg

%World:
%   Self explanatory

Temperature = 293.15; %K
Pressure = 1e5; %Pa
Humidity = 0.6; %RH


%% Model Generation

%Powertrain

Driveline = ScanSim.PowerTrain.Driveline(GearRatios,FinalDrive);
Driveline.Efficiency = DrivelineEfficiency;
Driveline.InertiaGearbox = InertiaGearbox;
Driveline.InertiaFinal = InertiaFinal;

Engine = ScanSim.PowerTrain.Engine(SpeedVector,TorqueMatrix,ThrottleVector);
Engine.Inertia = EngineInertia;

PowerTrain = ScanSim.PowerTrain.Model(Engine,Driveline);
PowerTrain.AddSoftLimiter(SoftLimit,SoftLimitGain);
PowerTrain.AddHardLimiter(HardLimit,HardLimitGain);

%Aero

Aero = ScanSim.Aerodynamics.Model(SCz,SCx,Abal);

%Chassis

Chassis = ScanSim.Chassis.Model(SprungMass);

%World

World = ScanSim.World.Model;
World.Temperature = Temperature;
World.Pressure = Pressure;
World.Humidity = Humidity;

%Wheels and tires

TireF = ScanSim.Wheel.Tire(FrontLoadedRadius,FrontEffectiveRadius,FrontTireMass);
TireF.Inertia = FrontTireInertia;
WheelF = ScanSim.Wheel.Model(TireF,FrontWheelMass);
WheelF.WheelInertia = FrontWheelInertia;

TireR = ScanSim.Wheel.Tire(RearLoadedRadius,RearEffectiveRadius,RearTireMass);
TireR.Inertia = RearTireInertia;
WheelR = ScanSim.Wheel.Model(TireR,RearWheelMass);
WheelR.WheelInertia = RearWheelInertia;

%Full Vehicle

VehicleModel = ScanSim.Vehicle.Model(World,Chassis,PowerTrain,Aero,WheelF,WheelF,WheelR,WheelR);

%% Solver Setup

Solver = ScanSim.Solver.StraightLine(VehicleModel,5,0.01);

%% Solve!

Result = Solver.Solve;

