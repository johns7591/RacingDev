%% Shift Light Calculation - Example File
%
%J.Scanlon 170308

%% User Entry For Shift Lights

% Delay array - the delay from each light to the next.  The last value is
% the delay to the shift itself.  Number of values in DelayArray is equal
% to number of lights.

DelayArray = [0.2 0.2 0.2 0.3];


%% Basic Setup
% Enter parameters for the sim here.  Note that EVERYTHING IS IN SI UNITS!
% I know, it's terrible.  But it makes the math so much easier.

%Chassis Setup:
%   Just corner weights for now

CornerWeightFL = 250; %kg
CornerWeightFR = 250; %kg
CornerWeightRL = 320; %kg
CornerWeightRR = 320; %kg

%Gearbox Parameters:
%   Just the gear ratios here

GearRatios = [2.43 1.94 1.65 1.47 1.33 1.21]; %out:in
FinalDrive = 2.6; %out:in

%World:
%   Self explanatory

Temperature = 293.15; %K
Pressure    = 1e5; %Pa
Humidity    = 0.6; %RH


%% Vehicle Properties
% These are things that may change less frequently throughout the weekend.
% Not to say they won't change, but just not as often.  EVERYTHING IS STILL
% IN SI UNITS!  That hasn't changed since the previous cell.

%Tires/Wheels:
%   Just care about effective and loaded radii for now.  I know they
%   change with speed and load, but haven't implemented yet.  Also, for now
%   please include all unsprung mass in the wheel mass.  Note that the way
%   I have this set up, you're only entering values for ONE WHEEL.  They
%   are then mirrored right to left because the car's probably symmetrical.
%   Rolling resistance is not coded yet.

FrontEffectiveRadius = 0.325; %m
FrontLoadedRadius    = 0.318; %m
FrontTireMass        = 4.5; %kg
FrontTireInertia     = 0.0; %kg*m^2
FrontWheelMass       = 5.0; %kg
FrontWheelInertia    = 0.0; %kg*m^2
FrontRollingResist   = 0.0; %N/N, positive
FrontHubloss         = 0.0; %N*m / RPS, positive

RearEffectiveRadius  = 0.351; %m
RearLoadedRadius     = 0.345; %m
RearTireMass         = 5.0; %kg
RearTireInertia      = 0.0; %kg*m^2
RearWheelMass        = 5.5; %kg
RearWheelInertia     = 0.0; %kg*m^2
RearRollingResist    = 0.0; %N/N, positive
RearHubloss          = 0.0; %N*m / RPS, positive

%Engine:
%   This is where you enter the basics about the engine.  I haven't
%   implemented correction factors yet.

ThrottleVector = [0;1]; %ratio
SpeedVector    = [0,16.6667,33.3334,50.0001,66.6668,83.3335,100.0002,116.6669,133.3336;]; %cycles/sec
TorqueMatrix   = [0,0,0,0,0,0,0,0,0;...
                  0,135.581795,298.279949,474.5362825,610.1180775,677.908975,677.908975,610.1180775,406.745385;]; %N*m
EngineInertia  = 0; %kg*m^2

%Limiter:
%   2-stage type or single stage can be entered.  Just delete or comment
%   out the soft limit lines to do a single stage.  This models the control
%   as proportional gain only, which is not 100% accurate, but the engine
%   guys can give you an approximation of a prop gain term.  Basically it's
%   torque cut ratio increase per RPS.

SoftLimit     = [117,117,117,117,117,117]; %cycles/sec (RPS)
SoftLimitGain = [0.06,0.06,0.06,0.06,0.06,0.06]; %cut increase per RPS
HardLimit     = [125,125,125,125,125,125]; %cycles/sec (RPS)
HardLimitGain = [0.12,0.12,0.12,0.12,0.12,0.12]; %cut increase per RPS

%Gearbox Parameters:
%   This should be familiar and straightforward.  You can enter a primary
%   gear here too if you have one, but that's rare on non-bikes.

DrivelineEfficiency = 0.9; %ratio
InertiaGearbox      = 0; %kg*m^2
InertiaFinal        = 0; %kg*m^2
ShiftCutDuration    = 0.05; %s
ShiftCutLevel       = 0.2; %ratio

%Aero:
%   Only SCx matters for now

SCx  = 1.2;
SCz  = 0;
Abal = 0.4;


%% Model Generation - Don't touch!

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
PowerTrain.ShiftCutDuration = ShiftCutDuration;
PowerTrain.ShiftCutLevel = ShiftCutLevel;

%Aero
Aero = ScanSim.Aerodynamics.Model(SCz,SCx,Abal);

%World
World = ScanSim.World.Model;
World.Temperature = Temperature;
World.Pressure = Pressure;
World.Humidity = Humidity;

%Wheels and tires
TireF = ScanSim.Wheel.Tire(FrontLoadedRadius,FrontEffectiveRadius,FrontTireMass);
TireF.Inertia = FrontTireInertia;
TireF.RollingResistance = FrontRollingResist;
WheelF = ScanSim.Wheel.Model(TireF,FrontWheelMass);
WheelF.WheelInertia = FrontWheelInertia;
WheelF.HubLoss = FrontHubloss;

TireR = ScanSim.Wheel.Tire(RearLoadedRadius,RearEffectiveRadius,RearTireMass);
TireR.Inertia = RearTireInertia;
TireR.RollingResistance = RearRollingResist;
WheelR = ScanSim.Wheel.Model(TireR,RearWheelMass);
WheelR.WheelInertia = RearWheelInertia;
WheelR.HubLoss = RearHubloss;

%Chassis
SprungMass = CornerWeightFL + CornerWeightFR + CornerWeightRL + CornerWeightRR - (2*WheelF.Mass + 2*WheelR.Mass);
Chassis = ScanSim.Chassis.Model(SprungMass);

%Full Vehicle
VehicleModel = ScanSim.Vehicle.Model(Chassis,PowerTrain,Aero,WheelF,WheelF,WheelR,WheelR);


%% Solver - Don't touch!

Solver = ScanSim.Solver.StraightLine(VehicleModel,World,10,0.01);
Solver.ShiftLightCalculator(DelayArray);
