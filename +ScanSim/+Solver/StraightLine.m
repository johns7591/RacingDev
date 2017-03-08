classdef StraightLine < ScanSim.Solver.Base
%STRAIGHTLINE Summary of this class goes here
%   Detailed explanation goes here
    
    %% Class Properties

    properties
        
        %STARTSPEED The starting speed for the simulation.  Must be >0
        %(m/s)
        StartSpeed@double;
        
        %TIMESTEP The time step for the sim.  Recommend 0.001s
        TimeStep@double;
        
    end
    
    %% Class Methods
    
    methods
        
        %% Class Constructor
        
        function obj = StraightLine(Vehicle,World,StartSpeed,TimeStep)
        %STRAIGHTLINE Straight line sim class constructor
            
            if StartSpeed <= 0
                error('ERROR: Negative and zero start speeds not supported');
            end
            
            obj = obj@ScanSim.Solver.Base(Vehicle,World);
            
            obj.StartSpeed = StartSpeed;
            obj.TimeStep = TimeStep;
            
        end
        
        %% Solver
        
        function Result = Solve(obj)
        %SOLVE Run a basic transient analysis of the vehicle
            
            % Display that it's running
            disp('Running straight line simulation solver');
            tic;
        
            % Iteration
            i = 1;
            
            % Position initial conditions.  These are the only things that
            % we'll evolve over time
            Result.BodyVx(i) = obj.StartSpeed;
            Result.BodyDx(i) = 0;
            Result.Time(i) = 0;
            
            % Pre-loop variables
            WOT = max(obj.Vehicle.PowerTrain.Engine.ControlVector);
            ShiftHysteresis = obj.Vehicle.PowerTrain.ShiftCutDuration * 10;
            ShiftTimer = ShiftHysteresis;
            
            % Stop condition
            StopCriteriaDur    = 1; %s
            StopCriteriaThresh = 0.01; %mph
            StopCounter        = 0; %s
                       
            while true
            
                % Aero forces (assume no wind for now)
                [Result.SCz(i),Result.SCx(i),Result.Abal(i)] = obj.Vehicle.Aerodynamics.GetAeroCoeffs;
                Result.DynPres(i) = 0.5*obj.World.GetAirDensity*(Result.BodyVx(i)^2);
                Result.Drag(i) = Result.DynPres(i) * Result.SCx(i);
                Result.Downforce(i) = Result.DynPres(i) * Result.SCz(i);             
                
                % Vertical loads
                % Not yet coded
                
                % Rolling resistance (zero for now)
                Result.RollingResistanceFL(i) = obj.Vehicle.WheelFL.Tire.GetRollingResistance(0);
                Result.RollingResistanceFR(i) = obj.Vehicle.WheelFR.Tire.GetRollingResistance(0);
                Result.RollingResistanceRL(i) = obj.Vehicle.WheelRL.Tire.GetRollingResistance(0);
                Result.RollingResistanceRR(i) = obj.Vehicle.WheelRR.Tire.GetRollingResistance(0);
                
                % Tire Radii - ultimately these will be based on Fz
                [Result.LoadedRadiusFL(i),Result.EffectiveRadiusFL(i)] = obj.Vehicle.WheelFL.Tire.GetRadii;
                [Result.LoadedRadiusFR(i),Result.EffectiveRadiusFR(i)] = obj.Vehicle.WheelFR.Tire.GetRadii;
                [Result.LoadedRadiusRL(i),Result.EffectiveRadiusRL(i)] = obj.Vehicle.WheelRL.Tire.GetRadii;
                [Result.LoadedRadiusRR(i),Result.EffectiveRadiusRR(i)] = obj.Vehicle.WheelRR.Tire.GetRadii;
                
                % Wheelspeeds
                Result.WheelSpeedFL(i) = Result.BodyVx(i) / (Result.EffectiveRadiusFL(i)*2*pi);
                Result.WheelSpeedFR(i) = Result.BodyVx(i) / (Result.EffectiveRadiusFR(i)*2*pi);
                Result.WheelSpeedRL(i) = Result.BodyVx(i) / (Result.EffectiveRadiusRL(i)*2*pi);
                Result.WheelSpeedRR(i) = Result.BodyVx(i) / (Result.EffectiveRadiusRR(i)*2*pi);
                
                % Total hub loss
                Result.HubLossFL(i) = obj.Vehicle.WheelFL.HubLoss * Result.WheelSpeedFL(i);
                Result.HubLossFR(i) = obj.Vehicle.WheelFR.HubLoss * Result.WheelSpeedFR(i);
                Result.HubLossRL(i) = obj.Vehicle.WheelRL.HubLoss * Result.WheelSpeedRL(i);
                Result.HubLossRR(i) = obj.Vehicle.WheelRR.HubLoss * Result.WheelSpeedRR(i);
                
                % Final drive speed
                Result.FinalSpeed(i) = (Result.WheelSpeedRL(i) + Result.WheelSpeedRR(i)) / 2;
                
                % Select the optimal gear
                OptGear = obj.Vehicle.PowerTrain.GetOptimalGear(Result.FinalSpeed(i));
                if ShiftTimer < ShiftHysteresis
                    Result.Gear(i) = Result.Gear(i-1);
                else
                    Result.Gear(i) = OptGear;
                end
                
                % Reset the timer if we shifted
                if i > 1
                    if Result.Gear(i) > Result.Gear(i-1)
                        ShiftTimer = 0;
                    end
                end
                
                % Cut if we're cutting            
                CutLevel = 1;
                if ShiftTimer < obj.Vehicle.PowerTrain.ShiftCutDuration
                    CutLevel = obj.Vehicle.PowerTrain.ShiftCutLevel;
                end

                % And the engine speed for that gear, and axle torque
                Result.OutputRatio(i) = obj.Vehicle.PowerTrain.Driveline.OutputRatios(Result.Gear(i));
                Result.EngineSpeed(i) = Result.FinalSpeed(i) * Result.OutputRatio(i);  
                Result.RearAxleTorque(i) = obj.Vehicle.PowerTrain.GetOutputTorque(Result.EngineSpeed(i),WOT,Result.Gear(i),CutLevel);
                
                % Get wheel torques.  Assume power evenly split between
                % rear wheels
                Result.WheelTorqueFL(i) = - Result.HubLossFL(i);
                Result.WheelTorqueFR(i) = - Result.HubLossFR(i);
                Result.WheelTorqueRL(i) = - Result.HubLossRL(i) + Result.RearAxleTorque(i)/2;
                Result.WheelTorqueRR(i) = - Result.HubLossRR(i) + Result.RearAxleTorque(i)/2;                           
                               
                % Tractive forces
                Result.TireFxFL(i) = Result.WheelTorqueFL(i) / Result.LoadedRadiusFL(i);
                Result.TireFxFR(i) = Result.WheelTorqueFR(i) / Result.LoadedRadiusFR(i);
                Result.TireFxRL(i) = Result.WheelTorqueRL(i) / Result.LoadedRadiusRL(i);
                Result.TireFxRR(i) = Result.WheelTorqueRR(i) / Result.LoadedRadiusRR(i);
                                 
                % Ineria
                InertiaFront = obj.Vehicle.WheelFL.Inertia + obj.Vehicle.WheelFR.Inertia;
                InertiaRear  = obj.Vehicle.WheelRL.Inertia + obj.Vehicle.WheelRR.Inertia...
                             + obj.Vehicle.PowerTrain.GetInertiaAtWheel(Result.Gear(i));
                         
                InertiaFront_Linear = InertiaFront / (((Result.LoadedRadiusFL(i)+Result.LoadedRadiusFR(i))/2)^2);
                InertiaRear_Linear  = InertiaRear  / (((Result.LoadedRadiusRL(i)+Result.LoadedRadiusRR(i))/2)^2);
                
                % Current Vehicle State
                Result.BodyFx(i) = Result.TireFxFL(i)...
                                 + Result.TireFxFR(i)...
                                 + Result.TireFxRL(i)...
                                 + Result.TireFxRR(i)...
                                 - Result.RollingResistanceFL(i)...
                                 - Result.RollingResistanceFR(i)...
                                 - Result.RollingResistanceRL(i)...
                                 - Result.RollingResistanceRR(i)...
                                 - Result.Drag(i);
                
                Result.BodyAx(i) = Result.BodyFx(i) / (obj.Vehicle.Mass + InertiaFront_Linear + InertiaRear_Linear);
                
                % Check if we should stop
                if Result.BodyAx(i) < StopCriteriaThresh
                    StopCounter = StopCounter + obj.TimeStep;
                    if StopCounter > StopCriteriaDur
                        break;
                    end
                else
                    StopCounter = 0;
                end
                
                % Loop Increment
                i = i+1;
                
                % New Vehicle State
                Result.BodyVx(i) = Result.BodyVx(i-1) + Result.BodyAx(i-1) * obj.TimeStep;
                Result.BodyDx(i) = Result.BodyDx(i-1) + Result.BodyVx(i-1) * obj.TimeStep;
                
                % Time
                Result.Time(i) = Result.Time(i-1) + obj.TimeStep;
                ShiftTimer = ShiftTimer + obj.TimeStep;
                            
            end
            
            disp(['Solve completed in ' num2str(toc) ' seconds']);
            
        end
        
        %% Other tools
        
        function Lights = ShiftLightCalculator(obj,DelayArray)
        %SHIFTLIGHTCALCULATOR Calculate shift lights with a relatively
        %constant time delay between eachother, and the shift point.
        %
        % Inputs:
        %       Delay array - the delay from each light to the next.  The
        %       last value is the delay to the shift itself.  Number of
        %       values in DelayArray is equal to number of lights.
            
            %Simulate
            Result = obj.Solve;
                        
            %Get number of gears
            NumGears = obj.Vehicle.PowerTrain.Driveline.GearCount;
            
            % Calculate lights for each gear
            Lights = nan(NumGears-1,length(DelayArray)+1);
            RowTitles = cell(1,NumGears-1);
            ColTitles = cell(1,length(DelayArray)+1);
            ColTitles{end} = 'ShiftPoint';
            for i = 1:NumGears-1
                
                % Time and engine speed for this gear
                timeInThisGear = Result.Time(Result.Gear == i);
                timeInNextGear = Result.Time(Result.Gear == i+1);
                if isempty(timeInNextGear)
                    warning(['Gear ' num2str(i+1) ' was unused, thus gear ' num2str(i) ' will not have shift light results.  Maybe the gearing is way off.']);
                    continue;
                end
                timeInThisGear = timeInThisGear - timeInThisGear(1);                
                engineSpeedData = Result.EngineSpeed(Result.Gear == i);
                
                % Title
                RowTitles{i} = ['Shift ' num2str(i) '-' num2str(i+1)];
                
                % Individual lights
                for j = 1:length(DelayArray)
                    
                    % Time till shift
                    timeToShift = sum(DelayArray(j:end));
                    
                    % Title
                    if i == 1                       
                        ColTitles{j} = ['Light ' num2str(j) ' @ T-' num2str(timeToShift) 's'];                
                    end

                    % Are we out of useful range?
                    if timeToShift > timeInThisGear(end)
                        warning(['Light ' num2str(j) ' for gear ' num2str(i) ' will occur before this gear, and will not have shift light results.  Perhaps your delays are way too long.']);
                        continue;
                    end
                    
                    % Set it
                    Lights(i,j) = interp1(timeInThisGear,engineSpeedData,timeInThisGear(end)-timeToShift);
                    
                end
                
                Lights(i,end) = engineSpeedData(end);
                
            end            
            
            % If no outputs requested, a plot appears!
            if nargout == 0
                figure('Position',[100 100 600 150],'Name','Shift Lights in RPM');
                uitable('Units','normalized','Position',...
                        [0.1 0.1 0.9 0.9],'Data',round(Lights.*60),'ColumnName',ColTitles,... 
                        'RowName',RowTitles);                
            end
            
        end
        
    end
    
end

