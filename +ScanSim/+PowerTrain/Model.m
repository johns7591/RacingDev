classdef Model < handle
%MODEL Model of a powertrain and some control parameters.
%
%J.Scanlon 170306
    
    %% Class Properties

    properties (SetAccess = immutable)
        
        %ENGINE The engine model applied to this powertrain
        Engine@ScanSim.PowerTrain.Engine;
        
        %DRIVELINE The driveline model applied to this powertrain
        Driveline@ScanSim.PowerTrain.Driveline;
        
    end
        
    properties (SetAccess = private)
                
        %SOFTLIMITSPEEDS Speed or speeds at which the soft limit activates.
        %Must either be the same length as number of gears, or if length 1
        %will be applied to all gears (1/s)
        SoftLimitSpeeds@double;
        
        %SOFTLIMITGAINS Gain for cut increase per increase in engine
        %speed (ratio*s) <- check out those weird units!
        SoftLimitGains@double;
        
        %HARDLIMITSPEEDS Speed or speeds at which the hard limit activates.
        %Must either be the same length as number of gears, or if length 1
        %will be applied to all gears (1/s)
        HardLimitSpeeds@double;
        
        %HARDLIMITGAINS Gain for cut increase per increase in engine
        %speed (ratio*s) <- check out those weird units!
        HardLimitGains@double;
        
    end
    
    properties
        
        %SHIFTCUTDURATION The amount of time for which shifting is cut (s)
        ShiftCutDuration@double = 0;
        
        %SHIFTCUTLEVEL The ratio of torque remaining during a shift (e.g. a
        %value of 0.2 means that 20% of total torque is available during
        %shift
        ShiftCutLevel@double = 1; 
        
    end

    %% Class Methods
    
    methods
        
        %% Class Constructor
        
        function obj = Model(Engine,Driveline)
        %MODEL The class constructor.  User must have already created
        %Engine and Driveline objects before instantiating this.
            
            % Set engine and driveline
            obj.Engine = Engine;
            obj.Driveline = Driveline;
            
            % And defaults
            obj.SoftLimitSpeeds = nan(Driveline.GearCount,1);
            obj.SoftLimitGains = nan(Driveline.GearCount,1);
            obj.HardLimitSpeeds = nan(Driveline.GearCount,1);
            obj.HardLimitGains = nan(Driveline.GearCount,1);
            
        end
        
        %% Other Methods
        
        function AddSoftLimiter(obj,SoftLimitSpeeds,SoftLimitGains)
        %ADDSOFTLIMITER Add a soft limiter by supplying the limit speed(s)
        %and corresponding proportional gain values.  We assume P only!
            
            % Error checks
            if length(SoftLimitGains) ~= length(SoftLimitSpeeds)
                error('ERROR: Each speed should have an associated gain');
            end           
            if length(SoftLimitSpeeds) == 1
                SoftLimitSpeeds = repmat(SoftLimitSpeeds,length(obj.Driveline.GearRatios),1);
                SoftLimitGains = repmat(SoftLimitGains,length(obj.Driveline.GearRatios),1);
            end
            if length(SoftLimitSpeeds) ~= length(obj.Driveline.GearRatios)
                error('ERROR: There must be an equal number of limits to the number of gears, or a global limit');
            end
            if any(SoftLimitGains < 0)
                error('ERROR: Gain values are decrease in torque per engine speed increase, thus must be positive');
            end
            
            % Set
            obj.SoftLimitSpeeds = SoftLimitSpeeds;
            obj.SoftLimitGains = SoftLimitGains;
            
        end
        
        function AddHardLimiter(obj,HardLimitSpeeds,HardLimitGains)
        %ADDHARDLIMITER Add a hard limiter by supplying the limit speed(s)
        %and corresponding proportional gain values.  We assume P only!
            
            % Error checks
            if length(HardLimitGains) ~= length(HardLimitSpeeds)
                error('ERROR: Each speed should have an associated gain');
            end           
            if length(HardLimitSpeeds) == 1
                HardLimitSpeeds = repmat(HardLimitSpeeds,length(obj.Driveline.GearRatios),1);
                HardLimitGains = repmat(HardLimitGains,length(obj.Driveline.GearRatios),1);
            end
            if length(HardLimitSpeeds) ~= length(obj.Driveline.GearRatios)
                error('ERROR: There must be an equal number of limits to the number of gears, or a global limit');
            end
            if any(HardLimitGains < 0)
                error('ERROR: Gain values are decrease in torque per engine speed increase, thus must be positive');
            end
            
            % Set
            obj.HardLimitSpeeds = HardLimitSpeeds;
            obj.HardLimitGains = HardLimitGains;
            
        end
        
        function Torque = GetOutputTorque(obj,SpeedArray,ControlArray,Gear,ActiveCut)
        %GETOUTPUTTORQUE Get torque value from a given engine speed
        %input and control input.  Same as in engine class, but augmented
        %with limiters, ratios, and efficiency
        %
        % Inputs:
        %       SpeedArray: An array of engine speeds (in 1/s)
        %       ControlArray: An array of engine control input, same length
        %       as the speed array.
        %       Gear: What gear are you in
        %       ActiveCut: Cut level if there is one
        
            %If no active cut
            if nargin == 4
                ActiveCut = 1;
            end
            if ActiveCut > 1
                error('ERROR: Engine cut ratio cannot be > 1');
            end
            
            %Basic torque output
            Torque = obj.Engine.GetEngineOutput(SpeedArray,ControlArray,'TorqueMatrix');
            OutputRatios = obj.Driveline.OutputRatios;
            
            %Correct with limiters
            LimiterCut = zeros(1,length(Torque));
            for i = 1:length(Torque)
                
                if SpeedArray(i) > obj.SoftLimitSpeeds(Gear) && SpeedArray(i) <= obj.HardLimitSpeeds(Gear)
                    LimiterCut(i) = (SpeedArray(i) - obj.SoftLimitSpeeds(Gear)) * obj.SoftLimitGains(Gear);
                elseif SpeedArray(i) > obj.HardLimitSpeeds(Gear)
                    if ~isnan(obj.SoftLimitSpeeds(Gear))
                        LimiterCut(i) = (obj.HardLimitSpeeds(Gear) - obj.SoftLimitSpeeds(Gear)) * obj.SoftLimitGains(Gear);
                    end       
                    LimiterCut(i) = LimiterCut(i) + (SpeedArray(i) - obj.HardLimitSpeeds(Gear)) * obj.HardLimitGains(Gear);
                end
                if LimiterCut(i) > 1
                    LimiterCut(i) = 1;
                end
                                
            end
            
            Torque = Torque .* OutputRatios(Gear) .* obj.Driveline.Efficiency .* ActiveCut .* (1-LimiterCut);
        
        end
        
        function Inertia = GetInertiaAtWheel(obj,Gear)
        %GETINERTIAATWHEEL Get the effective inertia of the powertrain,
        %resolved to the drive axle
        
            Inertia = obj.Driveline.InertiaFinal * obj.Driveline.FinalRatio + ...
                      obj.Driveline.InertiaGearbox * obj.Driveline.FinalRatio * obj.Driveline.GearRatios(Gear) + ...
                      obj.Engine.Inertia * obj.Driveline.OutputRatios(Gear);
            
        end
                
        function Gear = GetOptimalGear(obj,OutputSpeed)
        %GETOPTIMALGEAR Calculate the ideal gear to be in at this
        %output speed, where output speed is the angular velocity of the
        %drive wheels
           
            OutputRatios = obj.Driveline.OutputRatios;
            MaxControl = max(obj.Engine.ControlVector);
            
            OutputTorque = zeros(length(OutputRatios),1);
            for i = 1:length(OutputRatios)
                
                EngSpeed = OutputSpeed .* OutputRatios(i);
                OutputTorque(i) = obj.GetOutputTorque(EngSpeed,MaxControl,i);
                
            end
            
            [~,Gear] = max(OutputTorque);
        
        end
        
    end
    
end

