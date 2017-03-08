classdef Model < handle
%MODEL The full vehicle model
%
%J.Scanlon 170307
    
    %% Class Properties

    properties (SetAccess = immutable)
        
        %CHASSIS The chassis model
        Chassis@ScanSim.Chassis.Model;
        
        %POWERTRAIN The powertrain model
        PowerTrain@ScanSim.PowerTrain.Model;
        
        %AERODYNAMICS The aero model
        Aerodynamics@ScanSim.Aerodynamics.Model;
        
        %WheelFL The FL Wheel model
        WheelFL@ScanSim.Wheel.Model;
        
        %WheelFL The FR Wheel model
        WheelFR@ScanSim.Wheel.Model;
        
        %WheelFL The RL Wheel model
        WheelRL@ScanSim.Wheel.Model;
        
        %WheelFL The RR Wheel model
        WheelRR@ScanSim.Wheel.Model;
        
    end
    
    properties (Dependent = true)
        
        %MASS The total mass of the vehicle
        Mass;
        
    end
    
    %% Class Methods
    
    methods
        
        %% Getters and Setters
        
        function Mass = get.Mass(obj)
            
            Mass = obj.Chassis.Mass + obj.WheelFL.Mass + obj.WheelFR.Mass + obj.WheelRL.Mass + obj.WheelRR.Mass;
            
        end
        
        %% Class Constructor
        
        function obj = Model(Chassis,PowerTrain,Aerodynamics,WheelFL,WheelFR,WheelRL,WheelRR)
        %MODEL The full vehicle model constructor

            obj.Chassis = Chassis;
            obj.PowerTrain = PowerTrain;
            obj.Aerodynamics = Aerodynamics;
            obj.WheelFL = WheelFL;
            obj.WheelFR = WheelFR;
            obj.WheelRL = WheelRL;
            obj.WheelRR = WheelRR;
            
        end
        
    end
    
end

