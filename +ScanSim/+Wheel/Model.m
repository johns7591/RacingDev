classdef Model < handle
%MODEL The model of a wheel.
%
%J.Scanlon 030717
    
    %% Class Properties

    properties (SetAccess = immutable, GetAccess = private)
                
        %WHEELMASS The mass of a wheel (kg)
        WheelMass@double;

    end
    
    properties (SetAccess = immutable)
       
        %TIRE The tire model
        Tire@ScanSim.Wheel.Tire;
        
    end
    
    properties (Dependent = true)
        
        %MASS The mass of the wheel and tire (kg)
        Mass;
        
        %INERTIA The inertia of the wheel and tire (kg*m^2)
        Inertia;
        
    end
    
    properties
        
        %WHEELINERTIA The moment of inertia of the wheel (kg*m^2)
        WheelInertia@double = 0;
        
        %HUBLOSS Approximates bearing drag, first order with ang speed
        %(N*m*s) - loss of torque with speed
        HubLoss@double = 0;
        
    end
    
    %% Class Methods
    
    methods
        
        %% Getters and Setters
        
        function Mass = get.Mass(obj)
           
            Mass = obj.WheelMass + obj.Tire.Mass;
            
        end
        
        function Inertia = get.Inertia(obj)
            
            Inertia = obj.WheelInertia + obj.Tire.Inertia;            
            
        end
        
        %% Class Constructor
        
        function obj = Model(Tire,WheelMass)
        %MODEL Class constructor for the wheel model
            
            obj.Tire = Tire;
            obj.WheelMass = WheelMass;
            
        end
        
    end
    
end

