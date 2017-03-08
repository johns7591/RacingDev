classdef Engine < handle
%Engine Very simple model of an engine.
%
%J.Scanlon 170306

    %% Class Properties
    
    properties (SetAccess = immutable)
        
        %SpeedVector Engine speed vector (1/s)
        SpeedVector@double;
        
        %ControlVector Engine control vector, e.g. throttle blade angle
        %Units don't matter, but should generally be an increasing vector
        %with the max value corresponding to max torque.
        ControlVector@double;
        
        %TorqueMatrix Engine torque matrix (N*m)
        TorqueMatrix@double;

    end
    
    properties
        
        %Inertia The inertia of the engine (Kg*m^2)
        Inertia@double = 0;
        
    end
    
    properties (Dependent = true)
       
        %PowerMatrix Engine Power in (W)
        PowerMatrix;
        
    end
    
    %% Class Methods
    
    methods
        
        %% Getters and Setters
        
        function PowerMatrix = get.PowerMatrix(obj)
           
            PowerMatrix = bsxfun(@times,obj.TorqueMatrix,obj.SpeedVector);
            
        end
        
        function set.Inertia(obj,Inertia)
           
            if length(Inertia) > 1
                error('ERROR: Inertia must be a single value');
            end
            obj.Inertia = Inertia;
            
        end
        
        %% Class Constructor
        
        function obj = Engine(SpeedVector,TorqueMatrix,ControlVector,varargin)
        %Engine The class constructor for the engine model.
        %
        % Inputs:
        %       Engine Speed: double array of engine speed in 1/s
        %       Engine Torque: double matrix of engine torque in N*m
        %       Control: throttle angle or similar
        %       Other: Just enter the property name, followed by the val
        %
        % Note: speed and torque vectors must have same number of columns,
        %       control and torque vector the same number of rows.
        
            % Error checks
            [nrSp,ncSp] = size(SpeedVector);
            [nrTq,ncTq] = size(TorqueMatrix);
            [nrCt,ncCt] = size(ControlVector);
            
            if ncSp ~= ncTq
                error('ERROR: Torque and speed must have same col length');
            end
            if nrCt ~= nrTq
                error('ERROR: Torque and control must have same row length');
            end
            
            if nrSp~=1 || ncCt~=1
                error('ERROR: Speed and control must be vectors');
            end
            
            % Set the basics
            obj.SpeedVector = SpeedVector;
            obj.TorqueMatrix = TorqueMatrix;
            obj.ControlVector = ControlVector;
            
            % Did we enter other stuff?
            for i = 1:2:length(varargin)                
                obj.(varargin{i}) = varargin{i+1};            
            end
                        
        end
        
        %% General methods
        
        function Value = GetEngineOutput(obj,SpeedArray,ControlArray,PropertyName)
        %GETENGINEOUTPUT Get a property value from a given engine speed
        %input and control input
        %
        % Inputs:
        %       SpeedArray: An array of engine speeds (in 1/s)
        %       ControlArray: An array of engine control input, same length
        %       as the speed array.
            
            % Use interp, and let it do its own error checks
            Value = interp2(obj.SpeedVector,obj.ControlVector,obj.(PropertyName),SpeedArray,ControlArray);
            
        end
        
    end
    
end

