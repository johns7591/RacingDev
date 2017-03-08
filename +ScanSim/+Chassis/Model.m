classdef Model < handle
%MODEL The chassis model.  Just mass for now.
%
%J.Scanlon 170307
    
    %% Class Properties
    
    properties (SetAccess = immutable)
        
        %Sprung Mass (kg)
        Mass@double;
        
    end
    
    %% Class Methods
    
    methods
        
        %% Class Constructor
        
        function obj = Model(Mass)
        %MODEL The class constructor for the chassis model.
        %
        % Inputs
        %       Mass: Sprung mass
        
            obj.Mass = Mass;
            
        end
        
    end
    
end

