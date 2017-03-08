classdef Base < handle
%BASE Base class for all solvers.
%
%J.Scanlon 030717

    %% Class Properties
    
    properties
        
        %VEHICLE The full vehicle model
        Vehicle@ScanSim.Vehicle.Model;
        
        %WORLD The environment for the simulation
        World@ScanSim.World.Model;
        
    end
    
    %% Class Methods
    
    methods
        
        %% Class constructor
        
        function obj = Base(Vehicle,World)
        %BASE The class constructor
           
           obj.Vehicle = Vehicle;
           obj.World = World;
            
        end
        
    end
    
    %% Abstract Methods
    
    methods
    % Note: 2011b doesn't do abstraction yet, so I'm mimicing it here
    
        function Solve(obj)
            error('ERROR: This method must be overloaded by the parent class');
        end
        
    end
    
end

