classdef StraightLine
%STRAIGHTLINE Summary of this class goes here
%   Detailed explanation goes here
    
    %% Class Properties

    properties
        
        %VEHICLE The full vehicle model
        Vehicle@ScanSim.Vehicle.Model;
        
        %STARTSPEED The starting speed for the simulation.  Must be >0
        %(m/s)
        StartSpeed@double;
        
        %TIMESTEP The time step for the sim.  Recommend 0.01s
        TimeStep@double;
        
    end
    
    %% Class Methods
    
    methods
        
        %% Class Constructor
        
        function obj = StraightLine(Vehicle,StartSpeed,TimeStep)
        %STRAIGHTLINE Straight line sim class constructor
            
            if StartSpeed <= 0
                error('ERROR: Negative and zero start speeds not supported');
            end
            
            obj.Vehicle = Vehicle;
            obj.StartSpeed = StartSpeed;
            obj.TimeStep = TimeStep;
            
        end
        
        %% Solver
        
        function Result = Solve(obj)
            
            
            
        end
        
    end
    
end

