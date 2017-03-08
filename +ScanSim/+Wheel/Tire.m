classdef Tire < handle
%TIRE A very simple tire model
%
%J.Scanlon 170307

    %% Class Properties
    
    properties (SetAccess = immutable, GetAccess = private)
        
        %LOADEDRADIUS The radius of the tire under static conditions (m)
        LoadedRadius@double;
        
        %EFFECTIVERADIUS Think tank tracks (m)
        EffectiveRadius@double;
        
    end
        
    properties (SetAccess = immutable)
        
        %MASS Mass of the tire (kg)
        Mass@double;
        
    end
    
    properties
       
        %INERTIA Inertia of the tire (kg/m^2)
        Inertia@double = 0;
        
        %ROLLINGRESISTANCE Rolling resistance
        
        
    end
    
    %% Class Methods
    
    methods
        
        function obj = Tire(LoadedRadius,EffectiveRadius,Mass)
        %TIRE The class constructor for the tire model
           
            obj.LoadedRadius = LoadedRadius;
            obj.EffectiveRadius = EffectiveRadius;
            obj.Mass = Mass;
            
        end
        
        function [LoadedRadius,EffectiveRadius] = GetRadii(obj)
        %GETRADII Get the loaded and effective radii of the tire for the
        %given parameters
        
            LoadedRadius = obj.LoadedRadius;
            EffectiveRadius = obj.EffectiveRadius;
        
        end
        
    end
    
end

