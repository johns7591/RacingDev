classdef Driveline < handle
%Driveline Very simple model of a driveline and some powertrain control.
%
%Assumes rear wheel drive for now.  At the moment all control is pretty
%open loop, which makes pre-calculation really easy.
%
%J.Scanlon 170306

    %% Class Properties
    
    properties (SetAccess = immutable)
        
        %GEARRATIOS The ratios of individual selectable gears, not
        %inclusive of final and drop ratios (ratio)
        GearRatios@double;
        
        %FINALRATIO The final drive ratio (ratio)
        FinalRatio@double;
        
        %PRIMARYRATIO The ratio of the primary drive gear, if there is one
        %(ratio)
        PrimaryRatio@double;

    end
    
    properties
        
        %INERTIAGEARBOX Inertia of rotating assembly after the primary
        %reduction but before the final reduction (Kg*m^2)
        InertiaGearbox@double = 0;
        
        %INERTIAFINAL Inertia of rotating assembly after the final
        %reduction but before the wheels (Kg*m^2)
        InertiaFinal@double = 0;
        
        %EFFICIENCY The efficiency of the drivetrain (ratio).  We make the
        %assumption that efficiency is a constant power loss percentage.
        %This is the efficiency loss as would be measured at the half
        %shafts.
        Efficiency@double = 1;
        
    end
    
    properties (Dependent = true)
       
        %OUTPUTRATIOS Ratios from engine speed to wheel speed
        OutputRatios;
        
        %GEARCOUNT The amount of gears
        GearCount;
        
    end
    
    %% Class Methods
    
    methods
        
        %% Getters and Setters
        
        function OutputRatios = get.OutputRatios(obj)
           
            OutputRatios = obj.GearRatios .* obj.FinalRatio .* obj.PrimaryRatio;
            
        end
        
        function GearCount = get.GearCount(obj)
           
            GearCount = length(obj.GearRatios);
            
        end
        
        %% Class Constructor
        
        function obj = Driveline(GearRatios,FinalRatio,PrimaryRatio,varargin)
        %Driveline The class constructor for the driveline model.
        %
        % Inputs:
        %       Gear Ratios: list of selectable gear ratios, not including
        %       final drive or primary
        %       Final Ratio: final drive
        %       Primary Ratio: primary (drop) gear
        %       Other: Just enter the property name, followed by the val
        
            %Most cars don't have a primary gear.
            if nargin == 2
                PrimaryRatio = 1;
            end
            
            %Error checks
            if ~length(FinalRatio) == 1
                error('ERROR: You probably don''t have 2 final drives.');
            end
            if ~length(PrimaryRatio) == 1
                error('ERROR: You probably don''t have 2 primary gear sets.');
            end
            
            %Set the values
            obj.GearRatios = GearRatios;
            obj.FinalRatio = FinalRatio;
            obj.PrimaryRatio = PrimaryRatio;
            
            % Did we enter other stuff?
            for i = 1:2:length(varargin)                
                obj.(varargin{i}) = varargin{i+1};            
            end
            
        end
        
    end
    
end

