classdef Model < handle
%MODEL A very basic aero model
%
%I plan to add way more functionality to this
%
%J.Scanlon 170306
    
    %% Class Properties

    properties (SetAccess = immutable, GetAccess = private)
        
        %SCZ Lift coefficient, scaled to 1 m^2
        SCz@double;
        
        %SCX Drag coefficient, scaled to 1 m^2
        SCx@double;
        
        %Abal Aero balance, as a ratio
        Abal@double;
        
    end
    
    %% Class Methods
    
    methods
        
        %% Class Constructor
        
        function obj = Model(SCz,SCx,Abal)
        %MODEL The class constructor for the aero model
            
            %Set the class variables
            obj.SCz = SCz;
            obj.SCx = SCx;
            obj.Abal = Abal;
            
        end
        
        %% Other Methods
        
        function [SCz,SCx,Abal] = GetAeroCoeffs(obj)
        %GETAEROCOEFFS Get aero coefficients.  Ultimately this will be
        %calculated from vehicle state, but it's constant for now.
            
            SCz = obj.SCz;
            SCx = obj.SCx;
            Abal = obj.Abal;
            
        end
        
    end
    
end

