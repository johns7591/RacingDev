classdef Model < handle
%MODEL The model of the world.  For the time being it just describes
%atmospheric conditions.  Ultimately will be a track I guess.
%
%J.Scanlon 170306
    
    %% Class Properties

    properties
        
        %TEMPERATURE The ambient temperature in kelvin
        Temperature@double = 273.15;
        
        %PRESSURE The ambient pressure in pascals
        Pressure@double = 1e5;
        
        %HUMIDITY The humidity in ratio
        Humidity@double = 0;
        
    end
    
    %% Class Methods
    
    methods
        
        function AirDensity = GetAirDensity(obj)
            
            %Simple calculations and defining constants
            AirTempK = obj.Temperature;
            AirTempC = AirTempK - 273.15;
            Rd = 287.0531; %Universal gas constant for dry air
            Rv = 461.4964; %Universal gas constant for water vapour

            %Herman Wobus' ridiculous formula:
            eso = 6.1078;
            c = [0.99999683 -0.90826951e-2 0.78736169e-4 -0.61117958e-6 0.43884187e-8 -0.29883885e-10 0.21874425e-12 -0.17892321e-14 0.11112018e-16 -0.30994571e-19];
            p = (c(1)+AirTempC*(c(2)+AirTempC*(c(3)+AirTempC*(c(4)+AirTempC*(c(5)+AirTempC*(c(6)+AirTempC*(c(7)+AirTempC*(c(8)+AirTempC*(c(9)+AirTempC*(c(10)))))))))));

            Es = 100*(eso/(p^8)); %Pressure defined in Pascals
            Pv = obj.Humidity * Es; %Partial pressure of water vapour
            Pd = obj.Pressure - Pv; %Partial pressure of dry air
            AirDensity = (Pd/(Rd*AirTempK))+(Pv/(Rv*AirTempK)); %Local Air Density 
                        
        end        
        
    end
    
end

