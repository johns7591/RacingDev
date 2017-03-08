classdef CSV_Processor
%CSVPROCESSOR Process a CSV file in MATLAB.  At the moment it's only set up
%to calculate gap to leader, but can easily be extended to other exciting
%calculations!
%
%JScanlon 161202
    
    %% Class Properties

    properties (SetAccess = immutable, GetAccess = private)
        
        %DATA The class-internal data structure
        Data;
        
    end
    
    properties (SetAccess = immutable)
        
        %SEGMENTS Segments found in the CSV file
        Segments;
        
    end
    
    properties (Dependent = true)
        
        %NUMLAPS The max lap found in the CSV file
        NumLaps;
        
        %LAPS The list of laps
        Laps;
        
        %CARS The list of cars
        Cars;
        
        %CLASSES The list of classes
        Classes;
        
        %DRIVERS The list of drivers
        Drivers;
                
    end
    
    %% Class Methods
    
    methods
        
        %% Getters and Setters
        
        function Cars = get.Cars(obj)            
           Cars = unique({obj.Data.Car})';             
        end
        
        function Laps = get.Laps(obj)
            Laps = unique([obj.Data.Lap])';
        end
        
        function Classes = get.Classes(obj)
            Classes = unique({obj.Data.Class})';
        end
        
        function Drivers = get.Drivers(obj)
            Drivers = unique({obj.Data.Driver})';      
        end
        
        function NumLaps = get.NumLaps(obj)
            NumLaps = max(obj.Laps);          
        end
        
        %% Constructor
        
        function obj = CSV_Processor(CSVfile)
        %CSV_PROCESSOR Class Constructor.  Either call with an IMSA CSV as
        %the argument, or MATLAB will prompt you for one.
           
            % Error checks
            if nargin == 0
                [uiFile,uiPath]=uigetfile('*.csv','Select an IMSA Timing CSV!');
                if uiFile == 0
                    return;
                else
                    CSVfile = [uiPath uiFile];
                end
            end
            if ~ischar(CSVfile)
                error('ERROR: Input must be a string');
            end
            if exist(CSVfile,'file')==0
                error(['ERROR: The file ''' CSVfile ''' does not exist']);
            end
            
            % Open the file, get the data
            fid = fopen(CSVfile,'r');
            fData = fread(fid,'*char')';
            fclose(fid);
            
            % Format the file lines
            fData = strrep(fData,'"','');
            fData = regexp(fData,'(\r\n|[\r\n])','split')';
            emptyRows = cellfun('isempty',fData);
            fData = fData(~emptyRows);
            
            % Get the header and seg names
            header = regexp(fData{1},',','split');            
            obj.Segments = header(9:end);
            
            % Build up the lap crossing data
            lastwb = 0;
            wb = waitbar(lastwb,'Parsing file lines');
            dataLen = length(fData);
            dateRef = datenum('00:00','HH:MM');
            dayOffset = 0;
            for i = 2:dataLen
                
                fileLine = regexp(fData{i},',','split');
                
                for j = 1:8
                    
                    fieldName = strrep(header{j},' ','_');
                    
                    % Add the field by its type
                    switch fieldName
                        
                        % Laps stored as integer
                        case 'Lap'                            
                            tempData(i-1).(fieldName) = str2num(['uint16(' fileLine{j} ')']);
                            
                        % Handle time strings
                        case {'Lap_Time','Session_Time'}                            
                            colonCt = numel(strfind(fileLine{j},':'));                            
                            switch colonCt                                
                                case 0         
                                    dateFormat = 'SS.FFF';
                                case 1
                                    dateFormat = 'MM:SS.FFF';
                                case 2
                                    dateFormat = 'HH:MM:SS.FFF';
                                otherwise
                                    error(['ERROR: Invalid time string: ' fileLine{j}]);                                    
                            end
                            
                            tempData(i-1).(fieldName) = (datenum(fileLine{j},dateFormat)-dateRef) * 86400;        
                            % 24 hour races roll over... thanks Obama
                            % Filter reverse steps by half a day
                            if strcmp(fieldName,'Session_Time') && i > 2
                                tempData(i-1).(fieldName) = tempData(i-1).(fieldName)+dayOffset;
                                if  (tempData(i-2).(fieldName) - tempData(i-1).(fieldName)) >= 43200
                                    tempData(i-1).(fieldName) = tempData(i-1).(fieldName) + 86400;
                                    dayOffset = dayOffset+86400;
                                end
                            end
                        
                        % Cop out
                        otherwise                    
                            tempData(i-1).(fieldName) = fileLine{j};
                            
                    end
                    
                end
                
                % Add segments
                tempData(i-1).SegmentTimes = cellfun(@str2double,fileLine(9:end));
                
                % And update wait bar
                progressRat = i/dataLen;
                progressPct = round(progressRat*100);
                if round(progressPct) > lastwb
                    waitbar(progressRat);
                    lastwb = progressPct;
                end
                
            end
            close(wb);
            
            % Save data structure
            obj.Data = tempData;
            
        end
              
        %% Other Methods
        
        function Gap = GapToLeader(obj,Car,varargin)
        %GAPTOLEADER Get the gap in seconds to the leader, per race lap.
        %Lack of output arg implies plotting, or typing 'plot' as final
        %input argument.
            
            % Error check
            if ~ischar(Car) && ~iscell(Car)
                error('ERROR: Car must be entered as string or cell');
            end
            if ischar(Car)
                Car = {Car};
            end
           
            % Initialize loop
            laps = obj.Laps;
            numLaps = length(laps);            
            numCars = length(Car);
            Gap = nan(numLaps,numCars);
            
            % Temp
            tempData = obj.Data;
            tempLaps = [tempData.Lap];
            
            % Loop lap by lap
            for i = 1:numLaps
               
                allCross = tempData(tempLaps == laps(i));
                for j = 1:numCars
                    myCross = allCross(strcmp(Car{j},{allCross.Car}));                    
                    if ~isempty(myCross)
                        myCross = myCross(end);
                        Gap(laps(i),j) = myCross.Session_Time - min([allCross.Session_Time]);
                    end
                end
                
            end
            
            % Plotting
            if nargout == 0 || strcmpi(varargin{1},'plot')
               plot(Gap);
               title('Gap to leader');
               legend(Car);
               xlabel('Lap');
               ylabel('Gap (s)');               
            end

        end
        
        function Output = FilterByAttribute(obj,InputFilterType,InputFilter,OutputFilterType)
        %FILTERBYATTRIBUTE Filter the data set by an attribute.  Optionally
        %can have an output filter to only output a specific attribute.
        %
        %Filter types:
        %   'Car'
        %   'Driver'
        %   'Class'
           
            % Error check
            tempData = obj.Data;
            if nargin == 3
                OutputFilterType = '';
            end
            if ~ischar(InputFilterType)
                error('ERROR: Filter type must be entered as string');
            end
            if ~isfield(tempData,InputFilterType)
                error(['ERROR: Input filter ' InputFilterType ' is not valid']);
            end
            if ~ischar(OutputFilterType)
                error('ERROR: Output filter must be entered as string or ommitted');
            end
            if ~ischar(InputFilter) && ~iscell(InputFilter)
                error('ERROR: Input filter must be entered as string or cell');
            end
            if ischar(InputFilter)
                InputFilter = {InputFilter};
            end
            
            % Filter
            filter = false(1,length(tempData));
            for i = 1:length(InputFilter)
                filter = filter | strcmp(InputFilter{i},{tempData.(InputFilterType)});
            end            
            Output = tempData(filter);
            
            % Filter output
            if ~isempty(OutputFilterType)
                if ~isfield(tempData,OutputFilterType)
                    error(['ERROR: Output filter ' OutputFilterType ' is not valid']);
                end
                Output = unique({Output.(OutputFilterType)});
            end
            
        end
        
    end
    
end
