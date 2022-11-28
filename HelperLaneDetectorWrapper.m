classdef HelperLaneDetectorWrapper < matlab.System
    %HelperLaneDetectorWrapper Provides wrapper to the helperLaneDetector class.
    %
    % NOTE: The name of this System Object and it's functionality may
    % change without notice in a future release,
    % or the System Object itself may be removed.
    
    % Copyright 2020 The MathWorks, Inc.
    
    properties(Nontunable, Logical)
        % Enabling lane tracker
        EnableLaneTracker = true;
        
         % Display debug visualization windows
        EnableDisplays = true;
    end
    
    properties
       % Camera sensor parameters
        Camera = struct('ImageSize',[480 640],'PrincipalPoint',[320 240],...
            'FocalLength',[320 320],'Position',[1.8750 0 1.2000],...
            'PositionSim3d',[0.5700 0 1.2000],'Rotation',[0 0 0],...
            'LaneDetectionRanges',[8 17],'DetectionRanges',[8 50],...
            'MeasurementNoise',diag([6,1,1]));

         
    end
    
    properties (SetAccess='private', GetAccess='public')
        
         % Array useful for storing the history
        left_lane_arr = zeros(1,15);
        right_lane_arr = zeros(1,15);
        distance_left_lane = zeros(1,15);
        distance_right_lane = zeros(1,15);
        departure=0;
        departure_history= zeros(1,15);
        frame_counter=1;
    end
    
    properties (SetAccess='private', GetAccess='private', Hidden)
        % helperLaneDetector object
        LaneMarkerDetector
    end
    
    methods(Access = protected)
        %------------------------------------------------------------------
        % System object methods for Simulink integration
        %------------------------------------------------------------------
        function setupImpl(obj)
            % Camera setup
            %-------------
            camera = obj.Camera;     
            focalLength    = camera.FocalLength;
            principalPoint = camera.PrincipalPoint;
            imageSize      = camera.ImageSize;
            % mounting height in meters from the ground
            height         = camera.Position(3);  
            % pitch of the camera in degrees
            pitch          = camera.Rotation(2);  
            
            camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
            sensor        = monoCamera(camIntrinsics, height, 'Pitch', pitch);
            obj.LaneMarkerDetector = helperLaneDetector(sensor);
            % Lane marker detection parameters
            %---------------------------------
            % The percentage extent of the ROI a lane needs to cover. It can remove
            % noisy detections
            obj.LaneMarkerDetector.LaneXExtentThreshold = 0.55;
            % Sensitivity for the lane segmentation routine
            obj.LaneMarkerDetector.LaneSegmentationSensitivity = 0.25; 
            % Approximate lane marker width specified in meters
            obj.LaneMarkerDetector.ApproximateLaneMarkerWidth = 0.25;  
            % The percentage of inlier points required per unit length
            obj.LaneMarkerDetector.LaneStrengthThreshold = 0.24;   
            % Maximum number of lane marker detections from
            % findParabolicLaneBoundaries
            obj.LaneMarkerDetector.MaxNumLaneMarkersToDetect = 2;
            % Min and max lane detection range specified in meters
            obj.LaneMarkerDetector.LaneDetectionRanges = camera.LaneDetectionRanges;
            
            % Initialize the arrays
            obj.left_lane_arr = zeros(1,15);
            obj.right_lane_arr = zeros(1,15);

            obj.distance_left_lane = zeros(1,15);
            obj.distance_right_lane = zeros(1,15);

            obj.departure=0; % -1: left departure;  0: NO departure;  +1: right departure
            obj.departure_history= zeros(1,15);
            obj.frame_counter=1;

        end
        
        function stepImpl(obj,frame)
            
            % Detect lane boundaries
            [leftEgoBoundary,rightEgoBoundary, birdsEyeImage] = laneDetector(obj.LaneMarkerDetector, frame);

            [h,w] = size(birdsEyeImage);

            %% do image binarization
            regionA = cast(max(birdsEyeImage,[], 'all'), 'double');
            regionB = cast(min(birdsEyeImage,[], 'all'),'double');
            
            th_new = cast((regionA+regionB)/2, 'double');
            th_old = cast(1e3, 'double');
        
            % find threshold using iterative process
            while abs(th_new-th_old) > 1e-2
                th_old = th_new;
    
                A = birdsEyeImage(birdsEyeImage>th_old);
                regionA = sum(A)/numel(A);
        
                B = birdsEyeImage(birdsEyeImage<=th_old);
                regionB = sum(B)/numel(B);
        
                th_new = (regionA+regionB)/2;
            end

            % build binary image
            binaryImage = birdsEyeImage;
            for i = 1:h
                for j = 1:w      % consider only the current lane
                    if birdsEyeImage(i,j) <= th_new
                        binaryImage(i,j)=0;
                    else
                        binaryImage(i,j)=255;
                    end
                end
            end
   
            %% do lane recognition
            % Single window
            histogram = zeros(1,w);
            for i = 1:h
                for j = 1:w    
                    if binaryImage(i,j) ~= 0
                        histogram(1,j) = histogram(1,j)+1; 
                    end 
                end
            end
    
            figure(1)
            x = [1:length(histogram)];
            stem(x, histogram)
            
            % find the peak correspondings to lane
            [value, left_lane] = max(histogram(w/2-45:w/2));
            [value, right_lane] = max(histogram(w/2:w/2+45));
            left_lane = left_lane+w/2-45;
            right_lane = right_lane+w/2;
            % obj.left_lane_arr and obj.right_lane_arr are used as a buffer
            % so it should be filled with elements before used
            if obj.frame_counter <= length(obj.left_lane_arr)
                obj.left_lane_arr(obj.frame_counter) =  left_lane;
                obj.right_lane_arr(obj.frame_counter) = right_lane;
                obj.departure_history(obj.frame_counter) = obj.departure;

                obj.distance_left_lane(obj.frame_counter) = abs(left_lane-w/2);
                obj.distance_right_lane(obj.frame_counter) = abs(right_lane-w/2);
                obj.frame_counter = obj.frame_counter + 1;

            % When the buffer are filled then we treat the buffer as a circular register
            else
                % eliminate the first element of the vectors, representing
                % the value associated with the OLDEST frame obtained. In
                % this way we are creating circular arrays which updates
                % dynamically
                obj.left_lane_arr = obj.left_lane_arr(2:length(obj.left_lane_arr));
                obj.right_lane_arr = obj.right_lane_arr(2:length(obj.right_lane_arr));
                obj.distance_left_lane = obj.distance_left_lane(2:length(obj.distance_left_lane));
                obj.distance_right_lane = obj.distance_right_lane(2:length(obj.distance_right_lane));

                % insert the new value associated with the last frame
                obj.left_lane_arr = [obj.left_lane_arr, left_lane];
                obj.right_lane_arr = [obj.right_lane_arr, right_lane];
                obj.distance_left_lane = [obj.distance_left_lane, abs(left_lane-w/2)];
                obj.distance_right_lane = [obj.distance_right_lane , abs(right_lane-w/2)];

            end

            len_l = length(obj.distance_left_lane);
            len_r = length(obj.distance_right_lane);
            if (abs(obj.distance_left_lane(len_l) - obj.distance_right_lane(len_r) ) > 10) && (length(obj.distance_left_lane) >= 10)
                % implement the distance computation for the left lane
                avg_distance_l = mean(obj.distance_left_lane(len_l-14: len_l-7));
                last_distance_l = mean(obj.distance_left_lane(len_l-6: len_l));
                
                % implement the distance computation for the right lane
                avg_distance_r = mean(obj.distance_right_lane(len_r-14: len_r-7));
                last_distance_r = mean(obj.distance_right_lane(len_r-6: len_r));
                
                if (obj.departure ~= -1) && (abs(right_lane-w/2)<20) && (last_distance_r + 2 < avg_distance_r) 
                   %display("right departure");
                    curr_departure = +1;
                elseif  (obj.departure ~= +1) && (abs(left_lane-w/2)<20) && (last_distance_l + 2 < avg_distance_l)
                    %display("left departure");
                    curr_departure = -1;
                else 
                    curr_departure = 0;
                end
                % Structure which store the values of departure, the signal
                % that outputs the warning alarm
                obj.departure_history = obj.departure_history(2:length(obj.departure_history));
                obj.departure_history = [obj.departure_history, curr_departure];
                
            end

            if obj.frame_counter > length(obj.left_lane_arr)
                if sum(obj.departure_history(obj.frame_counter - length(obj.left_lane_arr) : obj.frame_counter - 1)) > 3
                    obj.departure = 1;

                elseif sum(obj.departure_history(obj.frame_counter - length(obj.left_lane_arr) : obj.frame_counter - 1)) < -3
                    obj.departure = -1;

                else
                    obj.departure = 0;
                end 
            end
            % Reject invalid lanes when lane tracker is enabled
            if obj.EnableLaneTracker
                [leftEgoBoundary,rightEgoBoundary] = rejectInvalidLanes(obj.LaneMarkerDetector,leftEgoBoundary,rightEgoBoundary);
                
            end
            % Pack lane boundaries to LaneSensor as expected by LaneFollowingDecisionLogicandControl
            % lanes = packLaneBoundaryDetections(obj,leftEgoBoundary,rightEgoBoundary);
            % display(lanes);
            % Display debugging windows in Normal simulation mode and when EnableDisplays is set.
            if(isempty(coder.target))
                if obj.EnableDisplays
                    displaySensorOutputs(obj, frame, leftEgoBoundary,rightEgoBoundary, false);
                end
            end
            
            
        end
        
        %------------------------------------------------------------------
        % packLaneBoundaryDetections method packs left and right lane
        % detections into a format expected by
        % LaneFollowingDecisionLogicandControl.
        function detections = packLaneBoundaryDetections(obj,leftEgoBoundary,rightEgoBoundary)
            %  Parameters of parabolicLaneBoundary object = [A B C]
            %  corresponds to the three coefficients of a second-degree
            %  polynomial equation:
            %                y = Ax^2 + Bx + C
            % Comparing this equation with lane model using 2nd order
            % polynomial approximation:
            %  y = (curvature/2)*(x^2) + (headingAngle)*x + lateralOffset
            %
            % This leads to the following relationship
            %   curvature           = 2 * A = 2 * Parameters(1)  (unit: 1/m)
            %   headingAngle        = B     = Parameters(2)      (unit: radians)
            %   lateralOffset       = C     = Parameters(3)      (unit: meters)
            %
            
            % Preallocate struct expected by controller
            DefaultLanesLeft = struct('Curvature',{single(obj.LaneMarkerDetector.DefaultLeftLaneParams(1))},...
                'CurvatureDerivative',{single(0)},...
                'HeadingAngle',{single(obj.LaneMarkerDetector.DefaultLeftLaneParams(2))},...
                'LateralOffset',{single(obj.LaneMarkerDetector.DefaultLeftLaneParams(3))},...
                'Strength',{single(0)},...
                'XExtent',{single([0,0])},...
                'BoundaryType',{LaneBoundaryType.Unmarked});
            
            DefaultLanesRight = struct('Curvature',{single(obj.LaneMarkerDetector.DefaultRightLaneParams(1))},...
                'CurvatureDerivative',{single(0)},...
                'HeadingAngle',{single(obj.LaneMarkerDetector.DefaultRightLaneParams(2))},...
                'LateralOffset',{single(obj.LaneMarkerDetector.DefaultRightLaneParams(3))},...
                'Strength',{single(0)},...
                'XExtent',{single([0,0])},...
                'BoundaryType',{LaneBoundaryType.Unmarked});
            
            field1 = 'Left'; field2 = 'Right';
            detections = struct(field1,DefaultLanesLeft,field2,DefaultLanesRight);                     
            
            % Pack left lane detections 
            detections.Left.Curvature(:)     = 2 * leftEgoBoundary.Parameters(1);
            detections.Left.HeadingAngle(:)  = leftEgoBoundary.Parameters(2); % Coordinate transform
            detections.Left.LateralOffset(:) = leftEgoBoundary.Parameters(3); % Coordinate transform
            detections.Left.Strength(:)      = leftEgoBoundary.Strength;
            detections.Left.XExtent(:)       = leftEgoBoundary.XExtent;
            detections.Left.BoundaryType(:)  = leftEgoBoundary.BoundaryType;
            
            % Pack right lane detections 
            detections.Right.Curvature(:)     = 2 * rightEgoBoundary.Parameters(1);
            detections.Right.HeadingAngle(:)  = rightEgoBoundary.Parameters(2); % Coordinate transform
            detections.Right.LateralOffset(:) = rightEgoBoundary.Parameters(3); % Coordinate transform
            detections.Right.Strength(:)      = rightEgoBoundary.Strength;
            detections.Right.XExtent(:)       = rightEgoBoundary.XExtent;
            detections.Right.BoundaryType(:)  = rightEgoBoundary.BoundaryType;

            % Shift detections to vehicle center as required by controller
            % Note: camera.PositionSim3d(1) represents the X mount location of the
            %       camera sensor with respect to the vehicle center
            if nnz(leftEgoBoundary.Parameters)
                detections.Left.LateralOffset(:) = polyval(...
                    leftEgoBoundary.Parameters, -obj.Camera.PositionSim3d(1));
                % Lane to left should always have positive lateral offset
                if detections.Left.LateralOffset < 0
                    detections.Left = DefaultLanesLeft;
                end
            end
            if nnz(rightEgoBoundary.Parameters)
                detections.Right.LateralOffset(:) = polyval(...
                    rightEgoBoundary.Parameters, -obj.Camera.PositionSim3d(1));
                % Lane to right should always have negative lateral offset
                if detections.Right.LateralOffset > 0
                    detections.Right = DefaultLanesRight;
                end
            end
        end
        
        %------------------------------------------------------------------
        % displaySensorOutputs method displays core information and
        % intermediate results from the monocular camera sensor simulation.
        % (obj, frame, leftEgoBoundary,rightEgoBoundary, false, lane_encoded)
        function isPlayerOpen = ...
                displaySensorOutputs(obj, frame, leftEgoBoundary,rightEgoBoundary, closePlayers) 
            sensor = obj.LaneMarkerDetector.Sensor;
            bottomOffset      = obj.LaneMarkerDetector.LaneDetectionRanges(1);
            distAheadOfSensor = obj.LaneMarkerDetector.LaneDetectionRanges(2);
            xVehiclePoints = bottomOffset:distAheadOfSensor;
            birdsEyeViewImage = obj.LaneMarkerDetector.BirdsEyeImage;
            birdsEyeConfig    = obj.LaneMarkerDetector.BirdsEyeConfig;
            birdsEyeViewBW    = obj.LaneMarkerDetector.BirdsEyeBW;
            if(~nnz(leftEgoBoundary.Parameters))
                leftEgoBoundary = parabolicLaneBoundary.empty;
            end
            if(~nnz(rightEgoBoundary.Parameters))
                rightEgoBoundary = parabolicLaneBoundary.empty;
            end
            birdsEyeWithOverlays = insertLaneBoundary(birdsEyeViewImage, leftEgoBoundary , birdsEyeConfig, xVehiclePoints, 'Color','Red');
            birdsEyeWithOverlays = insertLaneBoundary(birdsEyeWithOverlays, rightEgoBoundary, birdsEyeConfig, xVehiclePoints, 'Color','Green');
            
            frameWithOverlays = insertLaneBoundary(frame, leftEgoBoundary, sensor, xVehiclePoints, 'Color','Red');
            frameWithOverlays = insertLaneBoundary(frameWithOverlays, rightEgoBoundary, sensor, xVehiclePoints, 'Color','Green');
            
            imageROI = vehicleToImageROI(obj.LaneMarkerDetector);
            ROI = [imageROI(1) imageROI(3) imageROI(2)-imageROI(1) imageROI(4)-imageROI(3)];
            
            % Highlight candidate lane points that include outliers
            birdsEyeViewImage = insertShape(birdsEyeViewImage, 'rectangle', ROI); % show detection ROI
            birdsEyeViewImage = imoverlay(birdsEyeViewImage, birdsEyeViewBW, 'yellow');


            if obj.departure == 1
                 box_frame = insertText(frame, [340 120] ,'RIGHT DEPARTURE','FontSize',18,'BoxColor',...
                        'red','BoxOpacity',0.4,'TextColor','white');
            elseif obj.departure == -1
                box_frame = insertText(frame, [120 120] ,'LEFT DEPARTURE','FontSize',18,'BoxColor',...
                        'red','BoxOpacity',0.4,'TextColor','white');
            else
                 box_frame = insertText(frame, [240 120] ,'NO DEPARTURE','FontSize',18,'BoxColor',...
                        'green','BoxOpacity',0.4,'TextColor','white');
            end

            % Display the results
            frames = {frameWithOverlays, birdsEyeViewImage,  box_frame};        
            persistent players;
            if isempty(players)
                frameNames = {'Lane marker detections', 'Raw segmentation', 'Lane Departure Alarm'};
                players = helperVideoPlayerSet(frames, frameNames);
            end
            update(players, frames);
            
            % terminate the loop when the first player is closed
            isPlayerOpen = isOpen(players, 1);
            
            if (~isPlayerOpen || closePlayers) % close down the other players
                clear players;
            end
        end        
        
        function [lanes] = getOutputSizeImpl(obj) %#ok<MANU>
            % Return size for each output port
            lanes = 1;
        end
        
        function [lanes] = getOutputDataTypeImpl(obj) %#ok<MANU>
            % Return data type for each output port
            lanes = "LaneSensor";
        end
        
        function [lanes] = isOutputComplexImpl(obj) %#ok<MANU>
            % Return true for each output port with complex data
            lanes= false;
        end
        
        function [lanes] = isOutputFixedSizeImpl(obj) %#ok<MANU>
            % Return true for each output port with fixed size
            lanes = true;
        end
    end
    
    methods(Access = protected, Static)
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(....
                "Title","HelperLaneDetectorWrapper",...
                "Text",...
                "Detects lanes from camera image." + newline + newline +...
                "Enable display of debugging visualizations to show intermediate processing for lane detections.");
        end

        function flag = showSimulateUsingImpl
            % Return false if simulation mode hidden in System block dialog
            flag = true;
        end
    end
            
    
end
