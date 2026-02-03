classdef ObjectTracking_code < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                 matlab.ui.Figure
        ResetTrackingButton      matlab.ui.control.Button
        StopTrackingButton       matlab.ui.control.Button
        StartTrackingButton      matlab.ui.control.Button
        AddObjectButtonPushed    matlab.ui.control.Button
        ClearObjectsButton       matlab.ui.control.Button
        SelectObjectButton       matlab.ui.control.Button
        StopRecordingButton      matlab.ui.control.Button
        OpenWebcamButton         matlab.ui.control.Button
        RecordWebcamButton       matlab.ui.control.Button
        LoadSampleVideoButton_2  matlab.ui.control.Button
        LoadImageSequenceButton  matlab.ui.control.Button
        UIAxes                   matlab.ui.control.UIAxes
    end


    properties (Access = public)
        OriginalImage
        ProcessedImage
        SecondImage   
    end
    
    properties (Access = private)
        CurrentTemplateData
        CornerROI = []; 
        EdgeLiveMode = false;
        DOGLOG_BaseImage 
        DOGLOG_LiveMode = false; 
        VideoSource  
        ImageFiles
        ImageIndex
        ImageFolder
        VideoObj
        CurrentFrame
        FrameTimer
        BBoxes = {}
        ROIs = {}          
        Trackers = {}
        IsTracking = false;           
        TrackingTimer                 
        TrackedObjects = {};
        IsRecording = false;
        WriterObj


    end


    methods (Access = private)

        

        function imgIn = currentInput(app)
            if ~isempty(app.ProcessedImage)
                imgIn = app.ProcessedImage;
                return;
            elseif ~isempty(app.OriginalImage)
                imgIn = app.OriginalImage;
                return;
            end
        
            if ~isempty(app.CurrentFrame)
                imgIn = app.CurrentFrame;
            else
                imgIn = [];
            end
        end
                
        function showProcessed(app, img)
            app.ProcessedImage = img;
            imshow(img, 'Parent', app.UIAxes_2);
        end
    
        function captureFrame(app)
            frame = snapshot(app.VideoObj);
            app.CurrentFrame = frame;
        
            % Fast display (better than imshow every time)
            persistent hImg
            if isempty(hImg) || ~isvalid(hImg)
                cla(app.UIAxes);
                hImg = imshow(frame, 'Parent', app.UIAxes);
            else
                hImg.CData = frame;
            end
            drawnow limitrate
        
            % ✅ If recording, write the SAME frames being displayed
            if app.IsRecording && ~isempty(app.WriterObj)
                writeVideo(app.WriterObj, frame);
            end
        end

        function performTracking(app)
        % Get next frame
            if isempty(app.VideoSource)
                % Static image - track once and stop
                app.CurrentFrame = frame;
                frame = app.currentInput();   % ← USE EDITED OUTPUT
                app.updateAndVisualize(frame);

                app.stopTracking();
                return;
            end
            switch app.VideoSource
                case 'video'
                    if ~hasFrame(app.VideoObj), app.stopTracking(); return; end
                    frame = readFrame(app.VideoObj);
                case 'webcam'
                    frame = snapshot(app.VideoObj);
                case 'imageseq'
                    app.ImageIndex = app.ImageIndex + 1;
                    if app.ImageIndex > length(app.ImageFiles)
                        app.stopTracking(); return;
                    end
                    frame = imread(fullfile(app.ImageFolder, app.ImageFiles(app.ImageIndex).name));
            end
            
            app.CurrentFrame = frame;
            app.updateAndVisualize(frame);
        end

        function updateAndVisualize(app, frame)
            % Track all objects
            for i = 1:length(app.TrackedObjects)
                obj = app.TrackedObjects{i};
                if ~obj.isValid, continue; end
                
                % Track points
                [pts, valid] = step(obj.tracker, frame);
                validPts = pts(valid, :);
                
                if size(validPts, 1) < 3
                    app.TrackedObjects{i}.isValid = false;
                    continue;
                end
                
                % Update bbox using geometric transform
                try
                    oldPts = obj.initialPoints(valid, :);
                    tform = estimateGeometricTransform2D(oldPts, validPts, 'similarity', 'MaxDistance', 4);
                    
                    bbox = obj.bbox;
                    corners = [bbox(1), bbox(2);
                              bbox(1)+bbox(3), bbox(2);
                              bbox(1)+bbox(3), bbox(2)+bbox(4);
                              bbox(1), bbox(2)+bbox(4)];
                    newCorners = transformPointsForward(tform, corners);
                    
                    app.TrackedObjects{i}.bbox = [min(newCorners(:,1)), min(newCorners(:,2)), ...
                                                  range(newCorners(:,1)), range(newCorners(:,2))];
                    
                    setPoints(obj.tracker, validPts);
                    app.TrackedObjects{i}.initialPoints = validPts;
                catch
                    app.TrackedObjects{i}.isValid = false;
                end
            end
            
            imshow(frame, 'Parent', app.UIAxes);
            hold(app.UIAxes, 'on');
            for i = 1:length(app.TrackedObjects)
                obj = app.TrackedObjects{i};
                if obj.isValid
                    rectangle(app.UIAxes, 'Position', obj.bbox, 'EdgeColor', obj.color, 'LineWidth', 2);
                    text(app.UIAxes, obj.bbox(1), obj.bbox(2)-10, sprintf('Object %d', i), ...
                        'Color', obj.color, 'FontWeight', 'bold', 'BackgroundColor', [0 0 0 0.5]);
                end
            end
            hold(app.UIAxes, 'off');
            drawnow;
        end

        function stopTracking(app)
                app.IsTracking = false;
                if ~isempty(app.TrackingTimer) && isvalid(app.TrackingTimer)
                    stop(app.TrackingTimer);
                    delete(app.TrackingTimer);
                end
           end
        end
    

    % Callbacks that handle component events
    methods (Access = private)

       

        % Button pushed function: LoadSampleVideoButton_2
        function LoadSampleVideoButton_2Pushed(app, event)
            [file, path] = uigetfile({'*.mp4;*.avi'}, 'Select Video');
            if isequal(file,0); return; end
        
            app.VideoObj = VideoReader(fullfile(path,file));
            app.VideoSource = 'video';
        
            app.CurrentFrame = readFrame(app.VideoObj);
            imshow(app.CurrentFrame, 'Parent', app.UIAxes);
        end

        % Button pushed function: LoadImageSequenceButton
        function LoadImageSequenceButtonPushed(app, event)
            folder = uigetdir;
            if folder == 0; return; end
        
            files = dir(fullfile(folder,'*.jpg'));
            app.VideoSource = 'imageseq';
            app.ImageFiles = files;
            app.ImageIndex = 1;
            app.ImageFolder = folder;
        
            img = imread(fullfile(folder, files(1).name));
            app.CurrentFrame = img;
            imshow(img, 'Parent', app.UIAxes);
        end

        % Button pushed function: OpenWebcamButton
        function OpenWebcamButtonPushed(app, event)
             app.VideoObj = webcam;
            app.VideoSource = 'webcam';
        
            app.FrameTimer = timer( ...
                'ExecutionMode','fixedRate', ...
                'Period',0.05, ...
                'TimerFcn',@(~,~)captureFrame(app));
        
            start(app.FrameTimer);
        end

        % Button pushed function: RecordWebcamButton
        function RecordWebcamButtonPushed(app, event)
            if isempty(app.VideoObj) || ~isa(app.VideoObj,'webcam')
                uialert(app.UIFigure,'Open Webcam first.','Webcam Not Ready');
                return;
            end
        
            % Create writer once
            app.WriterObj = VideoWriter('recorded.avi','Motion JPEG AVI');
            app.WriterObj.FrameRate = 20;   % match your timer-ish speed
            open(app.WriterObj);
        
            app.IsRecording = true; 
        end

        % Callback function
        function StopRecordingButtonPushed(app, event)
                app.IsRecording = false;

    if ~isempty(app.WriterObj)
        close(app.WriterObj);
        app.WriterObj = [];
    end

        end

        % Button pushed function: StopRecordingButton
        function StopRecordingButtonPushed2(app, event)
            
        end

        % Button pushed function: SelectObjectButton
        function SelectObjectButtonPushed(app, event)
            frame = app.currentInput();
            if isempty(frame)
                uialert(app.UIFigure,'Load input first','Error');
                return;
            end
                
            imshow(frame,'Parent',app.UIAxes);
            roi = drawrectangle(app.UIAxes,'Color','cyan');
            wait(roi);
            bbox = roi.Position;
            delete(roi);
                
            gray = rgb2gray(frame);
            points = detectMinEigenFeatures(gray,'ROI',bbox);
        
            % ===== ADD START =====
            if points.Count < 5
                uialert(app.UIFigure,'Not enough features','Warning');
                return;
            end
        
            tracker = vision.PointTracker('MaxBidirectionalError',2);
            initialize(tracker, points.Location, frame);
        
            obj = struct( ...
                'tracker', tracker, ...
                'bbox', bbox, ...
                'initialPoints', points.Location, ...
                'color', rand(1,3), ...
                'isValid', true);
        
            app.TrackedObjects{end+1} = obj;

        end

        % Button pushed function: AddObjectButtonPushed
        function AddObjectButtonPushedPushed(app, event)
            SelectObjectButtonPushed(app, event);
        end

        % Button pushed function: ClearObjectsButton
        function ClearObjectsButtonPushed(app, event)
           app.BBoxes = {};
            app.ROIs = {};
            app.Trackers = {};
            app.TrackedObjects = {};   % ← REQUIRED
        
            if ~isempty(app.CurrentFrame)
                imshow(app.CurrentFrame,'Parent',app.UIAxes);
            end
        end

        % Button pushed function: StartTrackingButton
        function StartTrackingButtonPushed(app, event)
            if isempty(app.TrackedObjects)
                uialert(app.UIFigure,'Select objects first','No Objects');
                return;
            end
        
            app.IsTracking = true;
        
            % IMAGE → run once
            if isempty(app.VideoSource)
                app.performTracking();
                return;
            end
        
            % VIDEO / WEBCAM / IMAGE SEQUENCE
            app.TrackingTimer = timer( ...
                'ExecutionMode','fixedRate', ...
                'Period',0.02, ...
                'TimerFcn', @(~,~) app.performTracking());
        
            start(app.TrackingTimer);
        end

        % Button pushed function: StopTrackingButton
        function StopTrackingButtonPushed(app, event)
            app.IsTracking = false;
            
                if ~isempty(app.TrackingTimer) && isvalid(app.TrackingTimer)
                    stop(app.TrackingTimer);
                    delete(app.TrackingTimer);
                end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Color = [0.8196 0.8902 0.9412];
            app.UIFigure.Position = [100 100 1078 640];
            app.UIFigure.Name = 'MATLAB App';

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            title(app.UIAxes, 'Original Image')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.Position = [286 193 510 352];

            % Create LoadImageSequenceButton
            app.LoadImageSequenceButton = uibutton(app.UIFigure, 'push');
            app.LoadImageSequenceButton.ButtonPushedFcn = createCallbackFcn(app, @LoadImageSequenceButtonPushed, true);
            app.LoadImageSequenceButton.FontSize = 16;
            app.LoadImageSequenceButton.FontWeight = 'bold';
            app.LoadImageSequenceButton.Position = [44 413 184 29];
            app.LoadImageSequenceButton.Text = 'Load Image Sequence';

            % Create LoadSampleVideoButton_2
            app.LoadSampleVideoButton_2 = uibutton(app.UIFigure, 'push');
            app.LoadSampleVideoButton_2.ButtonPushedFcn = createCallbackFcn(app, @LoadSampleVideoButton_2Pushed, true);
            app.LoadSampleVideoButton_2.FontSize = 16;
            app.LoadSampleVideoButton_2.FontWeight = 'bold';
            app.LoadSampleVideoButton_2.Position = [44 468 162 29];
            app.LoadSampleVideoButton_2.Text = 'Load Sample Video';

            % Create RecordWebcamButton
            app.RecordWebcamButton = uibutton(app.UIFigure, 'push');
            app.RecordWebcamButton.ButtonPushedFcn = createCallbackFcn(app, @RecordWebcamButtonPushed, true);
            app.RecordWebcamButton.FontSize = 16;
            app.RecordWebcamButton.FontWeight = 'bold';
            app.RecordWebcamButton.Position = [224 75 163 29];
            app.RecordWebcamButton.Text = 'Record Webcam';

            % Create OpenWebcamButton
            app.OpenWebcamButton = uibutton(app.UIFigure, 'push');
            app.OpenWebcamButton.ButtonPushedFcn = createCallbackFcn(app, @OpenWebcamButtonPushed, true);
            app.OpenWebcamButton.FontSize = 16;
            app.OpenWebcamButton.FontWeight = 'bold';
            app.OpenWebcamButton.Position = [701 75 162 29];
            app.OpenWebcamButton.Text = 'Open Webcam';

            % Create StopRecordingButton
            app.StopRecordingButton = uibutton(app.UIFigure, 'push');
            app.StopRecordingButton.ButtonPushedFcn = createCallbackFcn(app, @StopRecordingButtonPushed2, true);
            app.StopRecordingButton.FontSize = 16;
            app.StopRecordingButton.FontWeight = 'bold';
            app.StopRecordingButton.Position = [460 75 162 29];
            app.StopRecordingButton.Text = 'Stop Recording ';

            % Create SelectObjectButton
            app.SelectObjectButton = uibutton(app.UIFigure, 'push');
            app.SelectObjectButton.ButtonPushedFcn = createCallbackFcn(app, @SelectObjectButtonPushed, true);
            app.SelectObjectButton.FontSize = 16;
            app.SelectObjectButton.FontWeight = 'bold';
            app.SelectObjectButton.Position = [47 307 116 28];
            app.SelectObjectButton.Text = 'Select Object';

            % Create ClearObjectsButton
            app.ClearObjectsButton = uibutton(app.UIFigure, 'push');
            app.ClearObjectsButton.ButtonPushedFcn = createCallbackFcn(app, @ClearObjectsButtonPushed, true);
            app.ClearObjectsButton.FontSize = 16;
            app.ClearObjectsButton.FontWeight = 'bold';
            app.ClearObjectsButton.Position = [44 167 118 28];
            app.ClearObjectsButton.Text = 'Clear Objects';

            % Create AddObjectButtonPushed
            app.AddObjectButtonPushed = uibutton(app.UIFigure, 'push');
            app.AddObjectButtonPushed.ButtonPushedFcn = createCallbackFcn(app, @AddObjectButtonPushedPushed, true);
            app.AddObjectButtonPushed.FontSize = 16;
            app.AddObjectButtonPushed.FontWeight = 'bold';
            app.AddObjectButtonPushed.Position = [44 234 166 28];
            app.AddObjectButtonPushed.Text = 'Add Another Object';

            % Create StartTrackingButton
            app.StartTrackingButton = uibutton(app.UIFigure, 'push');
            app.StartTrackingButton.ButtonPushedFcn = createCallbackFcn(app, @StartTrackingButtonPushed, true);
            app.StartTrackingButton.FontSize = 16;
            app.StartTrackingButton.FontWeight = 'bold';
            app.StartTrackingButton.Position = [884 262 122 28];
            app.StartTrackingButton.Text = 'Start Tracking';

            % Create StopTrackingButton
            app.StopTrackingButton = uibutton(app.UIFigure, 'push');
            app.StopTrackingButton.ButtonPushedFcn = createCallbackFcn(app, @StopTrackingButtonPushed, true);
            app.StopTrackingButton.FontSize = 16;
            app.StopTrackingButton.FontWeight = 'bold';
            app.StopTrackingButton.Position = [884 420 121 28];
            app.StopTrackingButton.Text = 'Stop Tracking';

            % Create ResetTrackingButton
            app.ResetTrackingButton = uibutton(app.UIFigure, 'push');
            app.ResetTrackingButton.FontSize = 16;
            app.ResetTrackingButton.FontWeight = 'bold';
            app.ResetTrackingButton.Position = [877 334 129 28];
            app.ResetTrackingButton.Text = 'Reset Tracking';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = ObjectTracking_code

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end