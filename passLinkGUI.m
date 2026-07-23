function passLinkGUI
%PASSLINKGUI Control panel for CANX-2 pass prediction and hardware link playback.
%
%      Launch the window using:  passLinkGUI
%
%   'Pass Prediction' tab   -> calls runPassPrediction.m
%   'Hardware Link' tab     -> writes the toggles to the
%                                  'base' workspace and then runs hardwareLink.m
%
%   Place this file in the same folder (or on the MATLAB path) as
%   runPassPrediction.m and hardwareLink.m.

    %% ---------------- Figure & Tabs ----------------
    fig = uifigure('Name', 'CANX-2 DCE Control Panel', 'Position', [100 100 950 720]);
    tg  = uitabgroup(fig, 'Position', [10 10 930 700]);

    tab1 = uitab(tg, 'Title', 'Pass Prediction');
    tab2 = uitab(tg, 'Title', 'Hardware Link Playback');

    %% ================= TAB 1: Pass Prediction =================
    g1 = uigridlayout(tab1);
    g1.RowHeight   = [repmat({32}, 1, 14), {160}];
    g1.ColumnWidth = {200, '1x', 110};
    g1.RowSpacing = 6;

    row = 0;

    % --- TLE file ---
    row = row + 1;
    uilabel(g1, 'Text', 'TLE File:');
    tleField = uieditfield(g1, 'text', 'Value', 'CANX-2.tle', 'Editable', 'off');
    tleField.Layout.Row = row; tleField.Layout.Column = 2;
    tleBtn = uibutton(g1, 'Text', 'Browse...');
    tleBtn.Layout.Row = row; tleBtn.Layout.Column = 3;
    tleBtn.ButtonPushedFcn = @(~,~) browseTLE();

    % --- Satellite name ---
    row = row + 1;
    uilabel(g1, 'Text', 'Satellite Name:');
    satNameField = uieditfield(g1, 'text', 'Value', 'CANX-2');
    satNameField.Layout.Row = row; satNameField.Layout.Column = [2 3];

    % --- Start date ---
    row = row + 1;
    uilabel(g1, 'Text', 'Start Date (UTC):');
    startDatePicker = uidatepicker(g1, 'Value', datetime('today'));
    startDatePicker.Layout.Row = row; startDatePicker.Layout.Column = [2 3];

    % --- Start time ---
    row = row + 1;
    uilabel(g1, 'Text', 'Start Time HH:mm:ss (UTC):');
    startTimeField = uieditfield(g1, 'text', ...
        'Value', string(datetime('now', 'TimeZone', 'UTC'), 'HH:mm:ss'));
    startTimeField.Layout.Row = row; startTimeField.Layout.Column = [2 3];

    % --- Duration ---
    row = row + 1;
    uilabel(g1, 'Text', 'Duration (days):');
    durationField = uieditfield(g1, 'numeric', 'Value', 1, 'Limits', [0.01 30]);
    durationField.Layout.Row = row; durationField.Layout.Column = [2 3];

    % --- Sample time ---
    row = row + 1;
    uilabel(g1, 'Text', 'Sample Time (s):');
    sampleTimeField = uieditfield(g1, 'numeric', 'Value', 15, 'Limits', [1 3600]);
    sampleTimeField.Layout.Row = row; sampleTimeField.Layout.Column = [2 3];

    % --- GS name ---
    row = row + 1;
    uilabel(g1, 'Text', 'Ground Station Name:');
    gsNameField = uieditfield(g1, 'text', 'Value', 'UBC-MCLD');
    gsNameField.Layout.Row = row; gsNameField.Layout.Column = [2 3];

    % --- GS latitude ---
    row = row + 1;
    uilabel(g1, 'Text', 'GS Latitude (deg):');
    gsLatField = uieditfield(g1, 'numeric', 'Value', 49.2606, 'Limits', [-90 90]);
    gsLatField.Layout.Row = row; gsLatField.Layout.Column = [2 3];

    % --- GS longitude ---
    row = row + 1;
    uilabel(g1, 'Text', 'GS Longitude (deg):');
    gsLonField = uieditfield(g1, 'numeric', 'Value', -123.2460, 'Limits', [-180 180]);
    gsLonField.Layout.Row = row; gsLonField.Layout.Column = [2 3];

    % --- Min elevation ---
    row = row + 1;
    uilabel(g1, 'Text', 'Min Elevation Angle (deg):');
    minElevField = uieditfield(g1, 'numeric', 'Value', 10.0, 'Limits', [0 90]);
    minElevField.Layout.Row = row; minElevField.Layout.Column = [2 3];

    % --- Carrier frequency ---
    row = row + 1;
    uilabel(g1, 'Text', 'Carrier Frequency (MHz):');
    freqField = uieditfield(g1, 'numeric', 'Value', 435, 'Limits', [1 1e5]);
    freqField.Layout.Row = row; freqField.Layout.Column = [2 3];

    % --- Show 3D viewer ---
    row = row + 1;
    uilabel(g1, 'Text', '3D Scenario Viewer:');
    showViewerSwitch = uiswitch(g1, 'slider', 'Items', {'Off', 'On'}, 'Value', 'On');
    showViewerSwitch.Layout.Row = row; showViewerSwitch.Layout.Column = 2;

    % --- Output folder ---
    row = row + 1;
    uilabel(g1, 'Text', 'Output Folder:');
    outputDirField = uieditfield(g1, 'text', 'Value', pwd, 'Editable', 'off');
    outputDirField.Layout.Row = row; outputDirField.Layout.Column = 2;
    outDirBtn = uibutton(g1, 'Text', 'Browse...');
    outDirBtn.Layout.Row = row; outDirBtn.Layout.Column = 3;
    outDirBtn.ButtonPushedFcn = @(~,~) browseOutputDir();

    % --- Run button ---
    row = row + 1;
    runBtn1 = uibutton(g1, 'Text', 'Run Pass Prediction', 'FontWeight', 'bold');
    runBtn1.Layout.Row = row; runBtn1.Layout.Column = [1 3];
    runBtn1.ButtonPushedFcn = @(~,~) runPredictionCallback();

    % --- Log area ---
    row = row + 1;
    logArea1 = uitextarea(g1, 'Value', {'Ready.'}, 'Editable', 'off');
    logArea1.Layout.Row = row; logArea1.Layout.Column = [1 3];

    %% ================= TAB 2: Hardware Link Playback =================
    g2 = uigridlayout(tab2);
    g2.RowHeight   = [repmat({32}, 1, 6), {1}, {32}, {260}];
    g2.ColumnWidth = {20, '1x'};
    g2.RowSpacing = 6;

    infoLabel = uilabel(g2, 'Text', ...
        ['These options enable/disable the sub-plots displayed in real-time' ...
        'by hardwareLink.m, which open in a separate window. The script requires' ...
        'a programmable attenuator and a SDR to be connected.']);
    infoLabel.WordWrap = 'on';
    infoLabel.Layout.Row = 1; infoLabel.Layout.Column = [1 2];

    rangePlotCheck = uicheckbox(g2, 'Text', 'Show Range Plot', 'Value', true);
    rangePlotCheck.Layout.Row = 2; rangePlotCheck.Layout.Column = [1 2];

    pathLossPlotCheck = uicheckbox(g2, 'Text', 'Show Path Loss Plot', 'Value', true);
    pathLossPlotCheck.Layout.Row = 3; pathLossPlotCheck.Layout.Column = [1 2];

    delayPlotCheck = uicheckbox(g2, 'Text', 'Show Delay Plot', 'Value', true);
    delayPlotCheck.Layout.Row = 4; delayPlotCheck.Layout.Column = [1 2];

    dopplerPlotCheck = uicheckbox(g2, 'Text', 'Show Doppler Plot', 'Value', true);
    dopplerPlotCheck.Layout.Row = 5; dopplerPlotCheck.Layout.Column = [1 2];

    tumbleCheck = uicheckbox(g2, 'Text', 'Enable Tumbling Simulation (tumbling_attenuation.m)', 'Value', false);
    tumbleCheck.Layout.Row = 6; tumbleCheck.Layout.Column = [1 2];
    tumbleCheck.ValueChangedFcn = @(~,~) toggleTumblePanel();

    % --- Tumbling options sub-panel: indented (column 2 only, leaving a
    % 20px gap in column 1) and shown/hidden when the checkbox toggles ---
    tumblePanel = uipanel(g2, 'Title', 'Tumbling Options', 'Visible', 'off');
    tumblePanel.Layout.Row = 7; tumblePanel.Layout.Column = 2;

    tp = uigridlayout(tumblePanel);
    tp.RowHeight   = repmat({28}, 1, 7);
    tp.ColumnWidth = {150, '1x'};
    tp.RowSpacing  = 4;

    uilabel(tp, 'Text', 'Test Case:');
    testCaseDropdown = uidropdown(tp, ...
        'Items', {'stable', 'drift', 'deployment', 'end-over-end', 'extreme'}, ...
        'Value', 'stable');
    testCaseDropdown.Layout.Row = 1; testCaseDropdown.Layout.Column = 2;

    uilabel(tp, 'Text', 'Dimensions X,Y,Z (m):');
    dimGrid = uigridlayout(tp, [1 3]);
    dimGrid.Layout.Row = 2; dimGrid.Layout.Column = 2;
    dimGrid.Padding = [0 0 0 0]; dimGrid.ColumnSpacing = 4;
    dimXField = uieditfield(dimGrid, 'numeric', 'Value', 0.1, 'Limits', [0.001 10]);
    dimYField = uieditfield(dimGrid, 'numeric', 'Value', 0.1, 'Limits', [0.001 10]);
    dimZField = uieditfield(dimGrid, 'numeric', 'Value', 0.3, 'Limits', [0.001 10]);

    uilabel(tp, 'Text', 'Mass (kg):');
    massField = uieditfield(tp, 'numeric', 'Value', 4, 'Limits', [0.01 1000]);
    massField.Layout.Row = 3; massField.Layout.Column = 2;

    uilabel(tp, 'Text', 'Antenna Type:');
    antennaTypeDropdown = uidropdown(tp, ...
        'Items', {'Half-Wave Dipole', 'Quarter-Wave Monopole', 'Dish'}, ...
        'Value', 'Half-Wave Dipole');
    antennaTypeDropdown.Layout.Row = 4; antennaTypeDropdown.Layout.Column = 2;
    antennaTypeDropdown.ValueChangedFcn = @(~,~) updateDishRadiusState();

    uilabel(tp, 'Text', 'Antenna Orientation:');
    antennaOrientationDropdown = uidropdown(tp, ...
        'Items', {'+X', '-X', '+Y', '-Y', '+Z', '-Z'}, 'Value', '+X');
    antennaOrientationDropdown.Layout.Row = 5; antennaOrientationDropdown.Layout.Column = 2;

    dishRadiusLabel = uilabel(tp, 'Text', 'Dish Radius (m):', 'Enable', 'off');
    dishRadiusLabel.Layout.Row = 6; dishRadiusLabel.Layout.Column = 1;
    dishRadiusField = uieditfield(tp, 'numeric', 'Value', 0.05, 'Limits', [0.001 10], 'Enable', 'off');
    dishRadiusField.Layout.Row = 6; dishRadiusField.Layout.Column = 2;

    tumbleShowPlotsCheck = uicheckbox(tp, 'Text', 'Show Tumbling Plots (pointing error / loss)', 'Value', false);
    tumbleShowPlotsCheck.Layout.Row = 7; tumbleShowPlotsCheck.Layout.Column = [1 2];

    runBtn2 = uibutton(g2, 'Text', 'Run Hardware Link', 'FontWeight', 'bold');
    runBtn2.Layout.Row = 8; runBtn2.Layout.Column = [1 2];
    runBtn2.ButtonPushedFcn = @(~,~) runHardwareLinkCallback();

    logArea2 = uitextarea(g2, 'Value', {'Ready.'}, 'Editable', 'off');
    logArea2.Layout.Row = 9; logArea2.Layout.Column = [1 2];

    %% ================= Callbacks (nested functions) =================

    function browseTLE()
        [f, p] = uigetfile({'*.tle', 'TLE files (*.tle)'}, 'Select a TLE file');
        if isequal(f, 0)
            return;
        end
        tleField.Value = fullfile(p, f);
    end

    function browseOutputDir()
        d = uigetdir(outputDirField.Value, 'Select output folder');
        if isequal(d, 0)
            return;
        end
        outputDirField.Value = d;
    end

    function runPredictionCallback()
        try
            % Build the UTC start time from the date picker + text field
            timeParts = split(strtrim(startTimeField.Value), ':');
            if numel(timeParts) ~= 3
                uialert(fig, 'The Start Time field must be set to the following format HH:mm:ss.', 'Invalid input');
                return;
            end
            hh = str2double(timeParts(1));
            mm = str2double(timeParts(2));
            ss = str2double(timeParts(3));
            if any(isnan([hh mm ss]))
                uialert(fig, 'The Start Time field must be set to the following format HH:mm:ss.', 'Invalid input');
                return;
            end

            startDateTime = datetime(startDatePicker.Value, 'TimeZone', 'UTC') + ...
                hours(hh) + minutes(mm) + seconds(ss);

            appendLog(logArea1, 'Launch of runPassPrediction...');
            drawnow;

            numPasses = runPassPrediction( ...
                "TLEFile",      tleField.Value, ...
                "SatName",      satNameField.Value, ...
                "StartTime",    startDateTime, ...
                "GSLat",        gsLatField.Value, ...
                "GSLon",        gsLonField.Value, ...
                "GSName",       gsNameField.Value, ...
                "MinElevation", minElevField.Value, ...
                "Frequency",    freqField.Value * 1e6, ...
                "DurationDays", durationField.Value, ...
                "SampleTime",   sampleTimeField.Value, ...
                "OutputDir",    outputDirField.Value, ...
                "ShowViewer",   strcmp(showViewerSwitch.Value, 'On'));

            appendLog(logArea1, sprintf('Done : %d pass(es) found and exported.', numPasses));
        catch ME
            appendLog(logArea1, sprintf('ERROR : %s', ME.message));
        end
    end

    function toggleTumblePanel()
        if tumbleCheck.Value
            tumblePanel.Visible = 'on';
            g2.RowHeight{7} = 230;
        else
            tumblePanel.Visible = 'off';
            g2.RowHeight{7} = 1;
        end
    end

    function updateDishRadiusState()
        if strcmp(antennaTypeDropdown.Value, 'Dish')
            dishRadiusLabel.Enable = 'on';
            dishRadiusField.Enable = 'on';
        else
            dishRadiusLabel.Enable = 'off';
            dishRadiusField.Enable = 'off';
        end
    end

    function runHardwareLinkCallback()
        try
            % hardwareLink.m is a script: write the plot toggles into the
            % base workspace, then run the script there so it can see them.
            assignin('base', 'showRangePlot',     rangePlotCheck.Value);
            assignin('base', 'showPathLossPlot',  pathLossPlotCheck.Value);
            assignin('base', 'showDelayPlot',     delayPlotCheck.Value);
            assignin('base', 'showDopplerPlot',   dopplerPlotCheck.Value);
            assignin('base', 'enableTumbleToggle', tumbleCheck.Value);

            % Tumbling sub-parameters (only used by hardwareLink.m if
            % enableTumbleToggle is true, but always sent so the script
            % has sensible values to read even on the first run).
            assignin('base', 'tumbleTestCase',           testCaseDropdown.Value);
            assignin('base', 'tumbleSatDimensions',      [dimXField.Value, dimYField.Value, dimZField.Value]);
            assignin('base', 'tumbleMass',               massField.Value);
            assignin('base', 'tumbleAntennaType',        antennaTypeDropdown.Value);
            assignin('base', 'tumbleAntennaOrientation', antennaOrientationDropdown.Value);
            assignin('base', 'tumbleDishRadius',         dishRadiusField.Value);
            assignin('base', 'tumbleShowPlots',          tumbleShowPlotsCheck.Value);

            appendLog(logArea2, 'Launch of hardwareLink.m...');
            drawnow;
            
            evalin('base', 'hardwareLink_3');
            
            appendLog(logArea2, 'hardwareLink_3.m done.');
        catch ME
            appendLog(logArea2, sprintf('ERROR : %s', ME.message));
        end
    end

    function appendLog(area, msg)
        existing = area.Value;
        if ischar(existing)
            existing = {existing};
        end
        area.Value = [existing; {char(msg)}];
        scroll(area, 'bottom');
    end

end