function numPasses = runPassPrediction(options)
% RUNPASSPREDICTION Compute satellite passes and export RF link parameters to CSV.
%
%   This function wraps the full passPrediction workflow (satellite
%   scenario setup, access computation, geometry/Doppler calculation,
%   and CSV export) so it can be run as a single command, either from
%   the MATLAB command window or directly from an OS terminal via
%   "matlab -batch". 
%
%   USAGE (from the MATLAB command window):
%       numPasses = runPassPrediction();
%       numPasses = runPassPrediction(TLEFile="ISS.tle", SatName="ISS", ...
%                                      GSLat=49.2606, GSLon=-123.2460);
%
%   USAGE (from an OS terminal, no MATLAB GUI):
%       matlab -batch "runPassPrediction"
%       matlab -batch "runPassPrediction(TLEFile='CANX-2.tle', SampleTime=10)"
%
%   NAME-VALUE ARGUMENTS (all optional, defaults match the original script):
%       TLEFile         (string)  TLE file name.                 Default: "CANX-2.tle"
%       SatName         (string)  Display name of the satellite.  Default: "CANX-2"
%       GSLat           (double)  Ground station latitude (deg).  Default: 49.2606
%       GSLon           (double)  Ground station longitude (deg). Default: -123.2460
%       GSName          (string)  Ground station display name.    Default: "UBC-MCLD"
%       MinElevation    (double)  Elevation mask angle (deg).     Default: 10.0
%       Frequency       (double)  Carrier frequency (Hz).         Default: 435e6
%       DurationDays    (double)  Prediction window length (days).Default: 0.5
%       SampleTime      (double)  Scenario/geometry sample time (s). Default: 15
%       OutputDir       (string)  Folder where CSV files are saved. Default: pwd
%
%   OUTPUT:
%       numPasses       Number of passes found and exported.
%
%   Each pass is exported to a CSV file named:
%       <SatName>_Pass_<index>_<yyyyMMdd_HHmmss>.csv
%   containing Range_m, Azimuth_deg, Elevation_deg, PathLoss_dB, Delay_s,
%   Doppler_Hz, and Rel_Velocity_mps for that pass, sampled at SampleTime.

   arguments
    options.TLEFile      (1,1) string   = "CANX-2.tle"
    options.SatName      (1,1) string   = "CANX-2"
    options.StartTime    (1,1) datetime = datetime("now", "TimeZone", "UTC")   % edit from the GUI or pass directly
    options.GSLat        (1,1) double   = 49.2606
    options.GSLon        (1,1) double   = -123.2460
    options.GSName       (1,1) string   = "UBC-MCLD"
    options.MinElevation (1,1) double   = 10.0
    options.Frequency    (1,1) double   = 435e6 % refer to DCE system diagram 

    options.DurationDays (1,1) double   = 1     % edit with the passPrediction.mlx stopTime       
    options.SampleTime   (1,1) double   = 15
    options.OutputDir    (1,1) string   = string(pwd)
    options.ShowViewer   (1,1) logical  = true       % false = scenario is not displayed
end

    %% 1. Define the time window for the pass prediction
    startTime = options.StartTime;
    if isempty(startTime.TimeZone)
        % If a naive (timezone-less) datetime was passed in (e.g. built from
        % a date picker + a text time field in the GUI), assume UTC.
        startTime.TimeZone = "UTC";
    end
    stopTime   = startTime + days(options.DurationDays);
    sampleTime = options.SampleTime;

    %% 2. Initialize the satellite scenario
    scenario = satelliteScenario(startTime, stopTime, sampleTime);

    %% 3. Load the satellite from a TLE file
    if ~isfile(options.TLEFile)
        error("runPassPrediction:missingTLE", ...
            "TLE file not found: %s", options.TLEFile);
    end
    sat = satellite(scenario, options.TLEFile, "Name", options.SatName);

    %% 4. Define the ground station
    gs = groundStation(scenario, options.GSLat, options.GSLon, ...
        "Name", options.GSName, "MaskElevationAngle", options.MinElevation);

    %% 5. Compute the access between the satellite and the ground station
    ac = access(sat, gs);

    %% 6. Extract pass intervals
    passes = accessIntervals(ac);
    numPasses = height(passes);

    %% Scenario display
    if options.ShowViewer && batchStartupOptionUsed
    warning("ShowViewer is not supported in -batch mode. Skipping visualization.");
    options.ShowViewer = false;
    end

    if numPasses == 0
        fprintf("No passes found in the requested window.\n");
        return;
    end

    % %% 7. Create output folder
    % folderTimeString = char(string(passes.StartTime(1), 'yyyyMMdd_HHmmss'));
    % foldername = sprintf('CANX2_Passes_%s', folderTimeString);
    % if ~exist(foldername, 'dir')
    %     mkdir(foldername);
    % end

    %% 7. Create output folder structure
    folderTimeString = char(string(passes.StartTime(1), 'yyyyMMdd_HHmmss'));
    
    % Parent data folder
    dataFolder = fullfile(options.OutputDir, "CANX2 Data");
    
    % Individual prediction folder
    foldername = sprintf('CANX2_Passes_%s', folderTimeString);
    
    % Full path: data/CANX2_Passes_20260708_120000
    outputFolder = fullfile(dataFolder, foldername);
    
    % Create folders if they do not exist
    if ~exist(outputFolder, 'dir')
        mkdir(outputFolder);
    end


    %% 8. Calculate essential fields for each pass and save to CSV
    fc = options.Frequency;
    c  = physconst('LightSpeed');
    lambda = c / fc;

    for i = 1:numPasses
        tStart = passes.StartTime(i);
        tEnd   = passes.EndTime(i);

        t = (tStart : seconds(sampleTime) : tEnd)';
        numSamples = length(t);

        az = zeros(numSamples, 1);
        el = zeros(numSamples, 1);
        r  = zeros(numSamples, 1);
        doppler_Hz = zeros(numSamples, 1);
        rel_velocity_mps = zeros(numSamples, 1);

        for j = 1:numSamples
            [az(j), el(j), r(j)] = aer(gs, sat, t(j));
            [doppler_Hz(j), ~, dopplerInfo] = dopplershift(sat, gs, t(j), "Frequency", fc);
            rel_velocity_mps(j) = dopplerInfo.RelativeVelocity;
        end

        delay = r / c;
        pathLoss = 20 * log10(4 * pi * r / lambda);

        currentPassTable = timetable(t, r, az, el, pathLoss, delay, doppler_Hz, rel_velocity_mps, ...
            'VariableNames', {'Range_m', 'Azimuth_deg', 'Elevation_deg', ...
                              'PathLoss_dB', 'Delay_s', 'Doppler_Hz', 'Rel_Velocity_mps'});

        currentPassTable.t.Format = 'yyyy-MM-dd HH:mm:ss.SSS';

        timeString = char(string(tStart, 'yyyyMMdd_HHmmss'));
        filename = sprintf('%s_Pass_%d_%s.csv', options.SatName, i, timeString);

        % Write the timetable to the CSV file
        filepath = fullfile(outputFolder, filename);
        writetimetable(currentPassTable, filepath);
        fprintf('Saved: %s\n', filepath);
    end

    %% 9. Optional: open satellite scenario viewer 
    % viewer must be launched after all the resource-intensive calculations have been completed
    % so as not to freeze the animation
    if options.ShowViewer && ~batchStartupOptionUsed()
        v = satelliteScenarioViewer(scenario, "PlaybackSpeedMultiplier", 60);
        groundTrack(sat, "LeadTime", 3600);
        assignin('base', 'satViewer', v);
        assignin('base', 'satScenario', scenario);
        play(scenario);
        drawnow;
    end

    fprintf('All %d pass(es) successfully exported to %s.\n', numPasses,outputFolder);
end

%[appendix]{"version":"1.0"}
%---
