%% 1. Define the time window for the pass prediction
startTime = datetime("now", "TimeZone", "UTC");
stopTime = startTime + days(1);
sampleTime = 1;

%% 2. Initialize the satellite scenario
scenario = satelliteScenario(startTime, stopTime, sampleTime);

%% 3. Load the Satellite from a TLE file
tleFile = "CANX-2.tle"; 
sat = satellite(scenario, tleFile, "Name", "CANX-2");

%% 4. Define a simple ground station in Vancouver 
gsLat = 49.2606;
gsLon = -123.2460;
gsMinElevationAngle = 10.0;
gs = groundStation(scenario, gsLat, gsLon, "Name", "UBC-MCLD", "MaskElevationAngle", gsMinElevationAngle);

%% 5. Compute the access between the satellite and the ground station
ac = access(sat, gs);

%% 6. Extract pass intervals, optional display in text and 3D visualization
passes = accessIntervals(ac);
%disp(passes);
%play(scenario);

%% 7. Calculate essential fields for each pass and save
% 1. Define RF parameters
fc = 437e6;
c = physconst('LightSpeed');
lambda = c / fc;

numPasses = height(passes);

folderTimeString = char(string(passes.StartTime(1), 'yyyyMMdd_HHmmss'));
foldername = sprintf('CANX2_Passes_%s', folderTimeString);
if ~exist(foldername, 'dir')
    mkdir(foldername);
end

for i = 1:numPasses
    tStart = passes.StartTime(i);
    tEnd = passes.EndTime(i);

    % Create a column vector for time at sampleTime resolution
    t = (tStart : seconds(sampleTime) : tEnd)'; 
    numSamples = length(t);

    % Preallocate output arrays for speed inside the loop
    az = zeros(numSamples, 1);
    el = zeros(numSamples, 1);
    r  = zeros(numSamples, 1);
    doppler_Hz = zeros(numSamples, 1);
    rel_velocity_mps = zeros(numSamples, 1);

    % 2. Inner loop for geometry and Doppler (requires scalar datetime)
    for j = 1:numSamples
        [az(j), el(j), r(j)] = aer(gs, sat, t(j));

        [doppler_Hz(j), ~, dopplerInfo] = dopplershift(sat, gs, t(j), "Frequency", fc);
        rel_velocity_mps(j) = dopplerInfo.RelativeVelocity;
    end

    % 3. Vectorized math for the derived RF parameters 
    delay = r / c; 
    pathLoss = 20 * log10(4 * pi * r / lambda); 

    % 4. Create the timetable for THIS pass
    currentPassTable = timetable(t, r, az, el, pathLoss, delay, doppler_Hz, rel_velocity_mps,...
        'VariableNames', {'Range_m', 'Azimuth_deg', 'Elevation_deg', ...
                          'PathLoss_dB', 'Delay_s', 'Doppler_Hz', 'Rel_Velocity_mps'});

    % 5. Generate a unique filename and save to CSV
    % Format the start time to be safe for filenames (removes spaces and colons)
    timeString = char(string(tStart, 'yyyyMMdd_HHmmss')); 
    filename = sprintf('CANX2_Pass_%d_%s.csv', i, timeString);

    % Write the timetable to the CSV file
    filepath = fullfile(foldername, filename);

    writetimetable(currentPassTable, filepath);

    fprintf('Saved: %s\n', filepath);
end

disp('All passes successfully exported to CSV files.');

%% 8. Plot one example pass
% fileInfo = dir('CANX2_Pass_2_*.csv');
% filename = fileInfo(1).name;
% fprintf('Loading data from: %s\n', filename);
% passData = readtimetable(filename);
% 
% figure('Name', 'Satellite Pass Metrics', 'Position', [100, 100, 900, 600]);
% 
% % --- Plot 1: Elevation Angle ---
% subplot(2, 2, 1);
% plot(passData.t, passData.Elevation_deg, 'LineWidth', 1.5, 'Color', '#0072BD');
% grid on;
% title('Elevation Angle');
% ylabel('Elevation (Degrees)');
% % A pass typically looks like a bell curve, peaking at the Time of Closest Approach (TCA)
% 
% % --- Plot 2: Range (Distance) ---
% subplot(2, 2, 2);
% % Converting meters to kilometers for readability
% plot(passData.t, passData.Range_m / 1000, 'LineWidth', 1.5, 'Color', '#D95319');
% grid on;
% title('Range (Distance to Ground Station)');
% ylabel('Range (km)');
% % Range will be at its minimum when Elevation is at its maximum
% 
% % --- Plot 3: Doppler Shift ---
% subplot(2, 2, 3);
% % Converting Hz to kHz for readability
% plot(passData.t, passData.Doppler_Hz / 1000, 'LineWidth', 1.5, 'Color', '#EDB120');
% grid on;
% title('Doppler Shift (437 MHz Carrier)');
% ylabel('Doppler (kHz)');
% % Doppler is positive as the satellite approaches, crosses zero at TCA, and becomes negative as it leaves
% 
% % --- Plot 4: Free Space Path Loss ---
% subplot(2, 2, 4);
% plot(passData.t, passData.PathLoss_dB, 'LineWidth', 1.5, 'Color', '#7E2F8E');
% grid on;
% title('Free Space Path Loss (FSPL)');
% ylabel('Loss (dB)');
% xlabel('Time (UTC)');
% 
% % Add a master title to the figure
% sgtitle(sprintf('Telemetry for %s', filename), 'Interpreter', 'none');
