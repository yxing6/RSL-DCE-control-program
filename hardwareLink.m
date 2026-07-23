%% Programmable Attenuator and SDR Link

% --- Plot visibility toggles -------------------------------------------
% These can be pre-set in the base workspace by passLinkGUI.m before this
% script is run (e.g. assignin('base','showRangePlot',false)). If they are
% not already defined (e.g. running this script directly), they default
% to true so the script behaves exactly as before.
if ~exist('showRangePlot',    'var'), showRangePlot    = true; end
if ~exist('showPathLossPlot', 'var'), showPathLossPlot = true; end
if ~exist('showDelayPlot',    'var'), showDelayPlot    = true; end
if ~exist('showDopplerPlot',  'var'), showDopplerPlot  = true; end
if ~exist('enableTumbleToggle','var'), enableTumbleToggle = false; end

% Tumbling sub-parameters (set by passLinkGUI.m; defaults match tumbling_attenuation.m)
if ~exist('tumbleTestCase',           'var'), tumbleTestCase           = "stable"; end
if ~exist('tumbleSatDimensions',      'var'), tumbleSatDimensions      = [0.1 0.1 0.3]; end
if ~exist('tumbleMass',               'var'), tumbleMass               = 4; end
if ~exist('tumbleAntennaType',        'var'), tumbleAntennaType        = "Half-Wave Dipole"; end
if ~exist('tumbleAntennaOrientation', 'var'), tumbleAntennaOrientation = "+X"; end
if ~exist('tumbleDishRadius',         'var'), tumbleDishRadius         = 0.05; end
if ~exist('tumbleShowPlots',          'var'), tumbleShowPlots          = false; end

% Sécurité : si un run précédent a planté avant la libération en fin de
% script, l'objet SDR peut encore tenir le matériel (pertinent car la GUI
% exécute ce script en boucle dans le workspace "base" via evalin).
if exist('SDR_RX', 'var')
    try release(SDR_RX); catch, end
end
if exist('SDR_TX', 'var')
    try release(SDR_TX); catch, end
end

% clearvars instead of clear so the toggles set above (or by the GUI)
% survive the workspace cleanup
clearvars -except showRangePlot showPathLossPlot showDelayPlot showDopplerPlot ...
    enableTumbleToggle tumbleTestCase tumbleSatDimensions tumbleMass ...
    tumbleAntennaType tumbleAntennaOrientation tumbleDishRadius tumbleShowPlots
clc;

% Define Programmable Attenuator Parameters
att_port = "COM3";                                                               % Check COM Port; 3 is for Howard
att_baudrate = 115200;       
test_channel = 1;

% Initialise Programmable Attenuator
fprintf("Opening serial connection to attenuator on %s...\n", att_port);
att = initProgATT(att_port, att_baudrate);

% Define SDR parameters
Platform = "B210";
SerialNum = "32418F5";
ChannelMapping = 1;
CenterFrequency = 435e6;            % 435 MHz Carrier Frequency
MasterClockRate = 56e6;                                             % 32e6 in DCETest But Increased to 56e6 For Anti-jitter
DecimationFactor = 56; InterpolationFactor = DecimationFactor;      % 32 in DCETest But Increased to 56 For Anti-jitter
fs = MasterClockRate / DecimationFactor;                       % 1 MSPS Sample Rate
rxGain = 25; txGain = 50;
delayBuffer = zeros(256e3,1);       % Memory array for time-delay emulation
SamplesPerFrame = 16384;                                            % 4096 in DCETest But Increased to 16384 For Anti-jitter
delaySDR = SamplesPerFrame/fs;      % Fixed physical hardware/USB loop latency calibration
phaseOffset = 0.0;
OutputDataType = "double"; 
enableTumble = enableTumbleToggle;  % Enable simulated tumbling of satellite (set via GUI, defaults to false)

% Initialize USRP RX and TX System Objects
disp("Initializing USRP SDR Hardware...");
[SDR_RX,SDR_TX] = initSDR(Platform,SerialNum,ChannelMapping,CenterFrequency, ...
    rxGain,txGain,MasterClockRate,DecimationFactor,InterpolationFactor, ...
    OutputDataType,SamplesPerFrame);

% Releases COM Port and SDR When Script Ends or Has Errors Mid-Run
cleanupAtt = onCleanup(@() clear('att')); 
cleanupRX = onCleanup(@() release(SDR_RX));
cleanupTX = onCleanup(@() release(SDR_TX));

% Synchronize B210 & Signal generator 
% Verify External 10 MHz Reference Lock Before Proceeding
disp("Checking external 10 MHz reference lock...");
pause(1);                                   % Give the radio a moment to attempt lock after object creation
if ~referenceLockedStatus(SDR_RX)           % SDR_RX & TX share the same clock : 1 test is enough
    error("SDR_RX is not locked to the external 10 MHz reference. Check REF OUT -> REF IN cabling and that the signal generator's reference output is enabled.");
end
disp("External reference locked successfully.");

% Flush the SDR Buffers to Discard Transient Startup Frames
disp("Flushing SDR buffers...");
flushSDR(SDR_RX, SDR_TX, fs, SamplesPerFrame, 10);

% Import Path from CSV
[file, path] = uigetfile('*.csv', 'Select a CSV File');
if isequal(file, 0)
    error("No CSV file selected.");
end
thisFile = fullfile(path, file);
fprintf('Reading %s...\n', file);
csv_table = readtable(thisFile);    % CSV data
csv_filename = string(file);
fprintf('Loaded %s\n', file);

% Set Up Pass Data Visualisation (Live Plot)
    % Column mapping confirmed from CSV header:
    % 1=t, 2=Range_m, 3=Azimuth_deg, 4=Elevation_deg, 5=PathLoss_dB, 6=Delay_s, 7=Doppler_Hz, 8=Rel_Velocity_mps
markerSize = 10;

% Build the ordered list of enabled plots based on the toggles above
% (and enableTumble, which gates the tumble plot regardless of the toggle
% since there is no tumble data unless enableTumble is true).
plotKeys    = {};
plotTitles  = {};
plotYLabels = {};
if showRangePlot
    plotKeys{end+1}    = 'range';
    plotTitles{end+1}  = 'Range vs. Time';
    plotYLabels{end+1} = 'Range (km)';
end
if showPathLossPlot
    plotKeys{end+1}    = 'pathloss';
    plotTitles{end+1}  = 'Path Loss vs. Time';
    plotYLabels{end+1} = 'Path Loss (dB)';
end
if showDelayPlot
    plotKeys{end+1}    = 'delay';
    plotTitles{end+1}  = 'Delay vs. Time';
    plotYLabels{end+1} = 'Delay (ms)';
end
if showDopplerPlot
    plotKeys{end+1}    = 'doppler';
    plotTitles{end+1}  = 'Doppler Shift vs. Time';
    plotYLabels{end+1} = 'Doppler (kHz)';
end
if enableTumble
    plotKeys{end+1}    = 'tumble';
    plotTitles{end+1}  = 'Pointing Loss (Tumbling) vs. Time';
    plotYLabels{end+1} = 'Attenuation (dB)';
end

numPlots = numel(plotKeys);
if numPlots == 0
    error("At least one plot must be enabled (showRangePlot, showPathLossPlot, showDelayPlot or showDopplerPlot).");
end

% Single dashboard window: scrolling data table on the left,
% the selected live plots on the right.
liveFig = uifigure('Name', 'Live Pass Metrics', 'Position', [100, 100, 1400, 750]);

leftPanel  = uipanel(liveFig, 'Title', 'Live Data Table', ...
    'Units', 'normalized', 'Position', [0.01 0.02 0.33 0.96]);
rightPanel = uipanel(liveFig, 'Title', 'Live Plots', ...
    'Units', 'normalized', 'Position', [0.35 0.02 0.64 0.96]);

tableColumnNames = {'Time (s)', 'Total Atten (dB)', 'Path Loss (dB)', ...
                     'Tumbling (dB)', 'Rician (dB)', 'Delay (ms)', 'Doppler (Hz)'};
liveTable = uitable(leftPanel, 'Units', 'normalized', 'Position', [0 0 1 1], ...
    'ColumnName', tableColumnNames, 'Data', zeros(0, numel(tableColumnNames)));

tl = tiledlayout(rightPanel,numPlots,1);
liveTitle = sgtitle(tl, 'Live playback of recorded pass data');

% Create one tile per enabled plot and keep handles in maps keyed by plotKeys
axMap   = containers.Map();
plotMap = containers.Map();
for k = 1:numPlots
    ax = nexttile(tl);
    p = scatter(ax, NaT, NaN, markerSize, 'b', 'filled');
    title(ax, plotTitles{k}); ylabel(ax, plotYLabels{k}); grid(ax, 'on');
    if k == numPlots
        xlabel(ax, 'Time');
    end
    axMap(plotKeys{k})   = ax;
    plotMap(plotKeys{k}) = p;
end

% Buffer for the scrolling table (same rows printed to the command window)
tableData = zeros(0, numel(tableColumnNames));

% Extract and Re-map Multi-parameter Channel Profiles From CSV Columns to Fit the Program Layout
totalPoints = height(csv_table);
channelProfile = zeros(totalPoints, 4);     % Columns: 1=Time, 2=Atten, 3=Delay, 4=Doppler

% Convert the Datetime Column Into Relative Elapsed Seconds Starting at 0
raw_times = datetime(csv_table{:, 1}); 
channelProfile(:,1) = seconds(raw_times - raw_times(1)); 

% Extract Attenuation From Column E (Column 5)
channelProfile(:,2) = csv_table{:, 5};

% Generate Path Loss Attenuation Vector
pathloss_att = channelProfile(:,2);

% Normalise Dynamic Attenuation Control by In-line Losses 
fixed_att = 125;            % 150 in DCETest
channelProfile(:,2) = round(channelProfile(:,2)/0.25)*0.25 - fixed_att;

% Generate CANX-2 Tumbling Attenuation Profile
tumble_att_dB = zeros(totalPoints,1);   % default: no tumbling, needed later whether or not enableTumble is true
if enableTumble
    tumbleComponents = tumbling_attenuation( ...
        channelProfile(:,1), CenterFrequency, ...
        TestCase=tumbleTestCase, ...
        SatDimensions=tumbleSatDimensions, ...
        Mass=tumbleMass, ...
        AntennaType=tumbleAntennaType, ...
        AntennaOrientation=tumbleAntennaOrientation, ...
        DishRadius=tumbleDishRadius, ...
        ShowPlots=tumbleShowPlots, ...
        ShowAnimation=false);
    tumble_att_dB = tumbleComponents.pointing_loss_dB;

    % Add Attenuation from Tumbling
    channelProfile(:,2) = channelProfile(:,2) + tumble_att_dB;

    fprintf('Tumbling Profile Generated (TestCase: %s)\n', tumbleTestCase);

    % Display initial tumble conditions
    fprintf('Initial Pointing Error:\n');
    fprintf('  Roll:  %.2f deg\n', rad2deg(tumbleComponents.InitialPointingError_rad(1)));
    fprintf('  Pitch: %.2f deg\n', rad2deg(tumbleComponents.InitialPointingError_rad(2)));
    fprintf('  Yaw:   %.2f deg\n', rad2deg(tumbleComponents.InitialPointingError_rad(3)));

    fprintf('Initial Angular Velocity:\n');
    fprintf('  Wx: %.4f deg/s\n', rad2deg(tumbleComponents.InitialAngularVelocity_rad_s(1)));
    fprintf('  Wy: %.4f deg/s\n', rad2deg(tumbleComponents.InitialAngularVelocity_rad_s(2)));
    fprintf('  Wz: %.4f deg/s\n', rad2deg(tumbleComponents.InitialAngularVelocity_rad_s(3)));
end

% Extract Pre-Calculated Delay From Column F (Column 6)
channelProfile(:,3) = csv_table{:, 6};                                         
% channelProfile(:,3) = zeros(size(csv_table{:, 6}));                   % Enable to Turn Delay Off                                         

% Extract Pre-Calculated Doppler Shift From Column G (Column 7)
channelProfile(:,4) = csv_table{:, 7};
% channelProfile(:,4) = zeros(size(csv_table{:, 7}));                   % Enable to Turn Doppler Shift Off

% Columns Used for Live Plot
range_col    = csv_table{:, 2} / 1000;   % Range_m -> km
pathloss_col = csv_table{:, 5};          % PathLoss_dB
delay_col    = csv_table{:, 6} * 1000;   % Delay_s -> ms
doppler_col  = csv_table{:, 7} / 1000;   % Doppler_Hz -> kHz

% Map of column data per plot key, used both for fixed Y-limits and live updates
dataMap = containers.Map();
dataMap('range')    = range_col;
dataMap('pathloss') = pathloss_col;
dataMap('delay')    = delay_col;
dataMap('doppler')  = doppler_col;
dataMap('tumble')   = tumble_att_dB;

% Reset Plot Buffers For This Pass
plot_times = NaT(totalPoints, 1, 'TimeZone', raw_times.TimeZone);
bufMap = containers.Map();
for k = 1:numPlots
    bufMap(plotKeys{k}) = NaN(totalPoints, 1);
end

set(liveTitle, 'String', sprintf('Live playback — %s', csv_filename)); 
for k = 1:numPlots
    set(plotMap(plotKeys{k}), 'XData', NaT, 'YData', NaN);
end

axList = cellfun(@(k) axMap(k), plotKeys, 'UniformOutput', false);
axArray = [axList{:}];
xlim(axArray, [raw_times(1), raw_times(end)]);
linkaxes(axArray, 'x');

% Fix the Plot Y-axes for the Entire Duration of the Plot (based on the values from CSV)
for k = 1:numPlots
    key = plotKeys{k};
    setFixedYLim(axMap(key), dataMap(key));
end
drawnow;

% Reset to Maximum Attenuation
fprintf("Start of Maximum Attenuation... \n")
setAttenuation(att, test_channel, 95);
pause(2.5);
fprintf("End of Maximum Attenuation. \n")

% Real-time Effect Application Loop
disp("Beginning playback loop.");
effectIndex = 1;
last_hardware_db = -1;     
loopTimer = tic;

while (effectIndex <= totalPoints)

    % Pull a live RF data frame from the USRP Receiver
    rx_data = SDR_RX();

    % Extract current parameters from processed profile matrix
    current_db        = channelProfile(effectIndex, 2); % total attenuation including tumbling (if enabled)
    current_pathloss  = pathloss_att(effectIndex);
    current_tumbleatt = tumble_att_dB(effectIndex);
    current_delay     = channelProfile(effectIndex, 3);
    current_fShift    = channelProfile(effectIndex, 4);

    % Apply a Doppler Shift and Time Delay to the digital waveform array
    % Subtract the known hardware processing lag (delaySDR) to prevent buffer overflows
    calibrated_delay = max(current_delay - delaySDR, 0);
    % Subtract freqOffsetHz to Obtain Desired Center Frequency
    [phaseOffset, delayBuffer, tx_data] = applyDigitalImpairments(...
        rx_data, current_fShift, phaseOffset, calibrated_delay, delayBuffer, SamplesPerFrame, fs);

    % Transmit the modified waveform out of the USRP Transmitter
    SDR_TX(tx_data);

    % Update Parameters (slower than the live RF pull)
    if (channelProfile(effectIndex, 1) <= toc(loopTimer))
        
        % Prevent sending negative numbers or out-of-bounds values to hardware
        current_db = max(0, current_db); 

        % ADD VALUE FOR RICIAN FADING WHEN ADDED
        fprintf("Time: %.2fs | Total Atten: %.2f dB |Path Loss Atten: %.2f dB | Pointing Loss (Tumbling): %.2f dB | Rician Fading (dB): %.2f dB | Delay: %.2f ms | Doppler: %.2f Hz\n", ...
            channelProfile(effectIndex, 1), current_db, current_pathloss, current_tumbleatt, 0, current_delay*1e3, current_fShift);

        % Send command to programmable attenuator
        if current_db ~= last_hardware_db
                setAttenuation(att, test_channel, current_db);
                last_hardware_db = current_db;
        end

        % Update live plot buffers up to the current row and redraw
        plot_times(effectIndex) = raw_times(effectIndex);
        for k = 1:numPlots
            key = plotKeys{k};
            buf = bufMap(key);
            colData = dataMap(key);
            buf(effectIndex) = colData(effectIndex);
            bufMap(key) = buf;
            set(plotMap(key), 'XData', plot_times(1:effectIndex), 'YData', buf(1:effectIndex));
        end

        % Update the scrolling data table (same values as the console log
% above). New rows are added at the bottom, and the view auto-scrolls
% down to keep the latest sample visible.
newRow = [channelProfile(effectIndex, 1), current_db, current_pathloss, ...
          current_tumbleatt, 0, current_delay*1e3, current_fShift];
tableData = [tableData; newRow];
liveTable.Data = tableData;
scroll(liveTable, 'bottom');

        drawnow limitrate;

        % Move to the next row in the CSV profile for the next second
        effectIndex = effectIndex + 1;
    end
end

% Reset to Maximum Attenuation
fprintf("Start of Maximum Attenuation... \n")
setAttenuation(att, test_channel, 95);
pause(2.5);
fprintf("End of Maximum Attenuation. \n")

% Release SDR RX/TX (reset to prevent "Busy" locks and power drops)
release(SDR_RX);
release(SDR_TX);

% End / Reset
clear att;
disp("Dynamic Channel Emulation Complete cleanly.");


%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Helper Functions %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialise Serial Connection to Programmable Attenuator
function att_serial = initProgATT(port, baudrate)
    att_serial = serialport(port, baudrate);
    configureTerminator(att_serial, "CR/LF"); 
end

% Set Attenuation on Specified Channel of Programmable Attenuator
function setAttenuation(connection, channel, attenuation)
    cmd = sprintf("SET %d %.02f\r\n", channel, attenuation);
    writeline(connection, cmd);  
end

% Initialise Drivers for SDR RX/TX
function [SDR_rx,SDR_tx] = initSDR(Platform,SerialNum,ChannelMapping,CenterFrequency,rxGain,txGain,MasterClockRate, ...
    DecimationFactor,InterpolationFactor,OutputDataType,SamplesPerFrame)

SDR_rx = comm.SDRuReceiver(Platform=Platform,SerialNum=SerialNum,ChannelMapping=ChannelMapping, ...
    CenterFrequency=CenterFrequency,Gain=rxGain,MasterClockRate=MasterClockRate,DecimationFactor=DecimationFactor, ...
    OutputDataType=OutputDataType,SamplesPerFrame=SamplesPerFrame,ClockSource="External",LocalOscillatorOffset=1e6);

SDR_tx = comm.SDRuTransmitter(Platform=Platform,SerialNum=SerialNum,ChannelMapping=ChannelMapping, ...
    CenterFrequency=CenterFrequency,Gain=txGain,MasterClockRate=MasterClockRate,InterpolationFactor=InterpolationFactor, ...
    ClockSource="External",LocalOscillatorOffset=1e6);
end

% Flush SDR RX/TX Buffers for Specified Duration
function flushSDR(SDR_RX,SDR_TX,fs,SamplesPerFrame,duration)
    for i = 1:(ceil(duration/(SamplesPerFrame/fs)))
        flush_data = SDR_RX();
        SDR_TX(flush_data);
    end
end

% Apply Channel Impairments Through the SDR
function [phaseOffset, delayBuffer, tx_data] = applyDigitalImpairments(data, fShift, phaseOffset, delay, delayBuffer, SamplesPerFrame, fs)

    % Compute and apply Doppler Shift
    t = (0:SamplesPerFrame-1)' / fs;
    phaseShift = 2 * pi * fShift * t;
    mod_data = data .* exp(1j * (phaseShift + phaseOffset));
    phaseOffset = mod(phaseOffset + phaseShift(end) + (2 * pi * fShift / fs), 2 * pi); 

    % Apply Delay Through Circularly Shifted Buffer               
    idx_shift = max(round(delay * fs), 1);      % the position (the number of samples) at which the new block of data is to be inserted into the buffer
    delayBuffer(idx_shift : idx_shift + SamplesPerFrame - 1) = mod_data;
    tx_data = delayBuffer(1:SamplesPerFrame);
    delayBuffer = [delayBuffer(SamplesPerFrame + 1 : end); zeros(SamplesPerFrame, 1)];
end

% Set Y-axis of a Subplot Based on the min/max of the Pass Data
    % with a small margin for readability (5% of the range, minimum 1 unit)
function setFixedYLim(ax, dataCol)
    yMax = max(dataCol, [], 'omitnan');
    yMin = min(dataCol, [], 'omitnan');
    if isempty(yMax) || isempty(yMin) || isnan(yMax) || isnan(yMin)
        return; % no valid data, keep default autoscaling
    end
    span = yMax - yMin;
    if span == 0
        margin = max(abs(yMax) * 0.05, 1);
    else
        margin = span * 0.05;
    end
    ylim(ax, [yMin - margin, yMax + margin]);
end
