%% Programmable Attenuator and SDR Link

clear; clc; 

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
MasterClockRate = 32e6;                                             
DecimationFactor = 32; InterpolationFactor = DecimationFactor;
fs = MasterClockRate / DecimationFactor;                       % 1 MSPS Sample Rate
rxGain = 25; txGain = 50;
delayBuffer = zeros(256e3,1);       % Memory array for time-delay emulation
%%%%%%%%%%
circBuffer   = zeros(bufferSize, 1);    % The static memory array
writePointer = 1;                       % Tracks where incoming RX data gets written
%%%%%%%%%%
SamplesPerFrame = 4096;                                            % 4096 in DCETest But Increased to 16384 For Anti-jitter
delaySDR = SamplesPerFrame/fs;      % Fixed physical hardware/USB loop latency calibration
phaseOffset = 0.0;
OutputDataType = "double"; 
enableTumble = false;               % Enable simulated tumbling of satellite

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

liveFig = figure('Name', 'Live Pass Metrics', 'Position', [100, 100, 900, 750]);
numPlots = 4 + enableTumble;
tl = tiledlayout(liveFig,numPlots,1);
liveTitle = sgtitle(tl, 'Live playback of recorded pass data');

% Initialize plot for range
ax1 = nexttile(tl);
plot_rng = scatter(ax1, NaT, NaN, markerSize, 'b', 'filled');
title(ax1, 'Range vs. Time'); ylabel(ax1, 'Range (km)'); grid(ax1, 'on');

% Initialize plot for path loss
ax2 = nexttile(tl);
plot_pl = scatter(ax2, NaT, NaN, markerSize, 'b', 'filled');
title(ax2, 'Path Loss vs. Time'); ylabel(ax2, 'Path Loss (dB)'); grid(ax2, 'on');

% Initialize plot for delay
ax3 = nexttile(tl);
plot_delay = scatter(ax3, NaT, NaN, markerSize, 'b', 'filled');
title(ax3, 'Delay vs. Time'); ylabel(ax3, 'Delay (ms)'); grid(ax3, 'on');

% Initialize plot for doppler shift
ax4 = nexttile(tl);
plot_dop = scatter(ax4, NaT, NaN, markerSize, 'b', 'filled');
title(ax4, 'Doppler Shift vs. Time'); ylabel(ax4, 'Doppler (kHz)'); xlabel(ax4, 'Time'); grid(ax4, 'on');

% Initialize plot for tumble related attenuation (if tumble is enabled)
if enableTumble
    ax5 = nexttile(tl);
    plot_tumble = scatter(ax5, NaT, NaN, markerSize, 'b', 'filled');
    title(ax5, 'Pointing Loss (Tumbling) vs. Time'); ylabel(ax5, 'Attenuation (dB)'); xlabel(ax5, 'Time'); grid(ax5, 'on');
    
    linkaxes([ax1, ax2, ax3, ax4, ax5], 'x');
else
    linkaxes([ax1, ax2, ax3, ax4], 'x');
end

% Extract and Re-map Multi-parameter Channel Profiles From CSV Columns to Fit the Program Layout
totalPoints = height(csv_table);
channelProfile = zeros(totalPoints, 4);     % Columns: 1=Time, 2=Atten, 3=Delay, 4=Doppler

% Convert the Datetime Column Into Relative Elapsed Seconds Starting at 0
raw_times = datetime(csv_table{:, 1}); 
channelProfile(:,1) = seconds(raw_times - raw_times(1)); 

% Extract Attenuation From Column E (Column 5)
channelProfile(:,2) = csv_table{:, 5};
t=15;                                                                   % Enable for Circular Buffer Delay Testing
channelProfile(1:t,2) = 150;                                            % Enable for Circular Buffer Delay Testing
channelProfile(t:end,2) = 130;                                          % Enable for Circular Buffer Delay Testing

% Generate Path Loss Attenuation Vector
pathloss_att = channelProfile(:,2);

% Normalise Dynamic Attenuation Control by In-line Losses 
fixed_att = 125;            % 150 in DCETest
channelProfile(:,2) = round(channelProfile(:,2)/0.25)*0.25 - fixed_att;

% Generate CANX-2 Tumbling Attenuation Profile
if enableTumble
    [tumble_att_dB] = tumbling_attenuation( ...
        channelProfile(:,1), ...
        ShowPlots=false, ...
        ShowAnimation=false);
    % Add Attenuation from Tumbling
    channelProfile(:,2) = channelProfile(:,2) + tumble_att_dB;
end

% Extract Pre-Calculated Delay From Column F (Column 6)
channelProfile(:,3) = csv_table{:, 6};                                         
% channelProfile(:,3) = zeros(size(csv_table{:, 6}));                   % Enable to Turn Delay Off                                         
channelProfile(:,3) = 0.1*ones(size(csv_table{:, 6}));                  % Enable for Circular Buffer Delay Testing                   

% Extract Pre-Calculated Doppler Shift From Column G (Column 7)
channelProfile(:,4) = csv_table{:, 7};
% channelProfile(:,4) = zeros(size(csv_table{:, 7}));                   % Enable to Turn Doppler Shift Off
channelProfile(1:t,4) = 7000;                                           % Enable for Circular Buffer Delay Testing           
channelProfile(t:end,4) = -7000;                                        % Enable for Circular Buffer Delay Testing           

% Generate CANX-2 Tumbling Attenuation Profile
tumble_att_dB = zeros(totalPoints,1);
if enableTumble
    [tumble_att_dB, components] = tumbling_attenuation( ...
        channelProfile(:,1), ...
        ShowPlots=false, ...
        ShowAnimation=false);
    fprintf('Random Tumbling Profile Generated\n');
    
    % Display initial tumble conditions
    fprintf('Initial Pointing Error:\n');
    fprintf('  Roll:  %.2f deg\n', rad2deg(components.InitialPointingError_rad(1)));
    fprintf('  Pitch: %.2f deg\n', rad2deg(components.InitialPointingError_rad(2)));
    fprintf('  Yaw:   %.2f deg\n', rad2deg(components.InitialPointingError_rad(3)));

    fprintf('Initial Angular Velocity:\n');
    fprintf('  Wx: %.4f deg/s\n', rad2deg(components.InitialAngularVelocity_rad_s(1)));
    fprintf('  Wy: %.4f deg/s\n', rad2deg(components.InitialAngularVelocity_rad_s(2)));
    fprintf('  Wz: %.4f deg/s\n', rad2deg(components.InitialAngularVelocity_rad_s(3)));
end

% Columns Used for Live Plot
range_col    = csv_table{:, 2} / 1000;   % Range_m -> km
pathloss_col = csv_table{:, 5};          % PathLoss_dB
delay_col    = csv_table{:, 6} * 1000;   % Delay_s -> ms
doppler_col  = csv_table{:, 7} / 1000;   % Doppler_Hz -> kHz

% Reset Plot Buffers For This Pass
plot_times = NaT(totalPoints, 1, 'TimeZone', raw_times.TimeZone);
rng_buf   = NaN(totalPoints, 1);
pl_buf    = NaN(totalPoints, 1);
delay_buf = NaN(totalPoints, 1);
dop_buf   = NaN(totalPoints, 1);
tumb_buf = NaN(totalPoints,1);

set(liveTitle, 'String', sprintf('Live playback — %s', csv_filename)); 
set(plot_rng,   'XData', NaT, 'YData', NaN);
set(plot_pl,    'XData', NaT, 'YData', NaN);
set(plot_delay, 'XData', NaT, 'YData', NaN);
set(plot_dop,   'XData', NaT, 'YData', NaN);

if enableTumble
    xlim([ax1, ax2, ax3, ax4, ax5], [raw_times(1), raw_times(end)]);
    set(plot_tumble,   'XData', NaT, 'YData', NaN);
    setFixedYLim(ax5, tumble_att_dB); % fix Y-axis of axis 5
else
    xlim([ax1, ax2, ax3, ax4], [raw_times(1), raw_times(end)]);
end

% Fix the Plot Y-axes for the Entire Duration of the Plot (based on the values ​​from CSV)
setFixedYLim(ax1, range_col);
setFixedYLim(ax2, pathloss_col);
setFixedYLim(ax3, delay_col);
setFixedYLim(ax4, doppler_col);
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
    % [phaseOffset, delayBuffer, tx_data] = applyDigitalImpairments(...                 %%%%%%%%%%
    %     rx_data, current_fShift, phaseOffset, calibrated_delay, delayBuffer, SamplesPerFrame, fs);
    %%%%%%%%%% 
    [phaseOffset, circBuffer, writePointer, tx_data] = applyDigitalImpairments(...  
        rx_data, current_fShift, phaseOffset, calibrated_delay, circBuffer, writePointer, SamplesPerFrame, fs);
    %%%%%%%%%%

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
        rng_buf(effectIndex)    = range_col(effectIndex);
        pl_buf(effectIndex)     = pathloss_col(effectIndex);
        delay_buf(effectIndex)  = delay_col(effectIndex);
        dop_buf(effectIndex)    = doppler_col(effectIndex);

        set(plot_rng,   'XData', plot_times(1:effectIndex), 'YData', rng_buf(1:effectIndex));
        set(plot_pl,    'XData', plot_times(1:effectIndex), 'YData', pl_buf(1:effectIndex));
        set(plot_delay, 'XData', plot_times(1:effectIndex), 'YData', delay_buf(1:effectIndex));
        set(plot_dop,   'XData', plot_times(1:effectIndex), 'YData', dop_buf(1:effectIndex));

        if enableTumble
            tumb_buf(effectIndex)   = tumble_att_dB(effectIndex);
            set(plot_tumble,   'XData', plot_times(1:effectIndex), 'YData', tumb_buf(1:effectIndex));
        end

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
clear; 
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

%%%%%%%%%%
% Apply Channel Impairments Through SDR
function [phaseOffset, circBuffer, writePointer, tx_data] = applyDigitalImpairments(...
    rx_data, fShift, phaseOffset, delay, circBuffer, writePointer, SamplesPerFrame, fs)
    
    % Compute and apply Doppler Shift to incoming data
    t = (0:SamplesPerFrame-1)' / fs;
    phaseShift = 2 * pi * fShift * t;
    mod_data = rx_data .* exp(1j * (phaseShift + phaseOffset));
    phaseOffset = mod(phaseOffset + phaseShift(end) + (2 * pi * fShift / fs), 2 * pi); 
    
    % Apply Delay Through Circularly Shifted Buffer
    writeIndices = mod((writePointer - 1) + (0:SamplesPerFrame-1), length(circBuffer)) + 1;         % Determine the block of indices where new data will be written
    circBuffer(writeIndices) = mod_data;                                                            % Writes new data                                                           
    delaySamples = max(round(delay * fs), 0);                                                       % Calculate how many samples back the read pointer needs to be
    assert(delaySamples < length(circBuffer), ...                                                   % Prevents invalid delay inputs
        'Requested delay exceeds circular buffer length.');
    readPointer = mod((writePointer - 1) - delaySamples, length(circBuffer)) + 1;                   % Calculate the Read Pointer position relative to where new data was just written (step backward by delaySamples)
    readIndices = mod((readPointer - 1) + (0:SamplesPerFrame-1), length(circBuffer)) + 1;           % Determine the block of indices where delayed data will be read
    tx_data = circBuffer(readIndices);                                                              % Transmits read data
    writePointer = mod((writePointer - 1) + SamplesPerFrame, length(circBuffer)) + 1;               % Advance the write pointer forward for the next frame's turn
end
%%%%%%%%%%

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