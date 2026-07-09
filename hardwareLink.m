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
fs = 32e6/32;                       % 1 MSPS Sample Rate
rxGain = 25; txGain = 50;
delayBuffer = zeros(256e3,1);       % Memory array for time-delay emulation
SamplesPerFrame = 4096;
delaySDR = SamplesPerFrame/fs;      % Fixed physical hardware/USB loop latency calibration
phaseOffset = 0.0;
OutputDataType = "double"; 
enableTumble = false; %enable simulated tumbling of satellite

% Initialize USRP RX and TX System Objects
disp("Initializing USRP SDR Hardware...");
[SDR_RX,SDR_TX] = initSDR(Platform,SerialNum,ChannelMapping,CenterFrequency, ...
    rxGain,txGain,MasterClockRate,DecimationFactor,InterpolationFactor, ...
    OutputDataType,SamplesPerFrame);

% Releases COM Port and SDR When Script Ends or Has Errors Mid-Run
cleanupAtt = onCleanup(@() clear('att')); 
cleanupRX = onCleanup(@() release(SDR_RX));
cleanupTX = onCleanup(@() release(SDR_TX));

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
tl = tiledlayout(liveFig, 4, 1);
liveTitle = sgtitle(tl, 'Live playback of recorded pass data');

ax1 = nexttile(tl);
plot_rng = scatter(ax1, NaT, NaN, markerSize, 'b', 'filled');
title(ax1, 'Range vs. Time'); ylabel(ax1, 'Range (km)'); grid(ax1, 'on');

ax2 = nexttile(tl);
plot_pl = scatter(ax2, NaT, NaN, markerSize, 'b', 'filled');
title(ax2, 'Path Loss vs. Time'); ylabel(ax2, 'Path Loss (dB)'); grid(ax2, 'on');

ax3 = nexttile(tl);
plot_delay = scatter(ax3, NaT, NaN, markerSize, 'b', 'filled');
title(ax3, 'Delay vs. Time'); ylabel(ax3, 'Delay (ms)'); grid(ax3, 'on');

ax4 = nexttile(tl);
plot_dop = scatter(ax4, NaT, NaN, markerSize, 'b', 'filled');
title(ax4, 'Doppler Shift vs. Time'); ylabel(ax4, 'Doppler (kHz)'); xlabel(ax4, 'Time'); grid(ax4, 'on');

linkaxes([ax1, ax2, ax3, ax4], 'x');

%% Pass Data Visualisation (Live Plot) Loop

    % Extract and Re-map Multi-parameter Channel Profiles From CSV Columns to Fit the Program Layout
    totalPoints = height(csv_table);
    channelProfile = zeros(totalPoints, 4);     % Columns: 1=Time, 2=Atten, 3=Delay, 4=Doppler

    % Convert the Datetime Column Into Relative Elapsed Seconds Starting at 0
    raw_times = datetime(csv_table{:, 1}); 
    channelProfile(:,1) = seconds(raw_times - raw_times(1)); 

    % Extract Attenuation From Column E (Column 5)
    channelProfile(:,2) = csv_table{:, 5};

    % Normalise Dynamic Attenuation Control by In-line Losses 
    fixed_att = 125;                                                                 % 150 in DCETest
    channelProfile(:,2) = round(channelProfile(:,2)/0.25)*0.25-fixed_att;

    % Extract Pre-Calculated Delay From Column F (Column 6)
    channelProfile(:,3) = csv_table{:, 6};                                         

    % Extract Pre-Calculated Doppler Shift From Column G (Column 7)
    channelProfile(:,4) = csv_table{:, 7};

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

    set(liveTitle, 'String', sprintf('Live playback — %s', csv_filename)); 
    set(plot_rng,   'XData', NaT, 'YData', NaN);
    set(plot_pl,    'XData', NaT, 'YData', NaN);
    set(plot_delay, 'XData', NaT, 'YData', NaN);
    set(plot_dop,   'XData', NaT, 'YData', NaN);
    xlim([ax1, ax2, ax3, ax4], [raw_times(1), raw_times(end)]);

    % Fix the Plot Y-axes for the Entire Duration of the Plot (based on the values ​​from CSV)
    setFixedYLim(ax1, range_col);
    setFixedYLim(ax2, pathloss_col);
    setFixedYLim(ax3, delay_col);
    setFixedYLim(ax4, doppler_col);

    drawnow;
    
    % Generate CANX-2 Tumbling Attenuation Profile
    if enableTumble
        [tumble_att_dB] = tumbling_attenuation( ...
            channelProfile(:,1), ...
            ShowPlots=false, ...
            ShowAnimation=false);
        % Add Attenuation from Tumbling
        channelProfile(:,2) = channelProfile(:,2)+ tumble_att_dB;
    end
    
    
    %% Real-time Effect Application Loop
    disp("Beginning playback loop.");
    loopTimer = tic;
    effectIndex = 1;
    
    n = 0;                                                                          % Counter Used For Testing Attenuation

    while (effectIndex <= totalPoints)
        % Pull a live RF data frame from the USRP Receiver
        rx_data = SDR_RX();
    
        % Extract current parameters from processed profile matrix
        current_db    = channelProfile(effectIndex, 2);
        current_delay = channelProfile(effectIndex, 3);
        current_fShift= channelProfile(effectIndex, 4);
    
        % Apply a Doppler Shift and Time Delay to the digital waveform array
        % Subtract the known hardware processing lag (delaySDR) to prevent buffer overflows
        calibrated_delay = max(current_delay - delaySDR, 0);
        [phaseOffset, delayBuffer, tx_data] = applyDigitalImpairments(...
            rx_data, current_fShift, phaseOffset, calibrated_delay, delayBuffer, SamplesPerFrame, fs);        
    
        % Transmit the modified waveform out of the USRP Transmitter
        SDR_TX(tx_data);
    
        % Update Parameters (slower than the live RF pull)
        if (channelProfile(effectIndex, 1) <= toc(loopTimer))
    
            n = n + 0;                                                              % Used For Testing Attenuation
            current_db = current_db + n;                                            % USed For Testing Attenuation
    
            % Prevent sending negative numbers or out-of-bounds values to hardware
            current_db = max(0, current_db); 
    
            fprintf("Time: %.2fs | Atten: %.2f dB | Delay: %.2f ms | Doppler: %.2f Hz\n", ...
                toc(loopTimer), current_db, current_delay*1e3, current_fShift);
    
            % Send command to programmable attenuator
            setAttenuation(att, test_channel, current_db);
    
            % Update live plot buffers up to the current row and redraw
            plot_times(effectIndex) = raw_times(effectIndex);
            rng_buf(effectIndex)   = range_col(effectIndex);
            pl_buf(effectIndex)    = pathloss_col(effectIndex);
            delay_buf(effectIndex) = delay_col(effectIndex);
            dop_buf(effectIndex)   = doppler_col(effectIndex);
    
            set(plot_rng,   'XData', plot_times(1:effectIndex), 'YData', rng_buf(1:effectIndex));
            set(plot_pl,    'XData', plot_times(1:effectIndex), 'YData', pl_buf(1:effectIndex));
            set(plot_delay, 'XData', plot_times(1:effectIndex), 'YData', delay_buf(1:effectIndex));
            set(plot_dop,   'XData', plot_times(1:effectIndex), 'YData', dop_buf(1:effectIndex));
            drawnow limitrate;
    
            % Move to the next row in the CSV profile for the next second
            effectIndex = effectIndex + 1;
        end
    end

%% End of Pass Data Visualisation (Live Plot) Loop

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
    OutputDataType=OutputDataType,SamplesPerFrame=SamplesPerFrame);

SDR_tx = comm.SDRuTransmitter(Platform=Platform,SerialNum=SerialNum,ChannelMapping=ChannelMapping, ...
    CenterFrequency=CenterFrequency,Gain=txGain,MasterClockRate=MasterClockRate,InterpolationFactor=InterpolationFactor);
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
    idx_shift = max(round(delay * fs), 1);
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