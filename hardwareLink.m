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
channelProfile(:,2) = round(channelProfile(:,2)/0.25)*0.25 - fixed_att;

% Extract Pre-Calculated Delay From Column F (Column 6)
channelProfile(:,3) = csv_table{:, 6};                                         

% Extract Pre-Calculated Doppler Shift From Column G (Column 7)
channelProfile(:,4) = csv_table{:, 7};

% Real-time Effect Application Loop
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

        % Move to the next row in the CSV profile for the next second
        effectIndex = effectIndex + 1;
    end
end

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

% Applying Channel Impairments Through the SDR
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
