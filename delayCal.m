
clear; 
clc;

Platform = "B210";
SerialNum = "32418F5";
ChannelMapping = 1;
CenterFrequency = 435e6;
MasterClockRate = 56e6;
DecimationFactor = 56; InterpolationFactor = DecimationFactor;
fs = 1e6;
rxGain = 25; txGain = 50;              
SamplesPerFrame = 16384;
OutputDataType = "double";

%Programmable attenuator's configuration for calibration
att_port = "COM3";
att_baudrate = 115200;
test_channel = 1;
calibAttenuation_dB = 0;            

att = initProgATT(att_port, att_baudrate);
setAttenuation(att, test_channel, calibAttenuation_dB);
pause(0.2);                        

[SDR_RX, SDR_TX] = initSDR(Platform, SerialNum, ChannelMapping, CenterFrequency, ...
    rxGain, txGain, MasterClockRate, DecimationFactor, InterpolationFactor, ...
    OutputDataType, SamplesPerFrame);

disp("Flushing...");
for i = 1:10
    flush_data = SDR_RX();
    SDR_TX(flush_data);
end

% % Barker 13 coded pulse
% barker13 = [1 1 1 1 1 -1 -1 1 1 -1 1 -1 1];
% samplesPerChip = 20;                                   
% barkerWaveform = repelem(barker13, samplesPerChip).'; 

%  m-seqeuence pulse
p = 7;
N = 2^p - 1; %127
state = ones(1,p);%inital state
seq = ones(1,N);
for i = 1:N
    seq(i) = state(end); %output bit
    % xor feeback on 6th and 7th register
    feedback = xor(state(6), state(7));
    % shift register
    state = [feedback state(1:end-1)];
end
%convert from [0,1] to [-1,+1]
seq = 2*seq - 1;
%Build cyclic code (3 sequences)
cyclic_code = repmat(seq, 1, 3);

pulseStart = 100;      
testPulse = zeros(SamplesPerFrame,1);
%testPulse(pulseStart : pulseStart+numel(barkerWaveform)-1) = barkerWaveform;
testPulse(pulseStart : pulseStart+numel(cyclic_code)-1) = cyclic_code;

nTrials = 30;
measuredDelaySamples = zeros(nTrials,1);

for k = 1:nTrials
    SDR_TX(testPulse);
    rx_data = SDR_RX();
    [c, lags] = xcorr(rx_data, testPulse);
    [~, idxMax] = max(abs(c));
    measuredDelaySamples(k) = lags(idxMax);

    flush_data = SDR_RX(); SDR_TX(flush_data);
end

release(SDR_RX); release(SDR_TX);
clear att;

delaySDR_measured_samples = median(measuredDelaySamples);
delaySDR_measured_seconds = delaySDR_measured_samples / fs;
delayStd_samples = std(measuredDelaySamples);

fprintf('Measured physical delay (via the actual hardware chain) : %.1f samples (%.3f ms), standard deviation = %.1f samples\n', ...
    delaySDR_measured_samples, delaySDR_measured_seconds*1e3, delayStd_samples);

%%
% Plot TX pulse, RX capture, and correlation (from the last trial)
figure('Name','RX/TX Loopback Check');
 
subplot(3,1,1);
plot(real(testPulse));
title('TX waveform (Barker-13 pulse, real part)');
xlabel('Sample'); ylabel('Amplitude'); grid on;
xlim([pulseStart-50, pulseStart+numel(barkerWaveform)+50]);
 
subplot(3,1,2);
plot(real(rx_data)); hold on;
plot(imag(rx_data));
legend('Real','Imag');
title('RX capture (last trial, full frame)');
xlabel('Sample'); ylabel('Amplitude'); grid on;
 
subplot(3,1,3);
plot(lags, abs(c));
hold on;
xline(measuredDelaySamples(end), 'r--', 'Detected delay');
title('Correlation magnitude |xcorr(rx\_data, testPulse)|');
xlabel('Lag (samples)'); ylabel('|c|'); grid on;


%% --- Log CSV of calibration ---
calibTable = table( ...
    datetime('now'), string(Platform), string(SerialNum), CenterFrequency, fs, ...
    SamplesPerFrame, rxGain, txGain, calibAttenuation_dB, nTrials, ...
    delaySDR_measured_samples, delaySDR_measured_seconds, delayStd_samples, ...
    'VariableNames', {'Timestamp','Platform','SerialNum','CenterFrequency_Hz','fs_Hz', ...
    'SamplesPerFrame','RxGain_dB','TxGain_dB','AttenuatorSetting_dB','NumTrials', ...
    'DelayMedian_samples','DelayMedian_s','DelayStd_samples'});

calibFolder = fullfile(pwd, 'DCE_Calibration_Log');
if ~exist(calibFolder, 'dir'); mkdir(calibFolder); end
calibLogFile = fullfile(calibFolder, 'hardware_delay_calibration_log.csv');

% Add a row at each calibration instead of overwritting the file
if isfile(calibLogFile)
    writetable(calibTable, calibLogFile, 'WriteMode', 'append');
else
    writetable(calibTable, calibLogFile, 'WriteMode', 'overwrite');
end
fprintf('Calibration saved in %s\n', calibLogFile);



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
    OutputDataType=OutputDataType,SamplesPerFrame=SamplesPerFrame,ClockSource="Internal",LocalOscillatorOffset=1e6);

SDR_tx = comm.SDRuTransmitter(Platform=Platform,SerialNum=SerialNum,ChannelMapping=ChannelMapping, ...
    CenterFrequency=CenterFrequency,Gain=txGain,MasterClockRate=MasterClockRate,InterpolationFactor=InterpolationFactor, ...
    ClockSource="Internal",LocalOscillatorOffset=1e6);
end

% Flush SDR RX/TX Buffers for Specified Duration
function flushSDR(SDR_RX,SDR_TX,fs,SamplesPerFrame,duration)
    for i = 1:(ceil(duration/(SamplesPerFrame/fs)))
        flush_data = SDR_RX();
        SDR_TX(flush_data);
    end
end