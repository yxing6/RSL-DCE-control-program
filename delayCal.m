
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

pause(0.5);   % addition : allow the PLL/LO on the B210 time to stabilise

% disp("Flushing...");
% for i = 1:10
%     flush_data = SDR_RX();
%     SDR_TX(flush_data);
% end

disp("Flushing...");

nCleanRequired = 20;   % required number of consecutive error-free frames
maxFlushIter   = 200;  % safeguard to prevent an infinite loop
consecClean    = 0;
iter           = 0;

flush_data = zeros(SamplesPerFrame,1);  % first flush TX empty

while consecClean < nCleanRequired && iter < maxFlushIter
    [flush_data, ~, rxOv] = SDR_RX();
    txUn = SDR_TX(flush_data);

    if ~rxOv && ~txUn
        consecClean = consecClean + 1;
    else
        consecClean = 0;   % start again from scratch as soon as an error occurs
    end
    iter = iter + 1;
end

if iter >= maxFlushIter
    warning('Flush not stabilised after %d iterations (persistent under/overrun).', maxFlushIter);
else
    fprintf('Flush stabilised after %d iterations (%d consecutive clean frames).\n', iter, consecClean);
end

% Barker 13 coded pulse
barker13 = [1 1 1 1 1 -1 -1 1 1 -1 1 -1 1];
samplesPerChip = 20;
barkerWaveform = repelem(barker13, samplesPerChip).';

pulseStart = 100;      
testPulse = zeros(SamplesPerFrame,1);
testPulse(pulseStart : pulseStart+numel(barkerWaveform)-1) = barkerWaveform;

% nTrials = 30;
% measuredDelaySamples = zeros(nTrials,1);
% 
% for k = 1:nTrials
%     txUnderrun = SDR_TX(testPulse);
%     [rx_data, ~, rxOverrun] = SDR_RX();
%     [c, lags] = xcorr(rx_data, testPulse);
%     [~, idxMax] = max(abs(c));
%     measuredDelaySamples(k) = lags(idxMax);
%     underrunLog(k) = txUnderrun;
%     overrunLog(k) = rxOverrun;
%     %flush_data = SDR_RX(); SDR_TX(flush_data);     %probably a major cause of the drift between trials
% end

% %%%% localization of the pulse
% nTrials = 30;
% measuredDelaySamples = zeros(nTrials,1);
% framesWaitedLog = zeros(nTrials,1);   % diagnostic: how many frames before finding the pulse
% 
% maxWaitFrames = 10;      % guardrail
% snrFactor = 8;            % The peak must exceed 8 times the median noise level of the correlation


%%%% continuous/periodic pulse
nCaptureFrames = 50;   % number of frames to capture continuously
rxBuffer = zeros(SamplesPerFrame*nCaptureFrames, 1);
underrunLog = false(nCaptureFrames,1);
overrunLog  = false(nCaptureFrames,1);

for k = 1:nCaptureFrames
    underrunLog(k) = SDR_TX(testPulse);
    [rxFrame, ~, overrunLog(k)] = SDR_RX();
    rxBuffer((k-1)*SamplesPerFrame+1 : k*SamplesPerFrame) = rxFrame;
end

if any(underrunLog) || any(overrunLog)
    warning('Underrun/overrun detected during the continuous capture: %d underrun(s), %d overrun(s). The data stream is not perfectly contiguous; this may affect the results.', ...
        sum(underrunLog), sum(overrunLog));
else
    disp('Continuous capture: no underruns or overruns; contiguous stream confirmed.');
end

release(SDR_RX); release(SDR_TX);
clear att;

% delaySDR_measured_samples = median(measuredDelaySamples);
% delaySDR_measured_seconds = delaySDR_measured_samples / fs;
% delayStd_samples = std(measuredDelaySamples);
%%%%%% modification for periodic pulse
[c, lags] = xcorr(rxBuffer, barkerWaveform);
cAbs = abs(c);

% retain only positive (causal) and valid lags
validIdx = lags >= 0 & lags <= (numel(rxBuffer) - numel(barkerWaveform));
cAbsValid = cAbs(validIdx);
lagsValid = lags(validIdx);

noiseFloor = median(cAbsValid);
[peakVals, peakLocs] = findpeaks(cAbsValid, 'MinPeakHeight', 8*noiseFloor, ...
    'MinPeakDistance', round(0.8*SamplesPerFrame));

peakLags = lagsValid(peakLocs);
peakModFrame = mod(peakLags, SamplesPerFrame);   % 'true' physical delay, independent of call jitter

fprintf('Number of periodic spikes detected: %d (expected ~%d)\n', numel(peakLags), nCaptureFrames);
fprintf('Physical delay (mod %d) : median = %.1f samples, std = %.1f samples\n', ...
    SamplesPerFrame, median(peakModFrame), std(peakModFrame));

delaySDR_measured_samples = median(peakModFrame);
delaySDR_measured_seconds = delaySDR_measured_samples / fs;
delayStd_samples = std(peakModFrame);
%%%%%%%%%

% % addition:determine whether an entire run switches to the other value
% fprintf('Delay min/max for this run : %d / %d samples (écart = %d)\n', ...
%     min(measuredDelaySamples), max(measuredDelaySamples), ...
%     max(measuredDelaySamples)-min(measuredDelaySamples));

%%%%%% periodic pulse
fprintf('Delay min/max for this run (peaks detected) : %d / %d samples (écart = %d)\n', ...
    min(peakModFrame), max(peakModFrame), ...
    max(peakModFrame)-min(peakModFrame));
%%%%%%

fprintf('Measured physical delay (via the actual hardware chain) : %.1f samples (%.3f ms), standard deviation = %.1f samples\n', ...
    delaySDR_measured_samples, delaySDR_measured_seconds*1e3, delayStd_samples);

% figure;
% stem(1:nTrials, measuredDelaySamples); hold on;
% plot(find(overrunLog | underrunLog), measuredDelaySamples(overrunLog | underrunLog), 'ro', 'MarkerSize',10);
% title('Delay per trial (red = under/overrun flagged)');

%%%%%%%%%% periodic pulse
figure;
stem(1:numel(peakModFrame), peakModFrame);
xlabel('Indice du pic détecté');
ylabel('Delay (samples, mod SamplesPerFrame)');
title('Delay physique par pic périodique détecté');
%%%%%%%

%%
% Plot TX pulse, RX capture, and correlation (from the last trial)
figure('Name','RX/TX Loopback Check');
 
subplot(3,1,1);
plot(real(testPulse));
title('TX waveform (Barker-13 pulse, real part)');
xlabel('Sample'); ylabel('Amplitude'); grid on;
xlim([pulseStart-50, pulseStart+numel(barkerWaveform)+50]);
 
subplot(3,1,2);
plot(real(rxBuffer(1:SamplesPerFrame*2))); hold on;
plot(imag(rxBuffer(1:SamplesPerFrame*2)));
legend('Real','Imag');
title('RX capture (first 2 frames of continuous buffer)');
xlabel('Sample'); ylabel('Amplitude'); grid on;
 
subplot(3,1,3);
zoomRange = (peakLags(1)-200) : (peakLags(1)+200);
zoomRange = zoomRange(zoomRange >= min(lagsValid) & zoomRange <= max(lagsValid));
[~, zoomIdxInValid] = ismember(zoomRange, lagsValid);
zoomIdxInValid = zoomIdxInValid(zoomIdxInValid > 0);
plot(lagsValid(zoomIdxInValid), cAbsValid(zoomIdxInValid));
hold on;
xline(peakLags(1), 'r--', 'Detected delay (1st peak)');
title('Correlation magnitude |xcorr(rxBuffer, barkerWaveform)| — zoom sur le 1er pic');
xlabel('Lag (samples)'); ylabel('|c|'); grid on;

% --- Log CSV of calibration ---
% calibTable = table( ...
%     datetime('now'), string(Platform), string(SerialNum), CenterFrequency, fs, ...
%     SamplesPerFrame, rxGain, txGain, calibAttenuation_dB, nTrials, ...
%     delaySDR_measured_samples, delaySDR_measured_seconds, delayStd_samples, ...
%     'VariableNames', {'Timestamp','Platform','SerialNum','CenterFrequency_Hz','fs_Hz', ...
%     'SamplesPerFrame','RxGain_dB','TxGain_dB','AttenuatorSetting_dB','NumTrials', ...
%     'DelayMedian_samples','DelayMedian_s','DelayStd_samples'});

%%% periodic pulse
calibTable = table( ...
    datetime('now'), string(Platform), string(SerialNum), CenterFrequency, fs, ...
    SamplesPerFrame, rxGain, txGain, calibAttenuation_dB, nCaptureFrames, numel(peakLags), ...
    delaySDR_measured_samples, delaySDR_measured_seconds, delayStd_samples, ...
    'VariableNames', {'Timestamp','Platform','SerialNum','CenterFrequency_Hz','fs_Hz', ...
    'SamplesPerFrame','RxGain_dB','TxGain_dB','AttenuatorSetting_dB','NumFramesCaptured','NumPeaksDetected', ...
    'DelayMedian_samples','DelayMedian_s','DelayStd_samples'});
%%%%


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