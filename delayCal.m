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

pause(0.5);

disp("Flushing...");
flushSDR(SDR_RX, SDR_TX, fs, SamplesPerFrame, 1);

% build m-seq
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
testPulse(pulseStart : pulseStart+numel(cyclic_code)-1) = cyclic_code;

nTrials = 30;
measuredDelaySamples = zeros(nTrials,1);

for k = 1:nTrials
    % transmit pulse
    txUnderrun = SDR_TX(testPulse);
    % receive frame
    [rx_data, ~, rxOverrun] = SDR_RX();
    % correlate
    [c,lags] = xcorr(rx_data,testPulse);
    [~,idxMax] = max(abs(c));
    measuredDelaySamples(k) = lags(idxMax);
end

% Statistics
delayMeanSamples = mean(measuredDelaySamples);
delayStdSamples  = std(measuredDelaySamples);

delayMean_ms = delayMeanSamples/fs*1000;
delayStd_ms  = delayStdSamples/fs*1000;


fprintf("\n SDR Delay \n");
fprintf("Mean delay: %.2f samples\n",delayMeanSamples);
fprintf("Mean delay: %.3f ms\n",delayMean_ms);
fprintf("Std deviation: %.3f ms\n",delayStd_ms);

release(SDR_RX); release(SDR_TX);
clear att;

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