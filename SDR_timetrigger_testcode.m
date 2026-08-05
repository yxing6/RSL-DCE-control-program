%% Simple timed TX/RX example for USRP B210
% Transmit and receive 1000 samples simultaneously.

clear;
clc;

%% Parameters
Fs = 1e6;          % Sample rate
Fc = 915e6;        % Center frequency
GainTX = 20;
GainRX = 20;
Nsamps = 1000;

%% Create transmitter
tx = comm.SDRuTransmitter( ...
    'Platform','B210', ...
    'SerialNum','32418F5', ...          % Leave blank if only one radio
    'CenterFrequency',Fc, ...
    'MasterClockRate',20e6, ...
    'InterpolationFactor',20, ...
    'Gain',GainTX);

%% Create receiver
rx = comm.SDRuReceiver( ...
    'Platform','B210', ...
    'SerialNum','32418F5', ...
    'CenterFrequency',Fc, ...
    'MasterClockRate',20e6, ...
    'DecimationFactor',20, ...
    'Gain',GainRX, ...
    'SamplesPerFrame',Nsamps, ...
    'OutputDataType','double');

%% Generate transmit signal
txData = complex(randn(Nsamps,1), randn(Nsamps,1));
txData = txData ./ max(abs(txData));

%% Read current USRP hardware time
currentTime = 5;

%% Schedule both devices 2 seconds in the future
triggerTime = currentTime + 2.0;

%% Configure timed operation
tx.EnableTimeTrigger = true;
rx.EnableTimeTrigger = true;

tx.TriggerTime = triggerTime;
rx.TriggerTime = triggerTime;

fprintf('Current Radio Time : %.6f s\n', currentTime);
fprintf('Scheduled Time     : %.6f s\n', triggerTime);

%% Wait until trigger time

disp('Waiting for trigger...');

%% Receive and transmit

% Start RX
[rxData,len,overflow] = rx();

% Start TX
underflow = tx(txData);

fprintf('Received %d samples.\n', len);

%% Release hardware
release(tx);
release(rx);