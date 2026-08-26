%% measure_hardware_latency.m
%
% Calibration du délai matériel pur de la chaîne :
%   USRP TX1  -->  Atténuateur programmable  -->  USRP RX2
%
% Méthode : on transmet une trame contenant une séquence de Zadoff-Chu
% (auto-corrélation quasi parfaite) à une position d'échantillon connue,
% on capture le retour sur RX2 en synchronisant TX et RX sur le même
% TriggerTime (comme dans doctorStrange_timeTrigger.m), puis on retrouve
% la position du préambule reçu par corrélation croisée. La différence
% entre position connue (TX) et position mesurée (RX) donne directement
% le délai matériel en nombre d'échantillons, sans dépendre du timing
% du PC hôte.
%
% Répété sur plusieurs essais pour obtenir moyenne + écart-type.

clear classes;
clear mex;
clear java;
clear;
clc;

%% ---------------- Paramètres attenuateur ----------------
att_port     = "COM3";                 % à adapter
att_baudrate = 115200;
test_channel = 1;
useAttenuator = true;                  % mettre false si vous ne voulez pas piloter l'atténuateur ici
calib_att_dB  = 20;                    % attenuation fixe pendant la calibration (évite de saturer l'ADC RX2)

if useAttenuator
    fprintf("Opening serial connection to attenuator on %s...\n", att_port);
    att = initProgATT(att_port, att_baudrate);
    setAttenuation(att, test_channel, calib_att_dB);
    cleanupAtt = onCleanup(@() clear('att'));
end

%% ---------------- Paramètres SDR ----------------
Platform          = "B210";
SerialNum         = "32418F5";
CenterFrequency   = 435e6;
MasterClockRate   = 32e6;
DecimationFactor  = 32;
InterpolationFactor = DecimationFactor;
fs                = MasterClockRate / DecimationFactor;   % 1 MSPS
rxGain            = 35;
txGain            = 60;
OutputDataType    = "double";

txChannelMapping  = 1;      % TX1
rxChannelMapping  = 1;      % RX2 


SamplesPerFrame   = 32768;

numTrials         = 7;     % nombre de mesures pour la statistique

%% ---------------- Séquence Zadoff-Chu ----------------
N  = 839;      % longueur (doit être premier)
u  = 25;       % index racine, gcd(u,N)=1
n  = (0:N-1).';
zc = exp(-1j*pi*u*n.*(n+1)/N);         % colonne complexe, |zc|=1

%% ---------------- Construction de la trame connue ----------------
% Frame 1 : silence (marge de garde avant)
% Frame 2 : préambule ZC au tout début, puis zéros
% Frame 3 : silence (marge de garde après, pour laisser le temps au signal
%           de revenir même si le délai matériel dépasse la fin de frame 2)
frame1 = complex(zeros(SamplesPerFrame,1));
frame2 = [zc; complex(zeros(SamplesPerFrame-N,1))];
frame3 = complex(zeros(SamplesPerFrame,1));

txFrames = {frame1, frame2, frame3};
numFrames = numel(txFrames);

% Position (en échantillons, à partir du 1er échantillon transmis après
% TriggerTime) où commence le préambule connu côté TX :
knownPreambleStartIdx = SamplesPerFrame + 1;   % début de frame2

%% ---------------- Initialisation SDR ----------------
disp("Initializing USRP SDR Hardware...");
[SDR_RX, SDR_TX] = initSDR(Platform, SerialNum, txChannelMapping, rxChannelMapping, ...
    CenterFrequency, rxGain, txGain, MasterClockRate, DecimationFactor, ...
    InterpolationFactor, OutputDataType, SamplesPerFrame);

cleanupRX = onCleanup(@() release(SDR_RX));
cleanupTX = onCleanup(@() release(SDR_TX));

disp("Checking external 10 MHz reference lock...");
pause(1);
if ~referenceLockedStatus(SDR_RX)
    error("SDR_RX is not locked to the external 10 MHz reference. Check REF OUT -> REF IN cabling.");
end
disp("External reference locked successfully.");

disp("Flushing SDR buffers...");
for i = 1:20
    [~, ~, ~] = SDR_RX();
end

%% ---------------- Boucle de mesure ----------------
delaySamples_all = nan(numTrials,1);
delay_s_all      = nan(numTrials,1);

for trial = 1:numTrials

    release(SDR_TX);
    release(SDR_RX);

    currentTime = getRadioTime(SDR_TX);
    TriggerTime = currentTime + 5;         % avant=3 :marge de sécurité, pas < ~2s

    SDR_TX.EnableTimeTrigger = true;
    SDR_TX.TriggerTime       = TriggerTime;
    SDR_RX.EnableTimeTrigger = true;
    SDR_RX.TriggerTime       = TriggerTime;

    rxFrames = cell(1,numFrames);
    for k = 1:numFrames
        SDR_TX(txFrames{k});
        rxFrames{k} = SDR_RX();
    end
    rxWaveform = cat(1, rxFrames{:});

    % Filtrage adapté (matched filter) : corrélation avec conj(zc) inversé
    mf = abs(conv(rxWaveform, conj(flipud(zc))));
    [peakVal, peakIdx] = max(mf);

    % Vérification grossière de qualité du pic (rapport pic / niveau moyen)
    noiseFloor = median(mf);
    peakToFloor_dB = 20*log10(peakVal / max(noiseFloor, eps));

    measuredPreambleStartIdx = peakIdx - N + 1;

    delaySamples_all(trial) = measuredPreambleStartIdx - knownPreambleStartIdx;
    delay_s_all(trial)      = delaySamples_all(trial) / fs;

    fprintf("Trial %2d/%2d : delay = %6d samples (%.3f ms) | pic/plancher = %.1f dB\n", ...
        trial, numTrials, delaySamples_all(trial), delay_s_all(trial)*1e3, peakToFloor_dB);

    pause(0.2);   % petite pause entre essais
end

%% ---------------- Statistiques ----------------
validMask   = ~isnan(delay_s_all);
meanDelay_s = mean(delay_s_all(validMask));
stdDelay_s  = std(delay_s_all(validMask));

fprintf("\n=== Résultat calibration ===\n");
fprintf("Délai matériel moyen : %.4f ms (%.1f échantillons @ %.0f Hz)\n", ...
    meanDelay_s*1e3, meanDelay_s*fs, fs);
fprintf("Écart-type           : %.4f ms\n", stdDelay_s*1e3);
fprintf("Nombre d'essais valides : %d / %d\n", sum(validMask), numTrials);

figure('Name','Calibration latence matérielle');
histogram(delay_s_all*1e3, 'BinMethod','integers');
xlabel('Délai mesuré (ms)'); ylabel('Nombre d''essais');
title(sprintf('Latence matérielle : %.3f ms \\pm %.3f ms (n=%d)', ...
    meanDelay_s*1e3, stdDelay_s*1e3, sum(validMask)));
grid on;

% Sauvegarde pour réutilisation dans verify_applied_delay.m
hardwareLatency_s = meanDelay_s;
hardwareLatency_std_s = stdDelay_s;
save('hardware_latency_calibration.mat', 'hardwareLatency_s', 'hardwareLatency_std_s', ...
     'delay_s_all', 'fs', 'SamplesPerFrame', 'N', 'u');
fprintf("Résultat sauvegardé dans hardware_latency_calibration.mat\n");

%% ---------------- Nettoyage ----------------
if useAttenuator
    setAttenuation(att, test_channel, 95);   % retour en attenuation max par sécurité
end
release(SDR_RX);
release(SDR_TX);
disp("Calibration terminée.");


%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Helper Functions %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%

function att_serial = initProgATT(port, baudrate)
    att_serial = serialport(port, baudrate);
    configureTerminator(att_serial, "CR/LF");
end

function setAttenuation(connection, channel, attenuation)
    cmd = sprintf("SET %d %.02f\r\n", channel, attenuation);
    writeline(connection, cmd);
end

function [SDR_rx, SDR_tx] = initSDR(Platform, SerialNum, txChannelMapping, rxChannelMapping, ...
    CenterFrequency, rxGain, txGain, MasterClockRate, DecimationFactor, InterpolationFactor, ...
    OutputDataType, SamplesPerFrame)

    SDR_rx = comm.SDRuReceiver(Platform=Platform, SerialNum=SerialNum, ChannelMapping=rxChannelMapping, ...
        CenterFrequency=CenterFrequency, Gain=rxGain, MasterClockRate=MasterClockRate, ...
        DecimationFactor=DecimationFactor, OutputDataType=OutputDataType, ...
        SamplesPerFrame=SamplesPerFrame, ClockSource="External", LocalOscillatorOffset=1e6, ...
        PPSSource="External");

    SDR_tx = comm.SDRuTransmitter(Platform=Platform, SerialNum=SerialNum, ChannelMapping=txChannelMapping, ...
        CenterFrequency=CenterFrequency, Gain=txGain, MasterClockRate=MasterClockRate, ...
        InterpolationFactor=InterpolationFactor, ClockSource="External", LocalOscillatorOffset=1e6, ...
        PPSSource="External");
end
