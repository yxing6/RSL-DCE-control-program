%% measure_hardware_latency.m
%
% Calibration du délai matériel pur de la chaîne :
%   USRP TX1 --> Atténuateur programmable --> USRP RX2
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
% CORRECTIF : toute la trame (silence + préambule + silence) est
% concaténée en UNE SEULE forme d'onde continue, transmise et reçue en
% UN SEUL appel step() par essai. Les appels step() répétés frame par
% frame introduisaient du jitter d'ordonnancement hôte (dizaines de ms,
% signe variable) qui masquait complètement la vraie latence matérielle
% (attendue de l'ordre de quelques dizaines à centaines d'échantillons
% à 1 MSPS).
%
% Répété sur plusieurs essais pour obtenir moyenne + écart-type.

clear classes;
clear mex;
clear java;
clear;
clc;

%% ---------------- Paramètres atténuateur ----------------
att_port      = "COM3";   % à adapter
att_baudrate  = 115200;
test_channel  = 1;
useAttenuator = true;     % mettre false si vous ne voulez pas piloter l'atténuateur ici
calib_att_dB  = 15;       % atténuation fixe pendant la calibration (évite de saturer l'ADC RX2)

if useAttenuator
    fprintf("Opening serial connection to attenuator on %s...\n", att_port);
    att = initProgATT(att_port, att_baudrate);
    setAttenuation(att, test_channel, calib_att_dB);
    cleanupAtt = onCleanup(@() clear('att'));
end

%% ---------------- Paramètres SDR ----------------
Platform            = "B210";
SerialNum            = "32418F5";
CenterFrequency      = 435e6;
MasterClockRate      = 32e6;
DecimationFactor     = 32;
InterpolationFactor  = DecimationFactor;
fs                   = MasterClockRate / DecimationFactor; % 1 MSPS
rxGain               = 35;
txGain               = 80;
OutputDataType       = "double";
ChannelMapping       = 1;
SamplesPerFrame      = 32768;   % taille d'une "sous-trame" logique (silence/preambule/silence)
numTrials            = 20;      % nombre de mesures pour la statistique (relevé de 2 à 20)

%% ---------------- Séquence Zadoff-Chu ----------------
N = 839;   % longueur (doit être premier)
u = 25;    % index racine, gcd(u,N)=1
n = (0:N-1).';
zc = exp(-1j*pi*u*n.*(n+1)/N);   % colonne complexe, |zc|=1

%% ---------------- Construction de la trame connue (buffer unique) ----------------
% Segment 1 : silence (marge de garde avant)
% Segment 2 : préambule ZC au tout début, puis zéros
% Segment 3 : silence (marge de garde après, pour laisser le temps au
%             signal de revenir même si le délai matériel dépasse la fin
%             du segment 2)
frame1 = complex(zeros(SamplesPerFrame,1));
frame2 = [zc; complex(zeros(SamplesPerFrame-N,1))];
frame3 = complex(zeros(SamplesPerFrame,1));

% Concaténation en UNE seule forme d'onde continue transmise en un seul
% appel step() -> plus de dépendance au timing d'enchaînement hôte entre
% appels successifs.
txWaveform = [frame1; frame2; frame3];
SamplesPerFrame_total = numel(txWaveform); % 3 * SamplesPerFrame

% Position (en échantillons, à partir du 1er échantillon transmis après
% TriggerTime) où commence le préambule connu côté TX :
knownPreambleStartIdx = SamplesPerFrame + 1; % début de frame2 dans le buffer concaténé

%% ---------------- Initialisation SDR ----------------
disp("Initializing USRP SDR Hardware...");
[SDR_RX, SDR_TX] = initSDR(Platform, SerialNum, ChannelMapping, ...
    CenterFrequency, rxGain, txGain, MasterClockRate, DecimationFactor, ...
    InterpolationFactor, OutputDataType, SamplesPerFrame_total);

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

    tRelease0 = tic;
    release(SDR_TX);
    release(SDR_RX);
    releaseElapsed = toc(tRelease0);

    % DIAGNOSTIC : le pattern d'overflow alterne parfaitement un trial sur
    % deux, meme avec un buffer 3x plus petit -> ce n'est probablement pas
    % une question de taille de buffer mais d'un etat/timing laisse par le
    % cycle release()+rearm precedent. On logue le temps de release() pour
    % voir s'il correle avec les essais qui echouent ensuite.
    fprintf('  [diag] release() a pris %.3f s\n', releaseElapsed);

    currentTime = getRadioTime(SDR_TX);
    TriggerTime = currentTime + 8; % marge augmentee (etait 5s) par securite

    SDR_TX.EnableTimeTrigger = true;
    SDR_TX.TriggerTime       = TriggerTime;
    SDR_RX.EnableTimeTrigger = true;
    SDR_RX.TriggerTime       = TriggerTime;

    % ---- Un seul appel TX et un seul appel RX pour toute la trame ----
    [~, txUnderflow] = SDR_TX(complex(txWaveform)); %#ok<ASGLU>
    if any(txUnderflow)
        fprintf('Underflow detecte au trial %d (TX)\n', trial);
    end

    [rxWaveform, ~, overflow] = SDR_RX();
    % --------------------------------------------------------------

    % CORRECTIF : un overflow RX signifie que le buffer host n'a pas ete
    % rempli correctement (souvent rempli de zeros) -> toute mesure de
    % delai issue de ce buffer est un artefact numerique sans valeur
    % physique (peakToFloor_dB = -Inf car noiseFloor = median(mf) = 0).
    % On rejette immediatement l'essai plutot que de le laisser polluer
    % les statistiques.
    if any(overflow ~= 0)
        fprintf('Overflow detecte au trial %d (RX) -> essai REJETE\n', trial);

        % DIAGNOSTIC/CORRECTIF : flush supplementaire pour vider un
        % eventuel etat residuel avant le prochain trial, au cas ou le
        % flag overflow serait rapporte avec un cycle de retard.
        release(SDR_RX);
        pause(0.8); % marge augmentee (etait 0.5s) avant le prochain trigger
        continue;   % delaySamples_all(trial) reste NaN, exclu des stats
    end

    % Filtrage adapté (matched filter) : corrélation avec conj(zc) inversé
    mf = abs(conv(rxWaveform, conj(flipud(zc))));

    %% test
    figure;
    plot(20*log10(mf));
    title(sprintf('correlation - trial %d', trial));
    %%%%%%%%%%%%%%

    [peakVal, peakIdx] = max(mf);

    % Vérification grossière de qualité du pic (rapport pic / niveau moyen)
    noiseFloor    = median(mf);
    peakToFloor_dB = 20*log10(peakVal / max(noiseFloor, eps));

    % Garde-fou supplementaire : meme sans overflow signale, un pic trop
    % faible par rapport au plancher de bruit est suspect (bruit pur,
    % pas de vraie correlation) -> on rejette aussi ces essais.
    minPeakToFloor_dB = 6; % seuil a ajuster selon le lien
    if ~isfinite(peakToFloor_dB) || peakToFloor_dB < minPeakToFloor_dB
        fprintf("Trial %2d/%2d : pic/plancher = %.1f dB (< %d dB) -> essai REJETE\n", ...
            trial, numTrials, peakToFloor_dB, minPeakToFloor_dB);
        pause(0.3);
        continue;
    end

    measuredPreambleStartIdx = peakIdx - N + 1;
    delaySamples_all(trial)  = measuredPreambleStartIdx - knownPreambleStartIdx;
    delay_s_all(trial)       = delaySamples_all(trial) / fs;

    fprintf("Trial %2d/%2d : delay = %6d samples (%.3f ms) | pic/plancher = %.1f dB\n", ...
        trial, numTrials, delaySamples_all(trial), delay_s_all(trial)*1e3, peakToFloor_dB);

    pause(0.3); % petite pause entre essais
end

%% ---------------- Statistiques ----------------
validMask   = ~isnan(delay_s_all);
meanDelay_s = mean(delay_s_all(validMask));
stdDelay_s  = std(delay_s_all(validMask));

fprintf("\n=== Résultat calibration ===\n");
fprintf("Délai matériel moyen : %.4f ms (%.1f échantillons @ %.0f Hz)\n", ...
    meanDelay_s*1e3, meanDelay_s*fs, fs);
fprintf("Écart-type            : %.4f ms\n", stdDelay_s*1e3);
fprintf("Nombre d'essais valides : %d / %d\n", sum(validMask), numTrials);

figure('Name','Calibration latence matérielle');
histogram(delay_s_all*1e3, 'BinMethod','integers');
xlabel('Délai mesuré (ms)'); ylabel('Nombre d''essais');
title(sprintf('Latence matérielle : %.3f ms \\pm %.3f ms (n=%d)', ...
    meanDelay_s*1e3, stdDelay_s*1e3, sum(validMask)));
grid on;

% Sauvegarde pour réutilisation dans verify_applied_delay.m
hardwareLatency_s     = meanDelay_s;
hardwareLatency_std_s = stdDelay_s;
save('hardware_latency_calibration.mat', 'hardwareLatency_s', 'hardwareLatency_std_s', ...
    'delay_s_all', 'fs', 'SamplesPerFrame', 'N', 'u');
fprintf("Résultat sauvegardé dans hardware_latency_calibration.mat\n");

%% ---------------- Nettoyage ----------------
if useAttenuator
    setAttenuation(att, test_channel, 95); % retour en atténuation max par sécurité
end
release(SDR_RX);
release(SDR_TX);
disp("Calibration terminée.");


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Helper Functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function att_serial = initProgATT(port, baudrate)
    att_serial = serialport(port, baudrate);
    configureTerminator(att_serial, "CR/LF");
end

function setAttenuation(connection, channel, attenuation)
    cmd = sprintf("SET %d %.02f\r\n", channel, attenuation);
    writeline(connection, cmd);
end

function [SDR_rx, SDR_tx] = initSDR(Platform, SerialNum, ChannelMapping, ...
    CenterFrequency, rxGain, txGain, MasterClockRate, DecimationFactor, InterpolationFactor, ...
    OutputDataType, SamplesPerFrame)

    SDR_rx = comm.SDRuReceiver(Platform=Platform, SerialNum=SerialNum, ChannelMapping=ChannelMapping, ...
        CenterFrequency=CenterFrequency, Gain=rxGain, MasterClockRate=MasterClockRate, ...
        DecimationFactor=DecimationFactor, OutputDataType=OutputDataType, ...
        SamplesPerFrame=SamplesPerFrame, ClockSource="External", LocalOscillatorOffset=1e6, ...
        PPSSource="External");

    SDR_tx = comm.SDRuTransmitter(Platform=Platform, SerialNum=SerialNum, ChannelMapping=ChannelMapping, ...
        CenterFrequency=CenterFrequency, Gain=txGain, MasterClockRate=MasterClockRate, ...
        InterpolationFactor=InterpolationFactor, ClockSource="External", LocalOscillatorOffset=1e6, ...
        PPSSource="External");
end