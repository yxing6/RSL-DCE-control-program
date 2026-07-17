%% test_hardware_delay.m
% Mesure du delai reel introduit par la chaine materielle
% (USRP B210 TX/RX + Attenuateur Programmable + cablage), par bouclage
% (loopback) : un signal de reference connu est transmis et recu en
% direct par le meme B210, ce qui permet de mesurer le delai physique
% de la chaine, independamment de tout delai logiciel simule.
%
% PROCEDURE DE TEST :
%   1. Cabler physiquement TX1 -> (Attenuateur Programmable [+ att. fixes
%      si besoin]) -> RX1 du B210, EN BOUCLE, a la place du chemin normal
%      vers la GTEM. Ceci isole le delai materiel pur (USB + pipeline SDR
%      + cablage), sans passer par un vrai canal RF / la GTEM.
%   2. Regler l'attenuateur a une valeur de securite pour ne pas saturer
%      l'entree RX avec le niveau de sortie TX (a ajuster selon txGain,
%      rxGain, et la marge de l'attenuateur programmable).
%   3. Generer un signal de reference connu (chirp) et le transmettre en
%      continu, frame par frame, tout en recevant simultanement (duplex).
%   4. Stocker le flux transmis (reference exacte) et le flux recu.
%   5. Chercher par intercorrelation, sur plusieurs fenetres temporelles,
%      le decalage entre le signal transmis et le signal recu -> delai
%      materiel mesure, avec verification de sa stabilite (jitter) dans
%      le temps.
%
% Le delai materiel mesure ici correspond a la variable delaySDR utilisee
% dans hardwareLink_v2.m pour calibrer le delai logiciel applique
% (calibrated_delay = max(current_delay - delaySDR, 0)).

clear; clc; close all;

%% Parametres Attenuateur Programmable
att_port = "COM3";
att_baudrate = 115200;
test_channel = 1;
safety_attenuation_dB = 60;   % Attenuation de securite pour le bouclage TX->RX.
                              % A AJUSTER selon txGain/rxGain pour ne pas
                              % saturer l'entree RX ni endommager le
                              % materiel. Augmenter si saturation observee.

%% Parametres SDR (identiques a hardwareLink_v2.m)
Platform = "B210";
SerialNum = "32418F5";
ChannelMapping = 1;
CenterFrequency = 435e6;
MasterClockRate = 56e6;
DecimationFactor = 56; InterpolationFactor = DecimationFactor;
fs = MasterClockRate / DecimationFactor;
rxGain = 25; txGain = 50;
SamplesPerFrame = 16384;
OutputDataType = "double";

%% Parametres du test
testDuration_s   = 3;          % duree du signal de reference transmis/recu
warmupFrames     = 20;         % frames de "chauffe" avant le test (flush)
commandedDelay_s = 0;          % delai logiciel optionnel a ajouter en plus
                                % du delai materiel (0 = mesure du materiel seul)
maxDelay_s       = 15e-3;      % marge pour la recherche de delai (materiel + logiciel)

numFrames = ceil(testDuration_s * fs / SamplesPerFrame);

%% Initialisation Attenuateur Programmable
fprintf("Opening serial connection to attenuator on %s...\n", att_port);
att = initProgATT(att_port, att_baudrate);
cleanupAtt = onCleanup(@() clear('att'));

fprintf("Reglage attenuation de securite : %.1f dB\n", safety_attenuation_dB);
setAttenuation(att, test_channel, safety_attenuation_dB);
pause(0.5);

%% Initialisation USRP RX/TX
disp("Initializing USRP SDR Hardware...");
[SDR_RX, SDR_TX] = initSDR(Platform, SerialNum, ChannelMapping, CenterFrequency, ...
    rxGain, txGain, MasterClockRate, DecimationFactor, InterpolationFactor, ...
    OutputDataType, SamplesPerFrame);

cleanupRX = onCleanup(@() release(SDR_RX));
cleanupTX = onCleanup(@() release(SDR_TX));

% disp("Checking external 10 MHz reference lock...");
% pause(1);
% if ~referenceLockedStatus(SDR_RX)
%     error("SDR_RX is not locked to the external 10 MHz reference. Check REF OUT -> REF IN cabling.");
% end
disp("Checking external 10 MHz reference lock...");
dummy = SDR_RX();      % declenche la connexion reelle au materiel (setup UHD)
pause(1);
if ~referenceLockedStatus(SDR_RX)
    error("SDR_RX is not locked to the external 10 MHz reference. Check REF OUT -> REF IN cabling.");
end
disp("External reference locked successfully.");

%% Flush des buffers SDR (chauffe / vidage des frames transitoires)
disp("Flushing SDR buffers...");
for i = 1:warmupFrames
    flush_data = SDR_RX();
    SDR_TX(flush_data);
end

%% Initialisation VFD (delai logiciel optionnel, 0 par defaut)
vfd = dsp.VariableFractionalDelay('InterpolationMethod', 'Farrow', ...
    'MaximumDelay', ceil(maxDelay_s * fs));

%% Generation du signal de reference (chirp large bande, bonne autocorrelation)
f0 = 20e3; f1 = 200e3;
totalDuration_s = numFrames * SamplesPerFrame / fs;
k_chirp = (f1 - f0) / (2 * totalDuration_s);

capturedTx = zeros(numFrames*SamplesPerFrame, 1);
capturedRx = zeros(numFrames*SamplesPerFrame, 1);

%% Boucle de transmission/reception simultanee (duplex)
fprintf('Transmission/reception de %d frames (%.1f s)...\n', numFrames, totalDuration_s);
for kf = 1:numFrames
    n0 = (kf-1) * SamplesPerFrame;
    t = (n0 + (0:SamplesPerFrame-1))' / fs;

    ref_frame = exp(1j * 2*pi * (f0*t + k_chirp*t.^2));

    % Delai logiciel optionnel (0 = passthrough, pour mesurer le materiel seul)
    tx_data = vfd(ref_frame, commandedDelay_s * fs);

    SDR_TX(tx_data);
    rx_data = SDR_RX();

    idxRange = n0 + (1:SamplesPerFrame);
    capturedTx(idxRange) = tx_data;
    capturedRx(idxRange) = rx_data;

    if mod(kf, 20) == 0
        fprintf('  Frame %d / %d\n', kf, numFrames);
    end
end

%% Reset securite + liberation des ressources
setAttenuation(att, test_channel, 95);
release(SDR_RX);
release(SDR_TX);
clear att;
disp("Acquisition terminee.");

%% Mesure du delai materiel par intercorrelation sur fenetres glissantes
maxLag = ceil(maxDelay_s * fs);
windowLen = 4 * SamplesPerFrame;              % fenetre d'analyse (> maxLag)
numWindows = floor(length(capturedTx) / windowLen);

windowTimes_s   = zeros(numWindows,1);
measuredDelay_s = nan(numWindows,1);

for w = 1:numWindows
    idxRange = (w-1)*windowLen + (1:windowLen);
    xin  = capturedTx(idxRange);
    xout = capturedRx(idxRange);

    [c, lags] = xcorr(xout, xin, maxLag);
    [~, iMax] = max(abs(c));
    measuredDelay_s(w) = lags(iMax) / fs;

    windowTimes_s(w) = (w-0.5) * windowLen / fs;
end

commandedDelayVec_s = commandedDelay_s * ones(numWindows,1);

%% Resultats numeriques
validIdx = ~isnan(measuredDelay_s);
meanDelay_ms = mean(measuredDelay_s(validIdx)) * 1e3;
minDelay_ms  = min(measuredDelay_s(validIdx)) * 1e3;
maxDelay_ms  = max(measuredDelay_s(validIdx)) * 1e3;
stdDelay_ms  = std(measuredDelay_s(validIdx)) * 1e3;

fprintf('\n--- Resultats mesure delai materiel ---\n');
fprintf('Delai moyen mesure  : %.4f ms\n', meanDelay_ms);
fprintf('Delai min           : %.4f ms\n', minDelay_ms);
fprintf('Delai max           : %.4f ms\n', maxDelay_ms);
fprintf('Ecart-type (jitter) : %.4f ms\n', stdDelay_ms);
fprintf('\n=> Valeur suggeree pour delaySDR (hardwareLink_v2.m) : %.6f s\n', meanDelay_ms*1e-3);

%% Graphiques
figure('Name', 'Mesure du delai materiel B210 + Attenuateur', 'Position', [100 100 900 750]);

subplot(3,1,1);
plot(windowTimes_s, commandedDelayVec_s*1e3, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Delai commande (logiciel)');
hold on;
plot(windowTimes_s, measuredDelay_s*1e3, 'r--x', 'DisplayName', 'Delai mesure (materiel, xcorr)');
yline(meanDelay_ms, 'k:', sprintf('Moyenne = %.3f ms', meanDelay_ms));
xlabel('Temps (s)'); ylabel('Delai (ms)');
title('Delai commande vs delai materiel mesure (bouclage TX->RX)');
legend('Location', 'best'); grid on;

% Alignement pour la comparaison visuelle : on decale capturedTx du delai moyen mesure
lagSamples = round(meanDelay_ms*1e-3 * fs);
zoomLen = 500;

subplot(3,1,2);
zStart = SamplesPerFrame*2 + 1;   % on evite le tout debut (transitoire)
idxTx = zStart : zStart+zoomLen-1;
idxRx = zStart+lagSamples : zStart+lagSamples+zoomLen-1;
plot(real(capturedTx(idxTx)), 'b', 'DisplayName', 'Emis (reference)'); hold on;
plot(real(capturedRx(idxRx)), 'r', 'DisplayName', 'Recu (realigne du delai mesure)');
legend('Location', 'best');
title('Signal emis vs recu, realigne du delai materiel mesure');
xlabel('Echantillon'); ylabel('Amplitude'); grid on;

subplot(3,1,3);
idxTxRaw = zStart : zStart+zoomLen-1;
idxRxRaw = zStart : zStart+zoomLen-1;
plot(real(capturedTx(idxTxRaw)), 'b', 'DisplayName', 'Emis (reference)'); hold on;
plot(real(capturedRx(idxRxRaw)), 'r', 'DisplayName', 'Recu (brut, non realigne)');
legend('Location', 'best');
title('Signal emis vs recu, SANS realignement (le decalage visible = delai materiel)');
xlabel('Echantillon'); ylabel('Amplitude'); grid on;


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
function [SDR_rx, SDR_tx] = initSDR(Platform, SerialNum, ChannelMapping, CenterFrequency, rxGain, txGain, MasterClockRate, ...
    DecimationFactor, InterpolationFactor, OutputDataType, SamplesPerFrame)

SDR_rx = comm.SDRuReceiver(Platform=Platform, SerialNum=SerialNum, ChannelMapping=ChannelMapping, ...
    CenterFrequency=CenterFrequency, Gain=rxGain, MasterClockRate=MasterClockRate, DecimationFactor=DecimationFactor, ...
    OutputDataType=OutputDataType, SamplesPerFrame=SamplesPerFrame, ClockSource="External", LocalOscillatorOffset=1e6);

SDR_tx = comm.SDRuTransmitter(Platform=Platform, SerialNum=SerialNum, ChannelMapping=ChannelMapping, ...
    CenterFrequency=CenterFrequency, Gain=txGain, MasterClockRate=MasterClockRate, InterpolationFactor=InterpolationFactor, ...
    ClockSource="External", LocalOscillatorOffset=1e6);
end