%% verify_applied_delay.m
%
% Vérifie que le délai numérique appliqué par applyDigitalImpairments
% (copiée à l'identique depuis doctorStrange_timeTrigger.m) correspond
% bien, une fois mesuré physiquement en sortie (TX1 -> atténuateur -> RX2),
% au délai demandé.
%
% Principe :
%  1) On construit un signal "source" connu contenant un préambule
%     Zadoff-Chu à une position d'échantillon connue.
%  2) On fait passer ce signal, hors-ligne, trame par trame, à travers
%     EXACTEMENT la même fonction applyDigitalImpairments que le
%     programme principal, avec un délai numérique voulu (current_delay).
%     Cela reproduit fidèlement ce que le buffer circulaire produirait
%     en fonctionnement réel.
%  3) On transmet le signal ainsi retardé, on capture le retour sur RX2
%     (TX et RX synchronisés sur le même TriggerTime), et on retrouve la
%     position du préambule reçu par corrélation croisée.
%  4) Délai total mesuré = latence matérielle + délai numérique appliqué.
%     On soustrait la latence matérielle (calibrée avec
%     measure_hardware_latency.m) pour obtenir le délai numérique mesuré,
%     à comparer au délai demandé.
%
% Lancez measure_hardware_latency.m AVANT ce script pour générer
% hardware_latency_calibration.mat.

clear classes;
clear mex;
clear java;
clear;
clc;

%% ---------------- Chargement de la calibration matérielle ----------------
calibFile = 'hardware_latency_calibration.mat';
if isfile(calibFile)
    calib = load(calibFile);
    hardwareLatency_s = calib.hardwareLatency_s;
    fprintf("Latence matérielle chargée : %.4f ms (depuis %s)\n", ...
        hardwareLatency_s*1e3, calibFile);
else
    warning("%s introuvable : exécutez d'abord measure_hardware_latency.m. hardwareLatency_s mis à 0.", calibFile);
    hardwareLatency_s = 0;
end

%% ---------------- Paramètres attenuateur ----------------
att_port      = "COM3";
att_baudrate  = 115200;
test_channel  = 1;
useAttenuator = true;
test_att_dB   = 30;    % même valeur que pendant la calibration, idéalement !!

if useAttenuator
    att = initProgATT(att_port, att_baudrate);
    setAttenuation(att, test_channel, test_att_dB);
    cleanupAtt = onCleanup(@() clear('att'));
end

%% ---------------- Paramètres SDR (identiques à doctorStrange_timeTrigger.m) ----------------
Platform            = "B210";
SerialNum           = "32418F5";
CenterFrequency     = 435e6;
MasterClockRate     = 32e6;
DecimationFactor    = 32;
InterpolationFactor = DecimationFactor;
fs                  = MasterClockRate / DecimationFactor;   % 1 MSPS
rxGain              = 10;
txGain              = 15;
OutputDataType      = "double";
ChannelMapping = 1;   
SamplesPerFrame = 20000;

%% ---------------- Délais à tester ----------------
% delaysToTest_s = [0, 0.001, 0.005, 0.010, 0.020, 0.050];  % en secondes
delaysToTest_s = [0, 0.010];  % en secondes
numTrialsPerDelay = 3;

%% ---------------- Séquence Zadoff-Chu ----------------
N  = 839;
u  = 25;
n  = (0:N-1).';
zc = exp(-1j*pi*u*n.*(n+1)/N);

%% ---------------- Construction du signal source connu ----------------
maxDelay_s      = max(delaysToTest_s);
maxDelaySamples = ceil(maxDelay_s * fs);

numLeadFrames  = 2;
numTrailFrames = ceil(maxDelaySamples / SamplesPerFrame) + 3;

sourcePreambleStartIdx = numLeadFrames*SamplesPerFrame + 1;

sourceWaveform = complex([ ...
    zeros(numLeadFrames*SamplesPerFrame, 1); ...
    zc; ...
    zeros(SamplesPerFrame - N, 1); ...
    zeros(numTrailFrames*SamplesPerFrame, 1) ]);

SamplesPerFrame_total = length(sourceWaveform);     % 3 * SamplesPerFrame

numSourceFrames = length(sourceWaveform) / SamplesPerFrame;
assert(mod(numSourceFrames,1)==0, 'sourceWaveform length must be a multiple of SamplesPerFrame');

%% ---------------- Initialisation SDR ----------------
disp("Initializing USRP SDR Hardware...");
[SDR_RX, SDR_TX] = initSDR(Platform, SerialNum, ChannelMapping, ...
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

% disp("Flushing SDR buffers...");
% for i = 1:20
%     [~, ~, ~] = SDR_RX();
% end

%% ---------------- Boucle de test sur les délais ----------------
results = struct('requested_s', {}, 'measured_total_s', {}, 'measured_digital_s', {}, ...
                  'error_s', {}, 'trial', {});
resultIdx = 1;

for d = 1:numel(delaysToTest_s)
    current_delay = delaysToTest_s(d);

    % --- 1) Génère le signal TX en repassant le signal source à travers
    %        EXACTEMENT applyDigitalImpairments, trame par trame ---
    circBuffer   = zeros(256e3, 1);    
    writePointer = 1;
    phaseOffset  = 0.0;
    fShift       = 0;   % pas de Doppler pour ce test

    txWaveform = complex(zeros(size(sourceWaveform)));
    for f = 1:numSourceFrames
        idxStart = (f-1)*SamplesPerFrame + 1;
        idxEnd   = f*SamplesPerFrame;
        rx_data_sim = sourceWaveform(idxStart:idxEnd);

        [phaseOffset, circBuffer, writePointer, tx_data] = applyDigitalImpairments(...
            rx_data_sim, fShift, phaseOffset, current_delay, circBuffer, writePointer, ...
            SamplesPerFrame_total, fs);

        txWaveform(idxStart:idxEnd) = tx_data;
    end

    % txFrames = mat2cell(txWaveform, SamplesPerFrame*ones(numSourceFrames,1), 1);

    for trial = 1:numTrialsPerDelay

        release(SDR_TX);
        release(SDR_RX);

        currentTime = getRadioTime(SDR_TX);
        TriggerTime = currentTime + 5;
        fprintf('current USRP time: %.9f s\n', currentTime);
        fprintf('trigger time: %.9f s\n', TriggerTime);

        SDR_TX.EnableTimeTrigger = true;
        SDR_TX.TriggerTime       = TriggerTime;
        SDR_RX.EnableTimeTrigger = true;
        SDR_RX.TriggerTime       = TriggerTime;

        % rxFrames = cell(1, numSourceFrames);
        % for f = 1:numSourceFrames
        %     SDR_TX(complex(txFrames{f}));
        %     rxFrames{f} = SDR_RX();
        % end
        % rxWaveform = cat(1, rxFrames{:});
        %%%%%%
        SDR_TX(complex(txWaveform));
        rt = getRadioTime(SDR_TX);
        fprintf('current USRP time: %.9f s\n', rt);

        [rxWaveform, ~, overflow, rxTimestamp] = SDR_RX();
        fprintf('RX timestamp: %.9f s\n', rxTimestamp);
        %%%%%%%

        if any(overflow ~= 0)
            fprintf('Overflow detected (delay=%.1f ms, trial %d) -> rejected trial \n', current_delay*1e3, trial);
            release(SDR_RX);
            pause(0.8);
            continue;
        end
        
        mf = abs(conv(rxWaveform, conj(flipud(zc))));
        [peakVal, peakIdx] = max(mf);
        noiseFloor = median(mf);
        peakToFloor_dB = 20*log10(peakVal / max(noiseFloor, eps));

        measuredPreambleStartIdx = peakIdx - N + 1;

        measuredTotalSamples = measuredPreambleStartIdx - sourcePreambleStartIdx;
        measuredTotal_s   = measuredTotalSamples / fs;
        measuredDigital_s = measuredTotal_s - hardwareLatency_s;
        error_s = measuredDigital_s - current_delay;

        fprintf(['Delai demande = %6.2f ms | mesure totale = %6.2f ms | ' ...
                 'mesure numerique = %6.2f ms | erreur = %+6.3f ms | pic/plancher = %.1f dB\n'], ...
            current_delay*1e3, measuredTotal_s*1e3, measuredDigital_s*1e3, error_s*1e3, peakToFloor_dB);

        results(resultIdx).requested_s        = current_delay;
        results(resultIdx).measured_total_s    = measuredTotal_s;
        results(resultIdx).measured_digital_s  = measuredDigital_s;
        results(resultIdx).error_s             = error_s;
        results(resultIdx).trial               = trial;
        resultIdx = resultIdx + 1;

        pause(0.2);
    end
end

%% ---------------- Résumé / graphique ----------------
resultsTable = struct2table(results);
summaryStats = varfun(@mean, resultsTable, 'InputVariables', {'measured_digital_s','error_s'}, ...
    'GroupingVariables', 'requested_s');
disp(summaryStats);

figure('Name','Vérification du délai appliqué');
scatter(resultsTable.requested_s*1e3, resultsTable.measured_digital_s*1e3, 40, 'filled');
hold on;
refLine = [0, max(delaysToTest_s)*1e3];
plot(refLine, refLine, 'k--');
xlabel('Délai demandé (ms)'); ylabel('Délai numérique mesuré (ms)');
legend('Mesures','Idéal (y = x)','Location','northwest');
title('Précision de applyDigitalImpairments');
grid on;

save('verify_applied_delay_results.mat', 'results', 'resultsTable', 'hardwareLatency_s');
fprintf("Résultats sauvegardés dans verify_applied_delay_results.mat\n");

%% ---------------- Nettoyage ----------------
if useAttenuator
    setAttenuation(att, test_channel, 95);
end
release(SDR_RX);
release(SDR_TX);
disp("Vérification terminée.");


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

function [SDR_rx, SDR_tx] = initSDR(Platform, SerialNum, ChannelMapping, ...
    CenterFrequency, rxGain, txGain, MasterClockRate, DecimationFactor, InterpolationFactor, ...
    OutputDataType, SamplesPerFrame)

    SDR_rx = comm.SDRuReceiver(Platform=Platform, SerialNum=SerialNum, ChannelMapping=ChannelMapping, ...
        CenterFrequency=CenterFrequency, Gain=rxGain, MasterClockRate=MasterClockRate, ...
        DecimationFactor=DecimationFactor, OutputDataType=OutputDataType, ...
        SamplesPerFrame=SamplesPerFrame, ClockSource="External", LocalOscillatorOffset=0, ...
        PPSSource="External");

    SDR_tx = comm.SDRuTransmitter(Platform=Platform, SerialNum=SerialNum, ChannelMapping=ChannelMapping, ...
        CenterFrequency=CenterFrequency, Gain=txGain, MasterClockRate=MasterClockRate, ...
        InterpolationFactor=InterpolationFactor, ClockSource="External", LocalOscillatorOffset=0, ...
        PPSSource="External");
end

% ---- Copie EXACTE de votre fonction, depuis doctorStrange_timeTrigger.m ----
function [phaseOffset, circBuffer, writePointer, tx_data] = applyDigitalImpairments(...
    rx_data, fShift, phaseOffset, delay, circBuffer, writePointer, SamplesPerFrame, fs)

    % Compute and apply Doppler Shift
    t = (0:SamplesPerFrame-1)' / fs;
    phaseShift = 2 * pi * fShift * t;
    mod_data = rx_data .* exp(1j * (phaseShift + phaseOffset));
    phaseOffset = mod(phaseOffset + phaseShift(end) + (2 * pi * fShift / fs), 2 * pi);

    % Apply Delay Through Circularly Shifted Buffer
    writeIndices = mod((writePointer - 1) + (0:SamplesPerFrame-1), length(circBuffer)) + 1;
    circBuffer(writeIndices) = mod_data;
    delaySamples = max(round(delay * fs), 0);
    assert(delaySamples < length(circBuffer), ...
        'Requested delay exceeds circular buffer length.');
    readPointer = mod((writePointer - 1) - delaySamples, length(circBuffer)) + 1;
    readIndices = mod((readPointer - 1) + (0:SamplesPerFrame-1), length(circBuffer)) + 1;
    tx_data = circBuffer(readIndices);
    writePointer = mod((writePointer - 1) + SamplesPerFrame, length(circBuffer)) + 1;
end
