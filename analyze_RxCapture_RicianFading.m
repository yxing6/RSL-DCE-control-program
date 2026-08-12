%% analyze_RxCapture_RicianFading.m
%
% Analyses a REAL IQ capture (output from the SDR TX chain -> attenuator
% -> SDR RX) and applies the same statistical tests as
% test_RicianFading.m (Rice distribution, K-factor, Doppler spectrum),
% in order to verify that the fading OBSERVED AT THE OUTPUT OF THE HARDWARE CHAIN
% does indeed correspond to the fading CONFIGURED upstream.
%
% ======================================================================
%  HARDWARE SETUP TO BE COMPLETED BEFORE COLLECTING DATA
% ======================================================================
%
% 1) WIRING (closed loop / loopback)
%    USRP_TX (RF output) --> Programmable attenuator --> USRP_RX (RF input)
%    RX and TX must be locked to the SAME
%    external 10 MHz reference (REF OUT -> REF IN), exactly as validated by
%    referenceLockedStatus(SDR_RX).
%
% 2) TEST SIGNAL: A CONSTANT CARRIER, NO MODULATED DATA
%    To statistically isolate pure fading (as in the software test
%    where testSig = ones(SamplesPerFrame,1)), you must TEMPORARILY replace
%    the actual modulated data with a signal of constant amplitude
%    (CW / pure carrier) BEFORE passing it through ricianChan, in
%    applyDigitalImpairments. Otherwise, the signal's content (modulation,
%    frequency offset, etc.) contaminates the measured envelope and distorts
%    the estimation of K on the RX side.
%    -> Keep fShift/Doppler at 0 for this test if possible, so as not to mix 
%       trajectory Doppler and fading Doppler.
%
% 3) CAPTURING RX FRAMES IN A FILE
%    In the main loop (around the line `rx_data = SDR_RX();`),
%    accumulate the received frames and save them at the end of the run:
%
%       rxCapture = complex(zeros(SamplesPerFrame*numFramesCapture, 1));
%       idx = 1;
%       for i = 1:numFramesCapture
%           rx_data = SDR_RX();
%           rxCapture(idx:idx+SamplesPerFrame-1) = rx_data;
%           idx = idx + SamplesPerFrame;
%           SDR_TX(tx_data);  % conserver le flux TX normal de la boucle
%       end
%       fs = ...;              % le SampleRate utilisé (doit être identique au TX)
%       K_nominal = ...;       % le KFactor configuré dans ricianChan pour ce run
%       fadeRate_nominal = ...;% le MaximumDopplerShift configuré
%       save('rxCapture_test.mat', 'rxCapture', 'fs', 'K_nominal', 'fadeRate_nominal', '-v7.3');
%
% 4) RECOMMENDED CAPTURE DURATION
%    As with the software test, sufficient coherence time must be covered
%    to obtain reliable statistics. Coherence time
%    approx. Tc = 0.423/fadeRate. Aim for at least ~2000–3000 x Tc of total
%    acquisition time for a good histogram (e.g. fadeRate=5 Hz -> Tc=85 ms -> aim for
%    a minimum of ~5–10 minutes of continuous acquisition). Be mindful of memory:
%    at fs=1 MHz, 10 minutes = 600M complex samples (~9.6 GB in
%    double) -> consider writing to disk in blocks (fwrite) rather
%    than keeping everything in RAM if your capture exceeds a few minutes.
%
% 5) BASELINE WITHOUT FADING 
%    Before taking a capture with fading enabled, take a
%    reference capture with fading disabled (enableFading=false, or a very
%    large K as in the main script) to characterise the
%    residual variation specific to the hardware alone (RX noise, gain drift,
%    attenuator imperfections). This 'baseline' variation must be
%    negligible compared to the variation induced by active fading — otherwise
%    the following test will be unable to distinguish between fading and hardware noise.
%    Save this capture to a separate file, e.g. 'rxCapture_baseline.mat'.
%
% ======================================================================

clear; clc; close all;

%% ------------------------------------------------------------------
%  Settings: paths to the capture files to be analysed
%% ------------------------------------------------------------------
% Leave 'captureFile' blank ("") so that the script automatically
% selects the most recent 'rxCapture_test_*.mat' file from the current folder 
captureFile  = '';
baselineFile = 'rxCapture_baseline.mat';   % capture without fading

if isempty(captureFile)
    d = dir('rxCapture_test_*.mat');
    if isempty(d)
        error(["No rxCapture_test_*.mat file found in the current folder.\n" ...
            "See the SETUP block at the top of this script to generate a capture" ...
            "(enableFading=true) from Capture_hardwareLink_wFading.m."]);
    end
    [~, iLatest] = max([d.datenum]);
    captureFile = d(iLatest).name;
    fprintf('No `captureFile` specified -> use the most recent one : %s\n', captureFile);
end

% The baseline is used automatically if the file exists.
% (No longer need to set a flag to true manually — the cause of the
%  previous problem: useBaseline had remained set to false even though the file existed.)
useBaseline = isfile(baselineFile);

%% ------------------------------------------------------------------
%  0) Loading of the real RX capture
%% ------------------------------------------------------------------
if ~isfile(captureFile)
    error(["Capture file not found: %s\n" ...
        "See the SETUP block at the top of this script to generate this file" ...
        "from an actual capture on the hardware."], captureFile);
end

fprintf('=== Analyze of the real RX capture : %s ===\n\n', captureFile);
S = load(captureFile, 'rxCapture', 'fs', 'K_nominal', 'fadeRate_nominal');

rxCapture        = S.rxCapture(:);
fs               = S.fs;
K_nominal        = S.K_nominal;
fadeRate_nominal = S.fadeRate_nominal;

fprintf('      %d loaded samples (fs=%g Hz, duration=%.1f s)\n', ...
    numel(rxCapture), fs, numel(rxCapture)/fs);
fprintf('      Expected configuration : K=%.2f, fadeRate=%.2f Hz\n\n', ...
    K_nominal, fadeRate_nominal);

%% ------------------------------------------------------------------
%  0bis) Subtraction of hardware baseline (optional)
%% ------------------------------------------------------------------
baselineCapture = [];   % reused in Section 5 for the superimposed plot
baselineFs      = [];

if useBaseline
    Sb = load(baselineFile, 'rxCapture', 'fs');
    baselineCapture = Sb.rxCapture(:);
    baselineFs      = Sb.fs;
    baselineEnv     = abs(baselineCapture);
    fprintf('      Baseline found (%s) : %d loaded samples, fs=%g Hz.\n', ...
        baselineFile, numel(baselineCapture), baselineFs);
    fprintf('      Hardware baseline (fading disabled) : coefficient of variation = %.4f\n', ...
        std(baselineEnv)/mean(baselineEnv));
    fprintf('      (should be significantly lower than that measured with active fading below)\n\n');
else
    fprintf(['      No baseline file (%s) found in the current folder.\n' ...
        '      -> The graph in section 5 will only display the trace with fading enabled.\n' ...
        '      -> Generate a capture without fading (see SETUP, point 5) and place it\n' ...
        '         in the same folder as this script to enable comparison.\n\n'], baselineFile);
end

%% ------------------------------------------------------------------
%  1) Temporal decorrelation (same logic as test_RicianFading.m)
%% ------------------------------------------------------------------
Tc = 0.423 / fadeRate_nominal;
decimFactor = max(1, round(Tc * fs));
envelope = abs(rxCapture(1:decimFactor:end));

fprintf('[1/4] Estimated coherence time : %.3f s -> decimation by %d\n', Tc, decimFactor);
fprintf('      %d selected quasi-independent samples.\n\n', numel(envelope));

if numel(envelope) < 200
    warning(['Fewer than 200 quasi-independent samples available: ' ...
        'the statistical tests below will be unreliable. ' ...
        'Increase the capture duration (see SETUP at the top of the script).']);
end

%% ------------------------------------------------------------------
%  2) Actual RX envelope distribution vs. Rice's theoretical PDF
%% ------------------------------------------------------------------
fprintf("[2/4] Comparison of the actual RX histogram with Rice's theoretical PDF...\n");

omega = mean(envelope.^2);
nu    = sqrt(K_nominal/(K_nominal+1) * omega);
sigma = sqrt(omega / (2*(K_nominal+1)));

figure('Name', 'real RX - Distribution enveloppe', 'Position', [50 50 900 500]);
histogram(envelope, 60, 'Normalization', 'pdf', 'FaceColor', [0.9 0.5 0.2], ...
    'EdgeColor', 'none', 'DisplayName', 'Real RX envelope (hardware)');
hold on;
x = linspace(0, max(envelope)*1.1, 500);
plot(x, ricePDF(x, nu, sigma), 'r-', 'LineWidth', 2, 'DisplayName', " Expected Rice's PDF (K nominal)");
xlabel('RX envelope amplitude'); ylabel('Probability density');
title(sprintf('Real RX : K nominal=%.1f, fadeRate=%.1f Hz (%d ech.)', ...
    K_nominal, fadeRate_nominal, numel(envelope)));
legend('Location', 'best'); grid on;

cdf_handle = @(x) riceCDF(x, nu, sigma);
[h_ks, p_ks, ks_stat] = simpleKSTest(envelope, cdf_handle, 0.05);
fprintf('      K-S Test : statistics = %.4f, p-value = %.4f\n', ks_stat, p_ks);
if h_ks == 0
    fprintf('      -> OK : The hardware chain faithfully reproduces a Rice distribution.\n\n');
else
    fprintf('      -> PLEASE NOTE : A significantly different RX distribution from Rice''s is expected.\n\n');
end

%% ------------------------------------------------------------------
%  3) Estimation of the ACTUAL K-factor as seen by RX 
%% ------------------------------------------------------------------
fprintf('[3/4] Estimation of K-factor from real RX signal...\n');

estK_hw = estimateKFactor(envelope);
errPct = 100 * abs(estK_hw - K_nominal) / K_nominal;

fprintf('      K configured (software) : %.2f\n', K_nominal);
fprintf('      K measured (hardware RX) : %.2f\n', estK_hw);
fprintf('      Error                  : %.1f %%\n\n', errPct);

if errPct < 20
    fprintf('      -> OK : The K value measured at the output of the hardware chain is consistent with the configured K value.\n\n');
else
    fprintf(['      -> PLEASE NOTE: significant discrepancy between the configured K and the measured K on the actual RX.\n' ...
        '         Possible causes: attenuator latency too slow to keep up with fading,\n' ...
        '         insufficient quantisation/resolution of the attenuator, excessive RX noise,\n' ...
        '         amplifier non-linearities, or ADC saturation during gain peaks.\n\n']);
end

%% ------------------------------------------------------------------
%  4) Doppler spectrum measured from the actual RX signal
%% ------------------------------------------------------------------
fprintf('[4/4] Verification of the Doppler spectrum measured on the actual receiver...\n');

rx_ac = rxCapture - mean(rxCapture);

% Adaptive FFT resolution: the previous fixed upper limit (nfft=4096) gave
% df=244 Hz with fs=1 MHz, i.e. just a single bin in the +/-5 Hz band — far
% too coarse to distinguish anything from one run to the next.
% The aim here is to have at least ~10 bins within the band +/-fadeRate_nominal.
target_df   = max(fadeRate_nominal / 10, 0.01);           % Hz
nfft_target = 2^nextpow2(fs / target_df);
nfft_maxData = 2^nextpow2(numel(rx_ac) / 8);               % keep enough segments to calculate the average (pwelch)
nfft_cap    = 2^22;                                          % a reasonable upper limit for the calculation/memory
nfft = min([nfft_target, nfft_maxData, nfft_cap]);
[pxx, freq] = pwelch(rx_ac, hamming(nfft), nfft/2, nfft, fs, 'centered');

figure('Name', 'Real RX - Doppler Spectrum', 'Position', [980 50 900 500]);
plot(freq, 10*log10(pxx), 'b'); hold on;
xline(fadeRate_nominal, 'r--', 'LineWidth', 1.5, 'DisplayName', '+MaxDopplerShift expected');
xline(-fadeRate_nominal, 'r--', 'LineWidth', 1.5, 'DisplayName', '-MaxDopplerShift expected');
xlim([-8*fadeRate_nominal, 8*fadeRate_nominal]);
xlabel('Frequency (Hz)'); ylabel('DSP (dB/Hz)');
title('Doppler spectrum measured from the actual RX signal'); legend('Location', 'best'); grid on;

df = freq(2) - freq(1);
inBand = freq >= -fadeRate_nominal & freq <= fadeRate_nominal;
nBinsInBand = sum(inBand);
fracInBand = 100 * sum(pxx(inBand)) * df / (sum(pxx) * df);
fprintf('      Measured power in +/-%.1f Hz : %.1f %%\n', fadeRate_nominal, fracInBand);
fprintf('      FFT resolution : nfft=%d, df=%.4f Hz, bin number within the band [-%.1f, +%.1f] Hz = %d\n', ...
    nfft, df, fadeRate_nominal, fadeRate_nominal, nBinsInBand);
if nBinsInBand <= 2
    fprintf(['      -> PLEASE NOTE: the tape contains very few FFT bins (%d). The measured ratio %%%%\n' ...
        '         is unreliable and may remain virtually constant from one run to the next\n' ...
        '         regardless of the true fadeRate/K -- increase the capture duration (and thus nfft)\n' ...
        '         or decrease fs; otherwise, this figure does not really distinguish between your runs.\n'], nBinsInBand);
end
fprintf('\n');

%% ------------------------------------------------------------------
%  5) RX power versus time plot (visual inspection of the fades)
%% ------------------------------------------------------------------
% t = (0:numel(rxCapture)-1)' / fs;
% rxPower_dB = 20*log10(abs(rxCapture) + eps);
% 
% figure('Name', 'RX réel - Power vs time', 'Position', [50 620 1830 350]);
% plot(t, rxPower_dB, 'b', 'DisplayName', 'Fading enabled'); hold on; grid on;
% 
% if ~isempty(baselineCapture)
%     % Realign the baseline to its own time axis (durations may vary)
%     t_bl = (0:numel(baselineCapture)-1)' / baselineFs;
%     bl_dB = 20*log10(abs(baselineCapture) + eps);
% 
%     % Centres the baseline on the average level of the fading capture,
%     % to compare the VARIABILITY of the two traces rather than their
%     % absolute gain (which may differ slightly between the two runs).
%     bl_dB_aligned = bl_dB - mean(bl_dB) + mean(rxPower_dB);
% 
%     plot(t_bl, bl_dB_aligned, 'Color', [0.5 0.5 0.5], 'DisplayName', ...
%         'Baseline (fading disabled, realigned to the same average level)');
% 
%     % --- Quantitative analysis: standard deviation ratio ---
%     std_fading   = std(rxPower_dB);
%     std_baseline = std(bl_dB);
%     stdRatio     = std_fading / std_baseline;
% 
%     fprintf('      std(power dB) fading enabled : %.2f dB\n', std_fading);
%     fprintf('      std(power dB) baseline      : %.2f dB\n', std_baseline);
%     fprintf('      Ratio std(fading)/std(baseline) : %.2f\n', stdRatio);
%     if stdRatio < 2
%         fprintf(['      -> PLEASE NOTE : ratio close to 1 -> fading is not distinguishable\n' ...
%             '         from hardware noise. The test is not valid as it stands.\n\n']);
%     elseif stdRatio < 5
%         fprintf(['      -> Modest ratio (2–5x): the fading is visible but remains close to\n' ...
%             '         equipment noise. This result should be interpreted with caution.\n\n']);
%     else
%         fprintf(['      -> OK : ratio >= 5x, fading clearly outweighs the hardware noise.\n\n']);
%     end
% end
% 
% xlabel('Time (s)'); ylabel('Power RX (dB, relative scale)');
% title('RX power time history: visual inspection of fade troughs');
% legend('Location', 'best');
% 
% fprintf('=== End of analyze ===\n');


%% Modification power computation
%% ================================
% 5) Inspection of temporal fading
%% ================================

fprintf('[4/4] Verification of temporal fading on a smoothed envelope...\n');

% Smoothing window
L = max(1000, round(0.1 * Tc * fs));

% Moving average power
%blEnv = movmean(abs(baselineCapture).^2, L);
rxEnv = movmean(abs(rxCapture).^2, L);
if ~isempty(baselineCapture)
    blEnv = movmean(abs(baselineCapture).^2, L);
end

% Conversion to dB
rxPower_dB = 10*log10(rxEnv + eps);
blPower_dB = 10*log10(blEnv + eps);

% Removal of the average offset
rxPower_dB = rxPower_dB - mean(rxPower_dB);
blPower_dB = blPower_dB - mean(blPower_dB);

stdFading = std(rxPower_dB);
stdBaseline = std(blPower_dB);

%fprintf('      std(baseline)     : %.2f dB\n',stdBaseline);
%fprintf('      Ratio             : %.2f\n',stdFading/stdBaseline);
fprintf('      std(fading enabled) : %.2f dB\n', stdFading);
if ~isempty(baselineCapture)
    fprintf('      std(baseline)     : %.2f dB\n', stdBaseline);
    fprintf('      Ratio             : %.2f\n', stdFading/stdBaseline);
end

figure;
plot((0:length(rxPower_dB)-1)/fs,rxPower_dB)
hold on
plot((0:length(blPower_dB)-1)/fs,blPower_dB)
grid on
xlabel('Time (s)')
ylabel('Amplitude (dB)')
legend('Fading enabled','Baseline')
title('Average envelope')

%% ==================
%  Helper Functions
%% ==================

function pdf = ricePDF(x, nu, sigma)
    z = x*nu/sigma^2;
    pdf = (x ./ sigma^2) .* exp(-(x.^2 + nu^2) / (2*sigma^2)) .* besseli(0, z, 1) .* exp(z);
    pdf(x < 0) = 0;
end

function cdf = riceCDF(x, nu, sigma)
    cdf = 1 - marcumq(nu/sigma, x/sigma, 1);
    cdf(x < 0) = 0;
end

function [h, p, D] = simpleKSTest(data, cdfHandle, alpha)
    n = numel(data);
    sortedData = sort(data(:));
    cdfTheo = cdfHandle(sortedData);
    cdfTheo = cdfTheo(:);

    stepUp   = (1:n)' / n;
    stepDown = (0:n-1)' / n;

    D = max(max(abs(stepUp - cdfTheo)), max(abs(stepDown - cdfTheo)));

    lambda = (sqrt(n) + 0.12 + 0.11/sqrt(n)) * D;
    p = 0;
    for k = 1:100
        p = p + 2 * (-1)^(k-1) * exp(-2 * k^2 * lambda^2);
    end
    p = max(min(p, 1), 0);

    h = double(p < alpha);
end

function estK = estimateKFactor(envelope)
    r2 = envelope.^2;
    mu2 = mean(r2);
    mu4 = mean(r2.^2);

    t = mu4/mu2^2 - 1;

    if t <= 0
        estK = Inf;
    elseif t >= 1
        estK = 0;
    else
        estK = ((1 - t) + sqrt(1 - t)) / t;
    end
end