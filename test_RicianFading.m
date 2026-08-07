%% test_RicianFading.m
%
% Script for statistical validation of Rice fading used in
% hardwareLink_wFading.m, WITHOUT dependence on SDR hardware / attenuator.
%
% This script:
%   1) Generates channel gain frames via comm.RicianChannel
%   2) Checks that the envelope distribution follows a Rice distribution
%      with the expected K-factor (comparison of histogram vs theoretical PDF)
%   3) Re-estimates the K-factor using the method of moments and compares it
%      to the configured value
%   4) Checks the Doppler spectrum (should resemble the Jakes spectrum,
%      with energy concentrated within +/- MaximumDopplerShift)
%   5) Tests the boundary cases: very large K (virtually no fading),
%      K = 0 (should degenerate into Rayleigh), fadeRate = 0 (gains virtually
%      constant over time)
%
% No SDR connection or attenuator required: this script can
% run on any PC with the Communications Toolbox.

clear; clc; close all;

%% ------------------------------------------------------------------
%  Test parameters (from hardwareLink_wFading.m)
%% ------------------------------------------------------------------
fs              = 1e6;      % Sample rate (Hz), same as initial script
SamplesPerFrame = 16384;     % Same as hardwareLink.m
numFrames       = 300;      % Number of frames to accumulate for statistics
K_nominal       = 10;       % KFactor nominal (typical for SatCom)
fadeRate_nominal = 5;       % Maximum Doppler Shift (Hz), slow fading 

fprintf('=== Rice fading validation test ===\n\n');

%% ------------------------------------------------------------------
% Generation of channel gains (nominal case, at 'RF' fs as in hardwareLink.m) 
% — used for the Doppler spectrum test (§4),
%     which does not require independent samples.
%% ------------------------------------------------------------------
fprintf('[1/5] Generation of %d frames (K=%d, fadeRate=%d Hz)...\n', ...
    numFrames, K_nominal, fadeRate_nominal);

allGains_nominal = generateChannelGains(fs, SamplesPerFrame, numFrames, ...
    K_nominal, fadeRate_nominal);

fprintf('      %d generated gain samples.\n\n', numel(allGains_nominal));

%% ------------------------------------------------------------------
%  1bis) A generation DEDICATED to statistical testing (§2/§3)
%% ------------------------------------------------------------------
% IMPORTANT: Distribution tests (histogram, K-S, estimation of K)
% assume INDEPENDENT samples. However, the coherence time of the
% channel is ~= 0.4/fadeRate (~80 ms here), which is considerably longer than a frame
% of 16,384 samples at fs=1 MHz (16 ms) . As a result, the 4.9M samples
% from Section 1 actually represent only ~50–60 independent realisations
% -> a 'spiky' histogram and an invalid K-S test (as it
% assumes independence, whereas here n=4.9M with strong correlation
% yields p-values that are artificially close to 0).
%
% Solution: the fs/fadeRate ratio is what governs the statistics of
% fading (not the absolute RF value of fs). We therefore simulate here at a much
% lower fs, over a much longer total duration, to cover much
% more coherence time — at a much lower computational cost — and then we
% thin the data at coherence-time intervals to retain only samples
% that are quasi-independent.
fs_stat        = 2000;              % Hz (>> 2*fadeRate, enough for comm.RicianChannel)
frameLen_stat  = 2000;              % 1 second per frame
numFrames_stat = 1200;              % 1200 s (20 min) of fading simulation

fprintf('[1bis] Generation dedicated to statistical testing : %d s simulated at fs=%d Hz...\n', ...
    numFrames_stat, fs_stat);

gains_stat = generateChannelGains(fs_stat, frameLen_stat, numFrames_stat, ...
    K_nominal, fadeRate_nominal);

% Approximate coherence time (Tc ~= 0.423/fadeRate)
Tc = 0.423 / fadeRate_nominal;
decimFactor = max(1, round(Tc * fs_stat));
envelope = abs(gains_stat(1:decimFactor:end));

fprintf('      Estimated coherence time : %.3f s -> decimation by %d\n', Tc, decimFactor);
fprintf('      %d quasi-independent samples selected for §2/§3.\n\n', numel(envelope));

%% ------------------------------------------------------------------
%  2) Envelope distribution vs Rice's theoretical PDF
%% ------------------------------------------------------------------
fprintf('[2/5] Comparison of a histogram with the theoretical Rice PDF...\n');

% Parameters of Rice's law as a function of K and the
% average percentage of the path (0 dB -> total power = 1)
omega = mean(envelope.^2);              % Average total power (LOS + multipaths)
nu    = sqrt(K_nominal/(K_nominal+1) * omega);   % Amplitude of LOS
sigma = sqrt(omega / (2*(K_nominal+1)));         % Std deviation of multipaths

figure('Name', 'Distribution envelope - Rice', 'Position', [50 50 900 500]);
histogram(envelope, 100, 'Normalization', 'pdf', 'FaceColor', [0.3 0.6 0.9], ...
    'EdgeColor', 'none', 'DisplayName', 'Simulated envelope');
hold on;
x = linspace(0, max(envelope)*1.1, 500);
pdf_theo = ricePDF(x, nu, sigma);
plot(x, pdf_theo, 'r-', 'LineWidth', 2, 'DisplayName', "Rice's theoretical PDF");
xlabel('Envelope amplitude'); ylabel('Probability density');
title(sprintf('Envelope distribution : K=%d, fadeRate=%d Hz (%d quasi-indep. samples)', ...
    K_nominal, fadeRate_nominal, numel(envelope)));
legend('Location', 'best'); grid on;

% Kolmogorov–Smirnov test against the theoretical Rice CDF
cdf_handle = @(x) riceCDF(x, nu, sigma);
[h_ks, p_ks, ks_stat] = simpleKSTest(envelope, cdf_handle, 0.05);
fprintf('      K-S Test : statistic = %.4f, p-value = %.4f\n', ks_stat, p_ks);
if h_ks == 0
    fprintf('      -> OK : distribution consistent with Rice model (H0 not rejected at 5%%).\n\n');
else
    fprintf('      -> PLEASE NOTE: distribution differs significantly from Rice''s.\n\n');
end

%% ------------------------------------------------------------------
%  3) Re-estimation of the K-factor using the method of moments
%% ------------------------------------------------------------------
fprintf('[3/5] Estimation of K-factor using the method of moments...\n');

estK = estimateKFactor(envelope);
errPct = 100 * abs(estK - K_nominal) / K_nominal;

fprintf('      K nominal   : %.2f\n', K_nominal);
fprintf('      K estimated    : %.2f\n', estK);
fprintf('      Error      : %.1f %%\n\n', errPct);

if errPct < 15
    fprintf('      -> OK: Estimated K consistent with the configured K.\n\n');
else
    fprintf('      -> PLEASE NOTE: If there is a significant discrepancy, check the number of frames (statistics) or the configuration.\n\n');
end

%% ------------------------------------------------------------------
%  4) Verification of Doppler spectrum
%% ------------------------------------------------------------------
fprintf('[4/5] Verification of Doppler spectrum...\n');

% The LOS component (average) is removed to isolate the diffuse component
gains_ac = allGains_nominal - mean(allGains_nominal);

[pxx, freq] = pwelch(gains_ac, hamming(2048), 1024, 2048, fs, 'centered');

figure('Name', 'Doppler Spectrum', 'Position', [980 50 900 500]);
plot(freq, 10*log10(pxx), 'b'); hold on;
xline(fadeRate_nominal, 'r--', 'LineWidth', 1.5, 'DisplayName', '+MaxDopplerShift');
xline(-fadeRate_nominal, 'r--', 'LineWidth', 1.5, 'DisplayName', '-MaxDopplerShift');
xlim([-5*fadeRate_nominal, 5*fadeRate_nominal]);
xlabel('Frequency (Hz)'); ylabel('DSP (dB/Hz)');
title(sprintf('Doppler spectrum of fading (expected to be concentrated in +/-%d Hz)', fadeRate_nominal));
legend('Simulated spectrum', 'Location', 'best'); grid on;

% Power fraction contained within the band [-fadeRate, +fadeRate]
% (frequency-resolution-weighted sum; more robust than trapz
%  when the selected band is narrow / has few bins)
df = freq(2) - freq(1);
inBand = freq >= -fadeRate_nominal & freq <= fadeRate_nominal;
powerInBand = sum(pxx(inBand)) * df;
powerTotal  = sum(pxx) * df;
fracInBand  = 100 * powerInBand / powerTotal;
fprintf('      Power contained within +/-%d Hz : %.1f %%\n', fadeRate_nominal, fracInBand);
if fracInBand > 70
    fprintf('      -> OK : most of the energy lies within the expected Doppler band.\n\n');
else
    fprintf('      -> PLEASE NOTE : energy scattered outside the expected Doppler band.\n\n');
end

%% ------------------------------------------------------------------
%  5) Boundary cases
%% ------------------------------------------------------------------
fprintf('[5/5] Test of boundary cases...\n');

% --- Case A : K very large -> virtually no fading (because enableFading=false)) ---
K_large = 1e6;
gains_largeK = generateChannelGains(fs, SamplesPerFrame, 50, K_large, 0);
env_largeK = abs(gains_largeK);
varRatio_largeK = std(env_largeK) / mean(env_largeK);
fprintf('      [K=1e6, fadeRate=0]  Envelope coefficient of variation : %.4f (expected ~0)\n', varRatio_largeK);
if varRatio_largeK < 0.01
    fprintf('      -> OK: the channel is virtually static; there is no significant fading.\n');
else
    fprintf('      -> PLEASE NOTE: unexpected variation for very large K.\n');
end

% --- Case B : K = 0 -> must degenerate into pure Rayleigh fading ---
%
explicitly prohibits KFactor=0
% ("KFactor cannot be all-zero"). MathWorks enforces this because the object is
% dedicated to the LOS component; the purely diffuse case must be handled via
% comm.RayleighChannel. In practical terms, this means that if the actual
% pipeline in hardwareLink_wFading.m (lines ~41–43) were ever configured
% with K=0 (e.g. a SatCom link with no direct path / complete NLOS), it
% would crash in exactly the same way -> bear this in mind if K can
% vary dynamically in your chain.
%
% Here, therefore, we use comm.RayleighChannel as the 'true' reference for
% the limiting case K=0, using the same decorrelated generation as in §2/§3.
gains_K0_raw = generateRayleighGains(fs_stat, frameLen_stat, numFrames_stat, fadeRate_nominal);
env_K0 = abs(gains_K0_raw(1:decimFactor:end));
omega_K0 = mean(env_K0.^2);
sigma_K0 = sqrt(omega_K0/2);
pdf_theo_K0 = @(x) rayleighPDF(x, sigma_K0);
x0 = linspace(0, max(env_K0)*1.1, 500);
cdf_handle_K0 = @(x) 1 - exp(-x.^2 ./ (2*sigma_K0^2));
[h_ks0, p_ks0] = simpleKSTest(env_K0, cdf_handle_K0, 0.05);
fprintf('      [K=0] Test K-S vs Rayleigh : p-value = %.4f -> %s\n', p_ks0, ...
    ternary(h_ks0==0, 'OK, Rayleigh-compatible', 'PLEASE NOTE: significant discrepancy'));

figure('Name', 'Boundary case K=0 (Rayleigh)', 'Position', [50 620 900 400]);
histogram(env_K0, 100, 'Normalization', 'pdf', 'FaceColor', [0.6 0.3 0.9], ...
    'EdgeColor', 'none', 'DisplayName', 'Simulated envelope (K=0)');
hold on;
plot(x0, pdf_theo_K0(x0), 'r-', 'LineWidth', 2, 'DisplayName', 'Rayleigh''s theoretical PDF');
xlabel('Amplitude'); ylabel('Probability density');
title('Boundary case K=0 : must degenerate into Rayleigh'); legend('Location', 'best'); grid on;

% --- Case C : fadeRate = 0 -> gains that remain virtually constant over time ---
gains_noDoppler = generateChannelGains(fs, SamplesPerFrame, 50, K_nominal, 0);
driftRatio = std(abs(gains_noDoppler)) / mean(abs(gains_noDoppler));
fprintf('      [fadeRate=0] Coefficient of temporal variation : %.4f (expected to be low)\n', driftRatio);

fprintf('\n=== End of tests ===\n');

%% ====================================================================
%  Helper Fonctions
%% ====================================================================

function allGains = generateChannelGains(fs, SamplesPerFrame, numFrames, K, fadeRate)
% Generates and concatenates the path gains from comm.RicianChannel
% across multiple frames, by modulating the channel with a constant signal
% (normalised CW carrier) to observe pure fading.

    ricianChan = comm.RicianChannel( ...
        'SampleRate', fs, ...
        'KFactor', K, ...
        'MaximumDopplerShift', fadeRate, ...
        'PathDelays', 0, ...
        'AveragePathGains', 0, ...
        'DirectPathDopplerShift', 0, ...
        'DirectPathInitialPhase', 0, ...
        'FadingTechnique', 'Filtered Gaussian noise', ...
        'PathGainsOutputPort', true);

    testSig = ones(SamplesPerFrame, 1);   % Constant signal : observes the fading on its own
    allGains = complex(zeros(SamplesPerFrame*numFrames, 1));

    idx = 1;
    for i = 1:numFrames
        [~, pathGains] = ricianChan(testSig);
        allGains(idx:idx+SamplesPerFrame-1) = pathGains;
        idx = idx + SamplesPerFrame;
    end

    release(ricianChan);
end

function allGains = generateRayleighGains(fs, SamplesPerFrame, numFrames, fadeRate)
% Equivalent to `generateChannelGains`, but for the limit case where K=0
% (`comm.RicianChannel` does not allow `KFactor=0`, so we use
% `comm.RayleighChannel` directly, which IS the limit as K approaches 0 of the Rice distribution).

    rayleighChan = comm.RayleighChannel( ...
        'SampleRate', fs, ...
        'MaximumDopplerShift', fadeRate, ...
        'PathDelays', 0, ...
        'AveragePathGains', 0, ...
        'FadingTechnique', 'Filtered Gaussian noise', ...
        'PathGainsOutputPort', true);

    testSig = ones(SamplesPerFrame, 1);
    allGains = complex(zeros(SamplesPerFrame*numFrames, 1));

    idx = 1;
    for i = 1:numFrames
        [~, pathGains] = rayleighChan(testSig);
        allGains(idx:idx+SamplesPerFrame-1) = pathGains;
        idx = idx + SamplesPerFrame;
    end

    release(rayleighChan);
end

function pdf = ricePDF(x, nu, sigma)
% PDF of Rice distribution : f(x) = x/sigma^2 * exp(-(x^2+nu^2)/(2*sigma^2)) * I0(x*nu/sigma^2)
    pdf = (x ./ sigma^2) .* exp(-(x.^2 + nu^2) / (2*sigma^2)) .* besseli(0, x*nu/sigma^2, 1) ...
          .* exp(x*nu/sigma^2 - x*nu/sigma^2);                      % besseli(...,1) is already growing exponentially
    % Correction: besseli(0, z, 1) returns I0(z)*exp(-|z|), so we
    % need to add |z| back into the exponential to obtain the correct value.
    z = x*nu/sigma^2;
    pdf = (x ./ sigma^2) .* exp(-(x.^2 + nu^2) / (2*sigma^2)) .* besseli(0, z, 1) .* exp(z);
    pdf(x < 0) = 0;
end

function cdf = riceCDF(x, nu, sigma)
% Rice's CDF using Marcum's Q-function : CDF(x) = 1 - Q1(nu/sigma, x/sigma)
    cdf = 1 - marcumq(nu/sigma, x/sigma, 1);
    cdf(x < 0) = 0;
end

function pdf = rayleighPDF(x, sigma)
    pdf = (x ./ sigma^2) .* exp(-x.^2 ./ (2*sigma^2));
    pdf(x < 0) = 0;
end

function [h, p, D] = simpleKSTest(data, cdfHandle, alpha)
% One-sample Kolmogorov-Smirnov test
%
% data      : vector of observations
% cdfHandle : function handle @(x) returning the theoretical CDF
% alpha     : significance threshold (e.g. 0.05)
%
% Returns:
%   h: 1 if H0 is rejected (different distribution), 0 otherwise
%   p: asymptotic p-value (Kolmogorov approximation)
%   D: Kolmogorov-Smirnov statistic (maximum deviation between the empirical and theoretical CDFs)

    n = numel(data);
    sortedData = sort(data(:));
    cdfTheo = cdfHandle(sortedData);
    cdfTheo = cdfTheo(:);

    stepUp   = (1:n)' / n;
    stepDown = (0:n-1)' / n;

    D = max(max(abs(stepUp - cdfTheo)), max(abs(stepDown - cdfTheo)));

    % Asymptotic p-value via the Kolmogorov distribution
    lambda = (sqrt(n) + 0.12 + 0.11/sqrt(n)) * D;

    p = 0;
    for k = 1:100
        p = p + 2 * (-1)^(k-1) * exp(-2 * k^2 * lambda^2);
    end
    p = max(min(p, 1), 0);   % within [0,1]

    h = double(p < alpha);
end

function estK = estimateKFactor(envelope)
% K-factor moment estimator
%
% Derivation: for X ~ Rice(nu, sigma), setting omega = E[X^2] = nu^2 + 2*sigma^2
% and t = Var(X^2)/E[X^2]^2 = (1+2K)/(1+K)^2, we solve for K:
%   K = [(1 − t) + √(1 − t)] / t
% where t is in (0, 1]. As t approaches 1, K approaches 0 (Rayleigh); as t approaches 0, K approaches infinity.
    r2 = envelope.^2;
    mu2 = mean(r2);
    mu4 = mean(r2.^2);

    t = mu4/mu2^2 - 1;   % = Var(X^2) / E[X^2]^2, must be within [0,1] in theory

    if t <= 0
        estK = Inf;      % virtually no fading (zero variance observed)
    elseif t >= 1
        estK = 0;        % no detectable LOS component (near-Rayleigh)
    else
        estK = ((1 - t) + sqrt(1 - t)) / t;
    end
end

function out = ternary(cond, valTrue, valFalse)
    if cond
        out = valTrue;
    else
        out = valFalse;
    end
end