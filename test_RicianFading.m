%% test_RicianFading.m
%
% Script de validation statistique du fading de Rice utilisé dans
% hardwareLink_wFading.m, SANS dépendance au matériel SDR / atténuateur.
%
% Ce script :
%   1) Génère des trames de gains de canal via comm.RicianChannel
%   2) Vérifie que la distribution d'enveloppe suit une loi de Rice
%      avec le K-factor attendu (comparaison histogramme vs pdf théorique)
%   3) Ré-estime le K-factor par la méthode des moments et le compare
%      à la valeur configurée
%   4) Vérifie le spectre Doppler (doit ressembler au spectre de Jakes,
%      énergie concentrée dans +/- MaximumDopplerShift)
%   5) Teste les cas limites : K tres grand (quasi pas de fading),
%      K = 0 (doit degenerer en Rayleigh), fadeRate = 0 (gains quasi
%      constants dans le temps)
%
% Aucune connexion SDR, aucun atténuateur requis : ce script peut
% tourner sur n'importe quel PC avec la Communications Toolbox.

clear; clc; close all;

%% ------------------------------------------------------------------
%  Paramètres de test (repris de hardwareLink_wFading.m)
%% ------------------------------------------------------------------
fs              = 1e6;      % Sample rate (Hz), identique au script principal
SamplesPerFrame = 16384;    % Taille de trame, identique au script principal
numFrames       = 300;      % Nombre de trames à accumuler pour les stats

K_nominal       = 10;       % KFactor nominal (SatCom typique)
fadeRate_nominal = 5;       % Maximum Doppler Shift (Hz), fading lent

fprintf('=== Test de validation du fading de Rice ===\n\n');

%% ------------------------------------------------------------------
%  1) Génération des gains de canal (cas nominal, à fs "RF" comme le
%     script principal) -- utilisé pour le test du spectre Doppler (§4),
%     qui lui n'a pas besoin d'échantillons indépendants.
%% ------------------------------------------------------------------
fprintf('[1/5] Génération de %d trames (K=%d, fadeRate=%d Hz)...\n', ...
    numFrames, K_nominal, fadeRate_nominal);

allGains_nominal = generateChannelGains(fs, SamplesPerFrame, numFrames, ...
    K_nominal, fadeRate_nominal);

fprintf('      %d échantillons de gain générés.\n\n', numel(allGains_nominal));

%% ------------------------------------------------------------------
%  1bis) Génération DÉDIÉE aux tests statistiques (§2/§3)
%% ------------------------------------------------------------------
% IMPORTANT : les tests de distribution (histogramme, K-S, estimation de K)
% supposent des échantillons INDÉPENDANTS. Or le temps de cohérence du
% canal est ~= 0.4/fadeRate (~80 ms ici), largement plus long qu'une trame
% de 16384 échantillons à fs=1 MHz (16 ms). Résultat : les 4.9M échantillons
% de la section 1 ne représentent en réalité que ~50-60 réalisations
% indépendantes -> histogramme "en pics" et test K-S invalide (car il
% suppose l'indépendance, alors qu'ici n=4.9M avec forte corrélation
% donne des p-values artificiellement proches de 0).
%
% Solution : le ratio fs/fadeRate est ce qui gouverne les statistiques du
% fading (pas la valeur RF absolue de fs). On simule donc ici à un fs bien
% plus bas, sur une durée totale bien plus longue, pour couvrir beaucoup
% plus de temps de cohérence -- à coût de calcul très inférieur -- puis on
% décime au pas du temps de cohérence pour ne garder que des échantillons
% quasi indépendants.
fs_stat        = 2000;              % Hz (>> 2*fadeRate, largement suffisant pour comm.RicianChannel)
frameLen_stat  = 2000;              % 1 seconde par trame
numFrames_stat = 1200;              % 1200 s (20 min) de fading simulé

fprintf('[1bis] Génération dédiée aux tests statistiques : %d s simulées à fs=%d Hz...\n', ...
    numFrames_stat, fs_stat);

gains_stat = generateChannelGains(fs_stat, frameLen_stat, numFrames_stat, ...
    K_nominal, fadeRate_nominal);

% Temps de cohérence approximatif (règle de Clarke : Tc ~= 0.423/fadeRate)
Tc = 0.423 / fadeRate_nominal;
decimFactor = max(1, round(Tc * fs_stat));
envelope = abs(gains_stat(1:decimFactor:end));

fprintf('      Temps de cohérence estimé : %.3f s -> décimation par %d\n', Tc, decimFactor);
fprintf('      %d échantillons quasi-indépendants retenus pour §2/§3.\n\n', numel(envelope));

%% ------------------------------------------------------------------
%  2) Distribution d'enveloppe vs pdf théorique de Rice
%% ------------------------------------------------------------------
fprintf('[2/5] Comparaison histogramme vs pdf de Rice théorique...\n');

% Paramètres de la loi de Rice en fonction de K et de la puissance
% moyenne du trajet (ici 0 dB -> puissance totale = 1)
omega = mean(envelope.^2);              % Puissance totale moyenne (LOS + diffus)
nu    = sqrt(K_nominal/(K_nominal+1) * omega);   % Amplitude du trajet direct (LOS)
sigma = sqrt(omega / (2*(K_nominal+1)));         % Écart-type des trajets diffus

figure('Name', 'Distribution enveloppe - Rice', 'Position', [50 50 900 500]);
histogram(envelope, 100, 'Normalization', 'pdf', 'FaceColor', [0.3 0.6 0.9], ...
    'EdgeColor', 'none', 'DisplayName', 'Enveloppe simulée');
hold on;
x = linspace(0, max(envelope)*1.1, 500);
pdf_theo = ricePDF(x, nu, sigma);
plot(x, pdf_theo, 'r-', 'LineWidth', 2, 'DisplayName', 'PDF de Rice théorique');
xlabel('Amplitude de l''enveloppe'); ylabel('Densité de probabilité');
title(sprintf('Distribution d''enveloppe : K=%d, fadeRate=%d Hz (%d ech. quasi-indep.)', ...
    K_nominal, fadeRate_nominal, numel(envelope)));
legend('Location', 'best'); grid on;

% Test de Kolmogorov-Smirnov contre la CDF de Rice théorique
% (implémentation maison, ne nécessite pas la Statistics Toolbox)
cdf_handle = @(x) riceCDF(x, nu, sigma);
[h_ks, p_ks, ks_stat] = simpleKSTest(envelope, cdf_handle, 0.05);
fprintf('      Test K-S : statistique = %.4f, p-value = %.4f\n', ks_stat, p_ks);
if h_ks == 0
    fprintf('      -> OK : distribution compatible avec Rice (H0 non rejetée à 5%%).\n\n');
else
    fprintf('      -> ATTENTION : distribution significativement différente de Rice.\n\n');
end

%% ------------------------------------------------------------------
%  3) Ré-estimation du K-factor par la méthode des moments
%% ------------------------------------------------------------------
fprintf('[3/5] Estimation du K-factor par la méthode des moments...\n');

estK = estimateKFactor(envelope);
errPct = 100 * abs(estK - K_nominal) / K_nominal;

fprintf('      K nominal   : %.2f\n', K_nominal);
fprintf('      K estimé    : %.2f\n', estK);
fprintf('      Erreur      : %.1f %%\n\n', errPct);

if errPct < 15
    fprintf('      -> OK : K estimé cohérent avec le K configuré.\n\n');
else
    fprintf('      -> ATTENTION : écart important, vérifier le nombre de trames (statistique) ou la config.\n\n');
end

%% ------------------------------------------------------------------
%  4) Vérification du spectre Doppler
%% ------------------------------------------------------------------
fprintf('[4/5] Vérification du spectre Doppler...\n');

% On retire la composante LOS (moyenne) pour isoler la partie diffuse
gains_ac = allGains_nominal - mean(allGains_nominal);

[pxx, freq] = pwelch(gains_ac, hamming(2048), 1024, 2048, fs, 'centered');

figure('Name', 'Spectre Doppler', 'Position', [980 50 900 500]);
plot(freq, 10*log10(pxx), 'b'); hold on;
xline(fadeRate_nominal, 'r--', 'LineWidth', 1.5, 'DisplayName', '+MaxDopplerShift');
xline(-fadeRate_nominal, 'r--', 'LineWidth', 1.5, 'DisplayName', '-MaxDopplerShift');
xlim([-5*fadeRate_nominal, 5*fadeRate_nominal]);
xlabel('Fréquence (Hz)'); ylabel('DSP (dB/Hz)');
title(sprintf('Spectre Doppler du fading (attendu concentré dans +/-%d Hz)', fadeRate_nominal));
legend('Spectre simulé', 'Location', 'best'); grid on;

% Fraction de puissance contenue dans la bande [-fadeRate, +fadeRate]
% (somme pondérée par la résolution fréquentielle, plus robuste que trapz
%  quand la bande sélectionnée est étroite / peu de bins)
df = freq(2) - freq(1);
inBand = freq >= -fadeRate_nominal & freq <= fadeRate_nominal;
powerInBand = sum(pxx(inBand)) * df;
powerTotal  = sum(pxx) * df;
fracInBand  = 100 * powerInBand / powerTotal;
fprintf('      Puissance contenue dans +/-%d Hz : %.1f %%\n', fadeRate_nominal, fracInBand);
if fracInBand > 70
    fprintf('      -> OK : la majorité de l''énergie est dans la bande Doppler attendue.\n\n');
else
    fprintf('      -> ATTENTION : énergie dispersée hors de la bande Doppler attendue.\n\n');
end

%% ------------------------------------------------------------------
%  5) Cas limites
%% ------------------------------------------------------------------
fprintf('[5/5] Test des cas limites...\n');

% --- Cas A : K très grand -> quasi pas de fading (comme enableFading=false) ---
K_large = 1e6;
gains_largeK = generateChannelGains(fs, SamplesPerFrame, 50, K_large, 0);
env_largeK = abs(gains_largeK);
varRatio_largeK = std(env_largeK) / mean(env_largeK);
fprintf('      [K=1e6, fadeRate=0]  Coefficient de variation enveloppe : %.4f (attendu ~0)\n', varRatio_largeK);
if varRatio_largeK < 0.01
    fprintf('      -> OK : le canal est quasi-statique, pas de fading significatif.\n');
else
    fprintf('      -> ATTENTION : variation inattendue pour K très grand.\n');
end

% --- Cas B : K = 0 -> doit dégénérer en fading de Rayleigh pur ---
%
% NOTE IMPORTANTE : comm.RicianChannel interdit explicitement KFactor=0
% ("KFactor cannot be all-zero"). MathWorks impose ça car l'objet est
% dédié à la composante LOS ; le cas purement diffus doit passer par
% comm.RayleighChannel. Concrètement, cela signifie que si le pipeline
% réel de hardwareLink_wFading.m (ligne ~41-43) était un jour configuré
% avec K=0 (ex: liaison SatCom sans trajet direct / NLOS complet), il
% plantera exactement de la même façon -> à garder en tête si K peut
% varier dynamiquement dans votre chaîne.
%
% Ici on utilise donc comm.RayleighChannel comme référence "vraie" pour
% le cas limite K=0, avec la même génération décorrélée que §2/§3.
gains_K0_raw = generateRayleighGains(fs_stat, frameLen_stat, numFrames_stat, fadeRate_nominal);
env_K0 = abs(gains_K0_raw(1:decimFactor:end));
omega_K0 = mean(env_K0.^2);
sigma_K0 = sqrt(omega_K0/2);
pdf_theo_K0 = @(x) rayleighPDF(x, sigma_K0);
x0 = linspace(0, max(env_K0)*1.1, 500);
cdf_handle_K0 = @(x) 1 - exp(-x.^2 ./ (2*sigma_K0^2));
[h_ks0, p_ks0] = simpleKSTest(env_K0, cdf_handle_K0, 0.05);
fprintf('      [K=0] Test K-S vs Rayleigh : p-value = %.4f -> %s\n', p_ks0, ...
    ternary(h_ks0==0, 'OK, compatible Rayleigh', 'ATTENTION, écart significatif'));

figure('Name', 'Cas limite K=0 (Rayleigh)', 'Position', [50 620 900 400]);
histogram(env_K0, 100, 'Normalization', 'pdf', 'FaceColor', [0.6 0.3 0.9], ...
    'EdgeColor', 'none', 'DisplayName', 'Enveloppe simulée (K=0)');
hold on;
plot(x0, pdf_theo_K0(x0), 'r-', 'LineWidth', 2, 'DisplayName', 'PDF Rayleigh théorique');
xlabel('Amplitude'); ylabel('Densité de probabilité');
title('Cas limite K=0 : doit dégénérer en Rayleigh'); legend('Location', 'best'); grid on;

% --- Cas C : fadeRate = 0 -> gains quasi constants dans le temps ---
gains_noDoppler = generateChannelGains(fs, SamplesPerFrame, 50, K_nominal, 0);
driftRatio = std(abs(gains_noDoppler)) / mean(abs(gains_noDoppler));
fprintf('      [fadeRate=0] Coefficient de variation temporel : %.4f (attendu faible)\n', driftRatio);

fprintf('\n=== Fin des tests ===\n');

%% ====================================================================
%  Fonctions locales
%% ====================================================================

function allGains = generateChannelGains(fs, SamplesPerFrame, numFrames, K, fadeRate)
% Génère et concatène les gains de trajet issus de comm.RicianChannel
% sur plusieurs trames, en excitant le canal avec un signal constant
% (porteuse CW normalisée) pour observer le fading pur.

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

    testSig = ones(SamplesPerFrame, 1);   % Signal constant : observe le fading seul
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
% Équivalent de generateChannelGains, mais pour le cas limite K=0
% (comm.RicianChannel interdit KFactor=0, donc on utilise directement
% comm.RayleighChannel, qui EST la limite K->0 de la distribution de Rice).

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
% PDF de la distribution de Rice : f(x) = x/sigma^2 * exp(-(x^2+nu^2)/(2*sigma^2)) * I0(x*nu/sigma^2)
    pdf = (x ./ sigma^2) .* exp(-(x.^2 + nu^2) / (2*sigma^2)) .* besseli(0, x*nu/sigma^2, 1) ...
          .* exp(x*nu/sigma^2 - x*nu/sigma^2);  % besseli(...,1) est déjà à échelle exponentielle
    % Correction : besseli(0, z, 1) retourne I0(z)*exp(-|z|), donc on
    % doit ré-ajouter |z| dans l'exponentielle pour obtenir la vraie valeur.
    z = x*nu/sigma^2;
    pdf = (x ./ sigma^2) .* exp(-(x.^2 + nu^2) / (2*sigma^2)) .* besseli(0, z, 1) .* exp(z);
    pdf(x < 0) = 0;
end

function cdf = riceCDF(x, nu, sigma)
% CDF de Rice via la fonction Q de Marcum : CDF(x) = 1 - Q1(nu/sigma, x/sigma)
    cdf = 1 - marcumq(nu/sigma, x/sigma, 1);
    cdf(x < 0) = 0;
end

function pdf = rayleighPDF(x, sigma)
    pdf = (x ./ sigma^2) .* exp(-x.^2 ./ (2*sigma^2));
    pdf(x < 0) = 0;
end

function [h, p, D] = simpleKSTest(data, cdfHandle, alpha)
% Test de Kolmogorov-Smirnov à un échantillon, implémentation maison
% (ne nécessite pas la Statistics and Machine Learning Toolbox).
%
% data      : vecteur d'observations
% cdfHandle : fonction handle @(x) donnant la CDF théorique
% alpha     : seuil de significativité (ex: 0.05)
%
% Retourne :
%   h : 1 si H0 rejetée (distribution différente), 0 sinon
%   p : p-value asymptotique (approximation de Kolmogorov)
%   D : statistique de Kolmogorov-Smirnov (écart max entre CDF empirique et théorique)

    n = numel(data);
    sortedData = sort(data(:));
    cdfTheo = cdfHandle(sortedData);
    cdfTheo = cdfTheo(:);

    stepUp   = (1:n)' / n;
    stepDown = (0:n-1)' / n;

    D = max(max(abs(stepUp - cdfTheo)), max(abs(stepDown - cdfTheo)));

    % p-value asymptotique via la distribution de Kolmogorov
    % (approximation de Stephens, standard pour un test K-S à un échantillon)
    lambda = (sqrt(n) + 0.12 + 0.11/sqrt(n)) * D;

    p = 0;
    for k = 1:100
        p = p + 2 * (-1)^(k-1) * exp(-2 * k^2 * lambda^2);
    end
    p = max(min(p, 1), 0);   % borne numérique dans [0,1]

    h = double(p < alpha);
end

function estK = estimateKFactor(envelope)
% Estimateur des moments du K-factor (Greenwood / Durgin), formule correcte.
%
% Dérivation : pour X ~ Rice(nu, sigma), en posant omega = E[X^2] = nu^2+2*sigma^2
% et t = Var(X^2)/E[X^2]^2 = (1+2K)/(1+K)^2, on résout pour K :
%   K = [(1-t) + sqrt(1-t)] / t
% avec t dans (0,1]. t->1 correspond à K->0 (Rayleigh), t->0 correspond à K->Inf.
    r2 = envelope.^2;
    mu2 = mean(r2);
    mu4 = mean(r2.^2);

    t = mu4/mu2^2 - 1;   % = Var(X^2) / E[X^2]^2, doit être dans (0,1] en théorie

    if t <= 0
        estK = Inf;      % quasi pas de fading (variance nulle observée)
    elseif t >= 1
        estK = 0;        % pas de composante LOS détectable (proche Rayleigh)
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