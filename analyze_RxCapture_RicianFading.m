%% analyze_RxCapture_RicianFading.m
%
% Analyse une capture IQ RÉELLE (sortie de la chaîne SDR TX -> atténuateur
% -> SDR RX) et lui applique les mêmes tests statistiques que
% test_RicianFading.m (distribution de Rice, K-factor, spectre Doppler),
% afin de valider que le fading OBSERVÉ EN SORTIE DE CHAÎNE MATÉRIELLE
% correspond bien au fading CONFIGURÉ en amont.
%
% ======================================================================
%  SETUP MATÉRIEL À METTRE EN PLACE AVANT DE CAPTURER LES DONNÉES
% ======================================================================
%
% 1) CÂBLAGE (boucle fermée / loopback)
%    USRP_TX (sortie RF) --> Atténuateur programmable --> USRP_RX (entrée RF)
%    Les deux USRP (RX et TX) doivent être verrouillés sur la MÊME référence
%    externe 10 MHz (REF OUT -> REF IN), exactement comme validé par
%    referenceLockedStatus(SDR_RX) dans hardwareLink_wFading.m.
%
% 2) SIGNAL DE TEST : UNE PORTEUSE CONSTANTE, PAS DE DONNÉES MODULÉES
%    Pour isoler statistiquement le fading pur (comme dans le test logiciel
%    où testSig = ones(SamplesPerFrame,1)), il faut TEMPORAIREMENT remplacer
%    les données modulées réelles par un signal d'amplitude constante
%    (CW / porteuse pure) AVANT le passage dans ricianChan, dans
%    applyDigitalImpairments. Sinon le contenu du signal (modulation,
%    décalage fréquentiel, etc.) contamine l'enveloppe mesurée et fausse
%    l'estimation de K côté RX.
%    -> Faites une copie de hardwareLink_wFading.m, et dans la boucle
%       principale, remplacez temporairement les données à moduler par
%       data = ones(SamplesPerFrame,1) (ou un ton CW) juste avant l'appel
%       à applyDigitalImpairments. Gardez le fShift/Doppler à 0 pour ce
%       test si possible, pour ne pas mélanger Doppler de trajectoire et
%       Doppler du fading.
%
% 3) CAPTURE DES TRAMES RX DANS UN FICHIER
%    Dans la boucle principale (autour de la ligne `rx_data = SDR_RX();`),
%    accumulez les trames reçues et sauvegardez-les à la fin du run :
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
% 4) DURÉE DE CAPTURE RECOMMANDÉE
%    Comme pour le test logiciel, il faut couvrir suffisamment de temps de
%    cohérence pour obtenir des statistiques fiables. Temps de cohérence
%    approx. Tc = 0.423/fadeRate. Viser au moins ~2000-3000 x Tc de capture
%    totale pour un bon histogramme (ex: fadeRate=5 Hz -> Tc=85ms -> viser
%    ~5-10 minutes de capture continue minimum). Attention à la mémoire :
%    à fs=1 MHz, 10 minutes = 600M échantillons complexes (~9.6 GB en
%    double) -> envisagez d'écrire par blocs sur disque (fwrite) plutôt
%    que tout garder en RAM si votre capture dépasse quelques minutes.
%
% 5) BASELINE SANS FADING (fortement recommandé)
%    Avant de capturer avec le fading actif, faites une capture de
%    référence avec le fading désactivé (enableFading=false, ou K très
%    grand comme dans le script principal) pour caractériser la variation
%    résiduelle propre au matériel seul (bruit RX, dérive de gain,
%    imperfections de l'atténuateur). Cette variation "plancher" doit être
%    négligeable devant la variation induite par le fading actif -- sinon
%    le test suivant ne pourra pas distinguer le fading du bruit matériel.
%    Sauvegardez cette capture dans un fichier séparé, ex: 'rxCapture_baseline.mat'.
%
% ======================================================================

clear; clc; close all;

%% ------------------------------------------------------------------
%  Paramètres : chemins des fichiers de capture à analyser
%% ------------------------------------------------------------------
captureFile  = 'rxCapture_test.mat';       % capture avec fading actif
baselineFile = 'rxCapture_baseline.mat';   % capture sans fading (optionnel mais recommandé)
useBaseline  = false;   % passer à true si baselineFile existe et doit être utilisé

%% ------------------------------------------------------------------
%  0) Chargement de la capture RX réelle
%% ------------------------------------------------------------------
if ~isfile(captureFile)
    error(['Fichier de capture introuvable : %s\n' ...
        'Voir le bloc SETUP en haut de ce script pour générer ce fichier ' ...
        'depuis une capture réelle sur le matériel.'], captureFile);
end

fprintf('=== Analyse de la capture RX réelle : %s ===\n\n', captureFile);
S = load(captureFile, 'rxCapture', 'fs', 'K_nominal', 'fadeRate_nominal');

rxCapture        = S.rxCapture(:);
fs               = S.fs;
K_nominal        = S.K_nominal;
fadeRate_nominal = S.fadeRate_nominal;

fprintf('      %d échantillons chargés (fs=%g Hz, durée=%.1f s)\n', ...
    numel(rxCapture), fs, numel(rxCapture)/fs);
fprintf('      Configuration attendue : K=%.2f, fadeRate=%.2f Hz\n\n', ...
    K_nominal, fadeRate_nominal);

%% ------------------------------------------------------------------
%  0bis) Soustraction de la baseline matérielle (optionnel)
%% ------------------------------------------------------------------
if useBaseline
    if ~isfile(baselineFile)
        warning('Baseline demandée mais fichier introuvable (%s) -> ignorée.', baselineFile);
    else
        Sb = load(baselineFile, 'rxCapture', 'fs');
        baselineEnv = abs(Sb.rxCapture(:));
        fprintf('      Baseline matérielle (fading désactivé) : coefficient de variation = %.4f\n', ...
            std(baselineEnv)/mean(baselineEnv));
        fprintf('      (doit être largement inférieur à celui mesuré avec fading actif ci-dessous)\n\n');
    end
end

%% ------------------------------------------------------------------
%  1) Décorrélation temporelle (même logique que test_RicianFading.m)
%% ------------------------------------------------------------------
Tc = 0.423 / fadeRate_nominal;
decimFactor = max(1, round(Tc * fs));
envelope = abs(rxCapture(1:decimFactor:end));

fprintf('[1/4] Temps de cohérence estimé : %.3f s -> décimation par %d\n', Tc, decimFactor);
fprintf('      %d échantillons quasi-indépendants retenus.\n\n', numel(envelope));

if numel(envelope) < 200
    warning(['Moins de 200 échantillons quasi-indépendants disponibles : ' ...
        'les tests statistiques ci-dessous seront peu fiables. ' ...
        'Augmentez la durée de capture (voir SETUP en haut du script).']);
end

%% ------------------------------------------------------------------
%  2) Distribution d'enveloppe RX réelle vs pdf de Rice théorique
%% ------------------------------------------------------------------
fprintf('[2/4] Comparaison histogramme RX réel vs pdf de Rice théorique...\n');

omega = mean(envelope.^2);
nu    = sqrt(K_nominal/(K_nominal+1) * omega);
sigma = sqrt(omega / (2*(K_nominal+1)));

figure('Name', 'RX réel - Distribution enveloppe', 'Position', [50 50 900 500]);
histogram(envelope, 60, 'Normalization', 'pdf', 'FaceColor', [0.9 0.5 0.2], ...
    'EdgeColor', 'none', 'DisplayName', 'Enveloppe RX réelle (hardware)');
hold on;
x = linspace(0, max(envelope)*1.1, 500);
plot(x, ricePDF(x, nu, sigma), 'r-', 'LineWidth', 2, 'DisplayName', 'PDF de Rice attendue (K nominal)');
xlabel('Amplitude de l''enveloppe RX'); ylabel('Densité de probabilité');
title(sprintf('RX réel : K nominal=%.1f, fadeRate=%.1f Hz (%d ech.)', ...
    K_nominal, fadeRate_nominal, numel(envelope)));
legend('Location', 'best'); grid on;

cdf_handle = @(x) riceCDF(x, nu, sigma);
[h_ks, p_ks, ks_stat] = simpleKSTest(envelope, cdf_handle, 0.05);
fprintf('      Test K-S : statistique = %.4f, p-value = %.4f\n', ks_stat, p_ks);
if h_ks == 0
    fprintf('      -> OK : la chaîne hardware reproduit fidèlement une distribution de Rice.\n\n');
else
    fprintf('      -> ATTENTION : distribution RX significativement différente de Rice attendue.\n\n');
end

%% ------------------------------------------------------------------
%  3) Estimation du K-factor RÉEL vu par le RX
%% ------------------------------------------------------------------
fprintf('[3/4] Estimation du K-factor à partir du signal RX réel...\n');

estK_hw = estimateKFactor(envelope);
errPct = 100 * abs(estK_hw - K_nominal) / K_nominal;

fprintf('      K configuré (logiciel) : %.2f\n', K_nominal);
fprintf('      K mesuré (hardware RX) : %.2f\n', estK_hw);
fprintf('      Écart                  : %.1f %%\n\n', errPct);

if errPct < 20
    fprintf('      -> OK : le K mesuré en sortie de chaîne matérielle est cohérent avec le K configuré.\n\n');
else
    fprintf(['      -> ATTENTION : écart important entre K configuré et K mesuré sur le RX réel.\n' ...
        '         Causes possibles : latence de l''atténuateur trop lente pour suivre le fading,\n' ...
        '         quantification/résolution insuffisante de l''atténuateur, bruit RX excessif,\n' ...
        '         non-linéarités des amplis, ou saturation ADC pendant les pics de gain.\n\n']);
end

%% ------------------------------------------------------------------
%  4) Spectre Doppler mesuré sur le signal RX réel
%% ------------------------------------------------------------------
fprintf('[4/4] Vérification du spectre Doppler mesuré sur le RX réel...\n');

rx_ac = rxCapture - mean(rxCapture);
nfft = min(4096, 2^nextpow2(numel(rx_ac)/8));
[pxx, freq] = pwelch(rx_ac, hamming(nfft), nfft/2, nfft, fs, 'centered');

figure('Name', 'RX réel - Spectre Doppler', 'Position', [980 50 900 500]);
plot(freq, 10*log10(pxx), 'b'); hold on;
xline(fadeRate_nominal, 'r--', 'LineWidth', 1.5, 'DisplayName', '+MaxDopplerShift attendu');
xline(-fadeRate_nominal, 'r--', 'LineWidth', 1.5, 'DisplayName', '-MaxDopplerShift attendu');
xlim([-8*fadeRate_nominal, 8*fadeRate_nominal]);
xlabel('Fréquence (Hz)'); ylabel('DSP (dB/Hz)');
title('Spectre Doppler mesuré sur le signal RX réel'); legend('Location', 'best'); grid on;

df = freq(2) - freq(1);
inBand = freq >= -fadeRate_nominal & freq <= fadeRate_nominal;
fracInBand = 100 * sum(pxx(inBand)) * df / (sum(pxx) * df);
fprintf('      Puissance mesurée dans +/-%.1f Hz : %.1f %%\n\n', fadeRate_nominal, fracInBand);

%% ------------------------------------------------------------------
%  5) Trace de puissance RX vs temps (inspection visuelle des fades)
%% ------------------------------------------------------------------
t = (0:numel(rxCapture)-1)' / fs;
rxPower_dB = 20*log10(abs(rxCapture) + eps);

figure('Name', 'RX réel - Puissance vs temps', 'Position', [50 620 1830 350]);
plot(t, rxPower_dB, 'b'); grid on;
xlabel('Temps (s)'); ylabel('Puissance RX (dB, échelle relative)');
title('Trace temporelle de la puissance RX : inspection visuelle des creux de fade');

fprintf('=== Fin de l''analyse ===\n');

%% ====================================================================
%  Fonctions locales (identiques à test_RicianFading.m)
%% ====================================================================

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