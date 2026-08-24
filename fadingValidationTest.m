%% test_rician_validation_onoff.m
% Comparaison statistique "fading ON" vs "fading OFF" à partir de DEUX
% captures réelles issues de hardwareLink.m :
%   - une capture avec enableFadingToggle = true  (K configuré, ex. 10)
%   - une capture avec enableFadingToggle = false (K = 1e50, fading quasi nul)
%
% Objectif : démontrer statistiquement que le fading appliqué augmente
% significativement la variance de l'enveloppe du signal reçu/transmis,
% et que cette variance correspond à ce qui est attendu pour le K
% configuré (et non à du bruit de fond ou une erreur d'implémentation).
%
% Pré-requis : deux fichiers de capture générés par hardwareLink.m
% (voir bloc "Rician Fading Validation Capture"), idéalement dans les
% MÊMES conditions RF (même niveau de sortie du générateur, mêmes
% réglages de gain RX/TX, delay = 0, fShift = 0), à l'exception du
% paramètre enableFadingToggle.

clear; clc; close all;

%% ---------------------------------------------------------------------
%  1) Charger les deux captures
%  ---------------------------------------------------------------------
fprintf('Sélectionnez la capture avec FADING ACTIVÉ (enableFadingToggle = true)...\n');
[fileOn, pathOn] = uigetfile('rician_capture_*.mat', 'Capture FADING ON');
if isequal(fileOn, 0), error('Aucun fichier sélectionné pour la condition ON.'); end
S_on = load(fullfile(pathOn, fileOn));

fprintf('Sélectionnez la capture avec FADING DÉSACTIVÉ (enableFadingToggle = false)...\n');
[fileOff, pathOff] = uigetfile('rician_capture_*.mat', 'Capture FADING OFF');
if isequal(fileOff, 0), error('Aucun fichier sélectionné pour la condition OFF.'); end
S_off = load(fullfile(pathOff, fileOff));

fprintf('\nCapture ON  : K configuré = %.2f, fadeRate = %.2f Hz, %d trames\n', ...
    S_on.K_configured, S_on.fadeRate_configured, numel(S_on.rx_data_capture));
fprintf('Capture OFF : K configuré = %.2e, fadeRate = %.2f Hz, %d trames\n', ...
    S_off.K_configured, S_off.fadeRate_configured, numel(S_off.rx_data_capture));

if S_on.K_configured <= S_off.K_configured
    warning(['Le K configuré de la capture "ON" n''est pas inférieur à celui de la capture "OFF". ' ...
             'Vérifiez que vous n''avez pas inversé les deux fichiers.']);
end

%% ---------------------------------------------------------------------
%  2) Fonction utilitaire : extraction du gain d'enveloppe |tx|/|rx|
%  ---------------------------------------------------------------------
extractGain = @(S) local_extractGain(S.rx_data_capture, S.tx_data_capture);

gain_on  = extractGain(S_on);
gain_off = extractGain(S_off);

fprintf('\nÉchantillons exploitables : ON = %d, OFF = %d\n', numel(gain_on), numel(gain_off));

%% ---------------------------------------------------------------------
%  3) Statistiques descriptives comparées
%  ---------------------------------------------------------------------
fprintf('\n=== Statistiques descriptives ===\n');
fprintf('                Moyenne     Écart-type     Variance\n');
fprintf('Fading ON   :   %.4f      %.4f         %.6f\n', mean(gain_on),  std(gain_on),  var(gain_on));
fprintf('Fading OFF  :   %.4f      %.4f         %.6f\n', mean(gain_off), std(gain_off), var(gain_off));
fprintf('Ratio de variance (ON / OFF) : %.1fx\n', var(gain_on)/var(gain_off));

%% ---------------------------------------------------------------------
%  4) Test statistique formel d'égalité des variances (F-test)
%  ---------------------------------------------------------------------
%  H0 : les deux échantillons ont la même variance (pas d'effet du fading)
%  H1 : les variances diffèrent (le fading a un effet mesurable)
fprintf('\n=== Test F de comparaison de variances (vartest2) ===\n');
[h, p, ci, stats] = vartest2(gain_on, gain_off);

fprintf('Statistique F : %.2f (dl = %d, %d)\n', stats.fstat, stats.df1, stats.df2);
fprintf('p-value       : %.3e\n', p);
if h == 1
    fprintf(['--> H0 rejetée (p < 0.05) : la variance diffère significativement entre ' ...
             'les deux conditions. Le fading a bien un effet mesurable et statistiquement ' ...
             'significatif sur le signal réellement transmis/reçu.\n']);
else
    fprintf(['--> H0 NON rejetée : aucune différence de variance significative détectée. ' ...
             'Cela suggère un problème : le fading ne serait pas réellement appliqué, ou les ' ...
             'deux captures ne diffèrent pas seulement par enableFadingToggle.\n']);
end

%% ---------------------------------------------------------------------
%  5) Test non-paramétrique complémentaire (Levene, plus robuste à la
%     non-normalité que le F-test classique)
%  ---------------------------------------------------------------------
fprintf('\n=== Test de Levene (robuste à la non-normalité) ===\n');
allData  = [gain_on(:); gain_off(:)];
allGroup = [repmat({'ON'}, numel(gain_on), 1); repmat({'OFF'}, numel(gain_off), 1)];

% Levene = ANOVA à un facteur sur les écarts absolus à la médiane du groupe
med_on  = median(gain_on);
med_off = median(gain_off);
absdev  = [abs(gain_on(:) - med_on); abs(gain_off(:) - med_off)];
[p_levene, tbl_levene] = anova1(absdev, allGroup, 'off');
fprintf('p-value (Levene) : %.3e\n', p_levene);
if p_levene < 0.05
    fprintf('--> Confirme le test F : différence de dispersion statistiquement significative.\n');
else
    fprintf('--> Ne confirme pas le test F : à examiner (voir remarque ci-dessus).\n');
end

%% ---------------------------------------------------------------------
%  6) Visualisation comparative
%  ---------------------------------------------------------------------
figure('Name', 'Comparaison Fading ON vs OFF');

subplot(2,2,1);
histogram(gain_off, 80, 'Normalization', 'pdf', 'FaceColor', [0.3 0.6 0.9]); hold on;
histogram(gain_on,  80, 'Normalization', 'pdf', 'FaceColor', [0.9 0.4 0.3]);
legend('Fading OFF', 'Fading ON');
xlabel('Gain d''enveloppe |tx\_data|/|rx\_data|'); ylabel('Densité');
title('Distributions comparées');
grid on;

subplot(2,2,2);
boxplot([gain_off(1:min(end,5000)); gain_on(1:min(end,5000))], ...
        [repmat({'OFF'}, min(numel(gain_off),5000), 1); repmat({'ON'}, min(numel(gain_on),5000), 1)]);
ylabel('Gain d''enveloppe');
title('Boxplot comparatif (échantillon)');
grid on;

subplot(2,2,3);
t_on  = (0:numel(S_on.fade_dB_capture)-1)  * (numel(S_on.rx_data_capture{1})  / S_on.fs_capture);
t_off = (0:numel(S_off.fade_dB_capture)-1) * (numel(S_off.rx_data_capture{1}) / S_off.fs_capture);
plot(t_on, S_on.fade_dB_capture, 'r.-'); hold on;
plot(t_off, S_off.fade_dB_capture, 'b.-');
legend('Fading ON', 'Fading OFF');
xlabel('Temps (s)'); ylabel('fade\_dB (dB)');
title('fade\_dB loggé au cours du temps');
grid on;

subplot(2,2,4);
bar(categorical({'OFF','ON'}), [var(gain_off), var(gain_on)]);
ylabel('Variance du gain d''enveloppe');
title(sprintf('Variance (ratio ON/OFF = %.1fx, p = %.1e)', var(gain_on)/var(gain_off), p));
grid on;

fprintf('\n=== Fin de la comparaison ON / OFF ===\n');

%% ---------------------------------------------------------------------
%  Fonction locale
%  ---------------------------------------------------------------------
function gainVec = local_extractGain(rx_cell, tx_cell)
    gainVec = [];
    numFrames = numel(rx_cell);
    for k = 1:numFrames
        rx_k = rx_cell{k};
        tx_k = tx_cell{k};
        if numel(rx_k) ~= numel(tx_k)
            continue; % trame ignorée (probablement delay non nul pendant la capture)
        end
        validIdx = abs(rx_k) > (0.05 * rms(abs(rx_k)));
        gain_k = abs(tx_k(validIdx)) ./ abs(rx_k(validIdx));
        gainVec = [gainVec; gain_k(:)]; %#ok<AGROW>
    end
end