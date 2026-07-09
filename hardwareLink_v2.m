%% Programmable Attenuator Link

clear; clc; 

% Hardware Connection Parameters
att_port = "COM3";                                                               % Check COM Port; 3 is for Howard
att_baudrate = 115200;       
test_channel = 1;
interPassDelay = 30; %simulated seconds between passes

% Import Path Loss from CSV - sélection d'un seul fichier

[csvFile, csvPath] = uigetfile('*.csv', 'Sélectionner un fichier CSV');
if isequal(csvFile, 0)
    error("No file selected.");
end

selpath = csvPath;
thisFile = fullfile(csvPath, csvFile);
fprintf('Reading %s...\n', csvFile);

numFiles = 1;
passData = {readtimetable(thisFile)};
passNames = string(csvFile);

fprintf('Loaded pass file: %s\n', csvFile);

% % Initialise Programmable Attenuator
% fprintf("Opening serial connection to attenuator on %s...\n", att_port);
% att = initProgATT(att_port, att_baudrate);
% % Releases COM Port When Script Ends or Has Errors Mid-Run
% cleanupAtt=onCleanup(@() clear('att')); 

%% Set up the live-plotting figure (created once, reused/reset for each pass)
% Column mapping confirmed from CSV header:
%   1=t, 2=Range_m, 3=Azimuth_deg, 4=Elevation_deg, 5=PathLoss_dB, 6=Delay_s, 7=Doppler_Hz, 8=Rel_Velocity_mps
markerSize = 10;

liveFig = figure('Name', 'Live Pass Metrics', 'Position', [100, 100, 900, 750]);
tl = tiledlayout(liveFig, 4, 1);
liveTitle = sgtitle(tl, 'Live playback of recorded pass data');

ax1 = nexttile(tl);
plot_rng = scatter(ax1, NaT, NaN, markerSize, 'b', 'filled');
title(ax1, 'Range vs. Time'); ylabel(ax1, 'Range (km)'); grid(ax1, 'on');

ax2 = nexttile(tl);
plot_pl = scatter(ax2, NaT, NaN, markerSize, 'b', 'filled');
title(ax2, 'Path Loss vs. Time'); ylabel(ax2, 'Path Loss (dB)'); grid(ax2, 'on');

ax3 = nexttile(tl);
plot_delay = scatter(ax3, NaT, NaN, markerSize, 'b', 'filled');
title(ax3, 'Delay vs. Time'); ylabel(ax3, 'Delay (ms)'); grid(ax3, 'on');

ax4 = nexttile(tl);
plot_dop = scatter(ax4, NaT, NaN, markerSize, 'b', 'filled');
title(ax4, 'Doppler Shift vs. Time'); ylabel(ax4, 'Doppler (kHz)'); xlabel(ax4, 'Time'); grid(ax4, 'on');

linkaxes([ax1, ax2, ax3, ax4], 'x');

for i = 1:length(passData)
    csv_filename = fullfile(selpath, passNames(i));
    csv_table = readtable(csv_filename);

    % Extract and Re-map the CSV Columns to Fit the Program Layout
    totalPoints = height(csv_table);
    attenuationTS = zeros(totalPoints, 2);
    
    % Convert the Datetime Column Into Relative Elapsed Seconds Starting at 0
    raw_times = datetime(csv_table{:, 1}); 
    attenuationTS(:,1) = seconds(raw_times - raw_times(1)); 
    
    % Extract Attenuation Directly from Column 5 (PathLoss_dB)
    attenuationTS(:,2) = csv_table{:, 5};
    
    % Normalise Dynamic Attenuation Control by In-line Losses 
    fixed_att = 100;                                                                 % 150 in DCETest
    attenuationTS(:,2) = round(attenuationTS(:,2)/0.25)*0.25 - fixed_att;

    % --- Columns used for live plotting (matches exported CSV header) ---
    range_col    = csv_table{:, 2} / 1000;   % Range_m -> km
    pathloss_col = csv_table{:, 5};          % PathLoss_dB
    delay_col    = csv_table{:, 6} * 1000;   % Delay_s -> ms
    doppler_col  = csv_table{:, 7} / 1000;   % Doppler_Hz -> kHz

    % Reset plot buffers for this pass
    plot_times = NaT(totalPoints, 1, 'TimeZone', raw_times.TimeZone);
    rng_buf   = NaN(totalPoints, 1);
    pl_buf    = NaN(totalPoints, 1);
    delay_buf = NaN(totalPoints, 1);
    dop_buf   = NaN(totalPoints, 1);

    set(liveTitle, 'String', sprintf('Live playback — %s', passNames(i)));
    set(plot_rng,   'XData', NaT, 'YData', NaN);
    set(plot_pl,    'XData', NaT, 'YData', NaN);
    set(plot_delay, 'XData', NaT, 'YData', NaN);
    set(plot_dop,   'XData', NaT, 'YData', NaN);
    xlim([ax1, ax2, ax3, ax4], [raw_times(1), raw_times(end)]);

    % --- Fixer les axes Y sur toute la durée du tracé, d'après les valeurs du CSV sélectionné ---
    setFixedYLim(ax1, range_col);
    setFixedYLim(ax2, pathloss_col);
    setFixedYLim(ax3, delay_col);
    setFixedYLim(ax4, doppler_col);

    drawnow;
    
    % Real-time Effect Application Loop
    disp("Beginning playback loop.");
    loopTimer = tic;
    effectIndex = 1;
    totalPoints = size(attenuationTS, 1);
    
    n = 0;                                                                          % Counter Used For Testing
    
    while (effectIndex <= totalPoints)
        % Check if the physical stopwatch has caught up to the next timestamp
        if (attenuationTS(effectIndex, 1) <= toc(loopTimer))
    
            current_db = attenuationTS(effectIndex, 2);
            
            n = n + 0;                                                              % Used For Testing
            current_db = current_db + n;                                            % USed For Testing
    
            % Prevent sending negative numbers or out-of-bounds values to hardware
            current_db = max(0, current_db); 
    
            fprintf("Time: %.2fs | Row: %d/%d | Setting Attenuator: %.2f dB\n", ...
                toc(loopTimer), effectIndex, totalPoints, current_db);
    
            % % Send command to physical hardware
            % setAttenuation(att, test_channel, current_db);

            % --- Update live plot buffers up to the current row and redraw ---
            plot_times(effectIndex) = raw_times(effectIndex);
            rng_buf(effectIndex)   = range_col(effectIndex);
            pl_buf(effectIndex)    = pathloss_col(effectIndex);
            delay_buf(effectIndex) = delay_col(effectIndex);
            dop_buf(effectIndex)   = doppler_col(effectIndex);

            set(plot_rng,   'XData', plot_times(1:effectIndex), 'YData', rng_buf(1:effectIndex));
            set(plot_pl,    'XData', plot_times(1:effectIndex), 'YData', pl_buf(1:effectIndex));
            set(plot_delay, 'XData', plot_times(1:effectIndex), 'YData', delay_buf(1:effectIndex));
            set(plot_dop,   'XData', plot_times(1:effectIndex), 'YData', dop_buf(1:effectIndex));
            drawnow limitrate;
    
            % Move to the next row in the CSV profile
            effectIndex = effectIndex + 1;
        end
    
        % Pause to prevent thrashing the laptop CPU kernel
        pause(0.01); 
    end

    % Fixed interpass delay before the next pass begins
    if i < length(passData)
        fprintf('Waiting %.1f s until next pass...\n', interPassDelay);
        pause(interPassDelay);
    end
end

clear att; 
disp("Profile playback complete.");


%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Helper Functions %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%

% Fixe l'axe Y d'un subplot en fonction du min/max des données de la passe,
% avec une petite marge pour la lisibilité (5% de l'étendue, mini 1 unité)
function setFixedYLim(ax, dataCol)
    yMax = max(dataCol, [], 'omitnan');
    yMin = min(dataCol, [], 'omitnan');

    if isempty(yMax) || isempty(yMin) || isnan(yMax) || isnan(yMin)
        return; % pas de données valides, on laisse l'auto-scale par défaut
    end

    span = yMax - yMin;
    if span == 0
        margin = max(abs(yMax) * 0.05, 1);
    else
        margin = span * 0.05;
    end

    ylim(ax, [yMin - margin, yMax + margin]);
end

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
