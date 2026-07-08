%% Programmable Attenuator Link

clear; clc; 

% Hardware Connection Parameters
att_port = "COM3";                                                               % Check COM Port; 3 is for Howard
att_baudrate = 115200;       
test_channel = 1;
interPassDelay = 30; %simulated seconds between passes

% Import Path Loss from CSV

selpath = uigetdir;
if isequal(selpath, 0)
    error("No folder selected.");
end

csvFiles = dir(fullfile(selpath, '*.csv'));
if isempty(csvFiles)
    error("No CSV files found in: %s", selpath);
end

numFiles = numel(csvFiles);
passData = cell(numFiles, 1);
passNames = strings(numFiles, 1);

% Build Pass Data Array
for k = 1:numFiles
    thisFile = fullfile(csvFiles(k).folder, csvFiles(k).name);
    fprintf('Reading %s...\n', csvFiles(k).name);

    passData{k} = readtimetable(thisFile);
    passNames(k) = string(csvFiles(k).name);
end

fprintf('Loaded %d pass file(s) from %s\n', numFiles, selpath);

% Initialise Programmable Attenuator
fprintf("Opening serial connection to attenuator on %s...\n", att_port);
att = initProgATT(att_port, att_baudrate);
% Releases COM Port When Script Ends or Has Errors Mid-Run
cleanupAtt=onCleanup(@() clear('att')); 

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
    
            % Send command to physical hardware
            setAttenuation(att, test_channel, current_db);
    
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
