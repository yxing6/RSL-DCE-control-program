%% Programmable Attenuator Link

clear; clc; 

% Hardware Connection Parameters
att_port = "COM3";                                                               % Check COM Port; 3 is for Howard
att_baudrate = 115200;       
test_channel = 1;

% Import Path Loss from CSV
csv_filename = "CANX2_Pass_1_20260707_221135.csv";                               % Only Pass 1 Implemented
if ~exist(csv_filename, 'file')
    error("Could not find CSV.", csv_filename);
end
disp("Reading attenuation profile from CSV...");
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

% Initialise Programmable Attenuator
fprintf("Opening serial connection to attenuator on %s...\n", att_port);
att = initProgATT(att_port, att_baudrate);

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