%% ALEASAT - UBC ground station simulation of Doppler Shift and Geometry
warning('off','all')

%% 1 - Set up a satellite scenario
% Specify the start time in UTC of the satellite scenario.
% startTime is chosen for graph display convinence, don't change
start_time = datetime(2026,5,1,1,30,00, 'TimeZone', 'UTC');
% Specify the simulation time
% stop_time = start_time + days(0.5);           % 5 days for longer simulation, can be changed
stop_time = start_time + hours(0.3);        % 0.3 hours for one path simulation, can be changed
% Set the sample time in seconds.
sample_time = 15;                           % change to 15 seconds for coarsing simulation.
ploting_time_pause = 0.5;                 % can use 1 to simulate a 1 second scatter plotting
% a vector of time from start_time to stop_time by interval of sample_time
time_vector = start_time:seconds(sample_time):stop_time;
% Create a satelliteScenario object
sc = satelliteScenario(start_time, stop_time, sample_time);
carrierFrequency = 437.3e6;                 % actual UHF frequency of ALEASAT

%% 2 - Add ALEASAT as a satellite to the scenario.
% these parameters are used in current ALEASIM built by the AOCS team
semiMajorAxis = 6828.14e3;            % In meters
eccentricity = 0.01;                  % unitless
inclination = 98;                     % In degrees
rightAscensionOfAscendingNode = 300;  % In degrees
argumentOfPeriapsis = 98;             % In degrees
trueAnomaly = 0;                      % In degrees
sat = satellite(sc,semiMajorAxis,eccentricity,inclination, ...
    rightAscensionOfAscendingNode,argumentOfPeriapsis,trueAnomaly, ...
    Name = "ALEASAT");

%% 3 - Add UBC gound station to the scenario.
lat = 49.261725;                     % latitude N
lon = -123.249569;                   % longtitude W
min_elevation_angle = 5.0;
gs = groundStation(sc, lat, lon, MaskElevationAngle = min_elevation_angle, Name = "UBC");

%% 4 - Add access analysis to the scenario
% obtain the table of intervals of access between ALEASAT and UBC ground station.
ac = access(sat, gs);
intvls = accessIntervals(ac);
% Play the scenario to visualize the ground stations.
play(sc, PlaybackSpeedMultiplier = sample_time)

%% 5 - Set up vectors to contain calculation result of doppler and angles.
% dopper shift in frequnecy by calling dopplershift function
time_out_vector = NaT(1, length(time_vector), 'TimeZone', 'UTC');
freq_doppler_vector = NaN(1, length(time_vector));
% relative velocity by calling dopplershift function
vel_doppler_vector = NaN(1, length(time_vector));
% angles between gs and sat by calling aer function
azimuth_angle_vector =  NaN(1, length(time_vector));
elevation_angle_vector = NaN(1, length(time_vector));
% distance between gs and sat by calling aer function
range_vector = NaN(1, length(time_vector));

%% 6 - setup plotting
figure('Name', 'ALEASAT Pass Metrics', 'Position', [100, 100, 800, 800]);
t = tiledlayout(3, 1);
line1 = 'Satellite Communication Parameters during simulated paths';
line2 = sprintf('%s - %s', start_time, stop_time);
sgtitle(t, {line1, line2});
markerSize = 10;
currentTimeText = annotation('textbox', [0.7, 0.93, 0.2, 0.05], ...
    'String', '', ...
    'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center', ...
    'FontSize', 12, ...
    'Color', 'black');

% Subplot 1: Doppler Shift vs. Time
ax1 = nexttile;
plot_freq = scatter(ax1, time_out_vector, freq_doppler_vector, markerSize, 'b');
title('Doppler Shift vs. Time');
xlim([time_vector(1)+minutes(4) time_vector(end)-minutes(5)]);
ylim([-12, 12]);
ylabel("Doppler Shift (kHz)");
grid on;

% Subplot 2: Elevation Angle vs. Time
ax2 = nexttile;
plot_elevation_angle = scatter(ax2, time_out_vector, elevation_angle_vector, markerSize, 'b');
title('Elevation Angle vs. Time');
xlim([time_vector(1)+minutes(4) time_vector(end)-minutes(5)]);
ylim([0, 90]); % Changed to 90 for full overhead pass visibility
ylabel("Elevation Angle (°)");
grid on;

% Subplot 3: Range vs. Time
ax3 = nexttile;
plot_range = scatter(ax3, time_out_vector, range_vector, markerSize, 'b');
title('Range vs. Time');
xlim([time_vector(1)+minutes(4) time_vector(end)-minutes(5)]);
xlabel("Simulation Time");
ylim([400, 2000]);
ylabel("Range (km)");
grid on;

linkaxes([ax1, ax2, ax3], 'x');  % Link x-axes so zooming affects all plots simultaneously

%% 7 - call each function every sample_time second
for s = 1:length(time_vector)
    % call aer function to calculate azimuth angle, elevation angle, and range
    [azimuth_angle_vector(s), elevation_angle, range_vector(s)] = ...
        aer(gs, sat, time_vector(s));
    
    elevation_angle_vector(s) = elevation_angle;
    
    % only calculate doppler shift when elevation_angle is greater than the
    % min_elevation_angle of the ground station
    if elevation_angle >= min_elevation_angle
        % call dopplershift function with the TimeIn argument
        [freq_doppler_vector(s), time_out_vector(s), DopplerInfoPerTime] = ...
            dopplershift(sat, gs, time_vector(s), Frequency = carrierFrequency);
        vel_doppler_vector(s) = DopplerInfoPerTime.RelativeVelocity;
    end
    
    % Update the displayed current time
    currentSimTime = datestr(time_vector(s), 'yyyy-mm-dd HH:MM:SS');
    set(currentTimeText, 'String', sprintf('Current Time: %s', currentSimTime));
    
    % Update the Doppler shift plot
    set(plot_freq, 'XData', time_out_vector(1:s), 'YData', freq_doppler_vector(1:s) / 1000);
    
    % Update the Elevation Angle plot
    set(plot_elevation_angle, 'XData', time_out_vector(1:s), 'YData', elevation_angle_vector(1:s));
    
    % Update the Range plot
    set(plot_range, 'XData', time_out_vector(1:s), 'YData', range_vector(1:s)/1000);
    
    % Refresh the plots
    drawnow;
    pause(ploting_time_pause)
end