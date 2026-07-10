function [pointing_loss_dB, components] = tumbling_attenuation(t, options)
% TUMBLING_ATTENUATION Simulate CanX-2 antenna pointing loss from tumbling.
%
% This function wraps the full attitude-dynamics workflow (rigid-body
% tumble simulation, optional active tumble correction, antenna pointing
% loss, and independent SDR-side random fade).
%
% USAGE (from the MATLAB command window):
%   [t, attenuation_dB] = tumbling_attenuation(DurationSec);
%   [t, attenuation_dB] = tumbling_attenuation(DurationSec, MaxTumbleRateDeg=5, ...
%       EnableTumbleCorrection=true, ShowAnimation=true);
%
% NAME-VALUE ARGUMENTS (all optional, defaults shown below):
%   SatName                (string)  Display name of the satellite. Default: "CANX-2"
%   Jx                     (double)  Roll-axis moment of inertia (kg*m^2). Default: 0.0366
%   Jy                     (double)  Pitch-axis moment of inertia (kg*m^2). Default: 0.0300
%   Jz                     (double)  Yaw-axis moment of inertia (kg*m^2). Default: 0.0058
%   BeamwidthDeg           (double)  Antenna half-power beamwidth (deg). Default: 34
%   SampleTime             (double)  Time step for the sim/animation (s). Default: 1
%   MaxTumbleRateDeg       (double)  Max |initial rate| per axis, uniform [-max,max] (deg/s). Default: 2
%   FadeTauSec             (double)  Random-fade correlation time (s). Default: 5
%   FadeSigmaDB            (double)  Random-fade steady-state 1-sigma depth (dB). Default: 0.5
%   AttenuationCapDB       (double)  Sidelobe-floor attenuation cap (dB). Default: 30
%   ShowPlots              (logical) Show the pointing-error/attenuation plots. Default: true
%   ShowAnimation          (logical) Show the 3D tumble animation. Default: true
%   PlaybackSpeed          (double)  Animation speed multiple of real-time. Default: 20
%
% OUTPUT:
%   attenuation_dB  - Attenuation due to tumbling (pointing loss), in dB
%
%   components      - Struct containing intermediate tumble parameters:
%   MomentOfInertia                 Satellite moments of inertia [kg*m^2]
%   InitialPointingError_rad        Initial roll/pitch/yaw error [rad]
%   InitialAngularVelocity_rad_s    Initial angular velocity [rad/s]
%   off_axis_angle_deg              Boresight off-axis angle [deg]
%   pointing_loss_dB                Antenna pointing loss [dB]
%   attenuation_dB                  Total tumble attenuation [dB]

arguments
    t                              (:,1) double  

    options.SatName                (1,1) string  = "CANX-2"
    options.Jx                     (1,1) double  = 0.0366   % roll axis, kg*m^2 (approximation)
    options.Jy                     (1,1) double  = 0.0300   % pitch axis, kg*m^2 (approximation)
    options.Jz                     (1,1) double  = 0.0058   % yaw axis, kg*m^2 (approximation)
    options.BeamwidthDeg           (1,1) double  = 34       % antenna half-power beamwidth, deg (approximation)
    options.SampleTime             (1,1) double  = 1        % time step, s
    options.MaxTumbleRateDeg       (1,1) double  = 0.5        % max |initial rate| per axis, deg/s
    options.AttenuationCapDB       (1,1) double  = 30       % sidelobe-floor cap, dB (approximation)
    options.ShowPlots              (1,1) logical = true
    options.ShowAnimation          (1,1) logical = true
    options.PlaybackSpeed          (1,1) double  = 100       % animation speed, x real-time
end


% Satellite Properties
tspan = t;
J = [options.Jx, options.Jy, options.Jz]; % Moment of interia 

% Initial Conditions (moderate, non-extreme tumble)
% All three axes randomized with normal distribution
sigma = options.MaxTumbleRateDeg / 3;
omega0 = deg2rad(sigma * randn(3,1));

%generate inital pointing error
%ask prof what range he wants of tumble
%used 10, 10, 180 before, could use 360
roll0  = deg2rad(10*randn);
pitch0 = deg2rad(10*randn);
yaw0   = deg2rad(180*rand);

q0 = euler2quat(roll0,pitch0,yaw0); %initial point
state0 = [omega0; q0];

% Run Attitude Dynamics
opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
[t_ode,state] = ode45(@(t,y) satellite_dynamics(t,y,J), ...
                      [tspan(1) tspan(end)], state0, opts);
% Interpolate attitude solution onto CSV time grid
q_ode = state(:,4:7);
q = zeros(length(t),4);
for i = 1:4
    q(:,i) = interp1(t_ode, q_ode(:,i), t, 'linear');
end

% Normalize quaternions
for k = 1:size(q,1)
    q(k,:) = q(k,:)/norm(q(k,:));
end

% Antenna Geometry (body-frame pointing loss)
antenna_body = [0;0;-1];
ground_station_inertial = [0;0;-1]; 

num_steps = length(t);
off_axis_angle_deg = zeros(num_steps,1);
pointing_loss_dB    = zeros(num_steps,1);

for k = 1:num_steps
    qk = q(k,:)';
    R = quat2rotm_scalar(qk);
    antenna_inertial = R * antenna_body;

    dot_product = dot(antenna_inertial, ground_station_inertial);
    dot_product = max(-1, min(1, dot_product)); % clamp for numerical safety
    theta = acos(dot_product);
    off_axis_angle_deg(k) = rad2deg(theta);

    % Parabolic beam roll-off, capped at the sidelobe floor
    pointing_loss_dB(k) = min(options.AttenuationCapDB, 12*(off_axis_angle_deg(k)/options.BeamwidthDeg)^2);
end

components = struct( ...
    'MomentOfInertia', J, ...
    'InitialPointingError_rad', [roll0, pitch0, yaw0], ...
    'InitialAngularVelocity_rad_s', omega0, ...
    'off_axis_angle_deg', off_axis_angle_deg, ...
    'pointing_loss_dB', pointing_loss_dB);

% Plotting
if options.ShowPlots
    figure('Color','w','Position',[100 100 900 700])

    subplot(2,1,1)
    plot(t, off_axis_angle_deg, 'LineWidth',2)
    grid on
    title(sprintf('%s Antenna Pointing Error', options.SatName))
    xlabel('Time (s)'); ylabel('Off-Axis Angle (deg)')

    subplot(2,1,2)
    plot(t, pointing_loss_dB, '--', 'LineWidth',1.2); hold on
    plot(t, attenuation_dB, 'LineWidth',2);
    legend('Pointing loss','Location','best')
    grid on
    title('Resulting Signal Attenuation Profile')
    xlabel('Time (s)'); ylabel('Attenuation (dB)')
end

if options.ShowAnimation
    animate_tumble(t, q, antenna_body, ground_station_inertial, ...
        options.PlaybackSpeed, options.SatName);
end

end

%% ============================================================
% Helper Functions
% ============================================================

% 3D Tumble Animation
function animate_tumble(t, q, antenna_body, ground_station_inertial, playback_speed, sat_name)
% Renders the satellite body (approximated as a 10x10x34 cm box) rotating
% with the antenna boresight(red) and the fixed ground-station direction (green).

if nargin < 5
    playback_speed = 50;
end
if nargin < 6
    sat_name = "Satellite";
end

% Body dimensions (m)
Lx = 0.10; Ly = 0.10; Lz = 0.34; %for CANX2 Cubsat

% Box vertices in body frame, centered at origin
verts0 = 0.5*[ -Lx -Ly -Lz;  Lx -Ly -Lz;  Lx  Ly -Lz; -Lx  Ly -Lz; ...
               -Lx -Ly  Lz;  Lx -Ly  Lz;  Lx  Ly  Lz; -Lx  Ly  Lz];
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

fig = figure('Color','w','Position',[100 100 650 600]);
ax = axes('Parent',fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
lim = 0.4;
xlim(ax,[-lim lim]); ylim(ax,[-lim lim]); zlim(ax,[-lim lim]);
xlabel(ax,'X (inertial)'); ylabel(ax,'Y (inertial)'); zlabel(ax,'Z (inertial)');
view(ax,135,20);

bodyPatch = patch('Parent',ax,'Vertices',verts0,'Faces',faces, ...
    'FaceColor',[0.3 0.6 0.9],'FaceAlpha',0.7,'EdgeColor','k');

antennaLine = plot3(ax,[0 0],[0 0],[0 0],'r-','LineWidth',3);
gsLine = plot3(ax,[0 0],[0 0],[0 0],'g--','LineWidth',2);
titleHandle = title(ax,'');
legend(ax,[antennaLine, gsLine],{'Antenna boresight','Ground station direction'}, ...
    'Location','northoutside');

% Downsample frames for rendering
step = 10;
frame_idx = 1:step:length(t);

    while isvalid(fig)
    
        playback_clock = tic;
        sim_time_at_start = t(frame_idx(1));
    
        for k = frame_idx
    
            if ~isvalid(fig)
                return
            end
    
            qk = q(k,:)';
            R = quat2rotm_scalar(qk);
    
            verts_k = (R*verts0')';
            set(bodyPatch,'Vertices',verts_k);
    
            antenna_inertial = R*antenna_body*(lim*0.9);
            set(antennaLine,...
                'XData',[0 antenna_inertial(1)],...
                'YData',[0 antenna_inertial(2)],...
                'ZData',[0 antenna_inertial(3)]);
    
            gs_vec = ground_station_inertial*(lim*0.9);
            set(gsLine,...
                'XData',[0 gs_vec(1)],...
                'YData',[0 gs_vec(2)],...
                'ZData',[0 gs_vec(3)]);
    
            set(titleHandle,'String',sprintf( ...
                '%s Tumble Animation | t = %.0f s (%.0fx speed)', ...
                sat_name, t(k), playback_speed));
    
            drawnow
    
            target_wall_time = (t(k)-sim_time_at_start)/playback_speed;
            actual_wall_time = toc(playback_clock);
    
            if target_wall_time > actual_wall_time
                pause(target_wall_time-actual_wall_time)
            end
        end    
    end
end

% Satellite Rotational Dynamics
function dstate = satellite_dynamics(~,state,J)
omega = state(1:3);
q = state(4:7);
wx = omega(1); wy = omega(2); wz = omega(3);
Jx = J(1); Jy = J(2); Jz = J(3);
tau_c = [0;0;0];

domega = [
    ((Jy-Jz)/Jx)*wy*wz + tau_c(1)/Jx;
    ((Jz-Jx)/Jy)*wx*wz + tau_c(2)/Jy;
    ((Jx-Jy)/Jz)*wx*wy + tau_c(3)/Jz
];

Omega = [
    0   -wx -wy -wz;
    wx   0   wz -wy;
    wy  -wz  0   wx;
    wz   wy -wx   0
];
dq = 0.5*Omega*q;

dstate = [domega; dq];
end

% Quaternion to Rotation Matrix (scalar-first)
function R = quat2rotm_scalar(q)
q = q/norm(q);
q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
R = [
    q0^2 + q1^2 - q2^2 - q3^2, 2*(q1*q2 - q0*q3),         2*(q1*q3 + q0*q2);
    2*(q1*q2 + q0*q3),         q0^2 - q1^2 + q2^2 - q3^2, 2*(q2*q3 - q0*q1);
    2*(q1*q3 - q0*q2),         2*(q2*q3 + q0*q1),         q0^2 - q1^2 - q2^2 + q3^2
];
end

% Euler Angles to Quaternion (ZYX)
function q = euler2quat(roll,pitch,yaw)
cr = cos(roll/2);  sr = sin(roll/2);
cp = cos(pitch/2); sp = sin(pitch/2);
cy = cos(yaw/2);   sy = sin(yaw/2);

q0 = cr*cp*cy + sr*sp*sy;
q1 = sr*cp*cy - cr*sp*sy;
q2 = cr*sp*cy + sr*cp*sy;
q3 = cr*cp*sy - sr*sp*cy;

q = [q0;q1;q2;q3];
q = q/norm(q);
end