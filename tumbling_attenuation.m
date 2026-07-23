function [components] = tumbling_attenuation(t, freq, options)
% TUMBLING_ATTENUATION Simulate antenna pointing loss due to CubeSat tumbling.
%
% This function simulates the attitude dynamics of a tumbling CubeSat and
% computes the resulting antenna pointing loss over time. The user can
% specify the satellite dimensions, mass properties, tumble severity,
% antenna type, and visualization options. The output includes the
% time-varying pointing loss along with intermediate attitude and pointing
% parameters.
%
% INPUTS:
%   t      - Time vector (s)
%   freq   - Operating frequency (Hz)
%
% NAME-VALUE ARGUMENTS (optional, defaults shown in the arguments block):
%   SatName             Display name of the satellite.
%   SatDimensions       Satellite dimensions (m).
%   Mass                Satellite mass (kg).
%   AntennaType         Antenna radiation pattern model. Supported options:
%                         - "Half-Wave Dipole"
%                         - "Quarter-Wave Monopole"
%                         - "Dish"
%   DishRadius          Dish radius (m), used only for the parabolic dish.
%   AttenuationCapDB    Maximum pointing loss applied to the radiation
%                       pattern (dB).
%   ShowPlots           Display pointing error and attenuation plots.
%   ShowAnimation       Display a 3-D spacecraft tumble animation.
%   PlaybackSpeed       Animation playback speed relative to real time.
%   TestCase            Test Cases. Supported options:
%                         - "stable"
%                         - "drift"
%                         - "deployment"
%                         - "end-over-end"
%                         - "extreme"
%
% OUTPUTS:
%   pointing_loss_dB    Antenna pointing loss (dB) as a function of time.
%
%   components          Structure containing intermediate simulation data:
%       TumbleType                  Selected tumble case.
%       MomentOfInertia             Principal moments of inertia (kg·m²).
%       InitialPointingError_rad    Initial attitude quaternion.
%       InitialAngularVelocity_rad_s Initial angular velocity vector (rad/s).
%       off_axis_angle_deg          Antenna boresight off-axis angle (deg).
%       pointing_loss_dB            Pointing loss (dB).

%%
arguments
    t                              (:,1) double                     % s
    freq                           (1,1) double                     % Hz
    options.SatName                (1,1) string  = "CANX-2"
    options.SatDimensions          (1,3) double  = [0.1, 0.1, 0.3]  % m
    options.Mass                   (1,1) double  = 4                % kg

    options.AntennaType            (1,1) string  = "Half-Wave Dipole"
    options.AntennaOrientation     (1,1) string  = "-z"

    options.DishRadius             (1,1) double = 1;             % m
    options.AttenuationCapDB       (1,1) double  = 60               % dB

    options.ShowPlots              (1,1) logical = true
    options.ShowAnimation          (1,1) logical = false
    options.PlaybackSpeed          (1,1) double  = 5                % animation speed, x real-time
    options.TestCase               (1,1) string = "stable" 
end

% Wavenumber
k0=2*pi/freq2wavelen(freq);
% Dish Radius (if applicable)
r=options.DishRadius;

%%

% Test Cases
switch lower(options.TestCase)

    case "stable"
        % Nominal Earth-pointing with negligible residual motion
        roll0  = 0;
        pitch0 = 0;
        yaw0   = 0;

        omega0 = deg2rad([0; 0; 0.1]);

    case "drift"
        % Small pointing error with slow attitude drift
        roll0  = deg2rad(3);
        pitch0 = deg2rad(-2);
        yaw0   = deg2rad(5);

        omega0 = deg2rad([0.2; 0.1; 0.4]);

    case "deployment"
        % Typical post-deployment tumble
        roll0  = deg2rad(360*rand);
        pitch0 = asin(2*rand - 1);
        yaw0   = deg2rad(360*rand);

        omega_axis = randn(3,1);
        omega_axis = omega_axis / norm(omega_axis);
        omega0 = deg2rad(60) * omega_axis;

    case "end-over-end"
        % Rotation about principal axis
        roll0  = 0;
        pitch0 = 0;
        yaw0   = 0;

        omega0 = deg2rad([90; 0; 0]);

    case "extreme"
        % High-rate tumble
        roll0  = deg2rad(360*rand);
        pitch0 = asin(2*rand - 1);
        yaw0   = deg2rad(360*rand);

        omega_axis = randn(3,1);
        omega_axis = omega_axis / norm(omega_axis);
        omega0 = deg2rad(180) * omega_axis;

    otherwise
        error('Unknown tumble scenario: %s', options.TestCase);

end


fprintf("\n--- CubeSat Tumbling Simulation ---\n")
fprintf("Satellite: %s\n", options.SatName)
fprintf("Scenario: %s\n", options.TestCase)
fprintf("Antenna: %s (%s mount)\n",...
    options.AntennaType,...
    options.AntennaOrientation)
fprintf("Frequency: %.2f MHz\n",freq/1e6)
fprintf("----------------------------------\n\n")

% Initial pointing error as a quatrion
q0 = euler2quat(roll0,pitch0,yaw0);

% Initial state
state0 = [omega0; q0];

m=options.Mass;
a=options.SatDimensions(1);
b=options.SatDimensions(2);
c=options.SatDimensions(3);

% Calculate principal moments of inertia
Jx = (1/12)*m*(b^2+c^2); % roll axis, kg*m^2
Jy = (1/12)*m*(a^2+c^2); % pitch axis, kg*m^2
Jz = (1/12)*m*(a^2+b^2); % yaw axis, kg*m^2
J = [Jx, Jy, Jz]; % Principal moments of inertia vector (assumes off-diagonal elements of the matrix are zero)

% Run Attitude Dynamics
opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
[t_ode,state] = ode45(@(t,y) satellite_dynamics(t,y,J), ...
                      [t(1) t(end)], state0, opts);
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

% Antenna and Ground Station Geometry (for pointing loss)
antenna_body = get_antenna_orientation(options.AntennaOrientation);
ground_station_inertial = [0;0;-1]; 

num_steps = length(t);
off_axis_angle_deg = zeros(num_steps,1);
pointing_loss_dB    = zeros(num_steps,1);

for k = 1:num_steps
    qk = q(k,:)';
    R = quat2rotm_scalar(qk);
    antenna_inertial = R * antenna_body;

    dot_product = dot(antenna_inertial, ground_station_inertial);
    dot_product = max(-1, min(1, dot_product)); % clamp
    theta = acos(dot_product);
    off_axis_angle_deg(k) = rad2deg(theta);

    %Calulate gain based on antenna type, user can add unique radiation
    %patterns for patch, helical etc. antennas
    switch lower(options.AntennaType)
        case "half-wave dipole"
            gain = (cos(pi/2*cos(theta))/sin(theta))^2;
            gain = max(gain, 10^(-options.AttenuationCapDB/10));
            pointing_loss_dB(k) = -10*log10(gain);    
        case "quarter-wave monopole"
            gain = (cos(pi/2*cos(theta))/sin(theta))^2;
            gain = max(gain, 10^(-options.AttenuationCapDB/10));
            pointing_loss_dB(k) = -10*log10(gain);    
        case "dish"
            if theta > pi/2
                gain = 10^(-options.AttenuationCapDB/10);   % rear hemisphere
            else
                u = k0*r*sin(theta);
                if abs(u) < 1e-10
                    E = 1;
                else
                    E = 2*besselj(1,u)/u;
                end
                gain = E^2;
            end
            gain = max(gain,10^(-options.AttenuationCapDB/10));
            pointing_loss_dB(k) = -10*log10(gain);
    end

end

%% Plot Antenna Pattern

% Radiation pattern
switch lower(options.AntennaType)
    case "half-wave dipole"
        theta_plot = linspace(0,pi,100);
        phi_plot   = linspace(0,2*pi,100);
        [theta_plot,phi_plot] = meshgrid(theta_plot,phi_plot);
        gain_plot = (cos(pi/2*cos(theta_plot))./sin(theta_plot)).^2;
    case "quarter-wave monopole"
        theta_plot = linspace(0,pi,100);
        phi_plot   = linspace(0,2*pi,100);
        [theta_plot,phi_plot] = meshgrid(theta_plot,phi_plot);
        gain_plot = (cos(pi/2*cos(theta_plot))./sin(theta_plot)).^2;
    case "dish"
        theta_plot = linspace(0,pi/2,100);
        phi_plot   = linspace(0,2*pi,100);
        [theta_plot,phi_plot] = meshgrid(theta_plot,phi_plot);
        u = k0*r*sin(theta_plot);
        gain_plot = ones(size(u));
        idx = abs(u)>1e-10;
        gain_plot(idx) = ...
            (2*besselj(1,u(idx))./u(idx)).^2;
end
gain_plot(~isfinite(gain_plot)) = 0;
gain_plot = gain_plot/max(gain_plot(:));

% Convert gain to radius
rho = sqrt(gain_plot);
X = rho.*sin(theta_plot).*cos(phi_plot);
Y = rho.*sin(theta_plot).*sin(phi_plot);
Z = rho.*cos(theta_plot);

% Rotate +Z antenna axis to actual mounting direction
z_axis = [0;0;1];
v = cross(z_axis,antenna_body);
s = norm(v);
c = dot(z_axis,antenna_body);

if s > 1e-10
    vx = [ 0 -v(3) v(2);
           v(3) 0 -v(1);
          -v(2) v(1) 0];
    R_ant = eye(3)+vx+vx^2*((1-c)/s^2);
else
    R_ant = eye(3);
end
P = R_ant*[X(:)';Y(:)';Z(:)'];
X = reshape(P(1,:),size(X));
Y = reshape(P(2,:),size(Y));
Z = reshape(P(3,:),size(Z));

figure
surf(X,Y,Z,gain_plot,'EdgeColor','none')

axis equal
grid on
xlabel("X")
ylabel("Y")
zlabel("Z")

title(sprintf("%s (%s mount)",...
    options.AntennaType,...
    options.AntennaOrientation))

colorbar
view(45,30)
%%

components = struct( ...
    'TestCase', options.TestCase, ...
    'MomentOfInertia', J, ...
    'InitialPointingError_rad', [roll0, pitch0, yaw0], ...
    'InitialAngularVelocity_rad_s', omega0', ...
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
    hold on
    plot(t, pointing_loss_dB, '-', 'LineWidth',1.2); hold on
    legend(sprintf('%s', options.AntennaType))
    grid on
    title('Pointing Loss Profile')
    xlabel('Time (s)'); ylabel('Attenuation (dB)')
end

    if options.ShowAnimation
        animate_tumble(t, q, a, b, c, antenna_body, ground_station_inertial, ...
            options.PlaybackSpeed, options.SatName);
    end

end

%% ============================================================
% Helper Functions
% ============================================================

% 3D Tumble Animation
function animate_tumble(t, q, Lx, Ly, Lz, antenna_body, ground_station_inertial, playback_speed, sat_name)
% Renders the satellite body rotating
% with the antenna boresight(red) and the fixed ground-station direction (green).

if nargin < 5
    playback_speed = 50;
end
if nargin < 6
    sat_name = "Satellite";
end

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
wx = omega(1); 
wy = omega(2); 
wz = omega(3);
Jx = J(1); 
Jy = J(2); 
Jz = J(3);

tau_c = [0;0;0]; %external torque

% Euler rotational dynamics (only valid if J is diagonal)
% Angular velocity derivative
domega = [
    ((Jy-Jz)/Jx)*wy*wz + tau_c(1)/Jx;
    ((Jz-Jx)/Jy)*wx*wz + tau_c(2)/Jy;
    ((Jx-Jy)/Jz)*wx*wy + tau_c(3)/Jz
];

% Quaternion propagation matrix
Omega = [
    0   -wx -wy -wz;
    wx   0   wz -wy;
    wy  -wz  0   wx;
    wz   wy -wx   0
];
% Quaternion derivative
dq = 0.5*Omega*q;
dstate = [domega; dq];
end

% Quaternion to Rotation Matrix
function R = quat2rotm_scalar(q)
q = q/norm(q); %normalize quaternion
q0 = q(1); 
q1 = q(2); 
q2 = q(3); 
q3 = q(4);
% build rotation matrix
R = [
    q0^2 + q1^2 - q2^2 - q3^2, 2*(q1*q2 - q0*q3),         2*(q1*q3 + q0*q2);
    2*(q1*q2 + q0*q3),         q0^2 - q1^2 + q2^2 - q3^2, 2*(q2*q3 - q0*q1);
    2*(q1*q3 - q0*q2),         2*(q2*q3 + q0*q1),         q0^2 - q1^2 - q2^2 + q3^2
];
end

% Euler Angles to Quaternion
function q = euler2quat(roll,pitch,yaw)
% compute half angle trig functions for roll
cr = cos(roll/2);  
sr = sin(roll/2);
% ... for pitch
cp = cos(pitch/2); 
sp = sin(pitch/2);
% ... for yaw
cy = cos(yaw/2);   
sy = sin(yaw/2);

%c compute quaternion components
q0 = cr*cp*cy + sr*sp*sy;
q1 = sr*cp*cy - cr*sp*sy;
q2 = cr*sp*cy + sr*cp*sy;
q3 = cr*cp*sy - sr*sp*cy;

q = [q0;q1;q2;q3];
q = q/norm(q);
end

function antenna_body = get_antenna_orientation(orientation)
    switch upper(orientation)
        case "+X"
            antenna_body = [1;0;0];
        case "-X"
            antenna_body = [-1;0;0];
        case "+Y"
            antenna_body = [0;1;0];
        case "-Y"
            antenna_body = [0;-1;0];
        case "+Z"
            antenna_body = [0;0;1];
        case "-Z"
            antenna_body = [0;0;-1];
        otherwise
            error("Unknown antenna orientation")
    end
end
