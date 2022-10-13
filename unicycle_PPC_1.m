clear all
close all
clc

addpath(genpath(pwd))
%% Robot model
d = 0.1; %10 cm
r = 0.05; %wheel radius 5 cm

%% Path

% Waypoints
wp = [5; 2];

T = 10;
dt = 0.01;
N = int16(T/dt);

omega_l = ones(1, N);
omega_r = omega_l;

u = [omega_l; omega_r];


%% SIMULATION WITH THE MODEL
x0 = [0; 0; pi/4];
x_model = cat(2, x0, zeros(3, N-1));
x_dot = zeros(3, N);
vlin = 1;
acceptable_tol = 0.01;

figure(1)
for i = 1:N-1
    c_t = cos(x_model(3, i));
    s_t = sin(x_model(3, i));
    
    b = wp - x_model(1:2, i); % distance from target
    ld = sqrt(sum(b.^2));
    mu = atan2(b(2), b(1)); % angle to destination
    alpha = mu - x_model(3, i);
    
    e = ld;
    
    if e <= acceptable_tol
        u_opt = [0 ; 0];
    else
        u_opt = [(ld - d * sin(alpha)); (ld + d * sin(alpha))] * (vlin/ (ld * r));
    end

    G_i = [r/2 * c_t, r/2 * c_t; ...
           r/2 * s_t, r/2 * s_t; ...
           -r/d, r/d];

%     x_dot(:, i) = G_i * u(:, i);
    x_dot(:, i) = G_i * u_opt;
    x_model(:, i+1) = x_dot(:, i)*dt + x_model(:, i);

    draw_unicycle(x_model(1, i), x_model(2, i), x_model(3, i), 1)
    

end

figure()
plot(x_model(1, :), x_model(2, :))
axis equal
