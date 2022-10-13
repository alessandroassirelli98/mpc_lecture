clear all
close all
clc

% Simulation data
T = 10;
dt = 0.01;
N = int16(T/dt);

addpath(genpath(pwd))
%% Robot model
d = 0.1; %10 cm
r = 0.05; %wheel radius 5 cm
xr = 0.01;
yr = 0;
delta = [xr; yr]; %reference point

%% Path
A = 0.5;
freq = 1;
scaling = 2*pi;
x = linspace(0, 5, N);
y = A * sin(freq * scaling * x);
xd = [x; y];
xd_dot = [diff(x); diff(y)];

%% SIMULATION WITH THE MODEL

% Initial conditions
x0 = [0; 0; pi/4];
x_model = cat(2, x0, zeros(3, N-1));
xp = zeros(2, N-1);
x_dot = zeros(3, N);

k = [10;10];

figure(1)
for i = 1:N-1
    ct = cos(x_model(3, i));
    st = sin(x_model(3, i));
    R = [ct, -st; st, ct];

    xp(:, i) = x_model(1:2, i) + R * delta;
    xp_dot = k .* (xd(:, i) - xp(:, i));

    G_i = [ ct, 0; ...
           st, 0; ...
           0, 1];
    
    u_opt = [ct, st; -1/xr * st, 1/xr * ct] * xp_dot;
    x_dot(:, i) = G_i * u_opt;

    x_model(:, i+1) = x_dot(:, i)*dt + x_model(:, i);
    

end

figure(1)
axis equal
plot(xp(1, :), xp(2, :))
hold on
plot(x_model(1, :), x_model(2, :), 'g.')
plot(xd(1, :), xd(2, :), 'r--')

figure(2)
for i=1:N-1
    draw_unicycle(x_model(1, i), x_model(2, i), x_model(3, i), 2)
end

