function [x_opt, u_opt] = solve_ocp(r, d, x0, xd, dt, N, w_x, w_u)
%   Implement an ocp with cost on position tracking and control reg

% r: wheel radius
% d: distance between wheels
% x0: initial state
% xd: [3XN] vector containing desired state at every node
% dt: integration timestep
% N: number of shooting nodes
% w_x: [3, 1] vector of state weights
% w_u: [2, 1] vector of control weights
import casadi.*

%% Model of the system
x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta');
omega_l = SX.sym('omega_l'); omega_r = SX.sym('omega_r');

state = [x; y; theta];
u = [omega_l, omega_r];
nx = length(state);
nu = length(u);

G = [(omega_l + omega_r)/2 * r * cos(theta); ...
     (omega_l + omega_r)/2 * r * sin(theta); ...
     (omega_r - omega_l)/d * r];

f = Function('f', {state, u}, {G});

%% Optimization stuff
v_max = 0.6; v_min = -v_max;
omega_max = pi/4; omega_min = -omega_max;

% Optimization variables
opti = Opti();
U = opti.variable(nu, N);
X = opti.variable(nx, N+1);

% Create the cost function
obj = 0;
opti.subject_to(X(:, 1) == x0)
for i=1:N
    u_i = U(:, i); st_i = X(:, i);
    obj = obj + (sumsqr((st_i - xd(:, i)) .* w_x)  + sumsqr(u_i .* w_u)) * dt;

    % Continuity constraint
    v = f(st_i, u_i);
    opti.subject_to(X(:, i+1) == X(:, i) + v* dt)

    % Path constraint
%     opti.subject_to(sum(u_i)/2 * r > 0);
end

% Final position constraint
opti.subject_to(X(:, N) == xd(:, end)) % You can enable or disable this and check how it affects the results

opts = struct;
opts.ipopt.print_level =0;
opts.print_time = 0;

opti.solver('ipopt', opts);
opti.minimize(obj);
opti.solve;

x_opt = opti.value(X);
u_opt = opti.value(U);

end