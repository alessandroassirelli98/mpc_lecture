function x_integrated = integrate_unicycle(r, d, x0, U, dt, N, Nk)
% Integrate the robot state starting from state x0, with control input U.
% It's possible to have a kick at the timestep Nk, at which a displacement
% is introduced

% r: radius of the wheels
% d: interaxial wheel distance
% x0: initial state
% U: control input dimension:[2, N-1]
% dt: timestep of the simulation
% T: simulation duration
% Nk: timestep at which the kick happens. If 0 no kick

x_integrated = cat(2, x0, zeros(3, N));
for i=1:N
    if Nk ~= 0 && i == Nk
        x_integrated(:, i) = x_integrated(:, i) + [0; 0.25; 0]; %randn(2, 1)*5e-1;
    end
    c_t = cos(x_integrated(3,i));
    s_t = sin(x_integrated(3,i));

    f = [(U(1, i) + U(2, i))/2 * r * c_t; ...
        (U(1, i) + U(2, i))/2 * r * s_t; ...
        (U(2, i) - U(1, i))/d * r ];
    
    x_integrated(:, i+1) = x_integrated(:, i) + f * dt;
end