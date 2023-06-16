function traj_d = circle(a,b,r, theta0, dtheta_des, dt, N)
% Get a vector containing the points of a circle to track.
% The last row is the angle spanning the circle

% a: offset in x
% b: offset in y
% theta0: initial angle
% dtheta_des: desired velocity 
% dt: timestep
% N: number of points describing the circle
theta = zeros(1, N);
for t=1:N
    theta(t) = theta0 + dtheta_des * ((t-1)*dt);
end

x = a + r*cos(theta);
y = b + r*sin(theta);

traj_d = [x; y; theta];

end