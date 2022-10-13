function triangle = draw_unicycle(x, y, theta, i)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
h = 0.25;
b = 0.1;
ct = cos(theta);
st = sin(theta);
R = [ct, -st, 0; st, ct, 0; 0, 0, 1];
A = [x; y; 0] + R * [- 0* h; b/2; 0];
B = [x; y; 0] + R * [- 0* h; -b/2; 0];
C = [x; y; 0] + R * [0.7* h; 0; 0];

figure(i)
triangle = fill([A(1), B(1), C(1)], [A(2), B(2), C(2)], 'red')
xlim([-1 6])
ylim([-1 4])

end