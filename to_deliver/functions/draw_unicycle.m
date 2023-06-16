function triangle = draw_unicycle(x_opt, xs, rob_diam)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
figure(500)
% Animate the robot motion
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);

r = rob_diam/2;  % obstacle radius
ang=0:0.005:2*pi;
xp=r*cos(ang);
yp=r*sin(ang);

for i=1:size(x_opt, 2)
    h_t = 0.14; w_t=0.09; % triangle parameters

    x1 = xs(1, i);
    y1 = xs(2, i);
    th1 = xs(3, i);
    
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
    fill(x1_tri, y1_tri, 'g'); % plot reference state
    hold on

    x1 = x_opt(1, i);
    y1 = x_opt(2, i);
    th1 = x_opt(3, i);

    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
    fill(x1_tri, y1_tri, 'g'); % plot reference state
    hold on;

    fill(x1_tri, y1_tri, [0.6350 0.0780 0.1840]); % plot robot position
%     plot(x1+xp,y1+yp, 'Color',[0.6350 0.0780 0.1840], 'LineStyle','--'); % plot robot circle
    

    hold off
    axis equal
    xlim([-0.2, 2.1])
    ylim([-0.2, 2.1])
    box on;
    grid on
    drawnow
end
hold on
plot(x_opt(1, :), x_opt(2, :), 'Color', [0 0.4470 0.7410], 'LineStyle','--')

end
