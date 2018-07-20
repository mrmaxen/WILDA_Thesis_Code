%% Plot Beacons

% get values for +-15 lines
th_plus = theta_actual + 15;
th_minus = theta_actual - 15;
r_p = sqrt( p(1)^2+p(2)^2 );

up = [r_p*cosd(th_plus), r_p*sind(th_plus)];
down = [r_p*cosd(th_minus), r_p*sind(th_minus)];

 if (p(2) < 0)
    up(2) = up(2)*-1;
    down(2) = down(2)*-1;
 end
 
 % plot beacons
plot(p1(1),p1(2),'ob'); hold on;
plot(p2(1),p2(2),'ob')
plot(p3(1),p3(2),'ob')

% plot tag actual position and +-15 lines
plot(p(1),p(2),'co')
plot([up(1) 0], [up(2) 0],'b')
plot([down(1) 0], [down(2) 0],'b')

clearvars up th_plus th_minus r_p
