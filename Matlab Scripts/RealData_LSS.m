%% Linear Least squares

PlotBeacons;
for i = 1:length(r1_)
    r1 = r1_(i,1); % grab one measurements at a time
    r2 = r2_(i,1);
    r3 = r3_(i,1);
    
    % calculate matrices
b = [ r1^2-p1(1)^2-p1(2)^2-r3^2+p3(1)^2+p3(2)^2;...
     r2^2-p2(1)^2-p2(2)^2-r3^2+p3(1)^2+p3(2)^2 ];
A = -2 * [ p1(1)-p3(1), p1(2)-p3(2);...
           p2(1)-p3(1), p2(2)-p3(2)];
pe = inv(A'*A)*A'*b; %get estimate coords
% positional error
thetaPosErr_LLS(i) = sqrt( (pe(1)-p(1))^2 + (pe(2)-p(2))^2 );
pq(1,i) = pe(1);% save every point in memory for later use, optional
pq(2,i) = pe(2);

% calculate angular position of estimate
theta_pe(i) = acosd(pe(1)/sqrt(pe(1)^2+pe(2)^2));
thetaErr_LSS(i) = theta_actual-theta_pe(i); % angular error
if (plot_ests == 1)
plot(pq(1,i),pq(2,i),'mo')%plot estimate
end
end

thetaMed_LSS = median(theta_pe);
for i = 1:length(theta_pe)
    thetaPrec_LSS(i) = theta_pe(i)-thetaMed_LSS;
end

theta_RMSE_LSS = sqrt( sum(theta_pe - theta_actual)^2/length(theta_pe) );

theta_MAE_LSS = 1/length(theta_pe) * sum(abs(theta_pe - theta_actual));

thetaPos_MAE_LLS = 1/length(thetaPosErr_LLS) * sum(abs(thetaPosErr_LLS));

clearvars theta_pe theta_sum

