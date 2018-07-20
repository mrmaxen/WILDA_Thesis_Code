%% WCBN

pe = [0;0];
pk = [pe(1); pe(2); pe(1)^2+pe(2)^2];
A = [p1(1), p1(2), -0.5;...
    p2(1), p2(2), -0.5;...
    p3(1), p3(2), -0.5];

pe = [0;0];
% plot(pe(1),pe(2),'m*')
pk = [pe(1); pe(2); pe(1)^2+pe(2)^2];
A = [p1(1), p1(2), -0.5;...
    p2(1), p2(2), -0.5;...
    p3(1), p3(2), -0.5];

pk = [0;0;0];
for i = 1:(length(r1_)/limit)
    pk = [0;0;0];
    for k = 1:limit % using 10 measurements each
        r1 = r1_(limit*(i-1) + k);
        r2 = r2_(limit*(i-1) + k);
        r3 = r3_(limit*(i-1) + k);

        bk = 0.5* [p1(1)^2+p1(2)^2-r1^2;...
            p2(1)^2+p2(2)^2-r2^2;...
            p3(1)^2+p3(2)^2-r3^2];
        Qk = diag([ 1/(bk(1)-A(1,:)*pk )^2, ...
            1/(bk(2)-A(2,:)*pk )^2, 1/(bk(3)-A(3,:)*pk )^2 ] ,0);
        ek = bk-A*pk;


        pk = pk + my*inv( sign(A')*Qk*sign(A) )*sign(A')*Qk*ek;
    end
PlotBeacons;
pe = [pk(1); pk(2)];
thetaPosErr_WCBN(i) = sqrt( (pe(1)-p(1))^2 + (pe(2)-p(2))^2 );
theta_pe(i) = acosd(pe(1)/sqrt(pe(1)^2+pe(2)^2));
thetaErr_WCBN(i) = theta_actual-theta_pe(i);
if (plot_ests == 1)
    plot(pe(1),pe(2),'mo')
end
end

thetaMed_WCBN = median(theta_pe);
for i = 1:length(theta_pe)
    thetaPrec_WCBN(i) = theta_pe(i)-thetaMed_WCBN;
end

% theta_ACC_WCBN = mean(abs(theta_pe - theta_actual));

theta_RMSE_WCBN = sqrt( sum(theta_pe - theta_actual)^2/length(theta_pe) );

theta_MAE_WCBN = 1/length(theta_pe) * sum(abs(theta_pe - theta_actual));

thetaPos_MAE_WCBN = 1/length(thetaPosErr_WCBN) * sum(abs(thetaPosErr_WCBN));

clearvars theta_pe theta_sum
