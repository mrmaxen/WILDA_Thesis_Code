%% Non-Linear Least Squares
PlotBeacons;

for i = 1:length(r1_) 
    r1 = r1_(i);
    r2 = r2_(i);
    r3 = r3_(i);
    %% Linear Least squares
    % gives initial guess
    b = [ r1^2-p1(1)^2-p1(2)^2-r3^2+p3(1)^2+p3(2)^2;...
        r2^2-p2(1)^2-p2(2)^2-r3^2+p3(1)^2+p3(2)^2 ];
    A = -2 * [ p1(1)-p3(1), p1(2)-p3(2);...
        p2(1)-p3(1), p2(2)-p3(2)];
    p_est = inv(A'*A)*A'*b;
    p_est0 = [p_est(1); p_est(2)];
    %% Iteration
    for count = 1:5
        %     disp(count)
        f = [ sqrt( (p_est(1)-p1(1))^2+(p_est(2)-p1(2))^2 )-r1;...
            sqrt( (p_est(1)-p2(1))^2+(p_est(2)-p2(2))^2 )-r2;...
            sqrt( (p_est(1)-p3(1))^2+(p_est(2)-p3(2))^2 )-r3];
        
        dfx = [-(2*p1(1) - 2*p_est(1))/(2*((p_est(2) - p1(2))^2 + (p1(1) - p_est(1))^2)^(1/2));...
            -(2*p2(1) - 2*p_est(1))/(2*((p_est(2) - p2(2))^2 + (p2(1) - p_est(1))^2)^(1/2));...
            -(2*p3(1) - 2*p_est(1))/(2*((p_est(2) - p3(2))^2 + (p3(1) - p_est(1))^2)^(1/2))];
        
        dfy = [(2*p_est(2) - 2*p1(2))/(2*((p_est(2) - p1(2))^2 + (p1(1) - p_est(1))^2)^(1/2));...
            (2*p_est(2) - 2*p2(2))/(2*((p_est(2) - p2(2))^2 + (p2(1) - p_est(1))^2)^(1/2));...
            (2*p_est(2) - 2*p3(2))/(2*((p_est(2) - p3(2))^2 + (p3(1) - p_est(1))^2)^(1/2))];
        
        J = [dfx(1), dfy(1);...
            dfx(2), dfy(2);...
            dfx(3), dfy(3)];
        
        g = [2*( f(1)*dfx(1) + f(2)*dfx(2) + f(3)*dfx(3) );...
            2*( f(1)*dfy(1) + f(2)*dfy(2) + f(3)*dfy(3) )];
        
        p_est = p_est - inv(J'*J)*J'*f;
    end
    pe = p_est;
    thetaPosErr_NLLS(i) = sqrt( (pe(1)-p(1))^2 + (pe(2)-p(2))^2 );

	if (plot_ests == 1)
		plot(pe(1),pe(2),'mo')
	end
    theta_pe(i) = acosd(pe(1)/sqrt(pe(1)^2+pe(2)^2));
    thetaErr_NLLSS(i) = theta_actual-theta_pe(i);
    %end iteration
end

thetaMed_NLLSS = median(theta_pe);
for i = 1:length(theta_pe)
    thetaPrec_NLLSS(i) = theta_pe(i)-thetaMed_NLLSS;
end

theta_RMSE_NLLSS = sqrt( sum(theta_pe - theta_actual)^2/length(theta_pe) );

theta_MAE_NLLSS = 1/length(theta_pe) * sum(abs(theta_pe - theta_actual));

thetaPos_MAE_NLLS = 1/length(thetaPosErr_NLLS) * sum(abs(thetaPosErr_NLLS));

clearvars theta_pe theta_sum