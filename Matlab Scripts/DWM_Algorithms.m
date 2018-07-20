% Load DWM data and run algorithms with it

%% setup
clear, clc, close all;
hold off;
plot_ests = 0;	% set to 1 to plot every estimate, takes longer

fileName = ''; % File containing data
p = [0;12.3];
load(fileName)
r1_ = imported_data(1:end,1)/1000;
r3_ = imported_data(1:end,2)/1000;
r2_ = imported_data(1:end,3)/1000;
Kalman_Filter;

hold on;
theta_actual = acosd(p(1)/sqrt(p(1)^2+p(2)^2));
p1 = [-0.15; 0]; p2 = [0; 0.15]; p3 = [0.15; 0];

Bias_Correction;

% Choose algorithms to run; 1 = run, 0 = not run
WBN = 1;
WCBN = 1;
LSS = 1;
LSS_K = 1;
NLLSS = 1;
initialVars = who; % save current variables to simplify later clearvars calls
 %% WBN
if exist('WBN')
    limit = 7; % number of iterations
    my = 1/5;   % stepsize; limit and my have been somewhat optimized
    figure(1)   
    WBN_RealData    % Run WBN Algorithm
    title('WBN');
    X = thetaErr_WBN;   % set variables for 'Evaluation'
    Z = thetaPrec_WBN;
    Evaluation;         % gives CDF, accuracy and precision
    thetaPrec_WBN = precision;
    thetaACC_WBN = accuracy;
    clearvars('-except',initialVars{:}, 'theta*', 'initialVars') % clear variables
end
%% WCBN
if exist('WCBN')
    limit = 7;
    my = 1/0.8;
    figure(4)
    WCBN_RealData;
    title('WCBN');
    X = thetaErr_WCBN;
    Z = thetaPrec_WCBN;
    Evaluation;
    thetaPrec_WCBN = precision;
    thetaACC_WCBN = accuracy;
    clearvars('-except',initialVars{:}, 'theta*', 'initialVars')
end
%% LSS without Kalman Filtering
if exist('LSS')
    figure(2);
    RealData_LSS;
    title('LSS');
    X = thetaErr_LSS;
    Z = thetaPrec_LSS;
    Evaluation;
    thetaPrec_LSS = precision;
    thetaACC_LSS = accuracy;
    clearvars('-except',initialVars{:}, 'theta*', 'initialVars')
end
%% LSS with Kalman Filtering
if exist('LSS_K')
    figure(3);
    LSS_Kalman;
    title('LSS Kalman');
    X = thetaErr_LSS_K;
    Z = thetaPrec_LSS_K;
    Evaluation;
    thetaPrec_LSS_K = precision;
    thetaACC_LSS_K = accuracy;
    clearvars('-except',initialVars{:}, 'theta*', 'initialVars')
end

%% NLLSS
if exist('NLLSS')
    figure(5)
    NonLinearLSS;
    title('NLLSS');
    X = thetaErr_NLLSS;
    Z = thetaPrec_NLLSS;
    Evaluation;
    thetaPrec_NLLSS = precision;
    thetaACC_NLLSS = accuracy;
    clearvars('-except',initialVars{:}, 'theta*', 'initialVars')
end

if(~plot_ests)
    close(1:6)  % if we didn't plot the estimates, close the empty figures
end

% clean up final workspace
clearvars thetaMe* thetaE* theta_actual plot_ests div j K_k P0 Q R