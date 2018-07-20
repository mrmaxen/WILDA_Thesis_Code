% Evaluation of algorithm output

% input thetaErr as X and thetaPrec as Z
%% Accuracy
t = linspace(0,20,length(X)); % create CDF
NewDist = fitdist(X', 'Normal');
Xcdf = cdf(NewDist,t);
figure(7)
if exist('thetaErr_WCBN') && (~exist('thetaErr_LSS'))
    g1 = plot(t,Xcdf,'c--');
else
g1 = plot(t,Xcdf);
end
set(g1, 'LineWidth',2)
title('Case 1: Accuracy');
legend('WBN', 'WCBN','LLS', 'LLS_{Kalman}',  'NLLS');
xlabel('Angular Error (degrees)')
ylabel('CDF')
% order: WBN, WCBN, LSS, LSS_K, NLLSS
hold on;

[c idx] = min(abs(Xcdf-0.95));  % get 95% value

accuracy = t(idx);
%% Precision
disp('Evaluating')
tp = linspace(0,20,length(Z));

NewDist = fitdist(abs(Z'), 'Normal');

Zcdf = cdf(NewDist,tp);
figure(8)
if exist('thetaErr_WCBN') && (~exist('thetaErr_LSS'))
    g2 = plot(tp,Zcdf,'c--');
else
    g2 = plot(tp,Zcdf);
end
set(g2, 'LineWidth',2)
title('Case 1: Precision');
legend('WBN', 'WCBN','LLS', 'LLS_{Kalman}',  'NLLS');
xlabel('Angular Error (degrees)')
ylabel('CDF')
hold on;
[c idx] = min(abs(Zcdf-0.95));

precision = tp(idx);
