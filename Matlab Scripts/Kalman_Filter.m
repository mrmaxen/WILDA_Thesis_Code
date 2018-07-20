%% Simple Kalman Filter
%   Takes a n x 3 array and filters each row separately
%% Kalman Filter
i = length(r1_);
% setup first step
Q = 0;
R = 1;
r0 = r1_(1); P0 = r0+1;
r1_c(1) = r0;
P_k(1) = P0;
K_k(1) = P_k(1)/( P_k(1) + R);
r1_c(1) = r1_c(1) + K_k(1)*(r1_(1)-r1_c(1));
P_k(1) = (1-K_k(1))*P_k(1);
% iterate over all measurements
for j = 2:i
    r1_c(j) = r1_c(j-1);
    P_k(j) = P_k(j-1)+Q;
    K_k(j) = P_k(j)/( P_k(j) + R);
    r1_c(j) = r1_c(j) + K_k(j)*(r1_(j)-r1_c(j));
    P_k(j) = (1-K_k(j))*P_k(j);
end

r0 = r2_(1); P0 = r0+10;
r2_c(1) = r0;
P_k(1) = P0;
K_k(1) = P_k(1)/( P_k(1) + R);
r2_c(1) = r2_c(1) + K_k(1)*(r2_(1)-r2_c(1));
P_k(1) = (1-K_k(1))*P_k(1);
for j = 2:i
    r2_c(j) = r2_c(j-1);
    P_k(j) = P_k(j-1)+Q;
    K_k(j) = P_k(j)/( P_k(j) + R);
    r2_c(j) = r2_c(j) + K_k(j)*(r2_(j)-r2_c(j));
    P_k(j) = (1-K_k(j))*P_k(j);
end

r0 = r2_(1); P0 = r0+10;
r3_c(1) = r0;
P_k(1) = P0;
K_k(1) = P_k(1)/( P_k(1) + R);
r3_c(1) = r3_c(1) + K_k(1)*(r3_(1)-r3_c(1));
P_k(1) = (1-K_k(1))*P_k(1);
for j = 2:i
    r3_c(j) = r3_c(j-1);
    P_k(j) = P_k(j-1)+Q;
    K_k(j) = P_k(j)/( P_k(j) + R);
    r3_c(j) = r3_c(j) + K_k(j)*(r3_(j)-r3_c(j));
    P_k(j) = (1-K_k(j))*P_k(j);
end

r1_c = r1_c';
r2_c = r2_c';
r3_c = r3_c';
filtered_data = [r1_c(1,:); r2_c(1,:);r3_c(1,:)];
clearvars r0