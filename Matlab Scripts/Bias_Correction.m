% Correct bias of measurements according to known distances
% Input r1_, r2_ and r3_ as well as the anchor positions

% Assume that r1_ is correct
% mean of r3_ should be equal to mean of r1_

m1 = mean(r1_);

m3 = mean(r3_);

% for k = 1:length(r3_)
%     r3_(k,1) = r3_(k,1) + (m1-m3);
% end
r3_ = r3_ + (m1-m3);

% Anchor 1 and 3 are on the x-axis

m2 = mean(r2_);

d = p2(2);

% for k = 1:length(r2_)
%     r2_(k,1) = r2_(k,1) + ((m1-0.15)-m2);
% end
r2_ = r2_  + ((m1-0.15)-m2);


clearvars d m1 m2 m3 k

