function plotInitialEstimate(obj)

import gtsam.*

figure;
hold on;
plot3DTrajectory(obj.initial_estimate, 'g-*');
plot3DPoints(obj.initial_estimate,'k-*');
hold off;

end