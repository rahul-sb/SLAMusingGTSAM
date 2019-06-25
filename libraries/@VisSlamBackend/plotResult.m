function plotResult(obj)

import gtsam.*

figure;
hold on;
plot3DPoints(obj.result, []);
plot3DTrajectory(obj.result, 'b-*', true, 0.3);
hold off;

end