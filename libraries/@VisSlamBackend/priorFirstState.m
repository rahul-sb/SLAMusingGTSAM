function priorFirstState(obj, state_idx, R, T)

import gtsam.*

obj.recent_pose = Pose3(Rot3(R),Point3(T));
obj.graph.add(PriorFactorPose3(obj.state_key(state_idx), obj.recent_pose, obj.noise_sigmas.odom));
obj.priorState(state_idx, R, T);

end