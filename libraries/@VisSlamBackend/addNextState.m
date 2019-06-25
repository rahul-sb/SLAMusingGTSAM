function addNextState(obj, new_state_idx, old_state_idx, R, T)

import gtsam.*

current_pose = Pose3(Rot3(R),Point3(T));
relative_pose = obj.recent_pose.between(current_pose);
obj.graph.add(BetweenFactorPose3(obj.state_key(old_state_idx), obj.state_key(new_state_idx), relative_pose, obj.noise_sigmas.odom));

% Update recent pose
obj.recent_pose = current_pose;
end