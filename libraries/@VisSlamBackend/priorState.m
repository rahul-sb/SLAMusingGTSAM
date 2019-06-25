function priorState(obj, state_idx, R, T)
import gtsam.*

obj.initial_estimate.insert(obj.state_key(state_idx), Pose3(Rot3(R), Point3(T)));

end