function addMeasurements(obj, state_idx, detected_tags)
% detected_tags - 9xn matrix: where n - num of tags and 9: tag_id,p1x,p1y,p2x,...p4y

import gtsam.*

if ~isempty(detected_tags)
    tag_idx = detected_tags(1,:);
    pixel_position = detected_tags(2:end,:);

    for i=1:length(tag_idx)
        for j=1:2:8
            obj.graph.add(GenericProjectionFactorCal3_S2(Point2(pixel_position(j,1), pixel_position(j+1,1)),...
                          obj.noise_sigmas.meas, obj.state_key(state_idx), obj.tags_key(ceil(j/2),tag_idx(i)), obj.camera_calib));
        end
    end
end
end