function [result, graph, initial_estimate, all_poses, landmarks] = getResult(obj, state_idxs, tag_idxs)

import gtsam.*

graph = obj.graph;
initial_estimate = obj.initial_estimate;
result = obj.result;

%% Get poses from graph
all_poses = zeros(length(state_idxs), 7);

for i = 1:length(state_idxs)
    try
        pose = result.at(obj.state_key(i));
        rot = pose.rotation.matrix;
        trans = pose.translation.vector;
        location = -rot' * trans;
        all_poses(i, :) = [location',  rotm2quat(rot)];
    end
end

%% Get location of landmarks
landmarks = nan(13, length(tag_idxs));
for i = 1:length(tag_idxs)
    landmarks(i,1) = tag_idxs(i);
    for j = 4:3:13
        tag_id = obj.tags_key((j-1)/3, tag_idxs(i));
        landmarks(i, j-2:j) = [result.at(tag_id).x;
                               result.at(tag_id).y;    
                               result.at(tag_id).z];
    end
end

end