
%% Load the data
[det_all, tag_size, K] = fn_loadFullSLAMData('DataMapping.mat');

%% Specify noise sigmas
noise.odom = [1 1 1 2 2 2]'*1e-1;
noise.meas = 2;
noise.tags = [2 2 2]'*1e-1;

%% Specify max no. of expected AR Tags in the environment
max_tags = 200;
max_states = length(det_all);

%% Setup required variables
tag_all = DataIO(13,max_tags); % Used to store the locations of the 4 corners 
                               % of the tags in world coordinates along with tag id
origin_tag_world = [0 0 0;tag_size 0 0;tag_size tag_size 0;0 tag_size 0]'; % World coordinates of the origin tag
tag_id = DataIO(1,max_tags);   % Used to keep track of the tags all the tags that have been detected

%% Import gtsam and initialize the graph
import gtsam.*

slam = VisSlamBackend(max_tags, max_states, noise, K, tag_size);

%% Fix the bottom left corner of the 1st ARTag detected as Origin
det_tags = fn_cleanDetections(det_all{1}); % Note: det_tags has dimension 9xn, where n - num of unique detections 
tag_all.store([det_tags(1); origin_tag_world(:)]); % Store tag details

[R,T] = fn_getPose(K, tag_all, det_tags); 
[new_tag_world, ~, ~] = fn_getTagsWorld(R, T, K, tag_all, det_tags);
tag_all.store(new_tag_world);
tag_id.overwrite(union(tag_id.retrieve, det_tags(1,:))); % Store all tag id's

%% Add the factors to the graph
slam.priorFirstState(1, R, T);
slam.addMeasurements(1, det_tags);
previous_state_id = 1;

for i=2:max_states
    det_tags = fn_cleanDetections(det_all{i});
    
    if isempty(det_tags)
        continue;
    end
    
    [R,T] = fn_getPose(K, tag_all, det_tags);
    
    if isempty(R)
        continue;
    end
    
    [new_tag_world, update_tag_world, update_idx] = fn_getTagsWorld(R, T, K, tag_all, det_tags);
    tag_all.store(new_tag_world);
    tag_all.insert(0, update_tag_world, update_idx);
    tag_id.overwrite(union(tag_id.retrieve, det_tags(1,:))); % Store all tag id's

    slam.addNextState(i, previous_state_id, R, T);
    slam.addMeasurements(i, det_tags);

    slam.priorState(i, R, T);
    previous_state_id = i;
end

%% Add priors to the tags
slam.priorOriginTag(tag_all.retrieve(1,1));
slam.priorTags(tag_all.retrieve);
tags_without_prior = setdiff(tag_id.retrieve, tag_all.retrieve(0,1));
slam.priorTags([tags_without_prior; rand(12, length(tags_without_prior))]);

%% Solve the graph and plot the result 
slam.plotInitialEstimate();
slam.solve();
slam.plotResult();
[result, graph, initial_estimate, all_poses, landmarks] = slam.getResult(1:max_states-1, tag_all.retrieve(0,1));



