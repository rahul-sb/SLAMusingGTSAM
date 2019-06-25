function [new_tag_world, update_tag_world, update_idx] = fn_getTagsWorld(R, T, K, tag_all, det_tags)
% Find the global coodinates of ALL the tags using the computed pose of the camera

list_ids = tag_all.retrieve(0,1);
[~,tag_all_idx,~] = intersect(list_ids, det_tags(1,:), 'stable');
tag_details = tag_all.retrieve(1,tag_all_idx);

% Get the global coordinates of ALL the detected tags w.r.t the world coordinates of the
% detected tags in the current frame. This recalculates the world
% coordinates of the already detected tags again.
tag_world = fn_getGlobalCoodinates(R, T, K, tag_details, det_tags);

if isempty(tag_world)
    new_tag_world = [];
    update_idx = [];
    update_tag_world = [];
else
    % New tags that have been detected
    [~, new_tag_world_idx] = setdiff(tag_world(1,:), list_ids);
    new_tag_world = tag_world(:, new_tag_world_idx);
    
    % Old tags that have to be updated
    [~,update_idx, tag_world_idx] = intersect(list_ids, tag_world(1,:), 'stable');
    update_tag_world = tag_world(:,tag_world_idx);
    
    % Do not update Origin Tag
    origin_id_idx = update_tag_world(1,:) == list_ids(1);
    if any(origin_id_idx)
        update_tag_world = update_tag_world(:,~origin_id_idx);
        update_idx = update_idx(2:end); % The first tag is the origin tag, if there is an origin tag
    end 
end
end