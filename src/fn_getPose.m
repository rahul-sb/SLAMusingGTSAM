function [R,T] = fn_getPose(K, tag_all, det_tags)
% Find the pose of the camera using global and camera coordinates of the
% tags (in the current frame) for which we already know the global coordinates.

% Find the tags for which we already know the global coordinates
[~,tag_all_idx, det_tag_idx] = intersect(tag_all.retrieve(0,1), det_tags(1,:), 'stable');
tags_world = tag_all.retrieve(1,tag_all_idx);

tag_img = fn_trans2img(det_tags(:,det_tag_idx)); % Transform to img homogeneous coordinates
tag_cam = fn_img2cam(K, tag_img); % Get in camera coordinates

% Perform RPnP if there are at least 4 points. This is the condition for
% RPnP to give a reasonable estimate of R and T.
if size(tag_cam,2) >= 4
    [R,T] = RPnP(reshape(tags_world(2:end,:),3,[]), tag_cam(1:2,:)); 
else
    R = [];
    T = [];
end

end