function tag_world = fn_getGlobalCoodinates(R, T, K, all_tags, det_tags)

if ~isempty(det_tags)
    P = K*[R,T]; % Get the projection matrix

    % Compute the scaling factor based on all detected tags and the projection
    % matrix
    tags_pos_world = all_tags(2:end,:);
    tags_img = P*[reshape(tags_pos_world,3,[]);ones(1,size(tags_pos_world,2)*4)];
    scaling_factor = mean(tags_img(3,:));
    
    % Compute the world coordinates of the remaining tags
    det_tags_img = [reshape(det_tags(2:end,:),2,[]);ones(1,size(det_tags,2)*4)]; % Mistake Corrected: I previously transposed the reshape function which gave completely differnet results
    tag_world = P\(det_tags_img.*scaling_factor);
    tag_world = tag_world./tag_world(4,:);
        
    % Remove any coordinates if they are Nan or zeros or Infinites
    if ~any(isnan(tag_world(4,:))) && ~any(abs(tag_world(1,:))>10) && ~any(abs(tag_world(2,:))>10) && any(tag_world(1,:)) && any(tag_world(2,:))
        tag_world = [det_tags(1,:);reshape(tag_world(1:3,:),12,[])];
    else
        tag_world = [];
    end
else
    tag_world = [];
end

end
