function priorTags(obj, tag_details)
% ArgIn: (obj, tag_details: 13xn vector: tag_id,px1,py1,pz1,px2,...p4z for 'n' tags) 

import gtsam.*

if ~isempty(tag_details)
    tag_idx = tag_details(1,:);
    
    % For each and every tag
    for i=1:length(tag_idx)
        position_xyz = reshape(tag_details(2:end,i),3,4);   % For all the 4 corners

        % For each corner
        for j=1:4
            obj.initial_estimate.insert(obj.tags_key(j,tag_idx(i)), Point3(position_xyz(:,j)));
        end
    end

    obj.addTagConstraints(tag_idx);
end

end