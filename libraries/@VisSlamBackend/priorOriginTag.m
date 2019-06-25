function priorOriginTag(obj, tag_details)
% ArgIn: (obj, tag_details: 13x1 vector: tag_id,px1,py1,pz1,px2,...p4z) 

import gtsam.*
tag_idx = tag_details(1);
position_xyz = reshape(tag_details(2:end),3,4);

for i=1:4
    obj.graph.add(PriorFactorPoint3(obj.tags_key(i, tag_idx), Point3(position_xyz(:,i)), obj.noise_sigmas.tags));
end

obj.addTagConstraints(tag_idx);

end