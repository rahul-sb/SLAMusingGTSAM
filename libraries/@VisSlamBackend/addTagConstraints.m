function addTagConstraints(obj, tag_idx)

import gtsam.*

for i=1:length(tag_idx)
    obj.graph.add(BetweenFactorPoint3(obj.tags_key(1,tag_idx(i)), obj.tags_key(2,tag_idx(i)), Point3(obj.tag_size, 0, 0), obj.noise_sigmas.tags));
    obj.graph.add(BetweenFactorPoint3(obj.tags_key(2,tag_idx(i)), obj.tags_key(3,tag_idx(i)), Point3(0, obj.tag_size, 0), obj.noise_sigmas.tags));
    obj.graph.add(BetweenFactorPoint3(obj.tags_key(3,tag_idx(i)), obj.tags_key(4,tag_idx(i)), Point3(-obj.tag_size, 0, 0), obj.noise_sigmas.tags));
    obj.graph.add(BetweenFactorPoint3(obj.tags_key(4,tag_idx(i)), obj.tags_key(1,tag_idx(i)), Point3(0, -obj.tag_size, 0), obj.noise_sigmas.tags));
end
end