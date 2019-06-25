function det_tags = fn_cleanDetections(det_all)
% Remove multiple detections and send the transposed matrix

if ~isempty(det_all)    
    det_all = det_all';
    [~,indices,~] = unique(det_all(1,:));
    det_tags = det_all(:,indices);

    if size(det_tags,1) ~= 9
        det_tags = [];
    end
else
    det_tags = [];
end
end