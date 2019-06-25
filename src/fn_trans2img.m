function tag_img = fn_trans2img(det_tags)
% Get tags in image coordinates [p1x,p2x,p3x,p4x;
%                                p1y,p2y,p3y,p4y;
%                                 1  ,1  ,1  ,1];
pixels = reshape(det_tags(2:end,:),2,[]);
tag_img = [pixels;ones(1,size(pixels,2))];
    
end