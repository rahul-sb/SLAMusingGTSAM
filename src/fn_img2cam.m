function tag_cam = fn_img2cam(K, tag_img)
% Transform from img coordinates to camera coordinates
tag_cam = K\tag_img;
end