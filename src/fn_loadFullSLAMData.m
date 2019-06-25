function [det_all, tag_size, K] = fn_loadFullSLAMData(file_name)

load(file_name);
load('CalibParams.mat');

det_all = DetAll;
tag_size = TagSize;

end