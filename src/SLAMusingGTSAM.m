function [LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                                IMU, LeftImgs, TLeftImgs, Mode)
% For Input and Output specifications refer to the project pdf

import gtsam.*
% Refer to Factor Graphs and GTSAM Introduction
% https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
% and the examples in the library in the GTSAM toolkit. See folder
% gtsam_toolbox/gtsam_examples

% This code doesn't use qIMUToC,TIMUToC,ImU and Mode.

% K - Calibration Matrix
% DetAll - list of all the tags and their relative positions in the image.
% LeftImgs - List of Images
global allTagWorld tagWorldDetected tagDetected nStates nTags;

rng(0);
DetAll = remove_no_detections(DetAll);
%% Define the number of states and also allocate matrix to store all the global positions
nStates = length(DetAll);     allPoses = nan(3,4,nStates);


%% Define the maximum number of tags and also allocate matrix to store the global positions of the tags
nTags = 1000;                   allTagWorld = nan(4*3,nTags);
tagDetected = false(nTags,1);   tagWorldDetected = false(nTags,1); 


%% Create keys for variables
i = uint64(nan(nStates,1));
j = uint64(nan(4,nTags));

for xi=1:nStates
   i(xi) = symbol('x',xi);    
end

for xi=1:nTags*4
   j(xi) = symbol('j',xi); 
end


%% Define Origin Tag and World Coordinates and store the coordinates and mention that the world cooedinates have been found
OriginTag = 10;     OriginTag_world = [0 0 0;TagSize 0 0;TagSize TagSize 0;0 TagSize 0]';
allTagWorld(:,OriginTag) = reshape(OriginTag_world,[],1);
tagWorldDetected(OriginTag) = true;


%% Find the Frame in which you detect the origin tag
[frameNumber,OriginTag_img,tagsInfo] = detect_origin_tag(OriginTag,DetAll);
OriginTag_cam = (K\OriginTag_img);


%% Find R,T from OriginTag camera and world positions
[R,T] = RPnP(OriginTag_world,OriginTag_cam(1:2,:));     allPoses(:,:,frameNumber) = [R,T];


%% Get the world coordinates of the other detected tags w.r.t the world coordinates of Origin Tag
get_tags_world_position(R,T,K,OriginTag_world,tagsInfo);


%% Create a graph container and initial estimates variable
disp('Making Graph');
graph = NonlinearFactorGraph;
initialEstimate = Values;


%% Specify Noise parameters for the graph and create an object for the calibrated camera
measurementNoiseSigma = 1.0;
tagNoiseSigma = [0.1 0.1 0.1]';
odometryNoiseSigma = [0.001 0.001 0.001 0.1 0.1 0.1]';

measurementNoise = noiseModel.Isotropic.Sigma(2,measurementNoiseSigma);
tagNoise = noiseModel.Diagonal.Sigmas(tagNoiseSigma);
odometryNoise  = noiseModel.Diagonal.Sigmas(odometryNoiseSigma);

calib_K = Cal3_S2(K(1,1),K(2,2),K(1,2),K(1,3),K(2,3));

%% Enter the prior for the 1st pose and the initial estimate
graph.add(PriorFactorPose3(i(frameNumber), Pose3(Rot3(R), Point3(T)), odometryNoise));
initialEstimate.insert(i(frameNumber), Pose3(Rot3(R), Point3(T)));


%% Enter the measurement values for the 1st node in graph
for yi=1:size(tagsInfo,1)
    for z=2:2:8
        graph.add(GenericProjectionFactorCal3_S2(Point2(tagsInfo(yi,z), tagsInfo(yi,z+1)), measurementNoise, i(frameNumber), j(z/2,tagsInfo(yi,1)), calib_K));
    end
end


%% Create the rest of the Graph

previousPose = Pose3(Rot3(R),Point3(T));

for xi = frameNumber+1:nStates
    %% Remove excess multiple detections 
    tagsInfo = remove_multiple_detections(DetAll{xi});
    
    
    %% Find the tags that are in common, i.e., that have been found previously
    [tagsWorld,tagsCam] = find_common_tags(tagsInfo,K);
    
    
    %% Find the R,T w.r.t these common tags
    [R,T] = RPnP(tagsWorld,tagsCam(1:2,:));     allPoses(:,:,xi) = [R,T];
    
    
    %% Store the world positions of the new tags (if any) from the current pose
    get_tags_world_position(R,T,K,tagsWorld,tagsInfo);
    
    
    %% Insert Pose in graph
    currentPose = Pose3(Rot3(R),Point3(T));
    relativePose = previousPose.between(currentPose);
    graph.add(BetweenFactorPose3(i(xi-1), i(xi), relativePose, odometryNoise));
    
    previousPose = currentPose;
    
    
    %% Insert Measurements in graph
    for yi=1:size(tagsInfo,1)
        for z=2:2:8
            graph.add(GenericProjectionFactorCal3_S2(Point2(tagsInfo(yi,z), tagsInfo(yi,z+1)), measurementNoise, i(xi), j(z/2,tagsInfo(yi,1)), calib_K));
        end
    end
    
    
    %% Insert initial estimates for the poses in the graph
    initialEstimate.insert(i(xi), Pose3(Rot3(R), Point3(T)));
    
    
end

%% Insert the prior for the Origin Tag
for xi=1:4
    graph.add(PriorFactorPoint3(j(xi,OriginTag), Point3(OriginTag_world(:,xi)), tagNoise));
end


%% Add constraints to the tags and add their world locations as prior
tagsFound = find(tagDetected);
for xi = 1:length(tagsFound)
    
    graph.add(BetweenFactorPoint3(j(1,tagsFound(xi)), j(2,tagsFound(xi)), Point3(TagSize, 0, 0), tagNoise));
    graph.add(BetweenFactorPoint3(j(2,tagsFound(xi)), j(3,tagsFound(xi)), Point3(0, TagSize, 0), tagNoise));
    graph.add(BetweenFactorPoint3(j(3,tagsFound(xi)), j(4,tagsFound(xi)), Point3(-TagSize, 0, 0), tagNoise));
    graph.add(BetweenFactorPoint3(j(4,tagsFound(xi)), j(1,tagsFound(xi)), Point3(0, -TagSize, 0), tagNoise));
    
    %% Insert initial estimates to the tags for those whose world location has and hasn't been found
    if tagWorldDetected(tagsFound(xi))
        for yi=3:3:12
            initialEstimate.insert(j(yi/3,tagsFound(xi)),Point3(allTagWorld(yi-2:yi,tagsFound(xi))));
        end    
    else
        for yi=1:4
            initialEstimate.insert(j(yi,tagsFound(xi)), Point3(randn(3, 1)));
        end        
    end
end

disp('Completed Making the entire graph');
%% Solve the graph
disp('Solving Graph');

optimizer = DoglegOptimizer(graph, initialEstimate);
result = optimizer.optimize();

disp('Solved the graph');


%% Plot Initial Estimate
cla
plot3DTrajectory(initialEstimate, 'g-*');
hold on;
plot3DPoints(initialEstimate,'k-*');

for x=1:3:10
    scatter3(allTagWorld(x,~isnan(allTagWorld(x,:))),allTagWorld(x+1,~isnan(allTagWorld(x,:))),allTagWorld(x+2,~isnan(allTagWorld(x,:))),'r');
end
hold off;


%% Plot Result
figure
plot3DPoints(result, []);
plot3DTrajectory(result, 'b-*', true, 0.3);


%% Retrieve landmarks from graph
LandMarksComputed = nan(length(tagsFound),13);
for xi = 1:length(tagsFound)
    for yi = 4:3:13
        LandMarksComputed(xi, yi-2:yi) = [result.at(j((yi-1)/3,tagsFound(xi))).x,    result.at(j((yi-1)/3,tagsFound(xi))).y,    result.at(j((yi-1)/3,tagsFound(xi))).z];
    end
end

LandMarksComputed(:,1) = tagsFound;


%% Retrieve poses from graph
AllPosesComputed = zeros(nStates, 7);

for x = frameNumber:nStates
    pose = result.at(i(x));
    rotationMaatrix = pose.rotation.matrix;
    translationMatrix = pose.translation.vector;
    location = -rotationMaatrix' * translationMatrix;
    AllPosesComputed(x, :) = [location',  rotm2quat(rotationMaatrix)];
end


end

function DetAll = remove_no_detections(DetAll)

emptyCells = cellfun(@isempty,DetAll);
DetAll(emptyCells) = [];

end

function tagsInfo = remove_multiple_detections(detectedTags)
global tagDetected;

[~,indices,~] = unique(detectedTags(:,1));
tagsInfo = detectedTags(indices,:);
tagDetected(tagsInfo(:,1)) = true;

end

function [frameNumber,OriginTag_img,tagsInfo] = detect_origin_tag(OriginTag,DetAll)
global tagDetected nStates;
detected = false;    x=1;
while ~detected && x < nStates
    tagsInfo = remove_multiple_detections(DetAll{x});
    idx = tagsInfo(:,1) == OriginTag;
    if any(idx)
        OriTag_img = tagsInfo(idx,:);
        OriTag_img = reshape(OriTag_img(2:end),2,[]);
        OriginTag_img = [OriTag_img;ones(1,4)];
        
        tagDetected(tagsInfo(:,1)) = true;
        frameNumber = x;
        detected = true;    
    end
    x = x+1;
end

if ~detected
    frameNumber = nan;
    OriginTag_img = nan(1,8);  
end

end

function get_tags_world_position(R,T,K,tagsWithRespectTo,tagsInfo)

global allTagWorld tagWorldDetected;

P = K*[R,T];

projectedImgPoints = P*[tagsWithRespectTo;ones(1,size(tagsWithRespectTo,2))];
lambda = mean(projectedImgPoints(3,:));

tagsImg = [reshape(tagsInfo(:,2:end)',2,[]);ones(1,(size(tagsInfo,1))*4)];

tagsWorld = P\(tagsImg.*lambda);
tagsWorld = tagsWorld./tagsWorld(4,:);

if ~any(isnan(tagsWorld(4,:))) && any(tagsWorld(1,:)) && any(tagsWorld(2,:)) && ~any(abs(tagsWorld(1,:))>15) && ~any(abs(tagsWorld(2,:))>15)
   tagWorldDetected(tagsInfo(:,1)) = true;
   allTagWorld(:,tagsInfo(:,1)) = reshape(tagsWorld(1:3,:),12,[]);   
end

end

function [tagsWorld,tagsCam] = find_common_tags(tagsInfo,K)
global nTags tagWorldDetected allTagWorld;

persistent index tempImg;
index = false(nTags,1);           tempImg = nan(nTags,8);

index(tagsInfo(:,1)) = true;       tempImg(index,:) = tagsInfo(:,2:end); 
common_tags = logical(tagWorldDetected.*index);

tagsImg    = [reshape(tempImg(common_tags,:)',2,[]);ones(1,sum(common_tags)*4)];
tagsCam    = K\tagsImg;
tagsWorld  = reshape(allTagWorld(:,common_tags),3,[]);

end
