%{
    This class abstracts the GTSAM toolbox. It is created so that it is
    easier to create the SLAM backend.

    Refer to Factor Graphs and GTSAM Introduction
    https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
    and the examples in the library in the GTSAM toolkit. See folder
    gtsam_toolbox/gtsam_examples
%}

classdef VisSlamBackend < handle
    properties (Access = private)
        graph
        initial_estimate
        noise_sigmas
        camera_calib
        tag_size
        state_key
        tags_key
        recent_pose
        result
    end
    
    methods        
        %% Constructor
        function obj = VisSlamBackend(max_num_tags, max_num_states, noise, K, tag_size)
            %{ 
                ArgIn:
                max_num_tags: maximum number of tags that can be expected
                              to be detected
                
                max_num_states: maximum number of states expected to be visited
            
                noise: struct with foll. variables -
                        1.odom - noise between successive states
                        2.meas - noise of the measurement i.e., camera
                        3.tags - noise in the (global) position of tags
                K    : 3x3 camera calibration matrix
            %}
            
            import gtsam.*
            
            % Create graph
            obj.graph = NonlinearFactorGraph;
            obj.initial_estimate = Values;      
            
            % Initialize noise sigmas
            obj.noise_sigmas.odom = noiseModel.Diagonal.Sigmas(noise.odom);
            obj.noise_sigmas.meas = noiseModel.Isotropic.Sigma(2,noise.meas);
            obj.noise_sigmas.tags = noiseModel.Diagonal.Sigmas(noise.tags);
            
            % Import the Camera calibration matrix
            obj.camera_calib = Cal3_S2(K(1,1),K(2,2),K(1,2),K(1,3),K(2,3));
            
            % Save the tag size
            obj.tag_size = tag_size;
            
            % Create keys for the list of states and tags
            obj.state_key = uint64(nan(1,max_num_states));
            obj.tags_key = uint64(nan(4,max_num_tags));
            
            for i=1:max_num_states
               obj.state_key(i) = symbol('i',i);    
            end

            for i=1:max_num_tags*4
               obj.tags_key(i) = symbol('j',i); 
            end            
        end
       
        %% List of other (public) functions:
        priorOriginTag(obj, tag_idx, position_xyz);
        priorFirstState(obj, state_idx, R, T);
        priorState(obj, state_idx, R, T);
        priorTags(obj, tag_idx, position_xyz); % ArgIn: (obj, tag_id, 3x4 positions of each of the 4 corners of the tag) 
        
        addNextState(obj, new_state_idx, old_state_idx, R, T);
        addMeasurements(obj, state_idx, tag_idx, pixel_position);
        addTagConstraints(obj, tag_idx, tag_size);
        
        solve(obj);
        [result, graph, initial_estimate, all_poses, landmarks] = getResult(obj, state_idxs, tag_idxs);
        
        plotInitialEstimate(obj);
        plotResult(obj);
    end
end