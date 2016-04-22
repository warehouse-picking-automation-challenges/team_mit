

function ret_json = ikTrajServer_internal(r, data_json, options)
    data = JSON.parse(data_json); 
    
    % 1. Get hand target pose
    q0 = cell2mat(data.q0)';
    
    target_hand_pos = [];
    target_hand_ori = [];
    if isfield(data, 'target_hand_pos')
        target_hand_pos = cell2mat(data.target_hand_pos)';  %[x,y,z]
    end
    if isfield(data, 'target_hand_ori')
        target_hand_ori = cell2mat(data.target_hand_ori)';  %[qw,qx,qy,qz]
    end
    if isfield(data, 'tip_hand_transform')
        options.tip_hand_transform = cell2mat(data.tip_hand_transform)';  %[x,y,z, qw,qx,qy,qz]
    end
    
    if isfield(data, 'straightness')
        options.straightness = data.straightness;
    end
    if isfield(data, 'pos_tol')
        options.pos_tol = data.pos_tol;
    end
    if isfield(data, 'ori_tol')
        options.ori_tol = data.ori_tol;
    end
    if isfield(data, 'inframebb')
        options.inframebb = data.inframebb;
        options.inframebb.tip_hand_transform = options.tip_hand_transform'; % xyzrpy
        options.inframebb.lb = cell2mat(options.inframebb.lb)';
        options.inframebb.ub = cell2mat(options.inframebb.ub)';
        options.inframebb.frame_mat = cellcelltomat(options.inframebb.frame_mat); % need to special treat matrix
    end
    if isfield(data, 'target_link')
        options.target_link = data.target_link;
    end
    if isfield(data, 'target_joint_bb')
        options.target_joint_bb = cell2mat(data.target_joint_bb{1});
    end
    if isfield(data, 'N')
        options.N = data.N;
    end
    if isfield(data, 'ik_only')
        options.ik_only = data.ik_only;
    end
    
    % 2. Get Other Object Pose (for collision avoidance), not used right
    % now
    if isfield(data, 'object_list')
        positions = zeros(3, length(data.object_list));
        orientations = zeros(4, length(data.object_list));

        % 2.1 Prepare objects info: parse obj pose information from message
        for i = 1:length(data.object_list)
            object = data.object_list{i};
            pose = object.pose;
            positions(1:3, i) = [pose.position.x; ...
                                 pose.position.y; ...
                                 pose.position.z];
            orientations(1:4, i) = [pose.orientation.w; ...
                                    pose.orientation.x; ...
                                    pose.orientation.y; ...
                                    pose.orientation.z];
        end
    end
    
    % 3. display options
    options
    
    % 4. do planning, will cache the robot
    [xtraj, snopt_info_iktraj, infeasible_constraint_iktraj, snopt_info_ik, infeasible_constraint_ik]...
                                  = runPlanning(r, q0, target_hand_pos, target_hand_ori, options);
                                  
    % 5. publish the data in xtraj with a standard ROS type
    q_traj = [];
    if ~isempty(xtraj)
        %ts = xtraj.pp.breaks;
        if isobject(xtraj)
            ts = linspace(xtraj.pp.breaks(1), xtraj.pp.breaks(end), length(xtraj.pp.breaks)*10);
            q_and_qdot = xtraj.eval(ts);
            q_traj = q_and_qdot(1:6, 1:end);  % this q's are in rad
        else 
            % for ik only
            q_traj = xtraj;
        end
    end
    
    ret.q_traj = q_traj;
    ret.snopt_info_iktraj = snopt_info_iktraj;
    ret.infeasible_constraint_iktraj = infeasible_constraint_iktraj;
    ret.snopt_info_ik = snopt_info_ik;
    ret.infeasible_constraint_ik = infeasible_constraint_ik;
    
    %ret_json = savejson('', ret, '/tmp/ret');
    ret_json = savejson('', ret);
end

function B = cellcelltomat(A)
    B = zeros(length(A), length(A{1}));
    for i=1:length(A)
        B(i,:) = cell2mat(A{i});
    end
end
