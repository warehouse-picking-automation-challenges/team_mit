function [xtraj, snopt_info_iktraj, infeasible_constraint_iktraj, snopt_info_ik, infeasible_constraint_ik] = ...
    runPlanning(r, q0, target_hand_pos, target_hand_ori, options)
% default starting pose
if nargin<1
    q0 = [0,0,0,0,0,0]';
end
if (nargin<2)
    % Somewhere out in front as an example
    target_hand_pos = [0.5, 0.0, 0.5]';
end
if (nargin<3)
    options = [];
end
if ~isfield(options,'visualize')
    options.visualize = false;
end
if ~isfield(options,'N')
    options.N = 10;
end
if ~isfield(options,'T')
    options.T = 1;
end
if ~isfield(options,'pos_tol')
    options.pos_tol = 0.0001;  % in meter
end
if ~isfield(options,'ori_tol')
    options.ori_tol = 0.01;  % in rad 0 to pi
end
if ~isfield(options,'straightness')
    options.straightness = 0.0;  % 0 to 1
end
if ~isfield(options,'target_link')
    options.target_link = 'link_6'; 
end
if ~isfield(options,'ik_only')
    options.ik_only = false; 
end
if (options.visualize)
    v = r.constructVisualizer();
end

hand_idx = findLinkId(r, options.target_link);

T = options.T;
N = options.N;
t_vec = linspace(0,T,N);
Allcons = cell(0,1);

pos_tol = options.pos_tol;
hand_pt = [0,0,0]';  % the transform of hand to tip is handled in python
hand_cons_pos = WorldPositionConstraint(r, hand_idx, hand_pt,...
    target_hand_pos-pos_tol,target_hand_pos+pos_tol,[T, T]);
Allcons{end+1} = hand_cons_pos;

hand_cons_orient = WorldQuatConstraint(r, hand_idx, target_hand_ori,...
    options.ori_tol, [(1.0-options.straightness)*T, inf]);
Allcons{end+1} = hand_cons_orient;


% constraint by in-frame bounding box
if isfield(options,'inframebb')
    hand_cons_inframebb = WorldPositionInFrameConstraint(r, hand_idx,...
        options.tip_hand_transform(1:3,:), options.inframebb.frame_mat,...
        options.inframebb.lb, options.inframebb.ub, [0, T]); 
    Allcons{end+1} = hand_cons_inframebb;
end

% constraint by joint pose bounding box %%%%%%%%
if isfield(options,'target_joint_bb')
    arm_cons_target_joint_bb = PostureConstraint(r, [T, T]); 
    arm_cons_target_joint_bb.setJointLimits(options.target_joint_bb(:,1), options.target_joint_bb(:,2), options.target_joint_bb(:,3));
    
    Allcons{end+1} = arm_cons_target_joint_bb;
end

% set iteration limit
ikoptions = IKoptions(r);
ikoptions = ikoptions.setIterationsLimit(1000000);
ikoptions = ikoptions.setDebug(true);

% compute seeds
q_start_nom = q0;

[q_end_nom,snopt_info_ik,infeasible_constraint_ik] = ...
    inverseKin(r, q_start_nom, q_start_nom, Allcons{:}, ikoptions);
    
if snopt_info_ik > 10
    fprintf('IK fail snopt_info: %d\n', snopt_info_ik);
    fprintf('Infeasible constraints: \n')
    disp(infeasible_constraint_ik)
    
    infeasible_constraint_iktraj = [];
    snopt_info_iktraj = 0;
    xtraj = [];
    return
end

% if ik_only, don't need to computer traj
if options.ik_only
    xtraj = q_end_nom;
    snopt_info_iktraj = 0;
    infeasible_constraint_iktraj = [];
    return 
end


q_start_nom = q_start_nom(1:length(q_end_nom), 1);  % remove hand joints, should do it in a better way
qtraj_guess = PPTrajectory(foh([0 T],[q_start_nom, q_end_nom]));

% do IKTraj
[xtraj,snopt_info_iktraj,infeasible_constraint_iktraj] = inverseKinTraj(r,...
    t_vec,qtraj_guess,qtraj_guess,...
    Allcons{:}, ikoptions);

%q_end = xtraj.eval(xtraj.tspan(end));
% Visualize the trajectory
if options.visualize && snopt_info_iktraj <= 10
    v.playback(xtraj);
end

if snopt_info_iktraj > 10
    fprintf('IK Traj fail snopt_info: %d\n', snopt_info_iktraj);
    fprintf('infeasible constraints: ')
    disp(infeasible_constraint_iktraj)
end




if options.straightness > 0
    % find current pos
    kinsol = r.doKinematics(q0(1:r.getNumPositions));
    pos0   = r.forwardKin(kinsol, hand_idx, hand_pt);
    hand_poses = repmat(pos0,1,N) + (target_hand_pos - pos0)*linspace(0,1,N);
    
    if options.visualize
        lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'apc');
        lcmgl.glColor3f(1,0,1);
        lcmgl.sphere(pos0,0.01,100,100);
        lcmgl.glColor3f(0,0,1);
        lcmgl.sphere(target_hand_pos,0.01,100,100);
        lcmgl.glColor3f(0,1,1);
    end
    
    for i=(N-1):-1:1
        if options.visualize; lcmgl.sphere(hand_poses(:,i),0.01,100,100); end
        
        cons = WorldPositionConstraint(r, hand_idx, hand_pt,...
          hand_poses(:,i)-0.001, hand_poses(:,i)+0.001, [t_vec(i) t_vec(i)]);
        if t_vec(i) < T * (1-options.straightness)
            break;
        end
        Allcons{end+1} = cons;
    end
    if options.visualize
        lcmgl.switchBuffers;
    end
    
    % do IKTraj
    qtraj_guess = xtraj;
    [xtraj,snopt_info_iktraj,infeasible_constraint_iktraj] = inverseKinTraj(r,...
        t_vec,qtraj_guess,qtraj_guess,...
        Allcons{:}, ikoptions);

    %q_end = xtraj.eval(xtraj.tspan(end));
    % Visualize the trajectory
    if options.visualize && snopt_info_iktraj <= 10
        v.playback(xtraj);
    end

    if snopt_info_iktraj > 10
        fprintf('IK Traj fail snopt_info: %d\n', snopt_info_iktraj);
        fprintf('infeasible constraints: ')
        disp(infeasible_constraint_iktraj)
    end
end

end

