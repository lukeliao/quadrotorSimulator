function [ desired_state ] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;

persistent choose_time 
persistent coeffs

% During first call
if isempty(t)    
    % Defined pts for {robot, point, dimension}
    avg_speed = 1; % m/s
    choose_time = cell(length(path), 1);
    for qn = 1:length(path)   
        % First see if we can condense the number of segments in the path 
        path_mod = condense_traj(map, path{qn});
        % Get the complete, 5-entry state for each vertex
        time_vec = zeros(size(path_mod,1),1);
        for s = 2:size(path_mod,1)
            % Keep fitting a polynomial until the acceleration is
            % acceptable
            avg_speed_temp = avg_speed;
            iter = 0;
            while iter < 50
                % Determine time at which it will reach that state
                dt = norm(path_mod(s,:) - path_mod(s-1,:))/avg_speed_temp;
                % Figure out coefficients (cell array with matrix for each
                % dim)
                coeffs{qn}{s-1} = get_coefficients(path_mod(s-1,:), path_mod(s,:), dt);
                % Map of times to decide which leg we're on
                time_vec(s) = time_vec(s-1) + dt;
                max_a = get_max_acc(coeffs{qn}{s-1}, dt);
                if max_a > 4.5
                    avg_speed_temp = 0.8 * avg_speed_temp;
                    iter = iter + 1;
                else
                    break;
                end
            end  
        end
        choose_time{qn} = time_vec;
    end    

% During sim
else
    
    % Determine which set of coefficients to use
    idx = find(choose_time{qn} > t, 1, 'first');
    if isempty(idx)
        idx = length(choose_time{qn});
        t = choose_time{qn}(end);
    end
    polynomials = coeffs{qn}{idx-1};
    % Choose the time into this segment
    dt = t - choose_time{qn}(idx-1);
    % Get position, velocity, and acceleration for this state
    pos = zeros(1,3);
    vel = zeros(1,3);
    acc = zeros(1,3);
    for d = 1:3
        p = polynomials{d}';
        pos(d) = sum(p .* dt.^(5:-1:0));
        vel(d) = sum((5:-1:1) .* p(1:5) .* dt.^(4:-1:0));
        acc(d) = 20 * p(1) * dt^3 + 12 * p(2) * dt^2 + 6 * p(3) * dt + 2 * p(4);
    end
    
    desired_state.pos = pos';
    desired_state.vel = vel';
    desired_state.acc = acc';
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
    
end
end

function max_a = get_max_acc(coeffs_all, dt)
% There's definitely a better, more analytical way to do this
tv = linspace(0, dt, 100);
acc = zeros(3, length(tv));
for d = 1:3
    coeffs = coeffs_all{d};
    acc(d,:) = 20*coeffs(1)*tv.^3 + 12*coeffs(2)*tv.^2 + 6*coeffs(3)*tv + 2*coeffs(4);
end
max_a = max(rownorm(acc'));
end

function n = rownorm(v)
n = sqrt(sum(v.^2,2));
end

function coeffs = get_coefficients(a, b, dt)
    % Get polynomials for quintic polynominal
    coeffs = cell(3,1);
    A = [zeros(1,5) 1; dt.^(5:-1:0); zeros(1,4) 1 0; (5:-1:0).*dt.^(4:-1:-1); ...
         0 0 0 2 0 0; 20*dt^3 12*dt^2 6*dt 2 0 0 ];
    for d = 1:3
       S = [a(d) b(d) 0 0 0 0]';
       coeffs{d} =  A\S;
    end
end

function cond_traj = condense_traj(map, path)
    % Put path into as few, direct segments as possible
    ref = single(path(1,:));
    cond_traj = ref;
    npoints = 55;
    for i = 2:size(path,1)
        shortcut = [linspace(ref(1), path(i,1), npoints)', ...
                    linspace(ref(2), path(i,2), npoints)', ...
                    linspace(ref(3), path(i,3), npoints)'];
        if any(collide(map, shortcut))
            ref = path(i-1,:);
            cond_traj = [cond_traj; ref];
            npoints = 50;
        end
        if  i == size(path,1)
            cond_traj = [cond_traj; path(i,:)];
        end
        npoints = npoints + 5;
    end
     plot3(cond_traj(:,1), cond_traj(:,2), cond_traj(:,3), 'c-')

end
