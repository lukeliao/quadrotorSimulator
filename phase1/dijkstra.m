function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
% 
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.

% Search mode
if nargin < 4
    astar = false;
end

% Show real-time animation
show_plot = 1;

% Greedy instead of astar
use_greedy = 1;

% Pull out necessary information
boundary = map.boundary_dim;
xy_res = map.xy_res;
z_res = map.z_res;
body_buffer = map.body_buffer;

if show_plot
    animate_frame;
    plot3(start(1), start(2), start(3), 'g.','MarkerSize',20);    
    plot3(goal(1), goal(2), goal(3),'b.','MarkerSize',20);
    xlim(map.boundary_dim([1 4]));
    ylim(map.boundary_dim([2 5]));
    zlim(map.boundary_dim([3 6]));
    plot_obstacles(map);       
    boxh = []; % handles of explored node boxes

    % Wait for a few seconds before continuing
    for i = 1:25
        pause(1/25);
        animate_frame;
    end
    grid_lines = plot_3d_grid(xy_res, z_res, boundary);
    for i = 1:25
        pause(1/25);
        animate_frame;
    end
end

% Calculate 'goal box'
goal_min = goal - [xy_res xy_res z_res];
goal_max = goal + [xy_res xy_res z_res];

% All possible expansion directions in terms of indices, steps, and cost
dpi = [eye(3,'int16'); -eye(3, 'int16')];
dp = bsxfun(@times, double(dpi), [xy_res xy_res z_res]);
dps = abs(sum(dp, 2));

% Initial condition
current = start;
parent = NaN(1,3);
cur_cost = 0;

% Keep track of neighbors and visited node
if astar
    neighbors = zeros(0,8,'single'); % Will need an extra field for heuristic
else    
    neighbors = zeros(0,7,'single');
end

% Neighbor indices
nis = zeros(0, 3, 'int16');

% Build visitor, cost, and collision maps
[visited offsets res min_m max_i] = create_visited_map(map, start);
isNeighbor = -1*single(ones(size(visited)));
col_map = create_collision_map(map, size(visited), offsets, res, min_m, max_i, body_buffer);

% Get starting and ending indices
ci = int16(real_to_idx(start, offsets, res, min_m, max_i, 0));
goali = int16(real_to_idx(goal, offsets, res, min_m, max_i, 0));

% Flags and info
no_path = 0;
num_expanded = 0;

% Search
while true
    


   % Determine nodes 'in consideration'
   incons = bsxfun(@plus, current, dp);
   inconsi = bsxfun(@plus, ci, dpi);
   % Consider each point, and proceed accordingly
   for j = 1:size(inconsi,1)
        if show_plot
         animate_frame;
        end
       % Extract point information
       point = incons(j,:);
       pidx  = inconsi(j,:);
       dpix = dpi(j,:);
       % Check if out of bounds
       if any(pidx < 1)
           continue;
       elseif any(pidx > max_i)
           continue;
       % Check if already neighbor
       elseif isNeighbor(pidx(1), pidx(2), pidx(3)) ~= -1
           continue;
       % Check if already visited 
       elseif ~isnan(visited(pidx(1), pidx(2), pidx(3), 1))
           continue;
       % Check if collision
       elseif col_map{pidx(3)}(pidx(1), pidx(2))         
           continue;
       end
       % Calculate tentative cost
       tent_cost = cur_cost + dps(j);
       % If just dijkstra
       if ~astar
           % Figure out where to put it in the neighbor list
           idx = find(tent_cost <= neighbors(:,7), 1, 'first');           
           if isempty(idx)
               neighbors = [neighbors; point, current, tent_cost];
               nis = [nis; pidx];
           elseif idx == 1
               neighbors = [point, current, tent_cost; neighbors];
               nis = [pidx; nis];
           else          
                neighbors = [neighbors(1:idx-1,:); [point, current, tent_cost]; neighbors(idx:end,:)];
                nis = [nis(1:idx-1,:); pidx; nis(idx:end,:)];
           end
           % Add cost to cost_map
           isNeighbor(pidx(1), pidx(2), pidx(3)) = tent_cost;
       else
           % If using A*
           hueristic = (use_greedy ~= 1)*tent_cost + est_dist(pidx, goali, res);
           % Figure out where to put it in the neighbor list
           idx = find(hueristic <= neighbors(:,8), 1, 'first');
           if isempty(idx)
               neighbors = [neighbors; point, current, tent_cost, hueristic];
               nis = [nis; pidx];
           elseif idx == 1
               neighbors = [point, current, tent_cost, hueristic; neighbors];
               nis = [pidx; nis];
           else          
                neighbors = [neighbors(1:idx-1,:); [point, current, tent_cost, hueristic]; neighbors(idx:end,:)];
                nis = [nis(1:idx-1,:); pidx; nis(idx:end,:)];
           end
           % Add cost to cost_map
           isNeighbor(pidx(1), pidx(2), pidx(3)) = hueristic;
       end

       % Incrememnt num_expanded
       num_expanded = num_expanded + 1;
       % Plot expanded point, if desired
       if show_plot
%            plot3([current(1) point(1)], [current(2) point(2)], [current(3) point(3)], 'b.-');
            boxh(end+1) = plot_box(point, xy_res, z_res);
            drawnow;
       end
   end
      
   % Check if there are no more neighbors -> no path
   if isempty(neighbors)
       no_path = 1;
       break;
   end
   
   % Fill parent in the current entry for visited (for tracing back)
   vidx = ci;
   visited(vidx(1), vidx(2), vidx(3), :) = parent;
   
   % Choose new current and parent (list is sorted)
   current = neighbors(1,1:3);
   ci = nis(1,1:3);
   parent = neighbors(1,4:6);   
   cur_cost = neighbors(1,7);
   % Remove from neighbors
   neighbors = neighbors(2:end,:);
   nis = nis(2:end,:);
   isNeighbor(vidx(1), vidx(2), vidx(3)) = 0;   
   
   % Plot, if desired
   if show_plot
%        plot3(current(1), current(2), current(3), 'yo');
       drawnow;
   end
      
   % See if we're close enough to the goal to stop
   if all(current >= goal_min) && all(current <= goal_max)
       break;
   end  

end

% Don't do anything if there as no path
path = zeros(0,3);
if no_path
    return;
end

% Trace steps
path = current;
current = parent;
while ~all(almostEqual(start, current))
    path = [current; path];    
    cidx = real_to_idx(current, offsets, res, min_m, max_i, 0); 
    current = squeeze(visited(cidx(1), cidx(2), cidx(3), :))';
end  

% Add start and end
path = [start; path; goal];   

if show_plot
     ph = plot3(path(:,1), path(:,2), path(:,3), 'g-');
    for i = 1:length(grid_lines)
        delete(grid_lines(i));
    end 
    for i = 1:50
        pause(1/25);
        animate_frame;
    end
    for i = 1:length(boxh)
        delete(boxh(i));
    end
    for i = 1:50
        pause(1/25);
        animate_frame;
    end  
    delete(ph)
end
end

function grid_lines = plot_3d_grid(xy_res, z_res, boundary)
% build vectors
xv = boundary(1)+xy_res/2:xy_res:boundary(4)-xy_res/2;
yv = boundary(2)+xy_res/2:xy_res:boundary(5)-xy_res/2;
zv = boundary(3)+xy_res/2:z_res:boundary(6)-z_res/2;
grid_lines = [];
% Plot vertical lines
for xi = min(xv):xy_res:max(xv)
    for yi = min(yv):xy_res:max(yv)
        grid_lines(end+1) = plot3([xi xi], [yi yi], [min(zv) max(zv)], '--','color',[0.8 0.8 0.8]);
    end
end
% Horizontal
for yi = min(yv):xy_res:max(yv)
    for zi = min(zv):z_res:max(zv)
        grid_lines(end+1) = plot3([min(xv) max(xv)], [yi yi], [zi zi], '--','color',[0.8 0.8 0.8]);
    end
end
% Last dim
for  xi = min(xv):xy_res:max(xv)
    for zi = min(zv):z_res:max(zv)
        grid_lines(end+1) = plot3([xi xi], [min(yv) max(yv)], [zi zi], '--','color',[0.8 0.8 0.8]);
    end
end
end

function col_map = create_collision_map(map, dims, offsets, res, min_m, max_i, body_buffer)
    % Create a collision map that can be indexed to see if there is a
    % collision
    col_mat = zeros(dims(1:3));
    res4 = repmat(res, length(map.block_dim(:,1)), 1);
    quad_body = repmat(body_buffer, size(map.block_dim, 1), 1);
    % Snap to grid points
    lower_bound = real_to_idx(map.block_dim(:,1:3)-quad_body-map.margin, offsets, res4, min_m, max_i, 1);
    upper_bound = real_to_idx(map.block_dim(:,4:6)+map.margin+quad_body, offsets, res4, min_m, max_i, -1);
    for i = 1:length(map.block_dim(:,1))
        col_mat(lower_bound(i,1):upper_bound(i,1),...
                lower_bound(i,2):upper_bound(i,2),...
                lower_bound(i,3):upper_bound(i,3)) = 1;
    end
    % Make sparse--can index faster
    col_map = cell(dims(3),1);
    for i = 1:dims(3)
        col_map{i} = sparse(squeeze(col_mat(:,:,i)));
    end
end

function [visited offsets res min_m dim] = create_visited_map(map, start)
    % Pull out data
    min_m = map.boundary_dim(1:3);
    max_m = map.boundary_dim(4:6);
    res = single([map.xy_res map.xy_res map.z_res]);
    % Find offsets
    offsets = mod(start - min_m, res);
    % Find dimensions of map
    dim = floor_approx((max_m - min_m - offsets)./res)+1;
    % Create map
    visited = NaN(dim(1), dim(2), dim(3), 3, 'single');
end

function v = floor_approx(v)
for i=1:length(v)
    if abs(round(v(i)) - v(i)) < 0.001
        v(i) = round(v(i));
    else
        v(i) = floor(v(i));
    end
end
end

function idx = real_to_idx(v, offsets, res, min_m, max_i, snap_dir)
    % res should match dimension of v
    if snap_dir == 0
        idx = round((bsxfun(@minus,v, min_m+offsets))./ res)+1;
    elseif snap_dir == -1
        idx = floor((bsxfun(@minus,v, min_m+offsets))./ res)+1;
    else
        idx = ceil((bsxfun(@minus,v, min_m+offsets))./ res)+1;
    end
    % Make sure it's in the map
    idx = bsxfun(@max, idx, [1 1 1]);
    idx = bsxfun(@min, idx, max_i);
end

function estimate = est_dist(point, goal, res)
    estimate = sum(single(abs(point-goal)).*res);    
%     estimate = norm(single(point-goal).*res);
end

function v = almostEqual(a,b)
    v = (abs(a-b) < 1e-4);
end

function boxh = plot_box(point, xy_res, z_res)
x = [point(1)-xy_res/2 point(1)+xy_res/2];
y = [point(2)-xy_res/2 point(2)+xy_res/2];
z = [point(3)-z_res/2 point(3)+z_res/2];
verts = [];
for xi = 1:2
    for yi = 1:2
        for zi = 1:2
            verts(end+1, 1:3) = [x(xi) y(yi) z(zi)];
        end
    end
end
faces = [1 3 4 2; 5 6 8 7;
         1 2 6 5; 3 4 8 7;
         1 3 7 5; 2 4 8 6];
 col = [184 234 242];
 edgecol = [122 122 122];
 boxh = patch('Faces',faces,'Vertices',verts,'FaceColor',col./255,...
        'EdgeColor',edgecol./255,'FaceAlpha',0.2,'EdgeAlpha', 0.6);  
end

function plot_obstacles(map)
% Show obstacles...taken from plot_path
for i = 1:length(map.block_dim(:,1))
    b = map.block_dim(i,:);
    x = [b(1) b(4)];
    y = [b(2) b(5)];
    z = [b(3) b(6)];
    col = b(7:9);
    verts = [];
    for xi = 1:2
        for yi = 1:2
            for zi = 1:2
                verts(end+1, 1:3) = [x(xi) y(yi) z(zi)];
            end
        end
    end
    faces = [1 3 4 2; 5 6 8 7;
             1 2 6 5; 3 4 8 7;
             1 3 7 5; 2 4 8 6];
   patch('Faces',faces,'Vertices',verts,'FaceColor',col./255,...
          'EdgeColor',col./255,'FaceAlpha',0.35, 'EdgeAlpha',0.7);  
%    hold on;
end
end