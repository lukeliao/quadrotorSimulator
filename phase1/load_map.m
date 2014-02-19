function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.

% Account for rounding error
xy_res = 0.01 * round(100*xy_res);
z_res = 0.01 * round(100*z_res);
margin = 0.01 * round(100*margin);

% Load parameters into map for later use
map.xy_res = xy_res;
map.z_res = z_res;
map.margin = margin;

% Load size of quad to avoid path
map.body_buffer = [0.15 0.15 0.05]; % m, from center

%% Parse text file:

fid = fopen(filename);
C = textscan(fid,'%s'); % returns cell array of white-space delimited words
fclose(fid);

% Get the indices of 'boundary' and 'block' occurences
boundary_idx = 0;
block_idx = [];
for i = 1:length(C{1})
    if strcmp(C{1}{i}, 'boundary')
        boundary_idx = i;
    elseif strcmp(C{1}{i}, 'block')
        block_idx(end+1) = i;
    end
end
% Store dimensions of boundary
map.boundary_dim = zeros(1,6);
for i = boundary_idx+1:boundary_idx+6
    map.boundary_dim(1,i-boundary_idx) = str2double(C{1}{i});
end
map.boundary6 = repmat(map.boundary_dim, 6, 1);
% Store dimensions of blocks
map.block_dim = zeros(length(block_idx), 9);
map.block6 = cell(length(block_idx),1);
for i = 1:length(block_idx)
    for j = block_idx(i)+1:block_idx(i)+9
        map.block_dim(i, j - block_idx(i)) = str2double(C{1}{j});
    end
    map.block6{i} = repmat(map.block_dim(i,1:6),6,1);
end
% Account for margins
% nblocks = length(block_idx);
% map.block_dim(:,1:3) = max(map.block_dim(:,1:3) - margin, repmat(map.boundary_dim(1:3), nblocks, 1));
% map.block_dim(:,4:6) = min(map.block_dim(:,4:6) + margin, repmat(map.boundary_dim(4:6), nblocks, 1));
% Don't let z_res exceed z_dim
map.z_res = min(z_res,  diff(map.boundary_dim([3 6])));
end

